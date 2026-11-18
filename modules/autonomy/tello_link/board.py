"""지도 그림(화이트보드) 1단계: 진입로와 사람 동선을 게이트웨이에 두고 모든 화면에 알린다(표준 라이브러리만).

계약 초안 map-board/0.1(4drone parts/drafts/map-whiteboard-plan-1117.md 5-1절). 좌표는 map(자율 비행 지도: 켠 자리 원점,
x 앞·y 왼쪽, m)이다. 발견 확인·오탐(detection_inbox.set_review)과 같은 길을 따른다.

- GET  /api/board   {schema_version, type: "board", run_id, rev, frame, width_m, walk_mps, grid_ver, items}(지운 것은 빼고 만든 순서)
- POST /api/board   {op: "add", client, item} · {op: "delete", client, id}
- ws(모든 화면)      {schema_version, type: "board", op: add·delete·route, rev, at, client, item, logged}
  route는 지도가 바뀌어 게이트웨이가 다시 계산한 동선(client "gateway"). logged면 비행 기록 줄(/api/events에도 남는다).
- 회차 폴더          board.jsonl(ws와 같은 줄), board.json(GET과 같은 모양, 바뀔 때마다 덮어씀)

그림 종류(1단계):
- entry 진입로: pts [[x, y]], text(없으면 "진입 n")
- route 동선: from {type: entry(id)·start·point, x, y}, to {type: person(n)·point, x, y}. 게이트웨이가 route_plan으로 계산해
  result {state: ok·partial·no_path·no_map, pts, length_m, walk_s, gap_m, …, grid_ver}를 붙인다.
  자율 비행 지도 번호가 바뀌면 RECALC_MIN_S에 한 번까지 다시 계산하고, 결과가 달라졌을 때만 알린다.
그림은 드론 명령이 아니다(드론에는 아무것도 보내지 않는다).
"""

from __future__ import annotations

import json
import math
import os
import threading
import time

import route_plan
from mode_arbiter import SCHEMA_VERSION
from tello_link import now_iso

MAX_ITEMS = 300        # 회차 하나의 그림 수(지운 것 포함)
MAX_TEXT = 60          # 진입로 이름 글자 수
MAX_COORD_M = 1000.0   # 좌표 상한(잘못 보낸 값 막기)
RECALC_MIN_S = 3.0     # 지도가 바뀌어 동선을 다시 계산하는 최소 간격
POLL_S = 1.0           # 지도 번호를 보는 간격
CLIENTS = ("monitor", "control")
CHANGED_M = 0.3        # 길이·남은 거리가 이만큼 넘게 바뀌어야 다시 알린다


class BoardError(ValueError):
    pass


class Board:
    def __init__(self, run_dir: str | None, run_id: str | None, emit=None, *, grid=None, discovered=None,
                 clock=time.monotonic):
        self.run_dir, self.run_id = run_dir, run_id
        self.emit = emit or (lambda msg: None)
        self.grid = grid or (lambda: None)                  # 자율 비행 지도(게이트웨이 autonomy_grid), 없으면 None
        self.discovered = discovered or (lambda: None)      # 찾은 장애물(autonomy_discovered)
        self.clock = clock
        self._lock = threading.Lock()                       # 그림·판 번호·기록 파일
        self._calc = threading.Lock()                       # 지도 비용 칸(같은 지도 판이면 동선끼리 같이 쓴다)
        self.items: dict[str, dict] = {}                    # id → 그림(지운 것도, 만든 순서)
        self.rev = 0
        self._seq = 0
        self._counts = {"entry": 0, "route": 0, "point": 0}
        self._cmap = None
        self._cmap_key = None
        self._map_note = "no_grid"                          # 지도가 없거나 못 쓸 때 까닭
        self._calc_key = None                               # 동선을 마지막으로 계산한 지도
        self._calc_at = -math.inf
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    # ---------- 화면 ----------
    def snapshot(self) -> dict:
        with self._lock:
            items = [dict(it) for it in self.items.values() if not it.get("deleted")]
            rev = self.rev
        return {"schema_version": SCHEMA_VERSION, "type": "board", "run_id": self.run_id, "rev": rev, "frame": "map",
                "width_m": route_plan.WIDTH_M, "walk_mps": route_plan.WALK_MPS, "grid_ver": self._calc_key_ver(),
                "items": items}

    def apply(self, msg) -> tuple[int, dict]:
        """화면이 보낸 바꿈. (HTTP 상태, 응답). 200 저장, 400 잘못된 그림, 404 모르는 그림, 413 그림이 너무 많음, 500 기록 실패."""
        if not isinstance(msg, dict):
            return 400, {"ok": False, "reason": "invalid_message"}
        client = msg.get("client") if msg.get("client") in CLIENTS else "monitor"
        op = msg.get("op")
        try:
            if op == "add":
                return self._add(msg.get("item"), client)
            if op == "delete":
                return self._delete(msg.get("id"), client)
        except BoardError as e:
            return 400, {"ok": False, "reason": "invalid_item", "detail": str(e)}
        return 400, {"ok": False, "reason": "invalid_op"}

    # ---------- 지도가 바뀌면 ----------
    def start(self) -> None:
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name="board-routes", daemon=True)
            self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)

    def _run(self) -> None:
        while not self._stop.wait(POLL_S):
            try:
                self.tick()
            except Exception:   # 동선 다시 계산이 게이트웨이를 멈추지 않게(다음에 다시)
                pass

    def tick(self) -> None:
        """지도 번호가 바뀌었으면(RECALC_MIN_S에 한 번까지) 동선을 다시 계산하고, 달라진 것만 알린다."""
        with self._lock:
            routes = [it["id"] for it in self.items.values() if it["kind"] == "route" and not it.get("deleted")]
        if not routes:
            return
        grid = self._grid_now()
        if grid is None:
            return   # 지도가 잠깐 없으면(자율 비행이 새로 연결됨) 마지막 결과를 둔다
        if self._key(grid) == self._calc_key or self.clock() - self._calc_at < RECALC_MIN_S:
            return
        with self._calc:
            cmap = self._costmap(grid)
            self._calc_key, self._calc_at = self._key(grid), self.clock()
            for rid in routes:
                with self._lock:
                    item = self.items.get(rid)
                    if item is None or item.get("deleted"):
                        continue
                    start, goal, old = (item["from"]["x"], item["from"]["y"]), (item["to"]["x"], item["to"]["y"]), item["result"]
                new = self._result(cmap, start, goal)
                if not _changed(old, new):
                    continue
                with self._lock:
                    if item.get("deleted"):
                        continue
                    item["result"] = new
                    self.rev += 1
                    item["rev"] = self.rev
                    msg = self._message("route", "gateway", item, logged=old.get("state") != new["state"])
                    self._record(msg)
                self.emit(msg)

    # ---------- 내부 ----------
    def _add(self, raw, client: str) -> tuple[int, dict]:
        if not isinstance(raw, dict):
            raise BoardError("item")
        kind = raw.get("kind")
        if kind not in ("entry", "route"):
            raise BoardError("kind")
        with self._lock:
            if len(self.items) >= MAX_ITEMS:
                return 413, {"ok": False, "reason": "too_many_items"}
        if kind == "entry":
            pts = raw.get("pts")
            if not isinstance(pts, list) or len(pts) != 1:
                raise BoardError("pts")
            x, y = _xy(pts[0])
            text = _text(raw.get("text"))
            body = {"pts": [[x, y]]}
        else:
            src, dst = raw.get("from"), raw.get("to")
            if not isinstance(src, dict) or not isinstance(dst, dict):
                raise BoardError("from/to")
            src = self._endpoint(src, ("entry", "start", "point"))
            dst = self._endpoint(dst, ("person", "point"))
            with self._calc:   # 지금 지도로 바로 계산(지도 판은 다시 계산과 같이 쓴다)
                grid = self._grid_now()
                cmap = self._costmap(grid) if grid else None
                result = self._result(cmap, (src["x"], src["y"]), (dst["x"], dst["y"]),
                                      None if grid else "no_grid")
            body = {"from": src, "to": dst, "result": result}
            text = None
        with self._lock:
            if len(self.items) >= MAX_ITEMS:
                return 413, {"ok": False, "reason": "too_many_items"}
            self._counts[kind] += 1
            if kind == "route" and body["to"]["type"] == "point":
                self._counts["point"] += 1
                body["to"]["label"] = f"지정 위치 {self._counts['point']}"
            self._seq += 1
            self.rev += 1
            item = {"id": f"b-{self._seq:04d}", "kind": kind, "n": self._counts[kind], **body,
                    "client": client, "at": now_iso(), "rev": self.rev}
            if kind == "entry":
                item["text"] = text or f"진입 {item['n']}"
            msg = self._message("add", client, item, logged=True)
            try:
                self._record(msg, strict=True)
            except OSError as e:   # 기록에 못 남기면 넣지 않는다(화면과 기록이 어긋나지 않게)
                self.rev -= 1
                self._seq -= 1
                self._counts[kind] -= 1
                if kind == "route" and body["to"]["type"] == "point":
                    self._counts["point"] -= 1
                return 500, {"ok": False, "reason": "store_failed", "detail": str(e)}
            self.items[item["id"]] = item
            self._write_snapshot()
        self.emit(msg)
        return 200, {"ok": True, "rev": item["rev"], "item": dict(item)}

    def _delete(self, item_id, client: str) -> tuple[int, dict]:
        with self._lock:
            item = self.items.get(item_id) if isinstance(item_id, str) else None
            if item is None or item.get("deleted"):
                return 404, {"ok": False, "reason": "not_found"}
            self.rev += 1
            gone = {**item, "deleted": True, "rev": self.rev}
            msg = self._message("delete", client, gone, logged=True)
            try:
                self._record(msg, strict=True)
            except OSError as e:
                self.rev -= 1
                return 500, {"ok": False, "reason": "store_failed", "detail": str(e)}
            self.items[item_id] = gone
            self._write_snapshot()
        self.emit(msg)
        return 200, {"ok": True, "rev": gone["rev"], "item": dict(gone)}

    def _endpoint(self, raw: dict, types: tuple) -> dict:
        kind = raw.get("type")
        if kind not in types:
            raise BoardError("type")
        if kind == "entry":   # 진입로는 그 그림의 자리·이름(만들 때 것)을 쓴다
            with self._lock:
                entry = self.items.get(raw.get("id")) if isinstance(raw.get("id"), str) else None
            if entry is None or entry["kind"] != "entry" or entry.get("deleted"):
                raise BoardError("entry")
            (x, y), = entry["pts"]
            return {"type": "entry", "id": entry["id"], "x": x, "y": y, "label": entry["text"]}
        x, y = _xy((raw.get("x"), raw.get("y")))
        if kind == "person":
            n = raw.get("n")
            if isinstance(n, bool) or not isinstance(n, int) or not 1 <= n <= 999:
                raise BoardError("n")
            return {"type": "person", "n": n, "x": x, "y": y, "label": f"사람 {n}"}
        return {"type": kind, "x": x, "y": y, "label": "출발" if kind == "start" else "찍은 자리"}

    def _grid_now(self) -> dict | None:
        try:
            grid = self.grid()
        except Exception:
            return None
        return grid if isinstance(grid, dict) else None

    def _found_now(self) -> dict | None:
        try:
            found = self.discovered()
        except Exception:
            return None
        return found if isinstance(found, dict) else None

    def _key(self, grid: dict):
        found = self._found_now()
        return grid.get("ver"), json.dumps((found or {}).get("obstacles"), sort_keys=True)

    def _calc_key_ver(self):   # 마지막으로 쓴 지도 번호(동선 결과마다 grid_ver도 있다)
        return self._cmap_key[0] if self._cmap_key else None

    def _costmap(self, grid: dict):
        """지도 한 판의 비용 칸(번호·찾은 장애물이 같으면 다시 만들지 않는다). 못 쓰는 지도면 None(까닭은 _map_note)."""
        key = self._key(grid)
        if key != self._cmap_key:
            try:
                self._cmap, self._map_note = route_plan.CostMap(grid, self._found_now()), None
            except ValueError as e:
                self._cmap, self._map_note = None, "grid_too_big" if "grid_too_big" in str(e) else "invalid_grid"
            self._cmap_key = key
        return self._cmap

    def _result(self, cmap, start, goal, reason: str | None = None) -> dict:
        if cmap is None:   # 지도가 없다(no_grid)·못 쓰는 지도(invalid_grid·grid_too_big)
            out = {"state": "no_map", "reason": reason or self._map_note or "no_grid", "pts": [], "length_m": None,
                   "walk_s": None, "gap_m": None, "snap_start_m": None, "snap_goal_m": None}
        else:
            out = route_plan.plan(cmap, start, goal)
        out["grid_ver"] = cmap.ver if cmap is not None else None
        out["at"] = now_iso()
        return out

    def _message(self, op: str, client: str, item: dict, *, logged: bool) -> dict:
        return {"schema_version": SCHEMA_VERSION, "type": "board", "op": op, "rev": item["rev"], "at": now_iso(),
                "client": client, "item": dict(item), "logged": logged}

    def _record(self, msg: dict, strict: bool = False) -> None:
        """board.jsonl에 한 줄. strict면 못 쓸 때 OSError(넣기·지우기는 되돌린다), 아니면(다시 계산) 넘어간다."""
        if not self.run_dir:
            return
        try:
            with open(os.path.join(self.run_dir, "board.jsonl"), "a", encoding="utf-8") as f:
                f.write(json.dumps(msg, ensure_ascii=False) + "\n")
        except OSError:
            if strict:
                raise
        if not strict:
            self._write_snapshot()

    def _write_snapshot(self) -> None:
        """board.json: 지금 그림 전부(GET /api/board와 같은 모양). 임시 파일에 쓰고 바꿔 끼운다. 못 쓰면 넘어간다(기록은 jsonl)."""
        if not self.run_dir:
            return
        snap = {"schema_version": SCHEMA_VERSION, "type": "board", "run_id": self.run_id, "rev": self.rev, "frame": "map",
                "width_m": route_plan.WIDTH_M, "walk_mps": route_plan.WALK_MPS, "grid_ver": self._calc_key_ver(),
                "items": [it for it in self.items.values() if not it.get("deleted")], "saved_at": now_iso()}
        path = os.path.join(self.run_dir, "board.json")
        try:
            with open(path + ".tmp", "w", encoding="utf-8") as f:
                json.dump(snap, f, ensure_ascii=False)
            os.replace(path + ".tmp", path)
        except OSError:
            pass


def _xy(pair) -> tuple[float, float]:
    if not isinstance(pair, (list, tuple)) or len(pair) != 2:
        raise BoardError("xy")
    out = []
    for v in pair:
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or abs(v) > MAX_COORD_M:
            raise BoardError("xy")
        out.append(round(float(v), 2))
    return out[0], out[1]


def _text(value) -> str | None:
    if value is None:
        return None
    if not isinstance(value, str):
        raise BoardError("text")
    text = " ".join(value.split())   # 줄바꿈·제어 문자는 띄어쓰기 하나로
    if len(text) > MAX_TEXT:
        raise BoardError("text")
    return text or None


def _changed(old: dict, new: dict) -> bool:
    """알릴 만큼 달라졌는지: 상태가 바뀌었거나 길이·남은 거리가 CHANGED_M 넘게 바뀜."""
    if old.get("state") != new.get("state"):
        return True
    for key in ("length_m", "gap_m"):
        a, b = old.get(key), new.get(key)
        if (a is None) != (b is None) or (a is not None and abs(a - b) > CHANGED_M):
            return True
    return False
