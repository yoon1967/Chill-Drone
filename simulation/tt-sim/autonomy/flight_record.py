"""4단계 비행 기록: 지도에 넣은 깊이 영상과 그 순간의 추정 위치, 전방 ToF. ROS와 무관.

비행 노드가 지도에 넣은 그대로(바닥 배율을 맞추기 전 깊이, 위치 맞추기 보정을 빼기 전 위치)를 남겨, 시뮬레이터
없이 지도 만들기와 위치 맞추기를 다시 돌려 볼 수 있게 한다 (tools/replay_map.py). 정답 위치는 따로 기록된
truth_log.csv와 시각으로 맞춘다.

chunk장마다 depth_NNNN.npz로 쓴다 (깊이는 float16). 비행을 강제로 꺼도 앞부분은 남는다. 임시 파일에 다 쓴 뒤
이름을 바꿔, 쓰는 도중에 꺼져도 깨진 파일이 남지 않는다 (시간 초과로 끌 때 마지막 덩어리가 깨졌다).
압축·쓰기(100장에 약 1초)는 따로 도는 스레드가 한다. 비행 노드 안에서 바로 쓰면 그동안 조종 주기가 멈췄다.
- depth_NNNN.npz: t (N,), pose (N, 4: x, y, z, yaw), depth (N, H, W)
- tof.npz: t (M,), pose (M, 4), range (M,)  (끝날 때 또는 chunk마다 덮어쓴다)
"""
from __future__ import annotations

import atexit
import json
import os
import queue
import threading
import zipfile
from pathlib import Path

import numpy as np


class FlightRecorder:
    def __init__(self, out_dir, chunk: int = 100, meta: dict | None = None):
        self.dir = Path(out_dir)
        self.dir.mkdir(parents=True, exist_ok=True)
        for old in list(self.dir.glob("depth_*.npz")) + list(self.dir.glob("*.part")) + list(self.dir.glob("tof.npz")):
            old.unlink()
        self.chunk = chunk
        self.n_chunks = 0
        self.frames: list[tuple[float, tuple, np.ndarray]] = []
        self.tof: list[tuple[float, tuple, float]] = []
        self._tof_written = 0
        self.count = 0
        (self.dir / "meta.json").write_text(json.dumps(meta or {}, ensure_ascii=False, indent=1), encoding="utf-8")
        self._jobs: queue.Queue = queue.Queue()
        self._writer = threading.Thread(target=self._write_loop, daemon=True)
        self._writer.start()
        atexit.register(self.close)   # 끝날 때 밀린 쓰기를 마친다

    def add_depth(self, t: float, pose, depth: np.ndarray) -> None:
        self.frames.append((t, tuple(pose), depth.astype(np.float16)))
        self.count += 1
        if len(self.frames) >= self.chunk:
            self._queue_writes()

    def add_tof(self, t: float, pose, distance: float) -> None:
        self.tof.append((t, tuple(pose), distance))

    def flush(self) -> None:
        """남은 것을 쓰고 다 쓸 때까지 기다린다."""
        self._queue_writes()
        self._jobs.join()

    def close(self) -> None:
        if self._writer.is_alive():
            self.flush()
            self._jobs.put(None)
            self._writer.join(timeout=30)

    def _queue_writes(self) -> None:
        if self.frames:
            frames, self.frames = self.frames, []
            self._jobs.put((f"depth_{self.n_chunks:04d}.npz",
                            {"t": np.array([f[0] for f in frames]), "pose": np.array([f[1] for f in frames], dtype=float),
                             "depth": np.stack([f[2] for f in frames])}))
            self.n_chunks += 1
        if len(self.tof) > self._tof_written:   # 새 ToF가 있을 때만 (전부 다시 쓴다)
            tof = list(self.tof)
            self._tof_written = len(tof)
            self._jobs.put(("tof.npz", {"t": np.array([f[0] for f in tof]), "pose": np.array([f[1] for f in tof], dtype=float),
                                        "range": np.array([f[2] for f in tof])}))

    def _write_loop(self) -> None:
        while True:
            job = self._jobs.get()
            try:
                if job is None:
                    return
                self._save(job[0], **job[1])
            finally:
                self._jobs.task_done()

    def _save(self, name: str, **arrays) -> None:
        tmp = self.dir / (name + ".part")
        with open(tmp, "wb") as f:
            np.savez_compressed(f, **arrays)
        os.replace(tmp, self.dir / name)


def load_record(rec_dir) -> dict:
    """{'t', 'pose', 'depth' (float32), 'tof_t', 'tof_pose', 'tof_range', 'meta', 'skipped'}. 깊이는 시각 순서.
    깨진 덩어리(이 기록기 전 버전에서 쓰다 꺼진 것)는 건너뛰고 'skipped'에 이름을 남긴다."""
    d = Path(rec_dir)
    chunks, skipped = [], []
    for f in sorted(d.glob("depth_*.npz")):
        try:
            z = np.load(f)
            chunks.append({k: z[k] for k in ("t", "pose", "depth")})
        except (zipfile.BadZipFile, OSError, ValueError, KeyError, EOFError):
            skipped.append(f.name)
    if not chunks:
        raise FileNotFoundError(f"{d}에 depth_*.npz가 없습니다")
    out = {"t": np.concatenate([c["t"] for c in chunks]),
           "pose": np.concatenate([c["pose"] for c in chunks]),
           "depth": np.concatenate([c["depth"] for c in chunks]).astype(np.float32)}
    if (d / "tof.npz").exists():
        tof = np.load(d / "tof.npz")
        out.update(tof_t=tof["t"], tof_pose=tof["pose"], tof_range=tof["range"])
    else:
        out.update(tof_t=np.zeros(0), tof_pose=np.zeros((0, 4)), tof_range=np.zeros(0))
    meta = d / "meta.json"
    out["meta"] = json.loads(meta.read_text(encoding="utf-8")) if meta.exists() else {}
    out["skipped"] = skipped
    return out
