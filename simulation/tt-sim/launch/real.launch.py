"""실기체(Tello EDU) 자율비행: 카메라로 지도를 만들며 사람을 찾는다(탐색). 실기체는 이것 하나만 한다.

2026-11-03 사용자 결정: 경로 비행(route)·장애물 1 m 앞 회피·지도 없이 둘러보기(wander)는 실기체에서 뺐다. 그 셋은
Gazebo(launch/mission.launch.py, scripts/run_mission.sh)에만 남는다.

띄우는 것: Tello 드라이버(영상) + 위치 추정 + 깊이 추정(nodes/depth_node.py, Depth Anything V2, ~/venvs/tt의 torch) +
YOLO 사람 탐지(nodes/detector_node.py, weights/yolo26n-seg.pt) + 비행 노드(mission_node, 카메라 지도 탐색) + 자율비행 웹 화면.
Gazebo는 띄우지 않는다. WSL2 기본(NAT) 네트워크면 Windows에서 tello_relay.py를 먼저 켠다 (modules/autonomy/tello_driver/README.md).

  ros2 launch launch/real.launch.py                  (기본 0.2 m/s, 120초 탐색)
  브라우저: http://localhost:8080 → 점검 신호등·YOLO 준비 확인 → 운용자 확인 체크 → [자율비행 시작]

지도 파일은 쓰지 않는다: 출발점이 가운데인 빈 지도(한 변 area m, 기본 20)를 기록 폴더에 만들고, 벽·가구는 드론이 카메라 깊이와
전방 ToF로 찾는다 (2026-11-02 결정: 실습실 지도 상자에 갇힌 비행이 있었다). 본 적 있는 빈 곳으로만 다니며 아직 못 본 곳을
찾아가고, search_time이 지나면 출발점으로 돌아와 내린다. YOLO가 준비되기 전(launch 뒤 20초쯤)에는 시작하지 않는다
(11/03 14:32: launch 3.8초 뒤 시작해 이륙하고 18초를 YOLO 없이 날았다). 만든 지도는 out_dir/camera_map.npz에 남고,
localize:=true면 그 지도에 깊이 영상을 대어 위치를 바로잡는다. flight_height(기본 1.0 m)는 위치 추정·비행 노드에 같이 준다.

기록: out_dir(기본 저장소 data/outputs/flights/<시각>, Git 제외)에 한 비행의 모든 것을 남긴다.
  run.json            실행 인자·지도·속도 보정값·코드 지문(파일 해시)·시각
  map.json            그때 쓴 지도 사본
  flight_log.csv      비행 노드: 시각·추정 위치·방향·상태 + 판단 근거(정면 ToF, 보낸 속도·회전)
  pose_inputs.csv     위치 추정이 받은 Tello 상태 문자열·전방 ToF 그대로 (tools/replay_pose.py로 다시 돌림)
  discovered.json     찾은 장애물
  tello_log/          드라이버: 보낸 명령·응답(rc·EXT tof? 포함)·상태값·사건
  video/              video.h264(받은 영상 그대로) + video_frames.csv(프레임마다 받은 시각)  ← video:=true
  bag/                ros2 bag(mcap): 원본 카메라 빼고 모든 토픽(깊이 영상 /tt/depth 포함)과 /rosout  ← bag:=true(기본)
  ros_log/            노드마다 로그 파일 (화면에 찍힌 줄 그대로)
  viewer.html         launch가 꺼지면(viewer:=true, 기본) 알아서 만드는 한 페이지: 영상·깊이·지도 위 경로·시계열·사건·로그를
                      같은 시각에 맞춰 본다 (tools/flight_viewer.py, 진행은 viewer.log). 목록은 flights/index.html
모든 시각은 같은 WSL 시계다. 더 볼 때: `python3 tools/flight_report.py <out_dir>`(요약), `tools/sync_frames.py <out_dir>`
(영상 프레임 ↔ ToF·상태·판단을 시각으로 맞춘 timeline.csv와 겹쳐 그린 영상).

속도 보정(v_full·v_dead·yaw_full_deg): 실기체 Tello는 작은 rc에 덜 움직인다. 2026-10-28 rc 앞 20에 약 0.12 m/s,
10-29 rc 40–42에 0.3–0.4 m/s를 쟀다. 둘을 이으면 rc ≈ 9.5에서 0 → v_dead 0.095, v_full 1.03(rc 100 환산).
10-29에 선형(v_full 0.6)으로 speed 0.25를 rc 42로 보내 실제 0.4 m/s로 날아 모니터에 닿았다. 회전은 rc 30에 약 15°/s(50°/s).

전방 ToF(front_tof:=true, 기본): 드라이버가 떠 있는 동안 RoboMaster TT 확장 키트에 `EXT tof?`를 물어 /tt/tof_front로 낸다.
비행 노드는 이것으로 지도에 없는 장애물을 피하고(avoid), 앞 0.35 m 안이 막히면 멈춘 뒤 물러난다. 키트가 없으면 false로 둔다.
ToF 지도 보정(pose_map)은 기본으로 끈다. 시험 공간 지도의 가구 좌표가 실측이 아니라서, 가구에 쏜 ToF로 위치를 고치면
오히려 틀어질 수 있다. 벽·가구를 줄자로 맞춘 지도에서만 켠다.

속도값 기준(vel_frame·vx_sign·vy_sign): Tello vgx·vgy는 기체 기준이 아니라 전원 켤 때 방향(Tello yaw 0) 기준이다
(2026-10-29 충돌 비행: 앞으로 날 때 속도 방향이 yaw와 같이 돌았다). 기체 기준(body)으로 읽으면 돌아선 뒤 위치 추정이
거꾸로 간다 → 기본 takeoff, vx_sign -1, vy_sign +1 (autonomy/tello_state.py). vel_scale(배율)은 아직 줄자로 확인하지 않았다:
비행 뒤 착륙점을 재서 vel_scale = 실제 이동 ÷ 추정 이동으로 다음 비행에 준다. 기록의 pose_inputs.csv로 다시 맞춰 볼 수 있다.
"""
import hashlib
import json
import os
import shutil
import socket
import subprocess
import sys
import time
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]
DRIVER = REPO / "modules" / "autonomy" / "tello_driver" / "tello_driver_node.py"
WEB = REPO / "modules" / "autonomy" / "autonomy_web" / "autonomy_web.py"
VENV_PY = Path(os.environ.get("TT_VENV", Path.home() / "venvs" / "tt")) / "bin" / "python"
# 드론 Wi-Fi에 붙으면 인터넷이 없다. 깊이·YOLO 모델은 내려받아 둔 것만 쓰게 한다. 안 그러면 Hugging Face에 묻느라
# 이름 풀이 실패를 다섯 번씩 다시 시도해 깊이 노드가 2분 넘게 늦게 떴다 (2026-10-29 실기체)
OFFLINE = {"HF_HUB_OFFLINE": "1", "TRANSFORMERS_OFFLINE": "1"}


def windows_host() -> str:
    """WSL에서 본 Windows 주소 (기본 경로의 게이트웨이). tello_relay.py가 여기서 기다린다."""
    out = subprocess.run(["ip", "route", "show", "default"], capture_output=True, text=True).stdout.split()
    return out[out.index("via") + 1] if "via" in out else "127.0.0.1"


def code_fingerprint() -> dict:
    """비행에 쓴 코드의 지문. WSL 사본에는 .git이 없어 파일 내용 해시로 남긴다 (같은 해시 = 같은 코드)."""
    files = sorted([*(ROOT / "nodes").glob("*.py"), *(ROOT / "autonomy").glob("*.py"), *(ROOT / "launch").glob("*.py"),
                    *(REPO / "modules" / "autonomy" / "tello_driver").glob("*.py"),
                    *(REPO / "modules" / "autonomy" / "tello_link").glob("*.py")])
    h = hashlib.sha1()
    for f in files:
        h.update(f.name.encode())
        h.update(f.read_bytes())
    newest = max(files, key=lambda f: f.stat().st_mtime)
    return {"sha1": h.hexdigest()[:12], "files": len(files), "newest": f"{newest.relative_to(REPO)}",
            "newest_mtime": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(newest.stat().st_mtime))}


def _setup(context):
    arg = lambda k: LaunchConfiguration(k).perform(context)  # noqa: E731
    out = Path(arg("out_dir") or REPO / "data" / "outputs" / "flights" / time.strftime("%Y%m%d_%H%M%S")).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    video = arg("video").lower() == "true"
    if arg("mode").strip() not in ("", "explore"):
        raise RuntimeError(f"실기체는 탐색(카메라로 지도를 만들며 사람 찾기)만 합니다. mode:={arg('mode')}는 뺐습니다 (2026-11-03). "
                           "경로 비행·둘러보기는 Gazebo(scripts/run_mission.sh)로만 시험합니다.")
    if arg("map").strip():
        print(f"[real.launch] map:={arg('map')} 은 무시합니다: 실기체는 늘 빈 지도로 뜹니다 (실습실 지도 안 씀)", flush=True)
    # 지도 없이: 출발점이 가운데인 빈 정사각형 지도(한 변 area m). 벽·가구는 드론이 카메라·ToF로 찾는다
    sys.path.insert(0, str(ROOT))
    from tools.make_room_map import open_map
    map_path = out / "map_open.json"
    map_path.write_text(json.dumps(open_map(float(arg("area")), float(arg("flight_height")), ceiling=float(arg("ceiling"))),
                                   ensure_ascii=False, indent=1), encoding="utf-8")
    # 높이 상한(11/03): 비우면 지도 천장(빈 지도는 ceiling 인자) − 0.5 m. 드라이버가 그 위로 상승을 막고, 경로는 그보다 0.15 m 아래까지 짠다
    ceiling = float(json.loads(map_path.read_text(encoding="utf-8"))["bounds"]["zmax"])
    height_limit = float(arg("height_limit")) if arg("height_limit").strip() else round(ceiling - 0.5, 2)
    # 카메라 깊이로 지도를 만들고 YOLO로 사람을 찾는다: 영상, 깊이·YOLO 가상환경(torch), YOLO 가중치가 다 있어야 한다.
    # 가중치는 weights/yolo26n-seg.pt(사람 윤곽 분할, develop 콘솔과 같은 모델) 또는 yolo11n.pt. 드론 Wi-Fi에서는 내려받을 수 없다
    weights = next((p for p in (ROOT / "weights" / "yolo26n-seg.pt", ROOT / "weights" / "yolo11n.pt") if p.exists()), None)
    if not video:
        raise RuntimeError("탐색은 카메라 영상으로 지도를 만들고 사람을 찾습니다. video:=true로 켜세요 (드라이버에 PyAV 필요).")
    if not VENV_PY.exists():
        raise RuntimeError(f"깊이·YOLO 가상환경이 없습니다: {VENV_PY} (scripts/setup_yolo.sh 또는 TT_VENV)")
    if weights is None:
        raise RuntimeError("YOLO 가중치가 없습니다: simulation/tt-sim/weights/yolo26n-seg.pt(또는 yolo11n.pt)를 미리 두세요 "
                           "(scripts/setup_yolo.sh, 인터넷 될 때).")
    if arg("tello") == "mirrored":
        net = ["-p", "host:=192.168.10.1", "-p", "cmd_port:=8889"]
    else:   # 기본: Windows 중계
        net = ["-p", f"host:={windows_host()}", "-p", "cmd_port:=9889"]
    # 영상을 풀려면 PyAV가 필요하다. YOLO 가상환경(시스템 패키지도 보임)에 설치해 두고 그 파이썬으로 띄운다
    driver_py = str(VENV_PY) if video and VENV_PY.exists() else "python3"
    # temp_max를 주면 이륙 상한과 비행 중 자동 착륙 기준을 둘 다 그 값으로 한다(기본은 드라이버의 80·85°C)
    temp_args = ["-p", f"temp_max_takeoff:={float(arg('temp_max'))}", "-p", f"temp_land:={float(arg('temp_max'))}"]         if arg("temp_max") else []
    # 배터리 기본(2026-11-02): 이륙 기준 없음, 비행 중 10% 이하면 그 자리 착륙 (드라이버 기본값).
    # battery_min을 주면 그 % 이상에서만 이륙한다(착륙 기준 10%는 그대로). 0이면 10% 착륙도 끈다
    if arg("battery_min"):
        b = float(arg("battery_min"))
        temp_args += ["-p", f"battery_min_takeoff:={b}", "-p", f"battery_land:={10.0 if b > 0 else 0.0}"]
    temp_args += ["-p", f"height_limit:={height_limit}"]
    # 비행 높이는 위치 추정(지도 보정 단면)·비행 노드(이륙 목표, 장애물 단면)가 같은 값을 써야 한다
    height = float(arg("flight_height"))
    if height >= height_limit:
        raise RuntimeError(f"flight_height {height} m가 height_limit {height_limit} m 이상입니다. "
                           "드라이버가 상승을 막아 비행 높이에 못 닿습니다. height_limit을 올리거나 flight_height를 낮추세요.")
    # 앞이 확실히 비면 올리는 속도: 기본 auto = speed의 2배(최대 0.5 m/s), 0이면 끔 (autonomy/speed.py)
    fast = float(arg("fast")) if arg("fast").strip().lower() != "auto" else round(min(2 * float(arg("speed")), 0.5), 2)
    # 실행 정보: 나중에 기록만 보고도 어떤 조건·코드로 날았는지 알 수 있게
    names = [a.name for a in generate_launch_description().entities if isinstance(a, DeclareLaunchArgument)]
    shutil.copyfile(map_path, out / "map.json")
    # 지도 테두리 밖으로는 날지 않는다: 실습실 지도를 복도에서 써서 출발점 둘레 상자에 갇힌 적이 있다 (11/02)
    mb = json.loads(map_path.read_text(encoding="utf-8")).get("bounds") or {}
    if mb:
        size = f"{mb['xmax'] - mb['xmin']:.1f}×{mb['ymax'] - mb['ymin']:.1f} m"
        print(f"[real.launch] 지도 {map_path.name}: 테두리 {size} 안에서만 납니다. 천장 {ceiling:g} m → 높이 상한 {height_limit:g} m "
              f"(가구 위로 넘을 때 경로는 {max(height, height_limit - 0.15):.2f} m까지)", flush=True)
    (out / "run.json").write_text(json.dumps({
        "started": time.strftime("%Y-%m-%d %H:%M:%S"), "host": socket.gethostname(),
        "args": {n: arg(n) for n in names}, "map": str(map_path), "out_dir": str(out),
        "code": code_fingerprint(),
    }, ensure_ascii=False, indent=1), encoding="utf-8")
    bag = arg("bag").lower() == "true"
    viewer = arg("viewer").lower() == "true"
    if viewer:
        # launch가 끝나면(Ctrl+C로 노드·bag이 다 닫힌 뒤) 기록 페이지를 만든다. 새 세션으로 띄워 Ctrl+C가 닿지 않게 한다
        py = str(VENV_PY) if VENV_PY.exists() else "python3"
        log = (out / "viewer.log").open("w", encoding="utf-8")
        subprocess.Popen(["bash", "-c", f'while kill -0 {os.getpid()} 2>/dev/null; do sleep 1; done; '
                                        f'exec "{py}" "{ROOT / "tools" / "flight_viewer.py"}" "{out}"'],
                         stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
    nodes = [
        # 노드마다의 로그 파일도 이 비행 폴더에 (기본 ~/.ros/log는 비행끼리 섞여 찾기 어렵다)
        SetEnvironmentVariable("ROS_LOG_DIR", str(out / "ros_log")),
        LogInfo(msg=f"기록 폴더: {out} (영상 {'저장' if video else '안 받음'}, bag {'저장' if bag else '안 함'}). "
                    + (f"끄면 기록 페이지를 만듭니다: {out / 'viewer.html'}" if viewer else
                       f"기록 페이지: tools/flight_viewer.py {out}")),
        ExecuteProcess(
            cmd=[driver_py, str(DRIVER), "--ros-args", *net, "-p", "local_cmd_port:=8889", "-p", "state_port:=8890",
                 "-p", f"video:={str(video).lower()}", "-p", f"log_root:={out / 'tello_log'}",
                 "-p", f"video_record:={out / 'video' if video else ''}",
                 "-p", f"camera_width:={int(arg('camera_width'))}",
                 "-p", f"rc_limit:={arg('rc_limit')}", "-p", f"front_tof:={arg('front_tof').lower()}",
                 "-p", f"v_full:={float(arg('v_full'))}", "-p", f"v_dead:={float(arg('v_dead'))}",
                 "-p", f"vz_dead:={float(arg('vz_dead'))}",
                 "-p", f"yaw_full_deg:={float(arg('yaw_full_deg'))}", "-p", f"rc_slew:={float(arg('rc_slew'))}",
                 *temp_args],
            output="screen",
        ),
        ExecuteProcess(
            cmd=["python3", str(ROOT / "nodes" / "pose_node.py"), "--ros-args", "-p", f"map:={map_path}",
                 "-p", f"input_log:={out / 'pose_inputs.csv'}",
                 # 실기체 h는 바닥 기준이 아닐 때가 있어(0.78 m에서 h 0–10 cm) 하방 ToF로 높이를 잡는다
                 "-p", "height_source:=tof", "-p", f"use_map:={arg('pose_map').lower()}",
                 "-p", f"vel_scale:={float(arg('vel_scale'))}", "-p", f"vel_frame:={arg('vel_frame')}",
                 "-p", f"yaw_sign:={float(arg('yaw_sign'))}", "-p", f"vx_sign:={float(arg('vx_sign'))}",
                 "-p", f"vy_sign:={float(arg('vy_sign'))}",
                 "-p", f"flight_height:={height}"],
            output="screen",
        ),
        ExecuteProcess(
            cmd=["python3", str(ROOT / "nodes" / "mission_node.py"), "--ros-args", "-p", f"map:={map_path}",
                 "-p", f"speed:={arg('speed')}", "-p", f"fast:={fast}", "-p", f"log_csv:={out / 'flight_log.csv'}",
                 "-p", f"discovered_out:={out / 'discovered.json'}", "-p", f"people_out:={out / 'people.json'}",
                 "-p", "driver:=true", "-p", "wait_start:=true", "-p", "pose_topic:=/tt/pose_est",
                 "-p", "wait_detect:=true",   # YOLO 결과가 와야 시작 요청을 받는다
                 "-p", f"hfov:={float(arg('hfov'))}",
                 # 깊이 배율은 정면 ToF로 맞춘다: 바닥으로 맞춘 배율이 1.30에 붙어 깊이를 30% 멀게 넣었다 (11/04 12:57)
                 "-p", "depth_scale_mode:=tof",
                 "-p", f"search_time:={float(arg('search_time'))}", "-p", f"arc_speed:={float(arg('arc_speed'))}",
                 "-p", f"return_home:={arg('return_home').lower()}",
                 "-p", f"flight_height:={height}",
                 # 카메라로 만든 지도 저장, (선택) 그 지도로 위치 바로잡기·기록
                 "-p", f"map_out:={out / 'camera_map.npz'}", "-p", f"localize:={arg('localize').lower()}",
                 "-p", f"map_radius:={float(arg('map_radius'))}", "-p", f"map_radius_tight:={float(arg('map_radius_tight'))}",
                 # 경로 높이 상한: 드라이버 height_limit보다 0.15 m 낮게 (사람·낮은 장애물 위로 넘을 때도 이 아래로)
                 "-p", f"max_height:={max(height, height_limit - 0.15):.2f}",
                 *(["-p", f"record:={out / 'record'}"] if arg("record").lower() == "true" else [])],
            output="screen",
        ),
        ExecuteProcess(
            cmd=["python3", str(WEB), "--ros-args", "-p", f"map:={map_path}", "-p", f"discovered:={out / 'discovered.json'}",
                 "-p", f"people:={out / 'people.json'}", "-p", "need_yolo:=true",
                 "-p", f"port:={arg('web_port')}"],
            output="screen",
        ),
        ExecuteProcess(   # YOLO 사람 탐지. 실기체 영상은 드라이버가 지연 없이 내므로 원본 토픽에 붙인다
            cmd=[str(VENV_PY), str(ROOT / "nodes" / "detector_node.py"), "--ros-args",
                 "-p", "image_topic:=/tt/camera/image_raw", "-p", f"out_dir:={out / 'detections'}",
                 "-p", f"model:={weights}", "-p", "depth_topic:=/tt/depth", "-p", f"rate:={float(arg('yolo_rate'))}"],
            output="screen", additional_env=OFFLINE,
        ),
        ExecuteProcess(   # 카메라 영상 → 깊이 영상 /tt/depth. 실기체 영상은 지연 없이 오므로 원본 토픽에 붙인다
            cmd=[str(VENV_PY), str(ROOT / "nodes" / "depth_node.py"), "--ros-args",
                 "-p", "image_topic:=/tt/camera/image_raw",
                 "-p", f"hfov:={float(arg('hfov'))}", "-p", f"depth_scale:={float(arg('depth_scale'))}",
                 "-p", f"rate:={float(arg('depth_rate'))}"],
            output="screen", additional_env=OFFLINE,
        ),
    ]
    if bag:
        # 카메라·깊이 영상(초당 수십 MB)은 빼고 나머지 토픽 전부와 /rosout(모든 노드의 로그 줄)을 남긴다.
        # 영상은 드라이버가 video/에 H.264 그대로 저장한다. 전방 ToF는 드라이버가 best_effort로 내는데 기록기가 토픽을 미리
        # 잡을 때 reliable로 구독해 bag에 안 남은 적이 있어(11/03 14:32) QoS를 맞춰 준다
        qos = out / "bag_qos.yaml"
        qos.write_text("/tt/tof_front:\n  reliability: best_effort\n  durability: volatile\n  history: keep_last\n  depth: 10\n",
                       encoding="utf-8")
        nodes.append(ExecuteProcess(
            cmd=["ros2", "bag", "record", "-s", "mcap", "-o", str(out / "bag"), "--all-topics",
                 "--exclude-regex", "/tt/camera/.*|.*/image.*", "--include-unpublished-topics",
                 "--qos-profile-overrides-path", str(qos)],
            output="log",
        ))
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("map", default_value="",
                              description="쓰지 않는다 (주면 무시). 실기체는 늘 출발점이 가운데인 빈 지도(한 변 area m)로 뜬다"),
        DeclareLaunchArgument("area", default_value="20", description="빈 지도의 한 변 (m). 이 테두리 밖으로는 날지 않는다"),
        DeclareLaunchArgument("out_dir", default_value="", description="기록 폴더 (기본: 저장소 data/outputs/flights/<시각>)"),
        DeclareLaunchArgument("tello", default_value="relay",
                              description="relay: Windows tello_relay.py를 거친다(WSL 기본 NAT), mirrored: WSL 미러 네트워크로 바로"),
        DeclareLaunchArgument("video", default_value="true",
                              description="Tello 영상 (꼭 켠다: 깊이 지도와 YOLO 사람 탐지가 영상을 쓴다, PyAV 필요)"),
        DeclareLaunchArgument("speed", default_value="0.2",
                              description="수평 최대 속도 (m/s, 실제 속도. v_full·v_dead로 rc로 바꾼다)"),
        DeclareLaunchArgument("rc_limit", default_value="80", description="rc 상한 (±100 중). 수평·상하·회전 모두에 걸린다"),
        DeclareLaunchArgument("v_full", default_value="1.03",
                              description="rc 100일 때 실제 수평 속도 (m/s, 무시 구간 위 기울기로 늘린 값). 실기체: rc 20 → 0.12, rc 40 → 0.35 m/s"),
        DeclareLaunchArgument("v_dead", default_value="0.095",
                              description="수평 rc 무시 구간 (rc 100 = 1). 이만큼 넘어야 움직이기 시작한다"),
        DeclareLaunchArgument("vz_dead", default_value="0.1",
                              description="상하 rc 무시 구간 (rc 100 = 1). Tello는 rc 상하 3–7을 무시해 비행 높이에 8 cm 모자란 채 멈췄다(11/02 0.92 m). 0이면 보정 안 함"),
        DeclareLaunchArgument("rc_slew", default_value="1.5",
                              description="rc 크기를 키울 때 초당 한도 (rc 100 = 1): 1.5면 0 → rc 45에 0.3 s. 줄이기·제동은 바로. "
                                          "0이면 바로 바꿈 (10/30까지의 동작: 0 → 27, 회전 0 → 80을 한 번에 보내 덜컥거렸다)"),
        DeclareLaunchArgument("camera_width", default_value="480",
                              description="ROS로 내는 카메라 영상 폭 (0이면 원본 960). 원본은 메시지가 커서 깊이 노드가 굶었다"),
        DeclareLaunchArgument("yaw_full_deg", default_value="50",
                              description="rc 100일 때 실제 회전 속도 (°/s). 2026-10-28 실기체: rc 30에 약 15°/s"),
        DeclareLaunchArgument("bag", default_value="true",
                              description="true면 카메라 영상 뺀 모든 토픽과 노드 로그를 out_dir/bag에 ros2 bag(mcap)으로 남긴다"),
        DeclareLaunchArgument("viewer", default_value="true",
                              description="true면 launch가 꺼진 뒤 out_dir/viewer.html(영상·지도·시계열·로그 한 페이지)을 만든다"),
        DeclareLaunchArgument("front_tof", default_value="true",
                              description="true: 확장 키트 전방 ToF로 지도에 없는 장애물을 피한다(EXT tof?). 키트가 없으면 false"),
        DeclareLaunchArgument("pose_map", default_value="false",
                              description="true면 전방 ToF를 지도에 대어 위치를 고친다 (지도를 줄자로 맞춘 뒤에만)"),
        DeclareLaunchArgument("vel_scale", default_value="1.0",
                              description="상태 속도값 배율 (줄자로 잰 이동 ÷ 추정 이동). 실기체 확인 전 1.0"),
        DeclareLaunchArgument("vel_frame", default_value="takeoff",
                              description="vgx·vgy 기준: takeoff(전원 켤 때 방향, 2026-10-29 실기체 확인) 또는 body(기체 앞)"),
        DeclareLaunchArgument("mode", default_value="explore",
                              description="explore만 된다 (탐색: 카메라로 지도를 만들며 사람 찾기). route·wander는 2026-11-03에 실기체에서 뺐다"),
        DeclareLaunchArgument("localize", default_value="false",
                              description="깊이 영상을 지금까지 만든 지도에 대어 위치를 바로잡는다 (시뮬레이션 4단계에서 권장)"),
        DeclareLaunchArgument("map_radius", default_value="0.4",
                              description="카메라 지도의 계획 여유(m, 막힌 칸에서 드론 중심까지). 드론 반폭 0.09 m를 뺀 만큼이 "
                                          "실제 간격이고 위치 오차만큼 먹힌다. 길이 없으면 map_radius_tight까지 줄여 본다"),
        DeclareLaunchArgument("map_radius_tight", default_value="0.2",
                              description="길이 없을 때만 줄여 보는 여유 (m). 0.2면 벽까지 약 0.1 m(11/02 요청). 위치 오차가 크면 0.3으로"),
        DeclareLaunchArgument("record", default_value="false",
                              description="깊이 영상·추정 위치·ToF를 out_dir/record에 기록한다 (tools/replay_map.py로 다시 돌림)"),
        DeclareLaunchArgument("search_time", default_value="120",
                              description="탐색 시간(초). 지나면 출발점으로 돌아와 착륙 (return_home:=false면 그 자리에 착륙)"),
        DeclareLaunchArgument("return_home", default_value="true",
                              description="탐색이 끝나면 출발점으로 돌아올지. false면 돌아가지 않고 그 자리에 내린다 (돌아오는 에너지를 아낌)"),
        DeclareLaunchArgument("fast", default_value="auto",
                              description="앞이 확실히 비면(카메라 정면 3 m·ToF 범위 밖·곧은 경로) 이 속도까지 (m/s). auto = speed의 2배(최대 0.5), 0 = 끔"),
        DeclareLaunchArgument("arc_speed", default_value="0.12",
                              description="방향을 틀 때 제자리에서 돌지 않고 이 속도(m/s)로 기수 쪽으로 나아가며 돈다. 0이면 제자리 회전"),
        DeclareLaunchArgument("vx_sign", default_value="-1.0",
                              description="Tello vgx 부호: 앞으로 날 때 vgx가 음수면 -1 (2026-10-28 실기체 확인), 양수면 1"),
        DeclareLaunchArgument("vy_sign", default_value="1.0",
                              description="Tello vgy 부호 (vel_frame takeoff 기준). 2026-10-29 실기체: yaw ψ로 앞으로 날 때 vgy ≈ −속도·sin ψ → +1"),
        DeclareLaunchArgument("yaw_sign", default_value="-1.0",
                              description="Tello yaw 부호: 위에서 볼 때 왼쪽(반시계)으로 돌릴 때 yaw가 줄면 -1, 늘면 1"),
        DeclareLaunchArgument("temp_max", default_value="",
                              description="기체 온도 기준(°C). 주면 이 온도 미만에서만 이륙하고 비행 중 이 이상이면 착륙. 비우면 80·85"),
        DeclareLaunchArgument("battery_min", default_value="",
                              description="이륙 최소 배터리(%). 비우면 이륙 기준 없음·비행 중 10% 이하 착륙, 0이면 배터리로 막지 않음"),
        DeclareLaunchArgument("flight_height", default_value="1.0",
                              description="비행 높이(m, 바닥에서 하방 ToF 기준). 이륙 뒤 이 높이에 8 cm 안으로 닿아야 출발하고, "
                                          "비행 중에는 이 높이를 지킨다(바꾸지 않음). height_limit보다 낮게. 전방 ToF는 이 높이의 것만 본다"),
        DeclareLaunchArgument("height_limit", default_value="",
                              description="이 높이(하방 ToF, m) 이상이면 상승 입력을 막는다. 비우면 천장 − 0.5 m (빈 지도 기본 2.0 m)"),
        DeclareLaunchArgument("hfov", default_value="0.955",
                              description="카메라 수평 화각 (rad). 팀 실측 보정(fx 927.5@960 → 54.7°)과 11/03 비행 영상의 회전 흐름(58–62°)으로 확정. "
                                          "전에 쓰던 1.4416(82.6°, DJI 스펙)은 틀려 깊이가 20–30% 짧고 광선이 옆으로 벌어졌다"),
        DeclareLaunchArgument("depth_scale", default_value="0.85",
                              description="깊이 추정에 곱하는 배율. 화각 공식값 1.12 대신 11/03 두 비행의 ToF와 맞춘 값(0.8–0.9). 0이면 공식값"),
        DeclareLaunchArgument("ceiling", default_value="2.5",
                              description="빈 지도의 천장 높이 (m). 등·배관 아래까지. 실습실은 2.7"),
        DeclareLaunchArgument("depth_rate", default_value="10",
                              description="깊이 추정 횟수 (초당). 11/04까지 5 — 비행 중 GPU 사용률 20%였고 배터리 상태로도 13–22번까지 나온다"),
        DeclareLaunchArgument("yolo_rate", default_value="10",
                              description="YOLO 사람 탐지 횟수 (초당). 11/04까지 5 (한 번 36–44 ms). 지나간 사람 판정은 횟수가 아니라 1초로 센다"),
        DeclareLaunchArgument("web_port", default_value="8080", description="자율비행 웹 화면 포트"),
        OpaqueFunction(function=_setup),
    ])
