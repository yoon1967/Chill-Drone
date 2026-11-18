"""시뮬레이션 + 자율비행(+ YOLO)을 한 번에 띄운다.

사용 예)
  python3 generate_world.py --seed 7 --stage 1
  ros2 launch launch/mission.launch.py world_dir:=$PWD/worlds/s7_stage1
  python3 generate_world.py --seed 7 --stage 2 --blockers 2
  ros2 launch launch/mission.launch.py world_dir:=$PWD/worlds/s7_stage2 detect:=true
  ros2 launch launch/mission.launch.py world_dir:=$PWD/worlds/s7_stage1 spawn:=2   # 비행 중 장애물 2개 생성
  ros2 launch launch/mission.launch.py world_dir:=$PWD/worlds/s7_stage1 pose:=gt    # 비교: 정답 위치로 비행
  python3 generate_world.py --seed 7 --stage 4
  ros2 launch launch/mission.launch.py world_dir:=$PWD/worlds/s7_stage4 detect:=true depth:=true   # 지도 없이 탐색

위치: 기본(pose:=est)은 정답 위치 없이 난다. Tello 상태값 → 위치 추정(pose_node.py) → 비행 노드.
정답 위치는 평가용으로 world_dir/truth_log.csv에만 남긴다.

명령 경로: 기본(link:=tello)은 실기체와 같은 길로 난다.
  비행 노드 → /tt/cmd_vel·/tt/flight_cmd → Tello 드라이버(modules/autonomy/tello_driver) → UDP SDK 명령(rc·takeoff·land)
  → gz_tello.py(Gazebo 드론을 Tello처럼 흉내) → /tt/sim/cmd_vel → Gazebo.  상태는 거꾸로 UDP → 드라이버 → /tt/state
  link:=direct는 예전처럼 /tt/cmd_vel을 Gazebo에 바로 넣고 sim_tello_state.py가 상태를 낸다(비교용).
  tof:=ext(link:=tello만)는 전방 ToF도 실기체 확장 키트처럼 받는다: Gazebo ToF는 gz_tello.py만 받고, 드라이버가 떠 있는 동안
  `EXT tof?`로 물어 /tt/tof_front에 한 줄짜리 새 값만 낸다(약 5Hz). 기본 tof:=direct는 Gazebo ToF 세 줄을 비행 노드가 바로 받는다.

웹 화면: web:=true면 자율비행 웹 화면(modules/autonomy/autonomy_web)을 함께 띄우고, 비행 노드는 화면의 [자율비행 시작]을
기다린다. 브라우저로 http://localhost:8080 (web_port)을 연다. 카메라·지도·상태를 보며 시작·착륙·비상 정지를 누른다.
  ros2 launch launch/mission.launch.py world_dir:=$PWD/worlds/s7_stage1 headless:=true web:=true
"""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT.parents[1] / "modules" / "autonomy" / "tello_driver" / "tello_driver_node.py"
WEB = ROOT.parents[1] / "modules" / "autonomy" / "autonomy_web" / "autonomy_web.py"
SIM_PORTS = {"cmd_port": 28889, "local_cmd_port": 28888, "state_port": 28890}   # 실기체(8889·8890)와 겹치지 않게


def _setup(context):
    world_dir = Path(LaunchConfiguration("world_dir").perform(context)).resolve()
    log_csv = LaunchConfiguration("log_csv").perform(context) or str(world_dir / "flight_log.csv")
    extra = []
    if LaunchConfiguration("detect").perform(context).lower() == "true":
        python = Path(os.environ.get("TT_VENV", Path.home() / "venvs" / "tt")) / "bin" / "python"
        extra.append(ExecuteProcess(
            cmd=[str(python), str(ROOT / "nodes" / "detector_node.py"), "--ros-args",
                 "-p", "use_sim_time:=true", "-p", f"out_dir:={world_dir / 'detections'}",
                 "-p", "depth_topic:=/tt/depth"],   # --depth도 켰으면 사람 마스크 안 깊이로 거리를 잰다
            output="screen",
        ))
    if LaunchConfiguration("depth").perform(context).lower() == "true":
        python = Path(os.environ.get("TT_VENV", Path.home() / "venvs" / "tt")) / "bin" / "python"
        extra.append(ExecuteProcess(   # 카메라 영상 → 깊이 영상 (4단계 지도 만들기)
            cmd=[str(python), str(ROOT / "nodes" / "depth_node.py"), "--ros-args", "-p", "use_sim_time:=true"],
            output="screen",
        ))
    pose = LaunchConfiguration("pose").perform(context)
    seed = LaunchConfiguration("seed").perform(context)
    link = LaunchConfiguration("link").perform(context)
    if link not in ("tello", "direct"):
        raise RuntimeError(f"link는 tello 또는 direct입니다: {link}")
    ext_tof = LaunchConfiguration("tof").perform(context) == "ext"
    if ext_tof and link != "tello":
        raise RuntimeError("tof:=ext는 link:=tello에서만 됩니다 (드라이버가 EXT tof?로 묻는다)")
    if link == "tello":
        if not DRIVER.exists():
            raise RuntimeError(f"Tello 드라이버가 없습니다: {DRIVER}. 팀 저장소 전체를 받거나 link:=direct로 실행하세요.")
        ports = [a for k, v in SIM_PORTS.items() if k != "local_cmd_port" for a in ("-p", f"{k}:={v}")]
        extra.append(ExecuteProcess(   # 시뮬레이션 전용: Gazebo 드론을 UDP Tello처럼
            cmd=["python3", str(ROOT / "nodes" / "gz_tello.py"), "--ros-args", *ports,
                 "-p", f"seed:={seed}", "-p", "use_sim_time:=true",
                 *(["-p", "front_tof_topic:=/tt/sim/tof_front"] if ext_tof else [])],
            output="screen",
        ))
        extra.append(ExecuteProcess(   # 실기체와 같은 드라이버. 주소·포트만 다르다
            cmd=["python3", str(DRIVER), "--ros-args", "-p", "host:=127.0.0.1", "-p", "bind_ip:=127.0.0.1",
                 *[a for k, v in SIM_PORTS.items() for a in ("-p", f"{k}:={v}")],
                 "-p", "operator_confirmed:=true", "-p", "source:=gazebo",
                 "-p", f"rc_limit:={int(LaunchConfiguration('rc_limit').perform(context))}",
                 "-p", f"height_limit:={float(LaunchConfiguration('height_limit').perform(context))}",
                 "-p", f"log_root:={world_dir / 'tello_log'}", "-p", "use_sim_time:=true",
                 "-p", f"front_tof:={str(ext_tof).lower()}"],
            output="screen",
        ))
    elif pose == "est":
        extra.append(ExecuteProcess(   # 시뮬레이션 전용: 실기체에서는 Tello 드라이버가 /tt/state를 낸다
            cmd=["python3", str(ROOT / "nodes" / "sim_tello_state.py"), "--ros-args",
                 "-p", f"seed:={seed}", "-p", "use_sim_time:=true"],
            output="screen",
        ))
    if pose == "est":
        extra.append(ExecuteProcess(
            cmd=["python3", str(ROOT / "nodes" / "pose_node.py"), "--ros-args",
                 "-p", f"map:={world_dir / 'known_map.json'}",
                 "-p", f"use_map:={LaunchConfiguration('pose_map').perform(context)}", "-p", "use_sim_time:=true",
                 "-p", f"input_log:={world_dir / 'pose_inputs.csv'}"],
            output="screen",
        ))
    extra.append(ExecuteProcess(   # 평가 전용 정답 기록
        cmd=["python3", str(ROOT / "tools" / "log_truth.py"), "--ros-args",
             "-p", f"out:={world_dir / 'truth_log.csv'}", "-p", "use_sim_time:=true"],
        output="screen",
    ))
    spawn = int(LaunchConfiguration("spawn").perform(context) or 0)
    if spawn > 0:
        extra.append(ExecuteProcess(
            cmd=["python3", str(ROOT / "tools" / "spawn_obstacles.py"), "--ros-args",
                 "-p", f"world_dir:={world_dir}", "-p", f"count:={spawn}", "-p", "use_sim_time:=true"],
            output="screen",
        ))
    web = LaunchConfiguration("web").perform(context).lower() == "true"
    if web:
        extra.append(ExecuteProcess(   # 자율비행 웹 화면. 시뮬레이션이라 정답 위치도 지도에 같이 그린다
            cmd=["python3", str(WEB), "--ros-args", "-p", f"map:={world_dir / 'known_map.json'}", "-p", "truth:=true",
                 "-p", f"discovered:={world_dir / 'discovered.json'}", "-p", f"people:={world_dir / 'people.json'}",
                 "-p", f"port:={LaunchConfiguration('web_port').perform(context)}", "-p", "use_sim_time:=true"],
            output="screen",
        ))
    # rcl은 빈 값(-p record:=)을 파라미터로 받지 못하고 노드가 바로 죽으므로 기록할 때만 넘긴다
    record = LaunchConfiguration("record").perform(context)
    record_arg = ["-p", f"record:={record}"] if record else []
    return extra + [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(ROOT / "launch" / "sim.launch.py")),
            launch_arguments={
                "world": str(world_dir / "world.sdf"),
                "headless": LaunchConfiguration("headless").perform(context),
                "cmd_topic": "/tt/sim/cmd_vel" if link == "tello" else "/tt/cmd_vel",
                "front_tof_topic": "/tt/sim/tof_front" if ext_tof else "/tt/tof_front",
            }.items(),
        ),
        ExecuteProcess(
            cmd=[
                "python3", str(ROOT / "nodes" / "wander_node.py"), "--ros-args",
                "-p", f"map:={world_dir / 'known_map.json'}",
                # depth:=true면 카메라 깊이 영상도 장애물 근거로 쓴다 (전방 ToF 빛줄기 밖, 화각 82°)
                "-p", f"depth_topic:={'/tt/depth' if LaunchConfiguration('depth').perform(context).lower() == 'true' else ''}",
                "-p", f"doors:={LaunchConfiguration('doors').perform(context).lower()}",
                "-p", f"speed:={LaunchConfiguration('speed').perform(context)}",
                "-p", f"fast:={float(LaunchConfiguration('fast').perform(context))}",
                "-p", f"log_csv:={log_csv}",
                "-p", f"discovered_out:={world_dir / 'discovered.json'}",
                "-p", f"explore_time:={float(LaunchConfiguration('search_time').perform(context))}",
                "-p", f"wait_start:={str(web).lower()}",
                "-p", f"pose_topic:={'/tt/pose_est' if pose == 'est' else '/tt/odometry_gt'}",
                "-p", "use_sim_time:=true",
            ] if LaunchConfiguration("mode").perform(context) == "wander" else [
                "python3", str(ROOT / "nodes" / "mission_node.py"), "--ros-args",
                "-p", f"map:={world_dir / 'known_map.json'}",
                "-p", f"speed:={LaunchConfiguration('speed').perform(context)}",
                "-p", f"fast:={float(LaunchConfiguration('fast').perform(context))}",
                "-p", f"log_csv:={log_csv}",
                "-p", f"discovered_out:={world_dir / 'discovered.json'}",
                "-p", f"people_out:={world_dir / 'people.json'}",
                "-p", f"map_out:={world_dir / 'camera_map.npz'}",
                "-p", f"search_time:={float(LaunchConfiguration('search_time').perform(context))}",
                "-p", f"avoid:={LaunchConfiguration('avoid').perform(context)}",
                "-p", f"arc_speed:={float(LaunchConfiguration('arc_speed').perform(context))}",
                "-p", f"localize:={LaunchConfiguration('localize').perform(context)}",
                "-p", f"depth_scale_mode:={LaunchConfiguration('depth_scale_mode').perform(context)}",
                *record_arg,
                "-p", f"driver:={str(link == 'tello').lower()}",
                "-p", f"wait_start:={str(web).lower()}",
                "-p", f"pose_topic:={'/tt/pose_est' if pose == 'est' else '/tt/odometry_gt'}",
                "-p", "use_sim_time:=true",
            ],
            output="screen",
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("world_dir", description="generate_world.py 출력 폴더 (world.sdf, known_map.json)"),
        DeclareLaunchArgument("headless", default_value="false", description="true면 Gazebo 화면 없이 실행"),
        DeclareLaunchArgument("speed", default_value="0.4", description="수평 최대 속도 (m/s)"),
        DeclareLaunchArgument("fast", default_value="0", description="앞이 확실히 비면 이 속도까지 (m/s). 0이면 끔"),
        DeclareLaunchArgument("log_csv", default_value="", description="비행 기록 CSV (기본: world_dir/flight_log.csv)"),
        DeclareLaunchArgument("avoid", default_value="true", description="false면 지도에 없는 장애물 앞에서 멈추고 중단"),
        DeclareLaunchArgument("detect", default_value="false", description="true면 YOLO 탐지 노드도 띄운다 (가상환경 필요)"),
        DeclareLaunchArgument("doors", default_value="false",
                              description="wander: true면 카메라로 본 틈(문)을 지나간다 (가구 사이를 문으로 볼 수 있어 기본 끔)"),
        DeclareLaunchArgument("depth_scale_mode", default_value="floor",
                              description="4단계 깊이 배율: floor(바닥으로, 시뮬레이션 기본) 또는 tof(정면 ToF로, 실기체와 같음)"),
        DeclareLaunchArgument("depth", default_value="false",
                              description="true면 카메라 깊이 추정 노드도 띄운다 (4단계 지도 만들기, 가상환경 필요)"),
        DeclareLaunchArgument("localize", default_value="false",
                              description="true면 4단계에서 깊이 영상을 예전 지도에 대어 위치를 바로잡는다 (4단계는 켜고 날린다)"),
        DeclareLaunchArgument("record", default_value="",
                              description="4단계 비행 기록 폴더 (깊이 영상·추정 위치·ToF, tools/replay_map.py용). 비우면 안 남긴다"),
        DeclareLaunchArgument("spawn", default_value="0", description="비행 중 드론 앞에 갑자기 만들 장애물 수 (시험용)"),
        DeclareLaunchArgument("pose", default_value="est", description="est: 정답 없이 위치 추정, gt: 시뮬레이션 정답(비교용)"),
        DeclareLaunchArgument("pose_map", default_value="true", description="false면 지도 보정 없이 속도 적분만"),
        DeclareLaunchArgument("seed", default_value="0", description="Tello 상태값 잡음 난수 seed"),
        DeclareLaunchArgument("rc_limit", default_value="30", description="Tello 흉내 드라이버의 rc 상한 (실기체 launch는 80). fast를 시험할 때 올린다"),
        DeclareLaunchArgument("height_limit", default_value="2.0", description="Tello 흉내 드라이버의 높이 상한 (m). 월드 천장 2.5 − 0.5 (실기체 launch와 같은 규칙)"),
        DeclareLaunchArgument("web", default_value="false",
                              description="true면 자율비행 웹 화면을 띄우고 화면의 시작 버튼을 기다린다"),
        DeclareLaunchArgument("web_port", default_value="8080", description="자율비행 웹 화면 포트"),
        DeclareLaunchArgument("link", default_value="tello",
                              description="tello: 실기체와 같은 드라이버·UDP SDK 명령으로 비행, direct: 속도를 Gazebo에 바로(비교용)"),
        DeclareLaunchArgument("mode", default_value="route",
                              description="route: mission_node(경로·탐색·지도 만들기), wander: wander_node(지도 없이 ToF로 둘러보기)"),
        DeclareLaunchArgument("arc_speed", default_value="0.12",
                              description="방향을 틀 때 기수 쪽으로 나아가며 도는 속도 (m/s). 0이면 제자리에서 돈다"),
        DeclareLaunchArgument("tof", default_value="direct",
                              description="direct: Gazebo 전방 ToF 세 줄을 바로, ext: 실기체 확장 키트처럼 드라이버가 EXT tof?로 물음"),
        DeclareLaunchArgument("search_time", default_value="300", description="3단계 사람 찾기 탐색 시간 (초)"),
        OpaqueFunction(function=_setup),
    ])
