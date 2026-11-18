"""Gazebo에 생성한 월드를 띄우고 ROS 2 토픽 브리지와 영상 지연 노드를 켠다.

사용 예)
  ros2 launch launch/sim.launch.py world:=$PWD/worlds/s7_stage1/world.sdf
  ros2 launch launch/sim.launch.py world:=$PWD/worlds/s7_stage1/world.sdf headless:=true   # 화면 없이

cmd_topic: Gazebo 드론 속도 입력으로 이어 줄 ROS 토픽. 기본 /tt/cmd_vel(비행 노드가 바로 움직임)이고,
mission.launch.py의 link:=tello에서는 /tt/sim/cmd_vel(gz_tello.py가 rc를 속도로 바꿔 냄)로 바꾼다.
front_tof_topic: Gazebo 전방 ToF(세 줄)를 내보낼 ROS 토픽. 기본 /tt/tof_front(비행 노드가 바로 받음)이고,
mission.launch.py의 tof:=ext에서는 /tt/sim/tof_front로 바꿔 gz_tello.py만 받는다(드라이버가 `EXT tof?`로 묻는다).
"""
import os
import tempfile
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ROOT = Path(__file__).resolve().parents[1]


def _gazebo(context):
    world = LaunchConfiguration("world").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower() in ("1", "true", "yes")
    gz_sim = os.path.join(FindPackageShare("ros_gz_sim").perform(context), "launch", "gz_sim.launch.py")
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim),
        launch_arguments={"gz_args": f"{'-s ' if headless else ''}-r {world}"}.items(),
    )]


def _bridge(context):
    config = ROOT / "config" / "bridge.yaml"
    text = config.read_text(encoding="utf-8")
    renames = {"/tt/cmd_vel": LaunchConfiguration("cmd_topic").perform(context)}
    tof = LaunchConfiguration("front_tof_topic").perform(context)
    for row in ("", "_up", "_down"):
        renames[f"/tt/tof_front{row}"] = f"{tof}{row}"
    changed = text
    for old_topic, topic in renames.items():
        old = f'- ros_topic_name: "{old_topic}"'
        assert old in text, f"bridge.yaml에서 {old_topic} 항목을 찾지 못했습니다"
        changed = changed.replace(old, f'- ros_topic_name: "{topic}"')
    if changed != text:
        config = Path(tempfile.mkdtemp(prefix="tt_bridge_")) / "bridge.yaml"
        config.write_text(changed, encoding="utf-8")
    return [Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": str(config), "use_sim_time": True}],
        output="screen",
    )]


def generate_launch_description() -> LaunchDescription:
    resource_path = os.pathsep.join(
        p for p in (str(ROOT / "models"), os.environ.get("GZ_SIM_RESOURCE_PATH", "")) if p
    )
    return LaunchDescription([
        DeclareLaunchArgument("world", description="generate_world.py가 만든 world.sdf의 절대 경로"),
        DeclareLaunchArgument("headless", default_value="false", description="true면 Gazebo 화면 없이 실행"),
        DeclareLaunchArgument("video_delay_ms", default_value="175", description="카메라 영상 지연 (ms)"),
        DeclareLaunchArgument("cmd_topic", default_value="/tt/cmd_vel",
                              description="Gazebo 드론 속도 입력으로 이어 줄 ROS 토픽"),
        DeclareLaunchArgument("front_tof_topic", default_value="/tt/tof_front",
                              description="Gazebo 전방 ToF를 내보낼 ROS 토픽 (위·아래 줄은 뒤에 _up·_down)"),
        # model://tt_drone 을 찾을 수 있게 모델 폴더를 Gazebo 검색 경로에 넣는다.
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        OpaqueFunction(function=_gazebo),
        OpaqueFunction(function=_bridge),
        ExecuteProcess(
            cmd=[
                "python3", str(ROOT / "scripts" / "latency_relay.py"),
                "--ros-args", "-p", ["delay_ms:=", LaunchConfiguration("video_delay_ms")], "-p", "use_sim_time:=true",
            ],
            output="screen",
        ),
    ])
