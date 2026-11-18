#!/usr/bin/env python3
"""자율비행: 지도에서 3D 경로를 계획해 날아간다. 가는 중에 지도에 없는 장애물을 찾으면 지도에 더하고
경로를 다시 짠다.
  1·2단계(목적지 있음): 목적지까지 날아가 착륙한다.
  3단계(목적지 없음): 사람 찾기 탐색. 카메라로 아직 못 본 곳을 골라 가서 둘러보기를 되풀이하고,
                      YOLO가 잡은 사람의 위치를 기록한다. YOLO 결과가 안 오면 둘러보거나 멈춰 보지 않고 바로 다음 자리로 간다. 더 볼 곳이 없거나 search_time이 지나면
                      출발점으로 돌아와 착륙한다 (찾은 사람은 people_out JSON).
  4단계(받은 지도 없음): 카메라 깊이(/tt/depth, depth_node.py)와 전방 ToF로 날면서 3D 지도를 만든다
                      (autonomy/camera_map.py). 본 적 있는 빈 곳으로만 다니며, '아는 빈 곳'과 '모르는 곳'의
                      경계를 골라 가서 둘러보기를 되풀이한다. 사람은 3단계처럼 기록한다. 지도는 map_out(npz).

순서: 위치 대기 → (wait_start면 시작 요청 대기 READY) → 이륙 → 경로 계획 → 경유점 추종 ⇄ 재계획 → 착륙 → 완료
웹 화면(modules/autonomy/autonomy_web)이 /tt/mission_cmd로 start(READY에서 출발)·land(그 자리에 착륙)를 보낸다.
탐색(3·4단계) 중에는 운용자가 웹 지도에서 찍은 자리(goto <x> <y>)를 다음 볼 자리로 삼는다: 가는 길·회피·문 지나기는 그대로
자동이고, 도착하면 둘러본 뒤 자율 탐색을 잇는다. 갈 수 없으면 알리고 하던 탐색을 잇는다 (autonomy/goto.py, 11/11).
탐색 시간이 다 돼 출발점으로 돌아가는 중에 찍어도 받는다: 그리로 가 둘러본 뒤 다시 돌아간다 (11/16).

지도에 없는 장애물 (avoid:=true, 기본)
  전방 ToF  같은 원뿔을 지도에 쏜 거리보다 0.2 m 넘게 가까운 값이 두 번 연속 나오면
            정면에 기둥을 세운다 (단일점 센서라 방향은 모른다). 세 줄(가운데·위·아래) 토픽의 최솟값을 쓴다.
  YOLO     의자·사람처럼 바닥에 선 물체를 상자 아래 변으로 위치를 추정해, 비슷한 자리에서
            두 번 보이면 기둥을 세운다. 영상이 늦게 오므로 촬영 시각의 위치로 계산한다.
            detector_node.py가 떠 있을 때만 쓴다.
  남은 경로가 새 기둥에 막히면 멈추고 지금 자리에서 다시 계획한다 (REPLAN).
  앞으로 가는 중 ToF가 stop_distance보다 가까우면 비상 정지 → 조금 뒤로 (BACKOFF) → 재계획.
  좁은 곳(유리 사이)에서 멈추거나 닿았으면 제자리에서 돌지 않고 들어온 자취를 그대로 되짚어 나온다 (EXIT, autonomy/near.py).
  움직이라고 하는데 2.5초 넘게 제자리면 (센서가 못 본 것에 걸림) 바로 앞을 막고 같은 순서로 피한다.
  avoid:=false면 1단계처럼 비상 정지 후 중단(ABORT)한다 (비교용).
  위치 다시 잡기 (RELOC): 아는 벽 근처의 ToF 장애물 때문에 길이 막히거나 같은 곳에서 비상 정지를 세 번 하면,
  추정 위치가 틀어진 채 굳었을 수 있다. 위치 추정 노드에 후보를 흩으라고(/tt/relocalize) 한 뒤
  제자리에서 천천히 한 바퀴 돌아 ToF로 주변 벽을 고루 잰다. 그 뒤 아는 벽에 붙은 ToF 장애물을 빼고 다시 계획한다.

입력  pose_topic (기본 /tt/pose_est)     위치 추정 노드(pose_node.py)의 추정 위치. 공분산에 불확실성 σ²
                                         (/tt/odometry_gt를 주면 시뮬레이션 정답으로 난다: 비교용)
      /tt/tof_front, /tt/tof_front_up, /tt/tof_front_down, /tt/detections, /tt/depth (4단계)
  위치가 불확실해지면(σ > max_pose_sigma) 그 자리에 착륙하고 ABORT로 끝낸다.
출력  /tt/cmd_vel                        기체 기준 속도 명령
      /tt/flight_cmd                     (driver:=true) 이륙·착륙 요청. Tello 드라이버가 받아 SDK 명령으로 보낸다.
                                         떠 있는지는 드라이버의 /tt/flight_status로 안다. 드라이버가 스스로 내리면
                                         (배터리·상태 끊김 등) ABORT로 끝내고, ABORT가 되면 착륙을 요청한다
      /tt/path                           계획 경로 (RViz용)
      /tt/mission_state                  현재 단계 문자열 (바뀔 때와 1초마다)
      /tt/operator_goal                  운용자가 찍은 자리의 상태 JSON {seq, t, state(none|pending|going|arrived), text, warn, click, goal}
                                         (바뀔 때, 가는 중이면 1초마다). 웹 화면이 기록과 지도에 보인다
      /tt/mission_note                   지금 하는 일과 이유 JSON {t, state, search{elapsed, limit}, returning, why_return, return_home, why_end}
                                         (단계가 바뀔 때와 1초마다). 웹 화면이 남은 탐색 시간·돌아가는 이유·내린 이유를 보인다 (11/16)
입력  /tt/mission_cmd                    start(READY에서 출발) | land(그 자리에 착륙) | goto <x> <y>(탐색 중: 찍은 자리를 다음 볼 자리로)
                                         | goto cancel. 자율비행 웹 화면이 보낸다
      discovered_out                     찾은 장애물 JSON (평가·그림용)

실행 예)
  python3 nodes/mission_node.py --ros-args -p map:=$PWD/worlds/s7_stage2/known_map.json -p use_sim_time:=true
"""
from __future__ import annotations

import csv
import dataclasses
import json
import math
import queue
import sys
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from autonomy import (AVOID_CLASSES, FollowerParams, ObstacleMemory, PlanError, TofRays, VoxelMap,  # noqa: E402
                      expected_range, explained_by_map, follow, ground_box, in_sight, inflate_box, is_known,
                      is_unexpected, near_known, near_person_box, path_clear, path_length, plan, tof_box,
                      with_boxes)
from autonomy.camera_map import CameraMap, CameraMapParams  # noqa: E402
from autonomy.explore import (PERSON_TALL, CoverageMap, PeopleLog, ViewpointBudget, people_risk,  # noqa: E402
                               person_in_view, person_seen_toward, person_top)
from autonomy import fresh as freshness  # noqa: E402  (step()에 지역 변수 fresh가 있다)
from autonomy.doorway import (DOOR_WIDTH, PASS_PEOPLE_CLEAR, Doorway, find_doorway, is_corridor, measure as measure_door,  # noqa: E402
                              open_axis, trail_gap)
from autonomy.follower import wrap  # noqa: E402
from autonomy.goto import SNAP_MAX as GOTO_SNAP_MAX, parse as parse_goto, snap as snap_goto  # noqa: E402
from autonomy.obstacles import ground_point  # noqa: E402
from autonomy.contact import ContactWatch, PushWatch, react_world, side_name  # noqa: E402
from autonomy.flight_record import FlightRecorder  # noqa: E402
from autonomy.scan_match import MapLocalizer  # noqa: E402
from autonomy.shape import (front_points, front_shape, rotate_scene, scene_ranges, see_through,  # noqa: E402
                            segment_points, shape_points)
from autonomy.near import (GAP_MIN_WIDTH, TRAIL_STEP, contact, forward_speed, late_hit, limit_turn, off_tail, pocket,  # noqa: E402
                           rear_unknown, retreat_body, retreat_direction, room_around, side_near, trail_cursor, trail_fold, trail_target,
                           trail_trim, turn_brake, world_to_body)
from autonomy.speed import FAST_FOV, boosted, fast_factor, straight_ahead  # noqa: E402
from autonomy.tello_state import parse as parse_tello_state  # noqa: E402
from autonomy.state_estimator import occupancy_slice  # noqa: E402
from autonomy.tello_state import YAW_SETTLE_DEG, YAW_SETTLE_WINDOW, describe_landing, yaw_span  # noqa: E402
from perception.depth import TofDepthScale, floor_scale  # noqa: E402

DRONE_RADIUS = 0.25          # 가드 포함 반지름 약 0.09 m + 여유. 지도를 이만큼 부풀려 계획한다.
FOUND_RADIUS = 0.3           # 찾은 장애물은 크기를 어림한 것이라 조금 더 부풀린다
FOUND_RADIUS_TIGHT = 0.2     # 그러면 길이 없을 때 줄여서 한 번 더 (드론 반폭 0.09 m + 0.11 m)
SENSOR_X = 0.05              # 기체 중심에서 카메라·ToF까지 앞쪽 거리 (models/tt_drone/model.sdf)
TOF_MAX = 1.2
TOF_SLOW_GAIN = 0.5          # 정면 ToF 거리 감속: 앞으로 속도 ≤ 이 값 × (ToF − stop_distance) (m/s per m)
TOF_SLOW_MIN = 0.05          # 감속해도 이만큼은 간다 (stop_distance에 닿아 비상 정지로 넘어가게)
TOF_SLOW_STOP = 0.45         # 감속이 겨누는 거리 (m). stop_distance(0.35)를 겨누면 비상 정지 직전까지 다가가 0.34 m가 됐다 (10/30)
TOF_RECHECK = 0.8            # 4단계: 앞으로 가는데 정면 ToF가 이 안이면 계획용 지도를 바로 다시 만들어 남은 경로를 본다
# 부드러운 정지 (11/02): 정면 ToF가 TOF_SLOW_STOP 안에 TOF_SOFT_HOLD 넘게 있고 그 자리가 남은 경로 위(TOF_SOFT_PATH 안)면,
# 최소 속도로 기어가 비상 정지(0.35 m)에 닿는 대신 멈추고 그 앞을 막힌 곳으로 넣어 다시 계획한다. 11/02 복도에서 막힌 경계를 보러
# 간 자리에서 0.05 m/s로 기어가 비상 정지 2번, 최소 0.24 m까지 다가갔다(127–143 s)
TOF_SOFT_HOLD = 0.3
TOF_SOFT_PATH = 0.3
#                              (MAP_REBUILD 1 s를 기다리면 0.46 m까지 가서야 경로가 막힌 것을 알았다, 10/30)
# 비상 정지 제동: 처음 BRAKE_TIME 동안은 세게 뒤로 밀어 앞으로 가던 관성을 끊는다. 2026-10-29 실기체는 0.4 m/s로
# 가다 0.32 m에서 멈췄지만 뒤로 0.15 m/s(rc −25)로는 못 멈춰 0.11 m까지 다가가 모니터에 닿았다
BRAKE_SPEED, BRAKE_TIME = 0.4, 0.5
# 돌기 전 제동 (11/05 13:04: 0.2 m/s로 가다 제자리 회전으로 바뀌면 도는 동안 0.1–0.27 m/s로 앞으로 밀려갔다, autonomy/near.py turn_brake)
TURN_BRAKE_START = 0.06          # 잰 앞 속도가 이보다 크면 돌기 전에 제동 (m/s). 멈춘 채 돌 때는 0.00이었다
TURN_BRAKE_GAIN = 1.5            # 뒤 속도 = 이 값 × 앞 속도,
TURN_BRAKE_LOW, TURN_BRAKE_HIGH = 0.12, 0.3   #   크기는 이 사이 (rc 무시 구간 0.095 m/s 위)
TURN_BRAKE_TIME = 0.8            # 이만큼(s) 밀어도 안 서면 그냥 돈다 (위치 추정 잡음일 수 있다)
# 유리 사이 추락 (2026-11-02 17:33 실기체, autonomy/near.py): 유리는 카메라 깊이에 안 보이고 정면 ToF도 0.18 m에서야 잡았다.
# 그 자리에서 회전 rc 80으로 제자리 회전하고, 비상 정지 뒤 '지금 기수의 반대쪽'(들어온 길이 아님)으로 물러나다 옆 유리에 걸려 뒤집혔다
GLASS_NEAR = 0.6         # 정면 ToF가 비어 있다가 이 안에서 처음 잡히고 카메라 지도에 없으면 유리·거울로 본다 (m)
GLASS_HALF = 0.6         # 그때 기수와 직각으로 양옆 이만큼을 단단히 막는다 (m). 깊이 장면의 기둥·벽 판단은 유리 너머를 본 것이라 쓰지 않는다
NEAR_TURN = 0.45         # 정면 ToF가 이 안이면 제자리에서 돌기 전에 들어온 길로 물러난다 (m)
NEAR_TURN_TRIES = 2      # 같은 자리(0.5 m 안)에서 그렇게 물러나는 횟수. 넘으면 천천히 돈다
SLOW_TURN_RANGE = 0.8    # 정면 ToF가 이 안이면 회전을 줄인다 (m)
SLOW_TURN_RATE = 0.3     # 그때 회전 속도 (rad/s, 약 17°/s). 전에는 0.8 rad/s(rc 80에서 잘림)로 0.2 m 앞에서 돌았다
SLOW_TURN_AFTER = 5.0    # 닿은 뒤 이 시간(s)은 늘 천천히 돈다
RETREAT_SPEED = 0.18     # 들어온 길로 물러나는 속도 (m/s, rc 약 27). 0.12(rc 20)는 실기체가 0.06–0.10 m/s로만 움직여 후진마다 2–3 s를
#                          더 썼다(11/03 16:12). 빠르게 다가가던 중(0.15 m/s 넘게)이면 처음 BRAKE_TIME은 BRAKE_SPEED로 끊는다
RETREAT_DIST = 0.3       # 물러나는 거리 (m)
RETREAT_TIME = 3.0
REAR_CHECK = RETREAT_DIST + 0.25   # 물러나기 전에 뒤쪽 이만큼(중심에서, 기체 뒤끝 0.1 + 물러날 0.3 + 여유 0.15)과
REAR_HALF = 0.15                   #   옆 ±이만큼이 '본 빈 칸'·'지나온 칸'이어야 물러난다. 아니면 물러나지 않고 천천히 돈다 (11/09 13:03)
CONTACT_YAW = math.radians(20)   # 물러나는 동안(회전 명령 0) 기수가 이만큼 저절로 돌면 닿은 것으로 본다 (추락 0.8초 전에 걸렸을 값)
CONTACT_TILT = 15.0              # 또는 기울기가 이만큼(°) 넘으면. 정상 비행은 rc 80에서도 3° 안팎
CONTACT_HOLD = 1.5               # 닿으면 입력 0으로 이만큼 머문 뒤 다시 계획한다 (Tello가 스스로 자세를 잡게)
# 모든 비행 단계의 닿음 (11/13 13:51 실기체, autonomy/contact.py): 세 번 부딪혔는데 위 검사(물러나기·후진 때만)는 한 번만 잡았다.
# 이제 어느 단계든 부딪힘·기대어 밂·크게 기욺·돌라는데 안 돎이면 닿은 쪽에 장애물을 세우고, 반대쪽으로 밀어
# 떼어 낸 뒤 CONTACT_HOLD 머물고 다시 계획한다. 입력 0만 보내면 Tello가 제자리를 지키려 닿은 쪽으로 계속 밀었다 — 문 앞에서
# 왼쪽 벽에 닿은 채 12초 동안 롤이 −3°에서 −60°까지 커져 운용자가 내렸다. 같은 곳에서 거듭 닿으면 그 자리에 내린다
# 떼어 내기는 시간이 아니라 거리로 (11/13 17:44 실기체): rc 26으로 0.8초 밀었더니 5 cm만 떨어져(두 번째는 0 cm) 머무는 동안 같은 기둥에
# 두 번 더 닿았고, 4.7초 사이의 그 세 번을 '같은 곳 세 번'으로 세어 내렸다. 이제 추정 위치로 CONTACT_PUSH_DIST 떨어질 때까지 밀고
# (CONTACT_PUSH_MAX까지), CONTACT_EPISODE 안에 같은 곳에서 이어지는 닿음은 한 사건으로 세어 0.15 m씩 더 민다
# 가라는데 안 감도 닿음이다 (11/14 Gazebo 안 보이는 벽: 문을 지나다 벽에 10.8초 붙어 있었는데 못 잡았다, autonomy/contact.py
# PushWatch). 닿은 쪽은 가라 한 쪽이다
CONTACT_PUSH_SPEED = 0.3         # 떼어 내는 속도 (m/s, rc 약 30)
CONTACT_PUSH_DIST = 0.3          # 닿은 곳에서 이만큼(m) 떨어질 때까지 민다. 한 사건에서 다시 닿으면 0.15 m씩 더
CONTACT_PUSH_MAX = 2.0           # 그래도 이 시간(s)이 지나면 그만 민다
CONTACT_EPISODE = 6.0            # 떼어 낸 뒤 이 시간(s) 안에 같은 곳에서 다시 닿으면 같은 사건이다 (한 번으로 센다)
CONTACT_PUSHES = 3               # 한 사건에서 이만큼 떼어 내도 또 닿으면 그 자리에 내린다
CONTACT_SETTLE = 0.3             # 떼어 낸 뒤 이만큼(s)은 닿음을 다시 보지 않는다 (놓이며 흔들린다)
CONTACT_EDGE = 0.12              # 닿은 쪽에 세우는 장애물의 앞면: 기체 중심에서 이만큼 (m, 가드 끝)
CONTACT_BOX = 0.3                # 그 장애물 상자 한 변 (m). 카메라가 모르는 면에 닿았으면 이 상자를 양옆 GLASS_HALF까지 줄지어 넣는다
CONTACT_LAND_N = 3               # 같은 곳(CONTACT_LAND_R m 안)에서 CONTACT_LAND_WINDOW초 안에 이만큼 닿으면 그 자리에 내린다
CONTACT_LAND_R = 1.0
CONTACT_LAND_WINDOW = 30.0
CONTACT_GROUND = 0.15            # 하방 ToF가 이 안이면 바닥에 내려앉은 것이라 닿음으로 보지 않는다 (드라이버가 grounded로 내린다.
#                                  10/30 15:05·17:44 기록: 내려앉은 뒤 '돌라는데 안 돎'·바닥 충격이 잡혔다)
# 옆 겨눠 보기 (11/13 17:44 실기체, autonomy/near.py side_near): 지도에는 빈 곳이던 자리에서 흰 기둥 모서리에 왼쪽 앞이 부딪혔다.
# 깊이 영상은 0.4초 전부터 왼쪽 7열을 0.6–0.7 m로 읽었지만, 단안 깊이는 1.3 m 안의 흰 면을 거리와 상관없이 그렇게 읽어(0.2 m 앞도
# 1.2 m 앞도) 1 m 안에서는 막힘으로 찍지 않는다. 정면 ToF 원뿔은 기체 바로 앞에서 좁아(0.2 m 앞 ±4 cm) 기체 폭의 양 끝을 못 본다.
# 그래서 경로를 따라 나아갈 때: ① 지금 기수로 찍은 장면이 온 만큼만 나아가고(돌면서 가기 포함 — 그 비행은 46°/s로 돌면서 0.12 m/s로
# 나아갔다), ② 양옆(ToF 원뿔 밖)이 가깝게 읽히는데 계획 지도에는 그쪽 PEEK_REACH 안에 아무것도 없으면, 나아가기 전에 그쪽으로
# 기수를 돌려 정면 ToF로 잰다. 무엇이 있으면 지도에 넣고 다시 계획하고, 비어 있으면(먼 면을 가깝게 읽은 것) 그대로 간다.
# 실기체 기록 6판(16분) 재연: 19번(시간으로 약 6%), 그 비행의 84.6초 왼쪽이 걸린다
PEEK_RAW = 0.95                  # 깊이 영상(배율 곱하기 전)이 이 안으로 읽은 열 = 가까운 면일 수 있다 (m)
PEEK_RUN = 3                     # 한쪽 옆에 이만큼 이어져야 (열 하나 약 2°)
PEEK_REACH = 1.3                 # 계획 지도에서 그쪽 이 안이 막혀 있으면 아는 것이라 겨눠 보지 않는다 (m)
PEEK_SCENE_YAW = math.radians(6)       # 나아갈 때 믿는 장면: 지금 기수와 이 안으로 찍혔으면 그대로 가고,
PEEK_SCENE_YAW_MAX = math.radians(12)  #   이만큼 넘게 어긋나면 나아가지 않는다 (그 사이는 전진 속도를 줄인다)
PEEK_SCENE_AGE = 0.6             # 그리고 받은 지 이 시간(s) 안이어야 한다. 아니면 나아가지 않고 기다린다
PEEK_SCENE_DEAD = 1.5            # 장면이 이보다 오래 안 오면(깊이 추정이 멈춤) 이 규칙 없이 간다 (정면 ToF는 그대로)
PEEK_DWELL = 0.4                 # 그쪽을 겨눈 채 ToF를 기다리는 시간 (s, ToF는 초당 10번)
PEEK_TIMEOUT = 4.0               # 겨누는 데 이보다 오래 걸리면 그 자리 값으로 판단한다
PEEK_MEMO = (0.6, math.radians(20), 20.0)   # 겨눠 본 자리 이 거리(m)·이 방향 안은 이 시간(s) 동안 다시 보지 않는다
# 후진 (11/03 저녁, 사용자: '정면 좌·우 1 m 안에 장애물이 모두 있을 때만 필요한 만큼 후진하고, 웬만하면 정면을 보고 난다.
# 옆으로 미끄러지는 것은 충돌 위협'): 막힌 주머니 — 돌 자리가 없고(둘레 TURN_ROOM 안 막힘) 정면 왼쪽·오른쪽 부채꼴 1 m 안이 모두
# 막힘 — 에서만, 들어온 자취(TRAIL_STEP 간격으로 남긴 위치)를 따라 기체 뒤쪽으로 곧게 후진한다. 돌 자리가 생기면 바로 끝내고 돌아서
# 정면을 보고 난다. 지나온 선은 실제로 비어 있던 곳이라 유리가 있어도 안전하다 (11/02 17:33: 40–90° 틀어진 채 나오다 옆 유리에 박았다).
# 16:12 비행은 되짚기 4번·29.9 s 중 13.9 s가 옆으로 미끄러진 것이었고 0.6 m 하한·'양옆 트임' 조건 때문에 필요보다 길었다
POCKET_RANGE = 1.0               # 정면 왼쪽·오른쪽 부채꼴에서 이 안이 막혔으면 '막힌 주머니' (m)
POCKET_SECTOR = (math.radians(20), math.radians(70))   # 부채꼴: 기수에서 이 각 사이 (양쪽, 10°마다 광선)
EXIT_MIN = 0.2                   # 적어도 이만큼 후진한 뒤 돌 자리가 생기면 끝낸다 (m)
EXIT_MAX = 1.5                   # 후진 최대 거리 (m)
EXIT_TIME = 20.0                 # 후진 최대 시간 (s)
EXIT_LOOKBACK = 0.3              # 자취에서 이만큼 뒤의 점을 겨눈다 (옆으로 밀렸어도 선 위로 돌아온다)
EXIT_FRONT = math.radians(30)    # 가는 방향이 기수에서 이 안이면 정면 ToF로도 멈춘다
EXIT_TRIES = 3                   # 같은 자리(0.7 m, 2분 안)에서 이만큼 후진해도 그대로면 후진을 그만두고 보통대로 계획한다
LATERAL_MAX = 0.05               # 물러나거나 후진할 때 옆으로 내는 속도 상한 (m/s): 옆은 보는 센서가 없다
TRAIL_SIDEWAYS = math.radians(45)   # 들어온 자취가 기체 뒤쪽에서 이만큼 넘게 벗어나면 '옆' — 돌 자리가 없을 때만 그쪽으로 간다
TRAIL_AHEAD = math.radians(100)  # 되짚을 자취가 기체 뒤쪽에서 이만큼 넘게 벗어나면(기수 쪽) 후진이 아니다: 되짚지 않고 보통대로 계획한다.
#                                  11/11 Gazebo 주머니: 자취를 잘못 잘라 기수 오른쪽 앞(뒤에서 139°)을 겨눈 채 센서 없는 쪽으로 밀려 벽에 닿았다
TRAIL_MAX = 6000                 # 자취 점 수 상한 (600 m)
TRAIL_WINDOW = 80                # 되짚을 때 보는 최근 자취 점 수 (8 m)
TRAIL_FOLD = math.radians(120)   # 자취가 이만큼 넘게 꺾인 점(U턴)까지만 되짚는다. 그 너머는 전에 들어갔다 나온 막다른 곳
# 볼 자리 (11/03 14:32 실기체: 노트북 보관함 앞은 카메라가 다 봤는데 선반 칸 안쪽만 '모르는 칸'이라, 그걸 보러 선반 앞
# 0.4 m까지 들어가 돌다 옆 선반 판에 닿아 뒤집혔다. 사용자: '공간이 다 기록됐으면 가 보지 않은 다른 곳으로')
FRONTIER_SHADOW = 0.3            # 막힌 칸에서 이 안의 모르는 칸은 경계로 치지 않는다 (가구·벽에 붙은 그늘)
VIEW_CLEAR = 0.45                # 볼 자리는 막힌 칸에서 이만큼 떨어진 곳만 (그 자리에서 제자리로 돌며 둘러본다)
VIEW_CLEAR_MIN = 0.2             # 그런 자리가 없으면(좁은 복도) 이만큼만 떨어진 자리도 고른다 (기체 모서리 0.14 m)
VIEW_STANDOFF = 1.2              # 경계에서 이 안의 넓은 자리에서 본다 (문 경계는 문틀 옆이라 좁다)
# 가운데로 다니기 (11/14 사용자 '어떤 공간이든 중간으로'): 실기체 12판에서 둘러본 자리 25곳은 벽까지 중앙 0.51 m(84%가 0.7 m 안)였고,
# 80%는 1 m 안에 0.5 m 넘게 더 넓은 자리가 있었다 — 후보 중 가는 길에서 가장 가까운 것을 골라 VIEW_CLEAR만 겨우 넘는 벽 옆이 됐다.
# 그 자리에서 제자리로 돌고(11/13 13:51은 문 앞에서 돌다 0.3 m 옆 흰 벽에 닿았다), 떠나는 길도 벽 옆에서 시작한다(벽 0.7 m 안으로 다닌
# 시간의 38%가 둘러본 자리 1 m 안). 이제 둘레가 VIEW_ROOM만큼 빈 후보를 먼저 고른다: 모자란 여유 1 m를 가는 거리 VIEW_ROOM_WEIGHT m로
# 친다. 실기체 끝 지도 73곳에서 고른 자리의 둘레가 중앙 0.50 → 0.98 m(0.7 m 안 88% → 32%), 가는 거리 4.07 → 4.50 m, 새로 보일 넓이는 그대로
VIEW_ROOM = 1.0
VIEW_ROOM_WEIGHT = 2.0
# 볼 자리 점수 (11/04 10:17 실기체: 가구·창문 벽으로 둘러싸인 구석의 작은 경계를 85초 동안 오가며, 7–9 m 떨어진 트인 라운지 쪽
# 경계는 한 번도 고르지 않았다. 경계 칸 수는 막힌 구석·벽 너머 틈에서도 크게 나오고, 거리 3 m 할인은 먼 곳을 너무 깎았다)
VIEW_RANGE = 3.0                 # 그 자리에서 이 안에 새로 보일 넓이로 센다 (광선은 막힌 칸에서 멈춤)
VIEW_GAIN_MIN = 0.3              # 새로 보일 넓이가 이보다 작으면(m²) 보러 가지 않는다
VIEW_COST_DISTANCE = 6.0         # 점수 = 넓이 / (1 + 거리 / 이 값) × 방향 계수 (전에는 3.0)
VP_GIVEUP_RADIUS = 1.5           # 포기한 볼 자리 둘레 이 안은 다시 고르지 않는다 (가는 길이 없던 자리는 0.6 m)
# 문 지나기 (11/05 10:51 실기체: 열린 문을 첫 볼 자리로 골랐는데, 길이 줄인 여유로만 나오고 문틀 둘레 지도가 0.5초마다 바뀌어
# 다시 계획하다 좁은 길 포기 규칙(재계획 3번)에 11초 만에 걸렸다. 포기 반경 1.5 m가 문까지 막아 '닫힌 공간'으로 끝냈다).
# 경로가 좁은 틈(autonomy/doorway.py)을 지나면: 문 앞 DOOR_PRE m(문 가운데 선 위)까지는 보통 경로로 가고, 거기서 문을 정면으로
# 본 뒤(DOOR_SETTLE) 다시 재고, 정면 ToF로 문이 비었는지 보고, 가운데 선을 따라 DOOR_SPEED로 곧게 DOOR_POST m까지 지난다.
# 지나는 동안은 지도로 다시 계획하지 않는다(문틀 칸이 흔들려도). 정면 ToF가 DOOR_STOP 안이거나 못 움직이면 그만두고 물러난다
DOOR_NARROW = 0.6                # 경로 점에서 막힌 칸 중심까지 이 안인 구간(폭 약 1.2 m 안)을 좁은 틈 후보로 본다 (m)
DOOR_PRE = 0.8                   # 문 가운데 앞 이 거리에서 문을 정면으로 본다 (m). 그 자리가 막혔으면 0.6·0.5 m
DOOR_POST = 0.8                  # 문 가운데를 지나 이만큼 더 곧게 간다 (m)
DOOR_NEAR = 0.4                  # 드론이 문 가운데에서 이보다 가까우면(이미 문에 들어섰으면) 곧게 지나기를 쓰지 않는다 (m)
DOOR_SPEED = 0.15                # 지날 때 속도 (m/s)
DOOR_LOOKAHEAD = 0.5             # 가운데 선 위 이만큼 앞의 점을 겨눈다 (옆으로 밀려도 기수로 선에 돌아온다, 옆 속도는 0.05 m/s까지)
DOOR_ALIGN = math.radians(8)     # 문 정면을 이 안으로 본다
DOOR_SETTLE = 0.6                # 정면을 본 뒤 기다리는 시간 (s): 깊이 영상(초당 10장)이 지도에 들어오게
DOOR_ACROSS = 0.2                # 다시 잰 가운데 선에서 이보다 벗어나 있으면 문 앞 자리로 다시 간다 (m, 한 번만)
DOOR_STOP = 0.25                 # 지날 때 정면 ToF 비상 정지 거리 (m). 문틀은 옆에 있고 정면은 비어야 한다
DOOR_BEYOND = 0.4                # 지나기 전에 정면 ToF가 문 가운데 너머 이만큼까지(ToF 1.2 m 안에서) 비어야 한다 (m)
DOOR_BEYOND_RUN = 0.2            # 지나는 동안 정면 ToF가 (빠져나갈 자리까지 남은 거리 + 이만큼)보다 가까우면 멈춘다 (m)
DOOR_THROUGH = 0.3               # 그때 문 가운데를 이만큼 지났으면 지난 것으로 본다 (기체 반폭 0.1 m + 여유)
DOOR_TIMEOUT = 25.0              # 문 앞에 닿은 뒤 이 시간 안에 못 지나면 그만둔다 (s)
DOOR_TRIES = 2                   # 같은 문(0.5 m 안)에서 이만큼 그만두면 그 문은 곧게 지나기를 쓰지 않는다 (보통 경로로)
REOPEN_TRIES = 3                 # 돌아갈 길이 닫혔을 때 들어온 틈 하나로 되나가기를 이만큼까지 시작한다 (그래도 안 되면 그 자리 착륙)
REOPEN_DIRECT = 0.3              # 그때 문 앞 자리가 이 안이면 가지 않고 그 자리에서 바로 문을 정면으로 본다 (m)
VP_GIVEUP_DOOR = 0.6             # 문 너머 볼 자리를 포기할 때 둘레 금지 반경 (m). 1.5 m는 문까지 막았다
DOOR_GAIN = 5.0                  # 문을 못 알아봤어도, 좁은 길로만 가는 자리의 새로 보일 넓이가 이만큼(m²) 넘으면 문 너머로 본다
# 문 앞에서 기다리기 (11/09 13:05 실기체): 사람이 오가는 0.78 m 통로에서 다시 잰 문 가운데가 잴 때마다 0.3 m씩 옮겨져 첫 시도를 그만뒀다.
# 잇달아 잰 가운데가 DOOR_SHAKE 넘게 다르거나 곧게 지나는 길에 찾은 사람이 닿으면(doorway.PASS_PEOPLE_CLEAR, 11/12 전에는 문 둘레 1.2 m
# 원) DOOR_WAIT까지 제자리에서 기다렸다 다시 잰다. 기다려도 사람이 그대로면 문으로 지나지 않는다 (사람 사이 틈은 문이 아니다 — Gazebo
# people_pair 11/10)
DOOR_WAIT = 6.0
DOOR_SHAKE = 0.2
# 기다리는 동안 그 사람 자리가 화면의 '떠났나' 확인 범위(가운데 ±16°, 0.8–3.5 m) 밖이면 그쪽으로 돌아 DOOR_LOOK초 본다. 떠났으면
# '지나간 사람'으로 지워지고 문을 지난다 (11/12 17:19: 문을 정면으로 본 채 기다려 −43°·−27°에 있던 그 자리를 한 번도 못 봤다). 사람마다 한 번
DOOR_LOOK = 1.5
DOOR_LOOK_MAX = 6.0              # 돌아보기 한 번의 시간 한도 (돌기 포함, s). 돌아본 시간은 문 시간 한도(DOOR_TIMEOUT)에 넣지 않는다
PERSON_VIEW = (0.8, 3.5)         # explore.person_in_view의 거리 범위: 이 밖이면 돌아봐도 못 센다
#                                  (10:51 문 13.9 m², Gazebo 문 월드 9.5–16 m²). 포기 반경만 작게 하고, 다시 보는 것은 볼 곳이 다 떨어진 뒤 한 번
# 카메라가 흰 벽·유리를 꿰뚫어 봄 (11/04: 흰 벽 0.4–0.9 m를 ToF가 잴 때 깊이는 1.1–1.5 m. 그 너머가 '본 빈 곳'이 되어 경계가 생겼다)
SEE_THROUGH_BEHIND = 0.2         # ToF 거리 + 이만큼 뒤부터
SEE_THROUGH_WIDEN = math.radians(5)   # ToF 원뿔보다 양옆 이만큼 넓은 부채꼴의 '본 빈 칸'을 모르는 칸으로 되돌린다
SEE_THROUGH_HOLD = 1.5           # 그 뒤 이 시간(s) 동안
SEE_THROUGH_YAW = math.radians(20)    # 그 기수 ± 이 안을 보는 깊이 영상은 ToF 거리까지만 넣는다
# 경로를 따라가다 돌려 할 때 (11/03 사용자: '주변에 사람이 없고 들어간 그대로 나와도 되면 후진'): 다음 경유점이 뒤쪽이고 막힌 주머니면
# 제자리에서 돌지 않고 후진한다(EXIT). 뒤를 보는 센서가 없어 사람이 가까웠으면 하지 않는다
REVERSE_TURN = math.radians(90)  # 이만큼 넘게 돌아야 하면
TURN_ROOM = 0.25                 # 둘레 이 안에 막힌 곳이 있으면 제자리에서 돌지 않는다 (기체 모서리는 중심에서 0.14 m까지 돈다)
PEOPLE_REAR = 2.0                # 이 안에
PEOPLE_REAR_TIME = 10.0          #   최근 이만큼 사이에 사람이 잡혔으면 후진하지 않고 천천히 돈다
# 뒤집힘 (11/03 14:32: roll −173°로 뒤집힌 뒤에도 다시 계획만 하며 5초 넘게 명령을 안 보냈다)
FLIP_TILT = 70.0                 # 기울기가 이만큼(°) 넘게
FLIP_HOLD = 0.3                  #   이어지면 추락으로 보고 비상 정지(모터 끔)
TRAIL_STATES = ("PLAN", "FOLLOW", "REPLAN", "LOOK", "RELOC", "HOMESPIN", "DOOR")   # 자취를 남기는 단계 (물러나거나 되짚을 때는 남기지 않는다)
TOF_HALF = math.radians(12.5)   # ToF 원뿔 반각 (model.sdf와 같게)
TOF_MARGIN = 0.2             # 지도보다 이만큼 넘게 가까워야 새 장애물로 본다
IMG_W, IMG_H, HFOV = 960, 720, 1.4416   # HFOV는 Gazebo 카메라 기본값(82.6°). 실기체는 파라미터 hfov로 실측 0.955 rad(54.7°)를 받는다
TOF_TOPICS = ("/tt/tof_front", "/tt/tof_front_up", "/tt/tof_front_down")   # 가운데 줄이 먼저
# 기둥·벽 구분 (autonomy/shape.py): 정면 ToF에 걸린 것이 깊이 장면에서 기둥이면 잰 폭만, 벽이면 그 폭을 다 막는다 (10/30 오후)
SHAPE_FRESH = 0.7                # 이보다 오래된 깊이 장면은 모양 판단에 쓰지 않는다 (s)
# 오래된 위치·전방 ToF로 움직이지 않는다 (autonomy/fresh.py, 11/04 외부 리뷰). 이 상태들의 명령에만 건다
FRESH_STATES = ("PLAN", "TAKEOFF", "FOLLOW", "REPLAN", "BACKOFF", "HOLD", "EXIT", "LOOK", "RELOC", "HOMESPIN", "DOOR", "UNSTICK")
SCENE_MAX = 4.0                  # scene_ranges가 깊이를 못 본 열에 채우는 거리 (m)
TOF_SCALE_YAW = math.radians(10)  # 깊이 배율을 ToF로 맞출 때 장면을 찍은 기수와 ToF를 잰 기수가 이 안이어야 한다
TOF_HIT_KEEP = 5.0               # ToF가 찍은 점을 이만큼 기억해, 옆으로 이어지면 벽으로 잇는다 (s). 20 s는 다른 가구끼리 이었다
TOF_JOIN_YAW = math.radians(45)  # 잇는 두 점은 비슷한 기수로 본 것이어야 한다 (돌면서 다른 가구를 본 점끼리 잇지 않게)
TOF_JOIN_FREE = 0.3              # 두 점 사이에 카메라가 본 빈 칸이 이 비율 넘게 있으면 잇지 않는다 (서로 다른 물건)
TOF_JOIN = (0.3, 1.5)            # 이어 붙일 ToF 점 사이 거리 (m)
HARD_WEIGHT = 1.5                # ToF로 확인한 칸에 더하는 점수 (camera_map.mark_hard)
TOF_UNFLY = 0.3                  # 정면 ToF가 면을 잰 자리 둘레 이 안(m)은 '지나온 칸'에서 뺀다 (11/05 13:04: 지나온 칸이 문짝·칸막이를 덮어
#                                  같은 자리로 다시 계획하며 정면 0.09–0.37 m까지 9번 다가감)
LAND_SETTLE = 0.05           # 착륙 전에 목적지 바로 위로 맞추는 수평 오차 (m)
LAND_SETTLE_TIME = 5.0       # 그래도 이 시간(초)이 지나면 그냥 내린다
LAND_NEAR = 0.25             # 경로 끝이 목적지에서 이보다 멀면(목적지가 막혀 가장 가까운 빈 칸에서 끝남) 맞추러 가지 않고 그 자리에 내린다 (m)
TAKEOFF_RETRY = 20.0         # driver:=true: 이륙 요청 뒤 이 시간(초) 안에 뜨지 않으면 다시 요청 (응답까지 실측 8.6 s)
TAKEOFF_TRIES = 3            # 그래도 안 뜨면 중단 (이륙 전 점검 실패 등. 드라이버 로그에 이유가 남는다)
TAKEOFF_TOL = 0.08           # 이륙 뒤 비행 높이에 이만큼 안이면 경로를 계획한다 (m)
# 이륙 전 기수 값 확인 (11/16 10:18 실기체, autonomy/tello_state.py yaw_span): 땅에서 Tello yaw 값이 흐르는 동안은 takeoff를 보내도
# Tello가 거절해 왔다(지난 요청 52번 중 거절 23번 모두, 그중 22번이 최근 10초에 2° 이상). 흐르는 동안은 시작 요청을 받지 않고 이유를
# 알린다 — 멈추면 저절로 띄우지는 않는다: 기체를 들었다 놓은 직후라 손이 가까이 있을 수 있다. 이 관계는 기록에서 본 것이라,
# TAKEOFF_AGAIN 안에 시작을 한 번 더 누르면 그대로 요청해 본다.
# Tello가 이륙을 바로 거절하면(error 응답) 저절로 다시 보내지도 않고 시작 대기로 돌아간다: 그 비행은 20초 뒤 다시 보낸 순간에
# 운용자가 기체를 들어 올리고 있었다(기울기 27°, 전방 ToF 2 cm) — 그때 받아들여졌다면 손에서 떴다. 응답이 없을 때(timeout)·
# 이륙 전 점검 실패는 전처럼 TAKEOFF_RETRY마다 TAKEOFF_TRIES번까지 다시 보낸다
TAKEOFF_AGAIN = 30.0
# 그 밖이면 이 속도(m/s) 이상으로 오르내린다. 실기체 Tello는 작은 rc를 무시한다:
# 2026-10-27 이륙 뒤 0.92 m에서 rc 7(약 0.07 m/s)로 25초 동안 오르지 못했다(이전 기준 3 cm)
CLIMB_MIN = 0.15
LAND_RETRY = 10.0            # 착륙 요청을 다시 보내는 간격 (초)
TOF_ROWS = (0.0, TOF_HALF, -TOF_HALF)
STUCK_TIME = 2.5
AIRBORNE = ("FOLLOW", "REPLAN", "BACKOFF", "HOLD", "EXIT", "LOOK", "RELOC", "HOMESPIN", "DOOR", "UNSTICK")
# 운용자가 웹 지도에서 찍은 자리 (11/11, autonomy/goto.py): 이 단계에서만 받는다 (이륙·출발점 위 한 바퀴·착륙 중에는 안 받는다).
# 받은 자리는 경로를 따라가는 중(FOLLOW)·다시 계획할 때(REPLAN)·둘러보기를 마친 뒤에 다음 볼 자리로 잡는다 —
# 물러나기·후진·문 지나기 같은 동작은 끊지 않고 마친 다음이다
GOTO_STATES = ("PLAN", "FOLLOW", "REPLAN", "BACKOFF", "HOLD", "EXIT", "LOOK", "RELOC", "DOOR")
GOTO_LOOK_AT = 0.3           # 찍은 곳에서 이 넘게 옮긴 자리로 갔으면(막혔거나 못 본 곳) 도착해서 찍은 쪽부터 본다 (m)
# 돌아오기 마무리 (11/09): 11/06 14:35는 위치가 1.3 m 밀려 지도에서 출발점이 벽 뒤에 갇혀 1 m 앞에서 길을 못 찾고 ABORT, 11/09 17:07은
# 출발점 둘레에서 150초를 헤맸다. 출발점 둘레가 지도에서 막혔으면 RETURN_RETARGET 안의 빈 자리로 한 번 다시 겨누고, 출발점 RETURN_NEAR
# 안에서 RETURN_STALL 동안 더 가까워지지 못하거나 길이 없으면 그 자리에 내리며, 돌아오기 시작 뒤 RETURN_MAX가 지나면 어디든 내린다
RETURN_NEAR = 2.0
RETURN_STALL = 30.0
RETURN_MAX = 120.0
RETURN_RETARGET = 1.5
HOME_SPIN_RATE = math.radians(15.0)   # 4단계 위치 맞추기: 출발점에 돌아와 내리기 전에 이 속도로 한 바퀴 돌며 위치를 맞춘다.
#                                       출발점 둘레는 이륙 직후(위치 오차가 거의 없을 때) 그린 지도라 맞추기 좋다. 돌아오는
#                                       마지막 15초 동안 맞추기가 안 붙어 0.1 → 0.3 m로 벌어진 채 내렸다 (seed 1)
RELOC_RATE = math.radians(6.0)   # 위치 다시 잡기 회전 속도. 위치 추정은 10°/s보다 빨리 돌면 ToF를 안 쓴다
RELOC_SPREAD = 0.3               # 그때 위치 후보를 흩는 폭 (m, 1σ)
RELOC_MAX = 3                    # 한 비행에서 위치 다시 잡기 최대 횟수
RELOC_STOPS = 3                  # 웨이포인트에 닿지 못한 채 비상 정지를 이만큼 하면 위치를 다시 잡는다
TURN_LOOK = 0.6                  # 3단계: 제자리에서 돈 뒤 출발 전에 멈춰 새로 보이는 쪽을 보는 시간 (s). YOLO가 돌 때만
DETECT_FRESH = 3.0               # 이 시간(s) 안에 탐지 결과가 왔으면 YOLO가 도는 것으로 본다
LOOKS_MAX = 6                    # 3단계: 탐색 자리에서 둘러보는 최대 횟수 (YOLO가 돌 때)
LOOKS_MAX_MAPPING = 3            # 4단계: 10/30 실기체는 자리마다 한 바퀴씩 돌아(9바퀴, 43 s) 비행의 1/4을 제자리에서 썼다.
#                                  2로 줄여 보니 실습실 Gazebo에서 지도가 덜 차 길이 없다는 재계획이 늘고 되레 더 맴돌았다(11/2). 지금 보는 쪽은 돌지 않는 것으로 줄인다
# YOLO 없이 날 때는 둘러보지 않고 바로 다음 자리로 간다. 볼 사람이 없는데 제자리에서 돌기만 하게 되므로.
# 본 곳은 날아가는 동안의 기수 방향으로 계속 쌓인다 (Gazebo 실습실: 1번 둘러보기와 0번의 본 곳 비교는 rooms/lab/README.md)
LOOKS_MAX_BLIND = 0
# 돌면서 가기: 제자리에서 돌아야 할 때(방향 오차가 turn_first_deg 초과) 기수 쪽으로 arc_speed로 조금씩 나아가며 돈다.
# 제자리에 머무는 시간을 줄인다(Gazebo 3단계 탐색에서 비행의 약 30%가 제자리 회전이었다). 기수 쪽으로만 움직이므로
# 전방 ToF가 계속 진행 방향을 본다. 앞 ARC_CLEAR 안에 무엇이 있거나 ARC_AHEAD 앞이 지도에서 막혀 있으면 제자리에서 돈다
ARC_CLEAR = 0.8
ARC_AHEAD = 0.3
ARC_MIN_DIST = 0.6               # 경유점이 이보다 가까우면 제자리에서 돈다 (지나쳐 버리지 않게)
PERSON_BOX = 0.5                 # 찾은 사람(확정)을 지도에 넣는 몸 크기 (m)
PERSON_MARGIN = 0.35             # 그 둘레 여유: 드론 반지름 0.25 m + 사람 위치 오차 0.1 m
PERSON_MARGIN_TIGHT = 0.15       # 길이 끝내 없을 때 줄여 보는 여유 (사람 중심에서 약 0.4 m)
PERSON_MARGIN_SOFT = 0.1         # 위험이 낮아 '둘레를 뺄' 때도 남기는 몸 둘레 하한 (사람 중심에서 0.35 m). 아예 빼면 계획이 사람 몸을 지나는
#                                  길을 내고 카메라 지도에 몸이 안 남아 있으면 그대로 간다 (Gazebo 사람 둘레 11/10: 사람 옆 0.25 m/s로 −0.08 m)
PERSON_SLOW_R = 1.0              # 찾은 사람이 이 안에 있으면 앞으로 가는 속도를 PERSON_SLOW_SPEED로 (정면 ToF는 옆 사람을 못 본다)
PERSON_SLOW_SPEED = 0.12
PERSON_WEAK_HITS = 10            # 그래도 길이 없으면 이보다 적게 잡힌 사람은 장애물에서 뺀다 (탐지 초당 10번 기준 1초, 전에는 5번·초당 5번)
PERSON_TOP_PAD = 0.1             # 잰 머리 높이 위로 더하는 몸 상자 여유 (m)
# 사람이 길을 막을 때 (2026-11-02 사용자 결정, '기다리기' 대신): 여유 줄이기 → 키를 잰 낮은 사람이면 높이를 올려 위로 넘기 →
# 부딪힐 위험(explore.people_risk: 0.6 m 안이거나 사방을 둘러쌈)이 높으면 그 자리에 착륙, 낮으면 찾은 사람 둘레를 빼고 탐색을 이어 간다.
# 사람 둘레를 빼도 카메라 깊이 지도의 몸·ToF·부드러운 정지·비상 정지는 그대로 피한다
CLIMB_NEAR = 3.0                 # 이 안에서 길을 막은 사람만 넘을지 본다 (m)
CLIMB_CLEAR = 0.1                # 넘을 때 부풀린 사람 상자 위로 더 두는 높이 (m, 지도 한 칸)
CLIMB_MAX = 3                    # 한 비행에서 높이를 올려 보는 횟수
CLIMB_TOP_N = 3                  # 넘을 사람은 머리 높이를 이만큼 넘게 쟀어야 한다 (제동 때 기수가 들려 작게 잰 한두 번을 믿지 않는다)
TOP_MAX_DIST = 3.5               # 머리 높이는 이 거리 안에서만 잰다 (멀수록 기울기 1°가 크게 틀린다)
PEOPLE_RELAX_LEAVE = 2.0         # 사람에 막혀 바꾼 것(높이·여유·사람 둘레)은 그 자리에서 이만큼 벗어나면 되돌린다 (m)
# 지나간 사람: 2026-10-30 복도에서 지나가던 사람·따라오던 운용자가 1 m 넘게 움직일 때마다 새 사람으로 쌓여, 지우지 않는
# 장애물 20개가 통로와 출발점을 막았다(볼 자리 8곳 중 2곳만 닿고 돌아오지 못함). 찾은 사람 자리를 카메라가 똑바로 봤는데
# 그쪽에 사람 상자가 없으면 세어, 이만큼 이어질 때 그 사람은 길을 막지 않는다(목록에는 남고, 다시 잡히면 다시 막는다)
PERSON_GONE_MISSES = 5           # 이만큼 이어지고
PERSON_GONE_TIME = 1.0           #   첫 번째부터 이만큼(s) 지나야 한다. 탐지가 초당 5번일 때 5번이 1초였다 — 11/04부터 실기체는 초당 10번
MAX_HEIGHT_BELOW_CEILING = 0.65  # max_height를 안 주면 천장 − 이 값까지 경로를 짠다 (드라이버 상한 = 천장 − 0.5, 경로는 그보다 0.15 아래)
# 밑 감시 (11/03): 하방 ToF(Tello 상태값 tof)가 떠 있는 중에 UNDER_MIN보다 짧으면 바로 밑에 가구·상자가 있다. 카메라가 꼭대기를 낮게 본
# 1.4 m 상자 위를 1.55 m로 지나다 0.13 m 내려앉아 얹힌 일(Gazebo 실습실). 수평은 멈추고 UNDER_CLEAR까지 오르며 그 자리를 꼭대기 높이로 막는다
UNDER_MIN = 0.35
UNDER_CLEAR = 0.6
UNDER_CLIMB = 0.3                # 오르는 속도 (m/s)
UNDER_FOOT = 0.2                 # 막는 자리 반폭 (m)
MAX_REPLANS = 30
MAX_REPLANS_MAPPING = 300        # 4단계는 지도가 계속 자라며 경로를 자주 막으므로 훨씬 많이 다시 짠다
MAP_REBUILD = 0.5                # 4단계: 카메라 지도로 계획용 지도를 다시 만드는 간격 (s). 한 번에 약 18 ms라 11/04부터 1.0 → 0.5
#                                  (새 막힘을 0.5초 빨리 보고 다시 계획한다. 깊이 영상도 실기체는 초당 10장)
# 남은 시간 (11/04 12:57 실기체: 탐색 120초 중 98초째에 10.4 m 밖 볼 자리(경로 12.9 m)를 골라 운용자가 내렸다)
VP_TIME_SPEED = 0.7              # 볼 자리는 (남은 탐색 시간 × 속도 × 이 값) m 안에서만 고른다 (12:57 평균 0.17 m/s = 0.2의 0.83배)
VP_MIN_REACH = 1.5               # 그래도 이 거리(m)까지는 고른다
VP_REACH_NOTE = 8.0              # 이보다 짧은 거리로 묶였을 때만 '남은 시간 안에 볼 곳이 없어'라고 로그에 적는다
# 닫힌 공간 (11/04 사용자: 카메라로 본 공간에 30×30 cm 넘는 통로가 없고 그보다 작은 구멍이나 막힌 곳(닫힌 창)뿐이면, 그 공간 탐색을
# 마치고 다른 곳을 찾아간다). 경계는 비행 높이로 닿는 것만 고르고(창턱·가구를 넘어야 닿는 곳은 닫힌 창 너머), 통로 입구는 경계로 친다
PASSAGE_MIN = 0.3                # 통로: 비행 높이 ±0.15 m 띠에서 이 폭(m) 넘게 트인 틈 (기체 20×20×10 cm, GAP_MIN_WIDTH와 같음)
PASSAGE_AREA = 1.0               #   그 너머 모르는 곳이 이 넓이(m²) 넘게 이어져야 통로 (선반 칸·가구 틈은 작다)
SPACE_REACH = 5.0                # '지금 공간': 드론에서 비행 높이로 이 거리(m) 안에 이어진 본 빈 곳
LOOK_DWELL = 1.0                 # 4단계: 둘러볼 방향을 본 뒤 깊이 영상이 지도에 들어올 때까지 기다리는 시간 (s)
LOOK_MIN_UNKNOWN = 25            # 4단계: 이보다 적은 모르는 칸이 보이면 그 방향은 볼 가치가 없다 (15는 거의 늘 돌았고, 40은 실습실 지도가 덜 찼다)
LOOK_MIN_TURN = math.radians(35)  # 4단계: 지금 보는 쪽(이만큼 안)은 오면서 찍었고 다음 자리로 가며 더 보므로 돌지 않는다
LOOK_FRESH_AREA = 6.0            # 4단계: 지도가 이만큼(m²)도 안 됐을 때(출발 직후)는 어느 쪽이든 돈다
MAP_RADIUS = 0.4                 # 4단계: 카메라 지도의 계획 여유 (파라미터 map_radius). 드론 반폭 0.09 m를 빼면 실제 0.31 m.
#                                  0.25(DRONE_RADIUS)로는 위치 오차 0.16–0.20 m에서 1–2 cm 닿았고(seed 1·4·7), 가구를 모르는
#                                  Gazebo 실습실에서는 0.03–0.06 m까지 붙었다 (2026-10-28: 깊이 지도가 비행 높이의 13–18%를 빈 곳으로 봄)
MAP_RADIUS_TIGHT = 0.2           # 4단계: 갈 곳·길이 없을 때 줄여 보는 지도 여유 (파라미터 map_radius_tight). 드론 반폭 0.09 m를 빼면
#                                  벽까지 약 0.1 m (11/02 사용자: 좁은 복도에서 벽과 10 cm 여유면 된다). 평소에는 0.4로 넓게 두고
#                                  길이 없을 때만 그 자리까지 줄인다. 전에는 0.3이었다: 0.15로 줄였을 때 실습실 가구에 붙었고, 0.25로도
#                                  줄인 직후 가구 모서리를 0.13 m로 스쳤다 (실제 간격 ≈ 여유 − 반폭 − 위치 오차)
APPROACH_DWELL = 3.0             # 4단계: 막힌 경계를 가까이 가서 볼 때 그쪽을 보는 시간 (s). 헛 장애물 한 칸을 다 비우려면
#                                  빗나간 광선이 십여 번 지나야 한다 (깊이 영상 초당 5장)
SAID_FRESH = 1.0                 # 내리거나 중단할 때 이 안(s)에 남긴 로그를 그 이유로 본다 (같은 주기 안에서 바로 앞에 남긴다)


class Said:
    """비행 노드 로거를 감싼다: 로그는 그대로 내고, 마지막으로 낸 말(INFO 이상)을 기억한다.
    내리거나 중단하는 자리는 모두 바로 앞에서 이유를 로그로 남기므로, 그때의 마지막 말이 곧 '왜 내렸는가'다 (Mission.end_reason).
    11/16 15:33 실기체: 웹 화면에는 단계 이름만 나와, 탐색 시간이 다 돼 돌아와 내린 것을 운용자가 알 수 없었다.
    수준마다 메서드를 따로 둔다 (rclpy는 부른 자리마다 수준을 기억한다)."""

    def __init__(self, logger):
        self._logger = logger
        self.last = None   # (말, 낸 때 time.monotonic())

    def info(self, text, **kw):
        self.last = (text, time.monotonic())
        return self._logger.info(text, **kw)

    def warn(self, text, **kw):
        self.last = (text, time.monotonic())
        return self._logger.warn(text, **kw)

    def warning(self, text, **kw):
        self.last = (text, time.monotonic())
        return self._logger.warning(text, **kw)

    def error(self, text, **kw):
        self.last = (text, time.monotonic())
        return self._logger.error(text, **kw)

    def fatal(self, text, **kw):
        self.last = (text, time.monotonic())
        return self._logger.fatal(text, **kw)

    def __getattr__(self, name):   # debug 등 나머지는 그대로
        return getattr(self._logger, name)


class Mission(Node):
    def __init__(self) -> None:
        super().__init__("tt_mission")
        self.declare_parameter("map", "")
        self.declare_parameter("pose_topic", "/tt/pose_est")
        self.declare_parameter("max_pose_sigma", 0.6)
        self.declare_parameter("flight_height", 1.0)
        self.declare_parameter("speed", 0.4)
        self.declare_parameter("fast", 0.0)   # 앞이 확실히 비면 이 속도까지 (m/s, autonomy/speed.py). 0이면 끔
        self.declare_parameter("stop_distance", 0.35)
        # 돌면서 가기 속도 (m/s). 0이면 예전처럼 제자리에서 돈다. 실기체 Tello가 작은 rc(상하 3–7)를 무시했으므로
        # rc 12 정도(v_full 1.0 가정)로 둔다. 회전 0.5 rad/s와 합치면 반지름 약 0.24 m로 돈다
        self.declare_parameter("arc_speed", 0.12)
        self.declare_parameter("log_csv", "")
        self.declare_parameter("goal", "")   # "x,y"로 주면 지도의 목적지 대신 쓴다
        self.declare_parameter("avoid", True)
        self.declare_parameter("discovered_out", "")
        self.declare_parameter("people_out", "")
        self.declare_parameter("search_time", 300.0)   # 3단계 탐색 시간 (초). 지나면 출발점으로 돌아온다
        # 탐색이 끝나면 출발점으로 돌아올지. false면 그 자리에 내린다 (11/11 09:32 실기체: 127초 탐색 뒤 돌아오는 데 62초를 쓰고도
        # 출발점 3.7 m 앞에서 내렸다 — 사용자: 돌아오느라 쓰는 에너지가 아깝다)
        self.declare_parameter("return_home", True)
        self.declare_parameter("map_out", "")           # 4단계: 카메라로 만든 지도 저장 경로 (.npz)
        # 4단계: 깊이 영상을 예전 지도에 대어 위치 바로잡기 (run_mission.sh --localize). 6개 맵 실제 비행에서 켜면 6번 중 4번,
        # 끄면 5번 중 2번 성공했고 위치 오차 최대가 0.62–1.02 → 0.24–0.50 m로 줄었다. 3단계까지와 같게 기본은 끔
        self.declare_parameter("localize", False)
        # 4단계: 카메라 지도 여유(막힌 칸에서 드론 중심까지, m)와 길이 없을 때 줄여 보는 하한. 위치 오차만큼 먹히므로
        # 실기체에서 위치 오차가 크면 늘린다(통로가 좁으면 길이 안 나온다)
        self.declare_parameter("map_radius", MAP_RADIUS)
        self.declare_parameter("map_radius_tight", MAP_RADIUS_TIGHT)
        # 경로를 짜는 가장 높은 높이 (m). 드라이버 높이 상한(height_limit)보다 낮아야 그 높이에 닿는다. real.launch는
        # height_limit − 0.15를 준다. 0이면 지도 천장 − MAX_HEIGHT_BELOW_CEILING. 가구·사람 위로 넘을 때 이 아래로만 오른다 (11/03)
        self.declare_parameter("max_height", 0.0)
        # 4단계: 지도에 넣은 깊이 영상·추정 위치·ToF를 이 폴더에 기록한다 (tools/replay_map.py로 시뮬레이터 없이 다시 돌린다)
        self.declare_parameter("record", "")
        # 4단계: 위치 추정 값이 실제보다 늦게 나오는 시간 (s). 깊이 영상·ToF를 지도에 넣을 때 (찍은 시각 + 이만큼)의 추정
        # 위치를 쓴다. 시뮬레이션 기록에서 추정 방향이 정답보다 50–58 ms 늦어, 돌 때 영상이 약 2.4° 틀어져 들어갔다.
        # 실기체에서는 제자리 회전 시험으로 다시 잰다
        self.declare_parameter("pose_lag", 0.05)
        # 실기체와 같은 Tello 드라이버로 날 때(launch link:=tello): 이륙·착륙은 드라이버에 요청한다.
        # 끄면 예전처럼 속도 명령만으로 바닥에서 오르내린다(link:=direct, Gazebo 전용)
        self.declare_parameter("driver", False)
        # 위치가 잡혀도 바로 뜨지 않고 /tt/mission_cmd "start"를 기다린다(READY). 자율비행 웹 화면에서 시작 버튼으로 띄운다
        self.declare_parameter("wait_start", False)
        self.declare_parameter("wait_detect", False)   # true면 YOLO 결과가 올 때까지 시작 요청을 받지 않는다 (실기체)
        self.declare_parameter("hfov", HFOV)           # 카메라 수평 화각 (rad): 깊이 광선·사람 위치·장면 판단에 쓴다
        # 깊이 배율을 맞추는 방법: floor(영상 아래 바닥으로, Gazebo) 또는 tof(정면 ToF로, 실기체 11/04 — perception.depth.TofDepthScale)
        self.declare_parameter("depth_scale_mode", "floor")
        self.hfov = float(self.get_parameter("hfov").value)

        map_path = self.get_parameter("map").value
        if not map_path:
            raise SystemExit("map 파라미터에 known_map.json 경로를 주세요.")
        data = json.loads(Path(map_path).read_text(encoding="utf-8"))
        self.height = float(self.get_parameter("flight_height").value)
        self.base_height = self.height   # 사람 위로 넘느라 올렸다가 되돌아올 비행 높이
        self.max_height_param = float(self.get_parameter("max_height").value)   # 천장을 읽은 뒤 정한다
        goal_param = self.get_parameter("goal").value
        if goal_param:
            gx, gy = (float(v) for v in goal_param.split(","))
            self.goal = np.array([gx, gy, self.height])
        elif data.get("goal"):
            # 높이는 비행 높이로 맞춘다. 지도의 goal.z는 지도를 만들 때의 높이라, flight_height를 다르게 주면 어긋난다
            self.goal = np.array([data["goal"]["x"], data["goal"]["y"], self.height])
        else:
            self.goal = None   # 목적지가 없는 지도(3·4단계): 이륙 뒤 가장 먼 곳을 고른다
        self.stop_distance = float(self.get_parameter("stop_distance").value)
        self.arc_speed = float(self.get_parameter("arc_speed").value)
        self.max_pose_sigma = float(self.get_parameter("max_pose_sigma").value)
        self.pose_sigma = 0.0     # 위치 추정 불확실성 (m). 정답 위치면 0, 모르면 inf (pose_sigma_known False)
        self.pose_sigma_known = True
        self.pose_t = -math.inf   # 위치를 받은 시각 (이 노드 시계)
        self.front_t = -math.inf  # 전방 ToF를 받은 시각
        self.pose_steps = self.front_steps = 0   # 받은 뒤 돈 제어 주기 수
        self.stale_logged = -math.inf
        self.lost = False
        self.avoid = bool(self.get_parameter("avoid").value)
        # 회피할 때는 ToF가 보는 방향(±12.5°)으로만 움직이도록 방향을 거의 맞춘 뒤 나아간다
        self.params = FollowerParams(speed=float(self.get_parameter("speed").value),
                                     turn_first_deg=12.0 if self.avoid else 35.0)
        self.door_params = dataclasses.replace(self.params, speed=min(DOOR_SPEED, self.params.speed))   # 문을 곧게 지날 때
        self.fast = float(self.get_parameter("fast").value)
        self.fast_logged = 0.0
        self.discovered_out = self.get_parameter("discovered_out").value

        self.ceiling = float(data["bounds"]["zmax"])
        self.max_height = max(self.height, self.max_height_param or round(self.ceiling - MAX_HEIGHT_BELOW_CEILING, 2))
        # 3단계 사람 찾기 탐색 (목적지가 없는 지도)
        self.search = self.goal is None
        self.start_xy = (float(data["spawn"]["x"]), float(data["spawn"]["y"]))
        self.search_time = float(self.get_parameter("search_time").value)
        self.return_home = bool(self.get_parameter("return_home").value)
        self.people_out = self.get_parameter("people_out").value
        self.people = PeopleLog(merge=1.0)
        self.returning = False
        self.why_return = None           # 출발점으로 돌아가는 이유 ("탐색 시간(120초)이 다 돼" 등, 웹 화면에 보인다)
        self.why_end = None              # 내리거나 중단한 이유 (set_state가 잡는다, 웹 화면에 보인다)
        self.return_t0 = None            # 돌아오기 시작 시각, 출발점까지 가장 가까웠던 거리와 그 시각 (return_check)
        self.return_best, self.return_best_t = math.inf, 0.0
        self.return_retargeted = False   # 출발점 둘레가 막혀 옆 빈 자리로 한 번 다시 겨눴는지
        self.plan_s, self.plan_done_t = 0.0, -math.inf   # 마지막 길 계획에 걸린 시간과 끝난 시각 (fresh_guard 문구)
        self.search_start = None
        self.look_target = None
        self.looks = 0
        self.bad_viewpoints: list[tuple] = []   # (x, y) 또는 (x, y, 반지름): 가 보려다 못 간 자리
        self.vp_budget = None      # 지금 볼 자리에 쓰는 재계획·시간 한도 (ViewpointBudget)
        self.door = None           # 지금 경로가 지나는 문: {"d": Doorway, "phase": approach|turn|through, ...}
        self.door_fails: list[list] = []    # [문 가운데 x, y, 그만둔 횟수]
        self.door_goals: list[tuple[float, float]] = []   # 문 너머라 작게(VP_GIVEUP_DOOR) 포기한 볼 자리: 끝내기 전에 한 번 더 본다
        self.door_retried = False
        self.doors_passed = 0
        self.doors_done: list = []          # 문 지나기로 지난 틈(Doorway, 지난 방향): 돌아갈 길이 닫히면 같은 절차로 되나간다 (reopen_door)
        self.doors_reopened = 0             # 그렇게 되나가기를 시작한 횟수 (끝 로그)
        self.reopen_count: dict = {}        # 틈마다 되나가기를 시작한 횟수 (REOPEN_TRIES까지)
        self.corridor_logged = None         # 마지막으로 '복도라 문 지나기를 안 씀'을 남긴 자리
        self.goal_area = 0.0       # 지금 볼 자리에서 새로 보일 넓이 (m²)
        self.vp_reach = math.inf   # 마지막으로 볼 자리를 고를 때 남은 시간으로 묶은 거리 (m)
        self.space_logged = None   # 마지막으로 '닫힌 공간'을 남긴 자리 (같은 판단을 되풀이해 남기지 않게)
        self.leg_from = None       # 마지막 볼 자리를 고른 자리: 가던 방향을 재는 기준 (travel_heading)
        self.visited: list[tuple[float, float]] = []
        self.rear_logged = -math.inf   # 마지막으로 '뒤를 몰라 물러나지 않음'을 남긴 시각
        self.rear_refused = 0          # 뒤를 몰라 물러나지 않은 횟수 (끝 로그, 2초 넘게 끊겼다 다시 걸릴 때만 하나 더)
        self.rear_last = -math.inf     # 마지막으로 뒤를 몰라 물러나지 않은 시각
        self.off_trail_refused = 0     # 그 가운데 뒤가 들어온 길이 아니라서 물러나지 않은 횟수 (11/16)
        self.ticks = 0
        # 4단계: 벽 하나 없는 지도(영역 경계와 출발점만)를 받으면 카메라로 지도를 만들며 탐색한다
        self.mapping = self.search and not data.get("walls")
        self.map_out = self.get_parameter("map_out").value
        self.map_built_at = -math.inf
        self.map_saved_at = -math.inf
        self.depth_frames = 0
        self.depth_scale = 1.0       # 마지막으로 맞춘 깊이 배율 (바닥이 안 보이는 장면·ToF 표본이 모자랄 때는 이 값을 쓴다)
        self.floor_frames = 0
        mode = str(self.get_parameter("depth_scale_mode").value).strip().lower()
        if mode not in ("floor", "tof"):
            raise SystemExit(f"depth_scale_mode는 floor 또는 tof입니다 (받은 값 {mode}).")
        self.tof_scale = TofDepthScale() if mode == "tof" else None
        self.scene_scale = 1.0       # self.scene을 만들 때 곱한 배율 (ToF와 비교할 때 되돌린다)
        self.scale_logged = (1.0, -math.inf)
        self.look_reached = None
        self.look_at = None          # 4단계: 도착하면 먼저 볼 곳 (막힌 경계)
        self.look_dwell = LOOK_DWELL
        self.approached: list[tuple[float, float]] = []   # 가까이 가서 다시 본 막힌 경계 무리의 중심
        self.approach_pending = None   # 그 가운데 지금 보러 가는 중인 것 (도착 전에 찍은 자리가 끼어들면 목록에서 되돌린다)
        self.map_radius_full = float(self.get_parameter("map_radius").value)
        # 틈 기준(2026-11-02 사용자): 기체 20×20×10 cm라 폭 30 cm보다 좁은 틈은 지나가지 않는다 → 여유 하한은 그 절반
        self.map_radius_tight = max(GAP_MIN_WIDTH / 2, min(float(self.get_parameter("map_radius_tight").value),
                                                           self.map_radius_full))
        self.map_radius = self.map_radius_full
        # 4단계 위치 맞추기: 추정 위치(속도 적분) → 지도 좌표 보정. 깊이 영상을 예전 지도에 대어 조금씩 고친다
        self.localize_on = bool(self.get_parameter("localize").value)
        self.pose_lag = float(self.get_parameter("pose_lag").value)
        self.localizer = MapLocalizer()
        self.correction = self.localizer.correction   # 위치 맞추기를 끄면 늘 0
        # 맞추기 한 번에 약 0.2 s라 조종 주기(20 Hz)를 붙잡지 않도록 따로 도는 스레드에서 한다 (가장 최근 영상 하나만 받는다)
        self.loc_jobs: queue.Queue = queue.Queue(maxsize=1)
        if self.localize_on:
            threading.Thread(target=self.localize_loop, daemon=True).start()
        record = self.get_parameter("record").value
        self.recorder = (FlightRecorder(record, meta={"hfov": self.hfov, "sensor_x": SENSOR_X, "tof_max": TOF_MAX,
                                                      "height": self.height, "bounds": data["bounds"],
                                                      "localize": self.localize_on, "pose_lag": self.pose_lag})
                         if record and self.mapping else None)
        if self.mapping:
            # 11/03 16:12 실기체: 1.0 m 단면 막힌 칸의 58%가 ToF 한 줄이 찍은 것이었고 유리 규칙이 찍은 단단한 칸도 카메라 광선 19장면쯤에
            # 지워졌다 → 단단한 칸은 1/10만 깎는다. 1 m 안 깊이로 막힘을 찍는 것(hit_min_range 0.6)은 Gazebo 1.4 m 복도에서 가까운 벽을
            # 20–30% 가깝게 찍어 복도가 0.5 m로 막혀 탐색이 바로 끝났으므로(11/03 저녁 corridor_L) 1.0 그대로 둔다
            self.cmap = CameraMap(data["bounds"], CameraMapParams(hard_miss=0.1))
            self.known = self.cmap.voxel_map()                        # 지금까지 본 막힌 칸
            self.known_inflated = self.cmap.planning_map(self.map_radius)
        else:
            if self.search:
                occ, origin, res = occupancy_slice(data, self.height, 0.1)
                self.coverage = CoverageMap(occ, origin, res)
            self.known = VoxelMap.from_known_map(data, res=0.1)      # 받은 지도
            self.known_inflated = self.known.inflate(DRONE_RADIUS)
        self.believed, self.inflated = self.known, self.known_inflated   # 찾은 장애물까지 더한 지도
        self.memory = ObstacleMemory()
        self.found_radius = FOUND_RADIUS
        self.candidates: list[tuple[dict, str, float]] = []   # 카메라로 한 번 본 후보 (상자, 출처, 시각)
        self.rays = TofRays.fan(7, TOF_HALF, rows=TOF_ROWS)
        self.tof_rows = {t: (math.inf, -math.inf) for t in TOF_TOPICS}   # 줄마다 (최솟값, 받은 시각)
        self.progress = None      # (x, y, 시각): 마지막으로 앞으로 나아간 때
        self.tof_streak = 0
        self.expected = math.inf  # 마지막으로 계산한 '지도대로라면' ToF 거리
        self.scene = None         # 최근 깊이 장면 (시각, 그때 기수, 열 각, 열 거리): 기둥·벽 판단용
        self.see_through = None   # (시각, 기수, ToF 거리): 카메라가 흰 벽·유리를 꿰뚫어 본 때 (check_see_through)
        self.see_through_streak = 0
        self.see_through_logged = -math.inf
        self.tof_hits: list = []  # ToF가 찍은 점 [((x, y), 시각)]
        self.shape_logged = (None, 0.0)
        self.tof_pose = None      # 마지막 ToF를 잰 순간의 위치
        self.map_dirty = False
        self.replans = 0
        self.people_key = ()      # 지도에 넣은 찾은 사람 자리 (바뀌면 지도를 다시 만든다)
        self.person_margin = PERSON_MARGIN
        self.drop_weak_people = False
        self.people_soft = False     # 위험이 낮아 찾은 사람 둘레 여유를 몸 둘레 하한(PERSON_MARGIN_SOFT)만 남겼다 (카메라가 본 몸도 남는다)
        self.person_slow_logged = -math.inf   # 마지막으로 '사람 옆이라 천천히'를 남긴 시각
        self.people_relaxed_at = None  # 사람에 막혀 여유·높이·사람 둘레를 바꾼 자리 (벗어나면 되돌린다)
        self.climbs = 0              # 사람 위로 넘으려 높이를 올린 횟수
        self.turned_at = -math.inf  # 마지막으로 제자리에서 돈 시각
        self.detect_at = -math.inf  # 마지막으로 탐지 결과(/tt/detections)를 받은 시각
        self.relocs = 0           # 위치 다시 잡기 횟수
        self.reloc = None         # {"last": 직전 yaw, "turned": 돈 각도}
        self.spin = None          # 4단계 출발점 위 한 바퀴 (같은 꼴). 한 번만 돈다
        self.stops = 0            # 웨이포인트에 닿은 뒤 비상 정지 횟수
        self.soft_since = None    # 정면 ToF가 TOF_SLOW_STOP 안에 들어온 때 (부드러운 정지)
        goal_text = (self.goal.round(2).tolist() if self.goal is not None
                     else f"없음 (벽·가구를 모르는 지도: 카메라 깊이·ToF로 지도를 만들며 탐색, 지도 여유 {self.map_radius_full}"
                          f"–{self.map_radius_tight} m)" if self.mapping
                     else "없음 (이륙 뒤 가장 먼 곳)")
        self.get_logger().info(f"지도 {self.known.occ.shape} 복셀, 목적지 {goal_text}, "
                               f"장애물 회피 {'켬' if self.avoid else '끔'}, 비행 높이 {self.height:.2f} m (가구 위로 넘을 때 "
                               f"{self.max_height:.2f} m까지, 천장 {self.ceiling:.1f} m)")

        self.pose = None          # (x, y, z, yaw), 지도 좌표 (4단계는 위치 맞추기 보정을 더한 값)
        self.raw_pose = None      # 위치 추정 노드가 준 그대로
        self.history: deque = deque(maxlen=250)   # (시각, raw pose) 최근 5초. 늦게 온 영상의 촬영 위치를 찾는다
        self.front = math.inf     # 전방 ToF (m)
        self.state = ""
        self.waypoints: list[np.ndarray] = []
        self.hold = None          # 이륙·착륙 때 붙잡을 수평 위치
        self.backoff = None       # 물러나기: {x, y, t, yaw, dir(들어온 길 방향), brake, then(끝나면 갈 단계)}
        self.front_prev = math.inf   # 직전 정면 ToF (유리 판단: 비어 있다가 가까이서 처음 잡힘)
        self.tilt = 0.0           # Tello 기울기 max(|pitch|, |roll|) (°)
        self.tof_down = None      # 하방 ToF (m, Tello 상태값 tof). 밑 감시
        self.ground_yaw: deque = deque()   # 최근 Tello yaw 값 (받은 시각, °) — 이륙 전 기수 값 확인
        self.start_refused_t = None        # 기수 값이 흘러 시작을 받지 않은 때 (TAKEOFF_AGAIN 안에 다시 누르면 받는다)
        self.takeoff_seen = None           # 이번 시작 전에 드라이버가 알려 준 마지막 takeoff 결과 (보낸 시각) — 옛 결과를 거르려고
        self.takeoff_holds = 0             # 기수 값이 흘러 시작을 받지 않은 횟수 (끝 로그)
        self.takeoff_refusals = 0          # Tello가 이륙을 거절한 횟수 (끝 로그)
        self.under_top = None     # 카메라 지도가 아는 기체 밑 가구 꼭대기 (m, 없으면 None) — 위치 추정에 알린다 (11/09)
        self.under_frames = 0     # 그것이 있다고 알린 횟수 (끝 로그)
        self.under_since = None   # 밑 감시가 오르기 시작한 때
        self.under_logged = -math.inf
        self.contact_until = 0.0  # 닿아서 머무는 끝 시각 (HOLD)
        self.watch = ContactWatch()   # 모든 비행 단계 닿음 감지 (autonomy/contact.py)
        self.push = PushWatch()       # 가라는데 안 감 (같은 곳)
        self.stall_count = 0          # 그것으로 잡은 닿음 수 (끝 로그)
        self.contact_hit = None   # Tello 상태로 잡은 닿음 (이유, 떼어 낼 방향, 시각) — step()이 처리한다
        self.contact_armed = 0.0  # 이 시각 전에는 닿음을 보지 않는다 (떼어 낸 직후)
        self.unstick = None       # 떼어 내기: {t, dir(지도 방향), z}
        self.contacts: list = []  # 최근 닿음 (시각, x, y) — 같은 곳에서 거듭 닿으면 내린다
        self.contact_last = None  # 닿은 쪽을 안 마지막 닿음 (시각, x, y, 닿은 지도 방향) — 다음에 쪽을 모르면 쓴다
        self.contact_episode = None   # 지금 닿음 사건 {t(마지막 닿음·떼어 냄), x, y, tries} — 이어지는 닿음은 한 번으로 센다
        self.contact_pushes = 0   # 떼어 낸 횟수 (끝 로그)
        self.peek = None          # 옆 겨눠 보기: {yaw(겨눌 지도 방향), t, at(겨눈 때), z, side}
        self.peek_memo: list = []   # 겨눠 본 곳 (시각, x, y, 지도 방향)
        self.peek_count, self.peek_found = 0, 0   # 겨눠 본 횟수·무엇이 있던 횟수 (끝 로그)
        self.contact_count = 0    # 닿음 횟수 (끝 로그)
        self.slow_turn_until = -math.inf
        self.near_spot, self.near_tries = None, 0   # 가까워 돌지 않고 물러난 자리와 횟수
        self.trail: list = []     # 지나온 자취 (raw pose, TRAIL_STEP 간격). 좁은 곳에서 되짚어 나올 때 쓴다
        self.exit = None          # 되짚어 나오기: {pts, base, cursor, x, y, t, yaw}
        self.exit_spot, self.exit_tries, self.exit_spot_t = None, 0, -math.inf
        self.reverse_logged = -math.inf   # '사람이 있어 후진 안 함' 경고를 낸 때
        self.pocket_cache = None          # (x, y, yaw, 시각, 결과)
        self.narrow_vp_logged = -math.inf
        self.door_logged = None   # 마지막으로 '문을 지납니다'를 남긴 문 가운데 (재계획마다 되풀이하지 않게)
        self.turn_brake_t0 = None   # 돌기 전 제동을 시작한 시각 (제동 중이 아니면 None)
        self.turn_brakes = 0        # 돌기 전 제동 횟수 (끝에 로그)
        self.turn_brake_logged = -math.inf
        self.flip_since = None    # 기울기가 FLIP_TILT 넘은 때
        self.crashed = False      # 뒤집혀 비상 정지했다 (중단해도 착륙 요청을 보내지 않는다)

        self.driver = bool(self.get_parameter("driver").value)
        self.flying = False       # driver:=true일 때 드라이버가 알려 주는 '떠 있음'
        self.flight_info = {}     # 드라이버 상태 JSON 마지막 값
        self.flight_req = None    # (요청, 보낸 시각, 횟수)
        self.forced_landing = None   # 비행 노드가 시키지 않았는데 내려앉은 이유
        self.wait_start = bool(self.get_parameter("wait_start").value)
        self.wait_detect = bool(self.get_parameter("wait_detect").value)
        self.start_requested = False
        self.land_requested = False  # 웹 화면 등에서 온 '그 자리에 착륙' 요청
        # 운용자가 웹 지도에서 찍은 자리 (11/11): 기다리는 것 하나, 가고 있는 것 하나
        self.goto_pending = None     # (x, y): 받았지만 아직 볼 자리로 잡지 않은 찍은 곳
        self.goto_active = None      # {"click": 찍은 곳, "goal": 갈 자리, "arrived": 도착해 둘러보는 중인가}
        self.goto_drop = False       # 가던 찍은 자리를 취소했다: 다음 FOLLOW·REPLAN에서 볼 자리를 다시 고른다
        self.goto_weak_dropped = False   # 찍은 자리로 가느라 몇 번만 잡힌 사람을 뺐다: 그 자리에서 벗어나면 되돌린다
        self.goto_seq = 0
        self.goto_last = None        # 마지막으로 낸 /tt/operator_goal 내용
        self.goto_count = [0, 0]     # 볼 자리로 잡은 횟수, 도착해 둘러본 횟수 (끝 로그)
        self.goto_pub = self.create_publisher(String, "/tt/operator_goal", 10)
        self.note_pub = self.create_publisher(String, "/tt/mission_note", 10)   # 지금 하는 일과 이유 (publish_note)
        self.cmd_pub = self.create_publisher(Twist, "/tt/cmd_vel", 10)
        if self.driver:
            self.flight_pub = self.create_publisher(String, "/tt/flight_cmd", 10)
            self.create_subscription(String, "/tt/flight_status", self.on_flight_status, 10)
        self.path_pub = self.create_publisher(PathMsg, "/tt/path", 1)
        self.state_pub = self.create_publisher(String, "/tt/mission_state", 1)
        self.reloc_pub = self.create_publisher(String, "/tt/relocalize", 10)
        self.under_pub = self.create_publisher(String, "/tt/under_map", 10)   # 카메라 지도가 아는 기체 밑 가구 꼭대기 → 위치 추정 노드 (11/09)
        # 4단계: 카메라 지도 단면을 웹 화면에 그리게 낸다 (막힘·의심·본 곳). 1초마다 지도를 다시 만들 때
        self.grid_pub = self.create_publisher(OccupancyGrid, "/tt/obstacle_grid", 1) if self.mapping else None
        self.create_subscription(Odometry, self.get_parameter("pose_topic").value, self.on_pose, 10)
        for topic in TOF_TOPICS:
            self.create_subscription(LaserScan, topic, lambda m, t=topic: self.on_tof(t, m), qos_profile_sensor_data)
        self.create_subscription(String, "/tt/detections", self.on_detections, 10)
        self.create_subscription(String, "/tt/mission_cmd", self.on_mission_cmd, 10)
        self.create_subscription(String, "/tt/state", self.on_tello_state, 10)   # 기울기 (물러나다 닿았는지)
        if self.mapping:
            self.create_subscription(Image, "/tt/depth", self.on_depth, 2)

        log_path = self.get_parameter("log_csv").value
        # 줄 단위로 바로 써서, 노드를 강제로 꺼도 기록이 남게 한다
        self.log = csv.writer(open(log_path, "w", newline="", buffering=1)) if log_path else None
        if self.log:
            # corr_*: 4단계 위치 맞추기 보정 (추정 → 지도 좌표). x, y, yaw는 보정한 뒤 값
            # 뒤 열은 판단 근거: 정면 ToF와 보낸 속도·회전 (비행 뒤 영상·ToF와 시각을 맞춰 본다, tools/sync_frames.py)
            self.log.writerow(["t", "x", "y", "z", "yaw", "state", "sigma", "corr_x", "corr_y", "corr_yaw",
                               "front_tof", "cmd_vx", "cmd_vy", "cmd_wz"])
        self.last_cmd = (0.0, 0.0, 0.0, 0.0)

        self.save_discovered()
        self.set_state("WAIT_POSE")
        self.create_timer(0.05, self.step)
        # 단계는 바뀔 때 내지만, 나중에 켠 화면(웹 등)도 알 수 있게 1초마다 한 번 더 낸다
        self.create_timer(1.0, self.republish)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def get_logger(self):
        """로그는 그대로 내되 마지막 말을 기억하는 로거 (Said). 내리는 이유를 웹 화면에 보일 때 쓴다."""
        said = self.__dict__.get("_said")
        if said is None:
            said = self._said = Said(super().get_logger())
        return said

    def republish(self) -> None:
        self.state_pub.publish(String(data=self.state))
        self.publish_note()
        if self.goto_last is not None and (self.goto_active is not None or self.goto_pending is not None):
            self.goto_pub.publish(String(data=self.goto_last))

    # ------------------------------------------------------------ 지금 하는 일과 이유 (11/16)

    def publish_note(self) -> None:
        """지금 하는 일과 이유를 /tt/mission_note로 낸다 (단계가 바뀔 때와 1초마다). 웹 화면이 남은 탐색 시간, 돌아가는 이유, 내린 이유를 보인다.
        11/16 15:33 실기체: 탐색 시간 120초가 다 돼 출발점으로 돌아와 내렸는데, 화면에는 '경로 비행 → 착륙 → 완료'만 나와 운용자가 이유를 몰랐다."""
        started = self.search and self.search_start is not None
        info = {"t": round(self.now(), 2), "state": self.state,
                "search": {"elapsed": round(self.search_elapsed(), 1), "limit": self.search_time} if started else None,
                "returning": bool(self.returning), "why_return": self.why_return if self.returning else None,
                "return_home": self.return_home, "why_end": self.why_end}
        self.note_pub.publish(String(data=json.dumps(info, ensure_ascii=False)))

    def end_reason(self) -> str:
        """내리거나 중단하는 이유 한 문장: 바로 앞에 남긴 로그 (내리는 자리마다 이유를 먼저 로그로 남긴다)."""
        last = getattr(self.get_logger(), "last", None)
        said = last[0] if last is not None and time.monotonic() - last[1] < SAID_FRESH else None
        if said is None:
            return "이유를 남기지 않고 내렸습니다 (터미널 로그를 보세요)"
        return f"{self.why_return} 출발점으로 돌아가던 중 — {said}" if self.returning and self.why_return else said

    # ------------------------------------------------------------ 운용자가 찍은 자리 (11/11)

    def goto_note(self, text: str, warn: bool = False) -> None:
        """찍은 자리의 지금 상태와 알릴 말을 /tt/operator_goal로 낸다 (웹 화면이 기록에 적고 지도에 그린다)."""
        self.goto_seq += 1
        a, p = self.goto_active, self.goto_pending
        state = "pending" if p is not None else "none" if a is None else "arrived" if a["arrived"] else "going"
        click = p if p is not None else a["click"] if a is not None else None
        goal = a["goal"] if a is not None and p is None else None
        self.goto_last = json.dumps({"seq": self.goto_seq, "t": round(self.now(), 2), "state": state, "text": text, "warn": bool(warn),
                                     "click": None if click is None else [round(v, 2) for v in click],
                                     "goal": None if goal is None else [round(v, 2) for v in goal]}, ensure_ascii=False)
        self.goto_pub.publish(String(data=self.goto_last))
        (self.get_logger().warn if warn else self.get_logger().info)(text)

    def goto_end(self, text: str, warn: bool = False) -> None:
        self.goto_active = None
        self.goto_note(text, warn)

    def search_elapsed(self) -> float:
        return self.now() - self.search_start if self.search_start is not None else 0.0

    def search_over(self) -> bool:
        return self.search_elapsed() >= self.search_time

    def goto_after(self) -> str:
        """찍은 자리를 둘러보거나 그만둔 뒤에 할 일을 말로 (탐색 시간이 남았으면 자율 탐색, 다 됐으면 돌아가기)."""
        if not self.search_over():
            return "자율 탐색을 이어갑니다"
        return f"탐색 시간({self.search_time:.0f}초)이 다 됐으므로 " + ("출발점으로 돌아갑니다" if self.return_home else "그 자리에 내립니다")

    def goto_refusal(self) -> str | None:
        """지금은 찍은 자리를 받을 수 없는 이유 (받을 수 있으면 None).
        탐색 시간이 다 된 뒤에도 받는다 — 11/16 15:33 실기체: 운용자가 더 보내려고 찍은 세 자리가 모두 탐색 시간(120초)이 끝난 뒤
        (2초·16초·24초 뒤)여서 '탐색 시간이 다 됐습니다'로 거절됐고, 기체는 그대로 돌아와 내렸다. 운용자가 찍은 것은 시간 한도보다 앞선다
        (찍은 자리를 둘러본 뒤에는 다시 돌아간다. 배터리·온도 자동 착륙과 [착륙]은 그대로다)."""
        if not self.search:
            return "목적지가 정해진 비행이라 탐색 중에만 받습니다"
        if self.state not in GOTO_STATES:
            return f"지금 단계({self.state})에서는 받지 않습니다 (이륙해 둘러본 뒤부터 돌아와 내리기 전까지)"
        return None

    def goto_request(self, cx: float, cy: float) -> None:
        """웹 지도에서 찍은 곳을 받아 둔다. 볼 자리로 잡는 것은 step()의 goto_take와 choose_viewpoint가 한다."""
        why = self.goto_refusal()
        if why:
            self.goto_note(f"찍은 자리 ({cx:.1f}, {cy:.1f})를 받지 않습니다: {why}.", warn=True)
            return
        self.goto_pending, self.goto_drop = (cx, cy), False
        when = {"FOLLOW": "바로", "REPLAN": "바로", "PLAN": "바로", "LOOK": "둘러보기를 마치면"}.get(self.state, "하던 동작을 마치면")
        if self.search_over():
            back = (f" 탐색 시간({self.search_time:.0f}초)은 다 됐지만 찍으셨으므로 " + ("돌아가던 것을 멈추고 " if self.returning else "")
                    + "다녀옵니다 (둘러본 뒤 " + ("출발점으로 돌아갑니다)." if self.return_home else "그 자리에 내립니다)."))
        else:
            back = " 돌아가던 것을 멈추고 탐색으로 돌아갑니다." if self.returning else ""
        self.goto_note(f"찍은 자리 ({cx:.1f}, {cy:.1f})를 받았습니다. {when} 그리로 갑니다.{back}")

    def goto_cancel(self) -> None:
        if self.goto_pending is None and self.goto_active is None:
            self.get_logger().info("취소할 찍은 자리가 없습니다.")
            return
        # 가던 중이면 다음 FOLLOW·REPLAN에서 볼 자리를 다시 고른다. 도착해 둘러보는 중이면 둘러보기는 마친다
        self.goto_drop = self.goto_active is not None and not self.goto_active["arrived"]
        self.goto_pending = None
        self.goto_end(f"찍은 자리를 취소했습니다. {self.goto_after()}.")

    def goto_take(self) -> bool:
        """받아 둔 찍은 자리가 있으면(또는 가던 것을 취소했으면) 하던 길을 멈추고 볼 자리를 다시 고르러 간다 (True)."""
        if self.goto_pending is None and not self.goto_drop:
            return False
        self.send()
        if self.goto_pending is not None and self.returning:
            # 돌아가던 중(더 볼 곳이 없어서, 또는 탐색 시간이 다 돼서): 탐색으로 되돌린다. 찍은 자리를 둘러본 뒤에 볼 곳이 없거나
            # 탐색 시간이 다 됐으면 choose_viewpoint가 다시 돌아가기를 시작한다
            self.returning, self.return_t0, self.return_best, self.return_retargeted = False, None, math.inf, False
        if self.approach_pending is not None:
            # 막힌 경계를 다시 보러 가던 길이 끊겼다: 그 경계를 나중에 다시 고를 수 있게 되돌린다 (11/11 Gazebo 방 두 개: 고르자마자
            # 찍은 자리가 끼어들어 가 보지도 않은 경계가 '가 본 곳'으로 남았고, 탐색이 반대쪽 방으로 돌아갔다)
            if self.approach_pending in self.approached:
                self.approached.remove(self.approach_pending)
            self.approach_pending = None
        self.goto_drop = False
        self.set_state("PLAN")
        return True

    def goto_spot(self, x: float, y: float, cx: float, cy: float, exact: bool = True):
        """찍은 곳 (cx, cy) → 갈 자리 (gx, gy, 찍은 곳에서 옮긴 거리). 받을 수 없으면 None (autonomy/goto.py snap).
        exact이고 찍은 칸이 평소 여유의 계획 지도에서 비었으면 그대로 간다. 아니면 줄인 여유의 계획 지도(경로 계획이 마지막에 쓰는 지도)에서
        드론과 이어진 빈 칸 가운데 찍은 곳에 가장 가까운 자리 — 평소 여유로는 문·좁은 통로가 닫혀 이어진 곳이 방 한 칸뿐일 수 있다."""
        m = self.inflated
        k = m.index((x, y, self.height))[2]
        if not 0 <= k < m.occ.shape[2]:
            return None
        if exact and not m.occupied_at((cx, cy, self.height)):
            return cx, cy, 0.0
        old = self.map_radius
        tight = self.mapping and self.map_radius_tight < old
        if tight:
            self.set_map_radius(self.map_radius_tight)
            m = self.inflated
        try:
            return snap_goto(~m.occ[:, :, k], m.origin[:2], m.res, (x, y), (cx, cy), exact=False)
        finally:
            if tight:
                self.set_map_radius(old)

    def goto_relax(self, x: float, y: float, cx: float, cy: float):
        """찍은 곳으로 가는 길을 찾은 장애물·사람의 둘레 여유가 막았을 때: 자율 탐색이 볼 자리·길이 없을 때 하는 순서대로 한 단계씩 줄여 본다
        (찾은 장애물 여유 0.2 m → 카메라로만 본 장애물 빼기 → 사람 둘레 0.15 m → 몇 번만 잡힌 사람 빼기 → 위험이 낮으면 몸 둘레 0.1 m만).
        찍은 곳 GOTO_SNAP_MAX 안까지 갈 수 있게 되는 첫 단계에서 멈추고 (갈 자리, 줄인 것들)을 돌려준다. 어느 단계로도 안 되면 모두 되돌리고 None.
        높이를 올려 사람 위로 넘기와 그 자리 착륙은 하지 않는다 (운용자가 찍었다고 내리지 않는다). 줄인 뒤에도 카메라가 본 몸·ToF·비상 정지와
        사람 1 m 안 감속은 그대로다.

        11/12 10:07 실기체: 드론 정북 3.75 m를 찍었는데 카메라 지도로는 직선이 다 비어 있었다. 찾은 사람 하나의 둘레 0.35 m와 YOLO '가까운 사람'
        상자 둘의 둘레 0.3 m가 그 사이 폭 1.1 m 입구를 닫아 '더 다가갈 곳이 없습니다'로 거절했다. 자율 탐색은 이럴 때 여유를 줄이는데
        찍은 자리는 그러지 않았다. 그 기록으로 다시 만든 지도에서는 카메라로만 본 장애물을 빼고 사람 둘레를 0.15 m로 줄이면 찍은 곳 0.4 m 앞까지 이어진다."""
        if not (self.search and self.avoid):
            return None
        saved = (self.found_radius, self.person_margin, self.people_soft, self.drop_weak_people, self.memory.items, self.map_radius)
        camera_only = [i for i in saved[4] if all(s.startswith("yolo") for s in i.sources)]

        def found_tight():
            if saved[4] and self.found_radius > FOUND_RADIUS_TIGHT:
                self.found_radius = FOUND_RADIUS_TIGHT
                return f"찾은 장애물 둘레 여유를 {FOUND_RADIUS_TIGHT} m로 줄이고"

        def drop_camera():
            if camera_only:
                self.memory.items = [i for i in saved[4] if i not in camera_only]
                return f"카메라로만 본 장애물 {len(camera_only)}개를 빼고"

        def people_tight():
            if self.blocking_people() and not self.people_soft and self.person_margin > PERSON_MARGIN_TIGHT:
                self.person_margin = PERSON_MARGIN_TIGHT
                return f"찾은 사람 둘레 여유를 {PERSON_MARGIN_TIGHT} m로 줄이고"

        def drop_weak():
            weak = [p for p in self.blocking_people() if p.hits < PERSON_WEAK_HITS]
            if weak and not self.people_soft and not self.drop_weak_people:
                self.drop_weak_people = True
                return f"몇 번만 잡힌 사람 {len(weak)}명을 빼고"

        def people_body():
            people = self.blocking_people()
            if people and not self.people_soft and people_risk(people, x, y) is None:
                self.people_soft = True
                return f"찾은 사람은 몸 둘레 {PERSON_MARGIN_SOFT} m만 남기고"

        tight = self.mapping and self.map_radius_tight < self.map_radius
        if tight:
            self.set_map_radius(self.map_radius_tight)
        k = self.inflated.index((x, y, self.height))[2]

        def reach():   # 지금 설정으로 찍은 곳 GOTO_SNAP_MAX 안까지 갈 수 있는 자리 (없으면 None)
            m = self.inflated
            got = snap_goto(~m.occ[:, :, k], m.origin[:2], m.res, (x, y), (cx, cy), exact=False) if 0 <= k < m.occ.shape[2] else None
            return got if got is not None and got[2] <= GOTO_SNAP_MAX else None

        done, spot = [], None
        # 지금도 닿는 곳이면 막은 것은 둘레 여유가 아니다: 아무것도 줄이지 않는다 (다시 불려도 한 단계씩 더 줄이지 않게)
        steps = () if reach() is not None else (found_tight, drop_camera, people_tight, drop_weak, people_body)
        for step in steps:
            what = step()
            if what is None:
                continue
            done.append(what)
            self.rebuild_maps()
            spot = reach()
            if spot is not None:
                break
        self.memory.items = saved[4]
        if spot is None:   # 줄여도 찍은 곳 가까이 못 간다: 쓸데없이 줄인 채로 두지 않는다
            self.found_radius, self.person_margin, self.people_soft, self.drop_weak_people = saved[:4]
        else:
            if any(w.startswith("카메라로만") for w in done):
                self.memory.drop_camera_only()   # 뺀 자리는 기억해 카메라로만은 다시 넣지 않는다 (가까이 가면 ToF·가까운 사람 인식이 다시 잡는다)
                self.save_discovered()
            if (self.person_margin, self.people_soft, self.drop_weak_people) != saved[1:4]:
                self.people_relaxed_at = self.people_relaxed_at or (x, y)   # 그 자리에서 2 m 벗어나면 되돌린다 (restore_after_people)
            if self.drop_weak_people and not saved[3]:
                self.goto_weak_dropped = True   # 찍은 자리 때문에 뺀 것은 같이 되돌린다 (돌아가는 길에서 뺀 것은 끝까지 둔다)
        if tight:
            self.set_map_radius(saved[5])   # 계획 지도도 다시 만든다
        else:
            self.rebuild_maps()
        return None if spot is None else (spot, done)

    def goto_choose(self, x: float, y: float, elapsed: float) -> bool:
        """받아 둔 찍은 자리를 다음 볼 자리로 잡는다 (True). 못 잡으면 알리고 False — 평소처럼 고른다.
        찍은 칸이 계획 지도에서 막혔거나 못 본 곳이면 이어진 빈 곳 가운데 가장 가까운 자리로 옮기고, 도착해서 찍은 쪽부터 본다.
        그 자리가 없거나 찍은 곳에서 멀면 찾은 장애물·사람 둘레 여유를 줄여 본다 (goto_relax).
        탐색 시간(elapsed ≥ search_time)이 다 됐어도 잡는다: 둘러본 뒤 choose_viewpoint가 돌아가기를 시작한다 (11/16, goto_refusal)."""
        cx, cy = self.goto_pending
        self.goto_pending, self.goto_active = None, None
        after = "자율 탐색을 잇습니다" if elapsed < self.search_time else \
            "출발점으로 돌아갑니다 (탐색 시간이 다 됨)" if self.return_home else "그 자리에 내립니다 (탐색 시간이 다 됨)"
        spot = self.goto_spot(x, y, cx, cy)
        relaxed = None
        if spot is None or spot[2] > GOTO_SNAP_MAX:
            better = self.goto_relax(x, y, cx, cy)
            if better is not None:
                spot, relaxed = better
        if spot is None:
            self.goto_note(f"찍은 자리 ({cx:.1f}, {cy:.1f}) 쪽으로는 지금 자리보다 더 다가갈 수 있는 본 빈 곳이 없습니다 "
                           f"(찍은 곳 {GOTO_SNAP_MAX} m 안에도 없고, 찾은 장애물·사람 둘레 여유를 줄여 봐도 같음). {self.goto_after()}.", warn=True)
            return False
        gx, gy, moved = spot
        self.goal = np.array([gx, gy, self.height])
        self.goal_area = 0.0
        self.vp_budget = None   # 경로가 나오면 make_plan이 새로 잡는다 (찍은 자리도 볼 자리 한도는 같다)
        self.leg_from = (x, y)
        # fallback: 이미 '갈 수 있는 가장 가까운 자리'로 옮겼는가 (아니면 찍은 칸까지 길이 없을 때 한 번 옮겨 본다, _make_plan)
        self.goto_active = {"click": (cx, cy), "goal": (gx, gy), "arrived": False, "fallback": moved > 0.05}
        self.goto_count[0] += 1
        dist = math.hypot(gx - x, gy - y)
        if moved > GOTO_LOOK_AT and self.mapping:
            self.look_at = (cx, cy)
        if relaxed:
            self.goto_note(self.goto_relaxed_text(cx, cy, gx, gy, moved, relaxed) + f" ({dist:.1f} m 거리)", warn=True)
        elif moved > GOTO_LOOK_AT:
            self.goto_note(f"찍은 자리 ({cx:.1f}, {cy:.1f})는 막혔거나 아직 못 본 곳이라, 거기서 {moved:.1f} m 떨어진 갈 수 있는 자리 "
                           f"({gx:.1f}, {gy:.1f})로 가서 찍은 쪽을 봅니다 ({dist:.1f} m 거리). 둘러본 뒤 {after}.")
        else:
            self.goto_note(f"찍은 자리 ({gx:.1f}, {gy:.1f})로 갑니다 ({dist:.1f} m 거리). 도착하면 둘러본 뒤 {after}.")
        return True

    @staticmethod
    def goto_relaxed_text(cx: float, cy: float, gx: float, gy: float, moved: float, relaxed) -> str:
        where = f"찍은 자리 ({gx:.1f}, {gy:.1f})로" if moved <= GOTO_LOOK_AT else f"찍은 곳에서 {moved:.1f} m 떨어진 ({gx:.1f}, {gy:.1f})로"
        return (f"찍은 자리 ({cx:.1f}, {cy:.1f})로 가는 길을 찾은 장애물·사람 둘레가 막고 있어, {' '.join(relaxed)} {where} 갑니다. "
                f"사람 {PERSON_SLOW_R:g} m 안에서는 {PERSON_SLOW_SPEED} m/s로 가고, 카메라가 본 몸·ToF·비상 정지는 그대로 피합니다.")

    def goto_fallback(self, x: float, y: float, err) -> bool:
        """찍은 자리까지 길이 없다 (True면 갈 자리를 바꿔 다시 계획한다).
        먼저 찾은 장애물·사람 둘레 여유를 줄여 본다(goto_relax — 가는 중에 새로 막혀도 다시). 그래도 안 되면 한 번, 드론과 이어진 빈 곳 가운데
        찍은 곳에 가장 가까운 자리로 옮긴다.
        11/11 Gazebo 방 두 개: 웹 지도에 본 곳으로 보이는 3.9 m 밖 자리가 계획 지도에서는 아직 이어지지 않아 '가는 길이 없습니다'로 끝냈다."""
        a = self.goto_active
        cx, cy = a["click"]
        better = self.goto_relax(x, y, cx, cy)
        if better is not None:
            (gx, gy, moved), relaxed = better
            a["goal"], a["fallback"] = (gx, gy), True
            self.goal = np.array([gx, gy, self.height])
            if moved > GOTO_LOOK_AT and self.mapping:
                self.look_at = (cx, cy)
            self.goto_note(self.goto_relaxed_text(cx, cy, gx, gy, moved, relaxed), warn=True)
            self.set_state("REPLAN")
            return True
        if a["fallback"]:
            return False
        a["fallback"] = True
        spot = self.goto_spot(x, y, cx, cy, exact=False)
        if spot is None or math.hypot(spot[0] - a["goal"][0], spot[1] - a["goal"][1]) < 0.05:
            return False
        gx, gy, moved = spot
        a["goal"] = (gx, gy)
        self.goal = np.array([gx, gy, self.height])
        if self.mapping:
            self.look_at = (cx, cy)
        self.goto_note(f"찍은 자리 ({cx:.1f}, {cy:.1f})까지는 아직 길이 이어지지 않아({err}), 그쪽으로 가장 가까이 갈 수 있는 자리 "
                       f"({gx:.1f}, {gy:.1f})(찍은 곳에서 {moved:.1f} m)로 가서 찍은 쪽을 봅니다.")
        self.set_state("REPLAN")
        return True

    # ------------------------------------------------------------ 입력

    def on_mission_cmd(self, msg: String) -> None:
        cmd = msg.data.strip()
        if cmd == "start":
            if self.state == "READY" and self.wait_detect and not self.detecting():
                # 11/03 14:32: launch 3.8초 뒤 시작해 YOLO(준비에 21.5초)가 뜨기 전 18초를 사람 탐지 없이 날았다
                self.get_logger().warn("YOLO가 아직 준비되지 않아 시작하지 않습니다 (탐지 결과가 와야 합니다. launch 뒤 20초쯤).")
            elif self.state == "READY":
                self.start_requested = True
                self.get_logger().info("시작 요청을 받았습니다.")
            else:
                self.get_logger().warn(f"시작 요청을 무시합니다 (지금 단계 {self.state}).")
        elif cmd == "land":
            self.land_requested = True
            self.get_logger().info(f"착륙 요청을 받았습니다 (지금 단계 {self.state}).")
        else:
            try:
                req = parse_goto(cmd)
            except ValueError as err:
                self.get_logger().warn(f"찍은 자리 명령을 읽지 못했습니다 ({cmd}): {err}")
                return
            if req is None:
                self.get_logger().warn(f"모르는 비행 노드 명령: {cmd}")
            elif req[0] == "cancel":
                self.goto_cancel()
            else:
                self.goto_request(req[1], req[2])

    def on_flight_status(self, msg: String) -> None:
        info = json.loads(msg.data)
        flying = bool(info.get("flying"))
        if self.flying and not flying and self.state not in ("LAND", "DONE", "ABORT"):
            auto = info.get("last_auto_land") or {}
            self.forced_landing = auto.get("reason") or info.get("landing_reason") or "알 수 없음"
        self.flying, self.flight_info = flying, info

    def start_blocked(self) -> bool:
        """시작 요청을 받지 않을까: 땅에서 Tello 기수 값이 흐르고 있으면 True (이유를 알린다). 멈춰 있거나 상태값이 아직 덜 쌓였으면,
        또는 TAKEOFF_AGAIN 안에 다시 누른 것이면 False."""
        now = self.now()
        span = yaw_span(self.ground_yaw, now)
        if span is None or span < YAW_SETTLE_DEG:
            self.start_refused_t = None
            return False
        if self.start_refused_t is not None and now - self.start_refused_t <= TAKEOFF_AGAIN:
            self.start_refused_t = None
            self.get_logger().warn(f"기체 기수 값이 아직 흐르지만(최근 {YAW_SETTLE_WINDOW:.0f}초에 {span:.0f}°) 다시 누르셨으므로 이륙을 "
                                   "요청해 봅니다.")
            return False
        self.start_refused_t = now
        self.takeoff_holds += 1
        self.get_logger().warn(f"기체 기수 값이 땅에서 흐르고 있어(최근 {YAW_SETTLE_WINDOW:.0f}초에 {span:.0f}°) 시작하지 않았습니다. 이 상태에서는 "
                               "Tello가 이륙을 거절해 왔습니다. 기체를 살짝 들었다 평평한 곳에 다시 내려놓고, 웹 화면의 기수 값이 멈추면 "
                               "다시 시작하세요 (방향이 0°에서 벗어났으면 이 프로그램을 다시 띄웁니다). 그래도 띄워 보려면 "
                               f"{TAKEOFF_AGAIN:.0f}초 안에 시작을 한 번 더 누르세요.")
        return True

    def takeoff_result(self) -> dict:
        """드라이버가 알려 준 마지막 takeoff 결과의 응답 기록 (없으면 빈 dict)."""
        r = self.flight_info.get("last_result") or {}
        return (r.get("result") or {}) if r.get("cmd") == "takeoff" else {}

    def takeoff_refused(self) -> bool:
        """Tello가 이번 이륙 요청을 바로 거절했으면(error 응답) 저절로 다시 보내지 않고 시작 대기로 돌아간다 (True)."""
        res = self.takeoff_result()
        if (self.flight_req is None or self.flight_req[0] != "takeoff" or res.get("reason") != "error_response"
                or res.get("sent_at") == self.takeoff_seen):
            return False
        self.takeoff_seen = res.get("sent_at")
        self.flight_req = None
        self.takeoff_refusals += 1
        span = yaw_span(self.ground_yaw, self.now())
        yaw = "" if span is None else (f" 기체 기수 값은 최근 {YAW_SETTLE_WINDOW:.0f}초에 {span:.0f}° 바뀌었습니다"
                                       + (" (흐르는 중)." if span >= YAW_SETTLE_DEG else " (멈춰 있음)."))
        self.get_logger().error("Tello가 이륙을 거절했습니다 (error 응답, 이유는 알려 주지 않습니다). 저절로 다시 보내지 않습니다 — 기체를 "
                                f"살펴도 됩니다.{yaw} 기체를 살짝 들었다 평평한 곳에 다시 내려놓고 기수 값이 멈추면 다시 시작하세요.")
        self.set_state("READY")
        return True

    def request_flight(self, cmd: str, retry: float) -> int:
        """드라이버에 이륙·착륙을 요청한다. 같은 요청은 retry초마다 한 번만. 지금까지 보낸 횟수를 돌려준다."""
        req = self.flight_req
        if req is None or req[0] != cmd or self.now() - req[1] > retry:
            n = 1 if req is None or req[0] != cmd else req[2] + 1
            self.flight_req = (cmd, self.now(), n)
            self.flight_pub.publish(String(data=cmd))
            self.get_logger().info(f"드라이버에 {cmd} 요청 ({n}번째)")
        return self.flight_req[2]

    def on_pose(self, msg: Odometry) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.raw_pose = (p.x, p.y, p.z, yaw)
        self.pose = self.correction.apply(self.raw_pose)
        var = max(msg.pose.covariance[0], msg.pose.covariance[7])
        # 위치 추정 노드는 오차를 모르면(속도 적분만, 실기체 기본) −1을 보낸다. 전에는 이것도 0(정확)으로 바꿨다 (11/04 외부 리뷰)
        self.pose_sigma_known = var >= 0
        self.pose_sigma = math.sqrt(var) if var > 0 else (0.0 if var == 0 else math.inf)
        self.pose_t, self.pose_steps = self.now(), 0
        self.history.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, self.raw_pose))
        if self.state in TRAIL_STATES and (not self.trail
                                           or math.hypot(p.x - self.trail[-1][0], p.y - self.trail[-1][1]) >= TRAIL_STEP):
            self.trail.append(self.raw_pose)
            if len(self.trail) > TRAIL_MAX:
                del self.trail[:TRAIL_MAX // 6]

    def raw_at(self, t: float):
        """t 시각의 위치 추정 노드 값 (보정 전, 앞뒤 값을 이어 보간). 0.1 s 안에 없으면 None."""
        if not self.history:
            return None
        h = self.history
        if t >= h[-1][0]:
            return h[-1][1] if t - h[-1][0] < 0.1 else None
        if t <= h[0][0]:
            return h[0][1] if h[0][0] - t < 0.1 else None
        i = next(k for k in range(len(h) - 1, 0, -1) if h[k - 1][0] <= t)
        (t0, a), (t1, b) = h[i - 1], h[i]
        if t1 - t0 > 0.2:
            return a if t - t0 < t1 - t else b
        w = (t - t0) / max(t1 - t0, 1e-9)
        yaw = a[3] + w * math.remainder(b[3] - a[3], 2 * math.pi)
        return tuple(a[k] + w * (b[k] - a[k]) for k in range(3)) + (yaw,)

    def pose_at(self, t: float):
        raw = self.raw_at(t)
        return None if raw is None else self.correction.apply(raw)

    def sensor_origin(self, pose) -> tuple[float, float, float]:
        x, y, z, yaw = pose
        return x + SENSOR_X * math.cos(yaw), y + SENSOR_X * math.sin(yaw), z

    def on_depth(self, msg: Image) -> None:
        """4단계: 깊이 영상을 찍은 순간의 위치로 지도에 넣는다. 이륙 전·높이를 못 맞춘 때는 넣지 않는다."""
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        raw = self.raw_at(stamp + self.pose_lag)
        if raw is None or self.state not in ("PLAN",) + AIRBORNE or raw[2] < 0.5:
            return   # 이륙 전·이륙 직후(기수가 들림)는 넣지 않는다. 가구 위로 올라가 있을 때는 넣는다 (11/03)
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        if self.recorder:
            self.recorder.add_depth(stamp, raw, depth)
        pose = self.correction.apply(raw)
        cam = self.sensor_origin(pose)
        if self.tof_scale is not None:
            # 실기체: 정면 ToF로 맞춘 배율 (바닥 배율은 1.30에 붙어 깊이를 30% 멀게 넣었다, 11/04 12:57)
            self.depth_scale = self.tof_scale.value(self.now())
            self.log_depth_scale()
        else:
            # 장면마다 흔들리는 전체 배율을 바닥으로 맞춘다. 배율이 틀리면 벽·가구가 앞당겨 찍혀 통로·문이 막혔다 (seed 1–3)
            scale = floor_scale(depth, self.hfov, cam[2])
            if scale is not None:
                self.depth_scale = scale
                self.floor_frames += 1
        depth = depth * self.depth_scale
        self.scene_scale = self.depth_scale
        self.scene = (self.now(), pose[3], *scene_ranges(depth, self.hfov, cam[2], pose[2]))   # 지금 높이 단면 (정면 ToF와 같은 높이)
        if self.localize_on and self.loc_jobs.empty():
            try:
                self.loc_jobs.put_nowait((stamp, depth, raw))
            except queue.Full:
                pass
        st = self.see_through
        if st is not None and self.now() - st[0] < SEE_THROUGH_HOLD and abs(wrap(pose[3] - st[1])) < SEE_THROUGH_YAW:
            # 흰 벽·유리를 꿰뚫어 보는 쪽: ToF가 잰 면까지만 비운다. 막힘은 찍지 않게 hit_min_range 앞까지 (면 모양은 ToF가 찍는다)
            depth = np.minimum(depth, min(st[2], self.cmap.p.hit_min_range - 0.05))
        # 정면 ToF로 맞춘 배율(실기체)이 1보다 작으면 바닥 문턱을 그만큼 올린다. 바닥으로 맞춘 배율(시뮬레이션)은 바닥이 제자리에 온다
        self.cmap.integrate_depth(depth, cam, pose[3], self.hfov, scale=self.depth_scale if self.tof_scale is not None else 1.0)
        self.depth_frames += 1

    def localize_loop(self) -> None:
        """4단계 위치 맞추기 스레드: 받은 영상을 예전 지도에 대어 보정(self.correction)을 조금씩 고친다."""
        while True:
            stamp, depth, raw = self.loc_jobs.get()
            pose = self.correction.apply(raw)   # 지금 보정으로 놓는다 (앞 영상의 보정이 그사이 바뀌었을 수 있다)
            try:
                self.localizer.update(stamp, self.cmap, depth, self.sensor_origin(pose), pose, self.hfov, raw=raw)
            except Exception as err:   # 맞추기가 실패해도 비행은 계속한다
                self.get_logger().error(f"위치 맞추기 오류: {err}")

    def on_tof(self, topic: str, msg: LaserScan) -> None:
        # REP 117: +inf는 범위 안에 없음, NaN은 잴 수 없음(버림), −inf는 너무 가까움. 전에는 NaN뿐인 줄도 '비었음'(inf)으로 봤다 (11/04)
        rng = [r for r in msg.ranges if not math.isnan(r)]
        if not rng:
            return
        near = msg.range_min if msg.range_min > 0 else 0.02
        valid = [near if r == -math.inf else r for r in rng if r == -math.inf or (math.isfinite(r) and r > 0)]
        now = self.now()
        self.tof_rows[topic] = (min(valid) if valid else math.inf, now)
        if topic != TOF_TOPICS[0]:
            return   # 세 줄이 거의 같이 오므로 가운데 줄이 올 때 한 번에 처리한다
        self.front_prev, self.front = self.front, min((r for r, t in self.tof_rows.values() if now - t < 0.3),
                                                      default=math.inf)
        self.front_t, self.front_steps = now, 0
        if not self.avoid or self.pose is None or self.state not in AIRBORNE or self.state == "RELOC":
            return
        # 측정한 순간의 위치·방향으로 지도와 비교한다 (돌고 있을 때 최신 위치를 쓰면 엉뚱한 벽을 장애물로 본다)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.mapping:   # 4단계: ToF는 지도에 바로 넣는다 (가까운 곳은 단안 깊이보다 정확하다)
            raw = self.raw_at(stamp + self.pose_lag) or self.raw_pose
            self.tof_pose = self.correction.apply(raw)
            if self.recorder:
                self.recorder.add_tof(stamp, raw, self.front)
            blind = not self.camera_knows(self.tof_hit_point())   # 지도에 없는 것 = 카메라가 못 보는 것 (흰 벽·유리·얇은 기둥)
            self.cmap.integrate_ray(self.sensor_origin(self.tof_pose), self.tof_pose[3], self.front, TOF_MAX)
            if self.front < TOF_MAX - 0.05:
                # ToF가 면을 잰 자리는 '지나온 칸'이 아니다 (위치가 틀어져 예전에 지나온 칸과 겹쳐도, 최신의 확실한 관측이 이긴다)
                z = self.tof_pose[2]
                self.cmap.unfly(self.tof_hit_point(), TOF_UNFLY, z - 0.35, z + 0.35)
            self.update_tof_scale()
            self.check_see_through()
            if blind and self.glass_check():
                return
            self.note_tof_shape(blind)
            return
        self.tof_pose = self.pose_at(stamp) or self.pose
        if self.glass_check():
            return
        if self.check_tof():
            self.tof_streak += 1
            if self.tof_streak >= 2:   # 한 번 튄 값은 무시한다
                self.add_tof_obstacle()
        else:
            self.tof_streak = 0

    def glass_check(self) -> bool:
        """정면 ToF가 비어 있다가 GLASS_NEAR 안에서 처음 잡혔으면 유리·거울로 보고 기수와 직각으로 양옆 GLASS_HALF를 막는다.

        다가오는 벽은 보통 1.2 m 근처에서 먼저 잡혀 줄어든다. 처음부터 가까우면 카메라도 ToF도 못 보던 것이다
        (2026-11-02 17:33 실기체: 유리를 0.18 m에서 처음 잡았고, 깊이 장면은 유리 너머를 봐 '기둥 폭 0.1 m'로 좁게만 막았다).
        """
        if not late_hit(self.front_prev, self.front, GLASS_NEAR, TOF_MAX):
            return False
        origin, yaw = self.sensor_origin(self.tof_pose), self.tof_pose[3]
        half = math.atan2(GLASS_HALF, self.front)
        if self.mapping:
            z0, z1 = max(0.2, self.tof_pose[2] - 0.5), min(self.ceiling, self.tof_pose[2] + 0.5)
            self.cmap.mark_hard(front_points(origin, yaw, self.front, -half, half, z0, z1, self.cmap.p.res), HARD_WEIGHT)
            self.update_camera_map(*self.pose[:3])
        else:
            self.add_obstacle(tof_box(origin, yaw, self.front, half, self.ceiling), "tof",
                              f"ToF {self.front:.2f} m에서 처음 잡힘(유리·거울로 봄)")
        self.get_logger().warn(f"앞 {self.front:.2f} m에서 처음 잡혔습니다(1.2 m 안이 비어 있다가). 카메라가 못 보는 유리·거울로 보고 "
                               f"양옆 {GLASS_HALF} m를 막습니다.")
        return True

    def on_tello_state(self, msg: String) -> None:
        try:
            s = parse_tello_state(json.loads(msg.data).get("raw", ""))
            pitch, roll = float(s.get("pitch", 0.0)), float(s.get("roll", 0.0))
            agx, agy = float(s.get("agx", 0.0)), float(s.get("agy", 0.0))
            self.tilt = max(abs(pitch), abs(roll))
            if isinstance(s.get("yaw"), (int, float)):
                now = self.now()
                self.ground_yaw.append((now, float(s["yaw"])))
                while self.ground_yaw and self.ground_yaw[0][0] < now - YAW_SETTLE_WINDOW - 2.0:
                    self.ground_yaw.popleft()
            tof = s.get("tof")
            self.tof_down = tof / 100.0 if isinstance(tof, (int, float)) and 0 < tof < 500 else None   # cm → m, 범위 밖은 없음
        except (ValueError, TypeError, AttributeError):
            return
        if self.state in AIRBORNE:
            # 부딪힘은 상태값 하나(0.1초)에만 나오므로 여기서 잡아 두고, 떼어 낼 방향도 그 값으로 정한다
            hit = self.watch.state(self.now(), agx, agy, pitch, roll)
            if hit and self.contact_hit is None:
                self.contact_hit = (hit, self.watch.away(), self.now())
        if self.mapping and self.pose is not None and self.state in AIRBORNE:
            # 하방 ToF는 바로 밑 물건까지 거리다. 밑에 지도의 가구가 있으면(깊이로 쌓은 지도 = 비전) 그 꼭대기를 위치 추정에 알려
            # 바닥 위 높이 = ToF + 꼭대기로 잡게 한다 (11/09 17:07: 조형물 위에서 높이가 0.67 m로 잡혀 천장에 눌림)
            x, y, _, _ = self.pose
            self.under_top = self.cmap.top_below(x, y, self.max_height - 0.2)   # 추정 높이가 아니라 경로 상한 아래에서 (되먹임 없게)
            self.under_frames += self.under_top is not None
            self.under_pub.publish(String(data=json.dumps({"t": round(self.now(), 4),
                                                            "top": None if self.under_top is None else round(self.under_top, 2)})))

    def past_xy(self) -> list:
        """최근 5초 위치 (오래된 것 → 최근, 지도 좌표)."""
        return [self.correction.apply(p)[:2] for _, p in self.history]

    def brake_before_turn(self, vz: float) -> bool:
        """제자리에서 돌려는데 아직 앞으로 가고 있으면, 돌지 않고 뒤로 짧게 밀어 관성을 끊는다(True: 이번 주기는 제동).
        앞 속도가 TURN_BRAKE_START 아래로 떨어지거나 TURN_BRAKE_TIME이 지나면 False (돌아도 된다)."""
        now = self.now()
        vx = turn_brake(forward_speed(self.history), TURN_BRAKE_START, TURN_BRAKE_GAIN, TURN_BRAKE_LOW, TURN_BRAKE_HIGH)
        if vx is None:
            self.turn_brake_t0 = None
            return False
        if self.turn_brake_t0 is None:
            self.turn_brake_t0 = now
            self.turn_brakes += 1
            if now - self.turn_brake_logged > 5.0:
                self.turn_brake_logged = now
                self.get_logger().info(f"돌기 전에 앞으로 {-vx / TURN_BRAKE_GAIN:.2f} m/s쯤 미끄러지고 있어 먼저 멈춥니다.")
        if now - self.turn_brake_t0 > TURN_BRAKE_TIME:
            return False
        self.send(vx, 0.0, vz, 0.0)
        return True

    def moving_fast(self) -> bool:
        """최근 0.5초에 0.15 m/s 넘게 움직였는가 (물러나기·되짚기 처음 BRAKE_TIME에 제동할지)."""
        if not self.history:
            return False
        recent = [p for t, p in self.history if t >= self.history[-1][0] - 0.5]
        return len(recent) >= 2 and math.hypot(recent[-1][0] - recent[0][0], recent[-1][1] - recent[0][1]) > 0.15 * 0.5

    def rear_blocked(self, x: float, y: float, z: float, yaw: float) -> float | None:
        """기체 뒤쪽 REAR_CHECK·옆 ±REAR_HALF 띠에서 '본 빈 칸'·'지나온 칸'이 아닌 첫 곳까지 거리. 다 알면 None.
        카메라 지도가 없는 모드(받은 지도로 나는 1–3단계)는 보지 않는다."""
        if not self.mapping:
            return None
        return rear_unknown(self.cmap.known_free_at, (x, y, z), yaw, REAR_CHECK, REAR_HALF)

    def note_rear(self, d: float | None, what: str, off: float | None = None) -> None:
        """물러나지 않는 까닭을 남긴다: 뒤 d m 안을 모르거나(d), 뒤가 들어온 길에서 off(rad)만큼 벗어났다."""
        now = self.now()
        # 돌기 전 확인은 막힌 동안 매 순간 다시 걸린다(Gazebo 사람 둘레 판 71번) — 한 번 막힌 일은 한 번으로 센다
        if now - self.rear_last > 2.0:
            self.rear_refused += 1
            self.off_trail_refused += d is None
        self.rear_last = now
        if now - self.rear_logged > 5.0:
            self.rear_logged = now
            if d is not None:
                self.get_logger().warn(f"{what} 뒤 {d:.2f} m 안이 카메라가 본 적 없는 곳(또는 막힘)이라 물러나지 않고 천천히 돕니다.")
            else:
                how = "들어온 길이 없어" if off is None or not math.isfinite(off) else f"기체 뒤쪽이 들어온 길에서 {math.degrees(off):.0f}° 벗어나 있어"
                self.get_logger().warn(f"{what} {how}(돈 뒤라 뒤는 지나온 곳이 아님) 물러나지 않고 천천히 돕니다.")

    def retreat_check(self, x: float, y: float, yaw: float):
        """곧게 뒤로 물러나도 되는가 → (뒤 띠에서 모르는 곳까지 거리 | None, 들어온 길에서 벗어난 각 | None, 자취 방향, 둘레 빈 거리).
        앞의 둘이 모두 None이어야 물러난다. 벗어난 각은 들어온 길이 아예 없으면 inf.

        물러나기는 기체 뒤쪽으로 곧게 간다. 그 뒤쪽은 (1) 카메라가 본 빈 곳이거나 지나온 칸이어야 하고(11/09 13:03), (2) 들어온 길 쪽
        (자취 방향에서 TRAIL_SIDEWAYS 안)이거나 뒤 띠가 모두 실제로 지나온 칸이어야 한다(11/16). 돌고 난 뒤의 뒤쪽은 지나온 곳이 아니다 —
        카메라가 '봤다'는 것만으로는 못 믿는다: 단안 깊이는 얇은 문틀·흰 벽을 못 찍거나 어긋나게 찍는다.
        11/16 Gazebo 방 두 개: 폭 0.9 m 문을 +x로 지난 직후(문 가운데를 실제 벽보다 0.5 m 앞으로 잡아 아직 문틀 사이였다) 둘러보려고 −80°로
        돌다 한쪽 문틀이 정면 0.36 m에 잡혔고, 뒤(들어온 길에서 80° 벗어난 쪽)로 0.35 m 물러나 반대편 문틀 0.04 m까지 붙었다.
        그 문틀은 지도에 없었고 뒤 띠는 '본 빈 곳'이었다. 돌 자리가 없어(둘레 TURN_ROOM 안이 막힘) 자취 방향으로 가는 경우는 그대로다
        (retreat_body: 지나온 선으로 간다). 정면이 TURN_ROOM 안으로 바짝 붙었을 때도 그대로 물러난다 — 제자리에서 돌면 기체 모서리
        (중심에서 0.14 m)가 닿는다. 그보다 멀면 물러나지 않아도 천천히 돌 수 있다.
        지난 실기체 물러나기 44번(11/03–11/13)에 대 보면 22번이 들어온 길에서 45° 넘게 벗어난 쪽이었고, 그 가운데 7번에 닿은 흔적
        (기수가 저절로 10°+ 돎·수평 가속 0.3 g+·바로 멈춤)이 있었다 — 들어온 길 쪽 22번에서는 2번(하나는 이미 뒤집힌 뒤).
        그 22번 중 정면이 0.25 m 넘게 떨어져 있던 19번이 이제 '물러나지 않고 천천히 돌기'가 된다."""
        z = self.pose[2]
        rear = self.rear_blocked(x, y, z, yaw)
        if rear is not None:
            return rear, None, None, 0.0
        past = [self.correction.apply(p)[:2] for p in self.trail[-30:]] or self.past_xy()   # 자취: 물러나거나 후진한 자리는 뺀 것
        direction = retreat_direction(past, (x, y), RETREAT_DIST)
        room = room_around(self.occupied_near, (x, y, z), 1.0)
        off = None
        if self.mapping:
            angle = math.inf if direction is None else off_tail(direction, yaw)   # 들어온 길이 없으면(막 떴으면) inf
            along_trail = direction is not None and (angle <= TRAIL_SIDEWAYS or room < TURN_ROOM)
            can_turn = not (math.isfinite(self.front) and self.front < TURN_ROOM)   # 정면이 바짝 붙었으면 돌 자리가 없다
            if not along_trail and can_turn and rear_unknown(self.cmap.flown_at, (x, y, z), yaw, REAR_CHECK, REAR_HALF) is not None:
                off = angle
        return None, off, direction, room

    def start_backoff(self, x: float, y: float, yaw: float, then: str = "REPLAN") -> bool:
        """들어온 길로 천천히 물러난다. 시작했으면 True.
        실제 움직임은 기체 뒤쪽으로 곧게라, 뒤 띠가 아는 빈 곳이 아니거나(11/09 13:03) 뒤가 들어온 길 쪽이 아니면(돌고 난 뒤, 11/16)
        물러나지 않고 멈춘 채 then으로 간다 (retreat_check)."""
        rear, off, direction, room = self.retreat_check(x, y, yaw)
        if rear is not None or off is not None:
            self.send()
            self.note_rear(rear, "물러나려 했지만", off)
            self.slow_turn_until = self.now() + SLOW_TURN_AFTER
            self.set_state(then)
            return False
        self.backoff = {"x": x, "y": y, "z": self.pose[2], "t": self.now(), "yaw": yaw,
                        "dir": direction, "brake": self.moving_fast(), "then": then, "room": room}
        self.set_state("BACKOFF")
        return True

    def retreat_cmd(self, direction, yaw: float, speed: float, room: float,
                    lateral: float = LATERAL_MAX) -> tuple[float, float]:
        """물러나거나 후진할 때의 기체 기준 속도 (autonomy/near.py retreat_body): 기체 뒤쪽으로 곧게, 옆은 lateral까지만."""
        return retreat_body(direction, yaw, speed, room, TURN_ROOM, lateral, TRAIL_SIDEWAYS)

    def pocket(self, x: float, y: float, z: float, yaw: float) -> tuple[bool, float, float, float]:
        """막힌 주머니인가 (autonomy/near.py pocket): 카메라 지도(막힘 확정)만 본다 — 유리는 지도에 없을 수 있다.
        5 cm·5° 안에서 0.5 s 안에 잰 값이 있으면 그것을 쓴다 (제자리 회전 틱마다 광선 수백 개를 쏘지 않게)."""
        c = self.pocket_cache
        now = self.now()
        if c is not None and now - c[3] < 0.5 and math.hypot(x - c[0], y - c[1]) < 0.05 and abs(wrap(yaw - c[2])) < math.radians(5):
            return c[4]
        out = pocket(self.occupied_near, (x, y, z), yaw, TURN_ROOM, POCKET_RANGE, POCKET_SECTOR)
        self.pocket_cache = (x, y, yaw, now, out)
        return out

    def try_exit(self, x: float, y: float, z: float, yaw: float) -> bool:
        """막힌 주머니에서 멈췄으면 들어온 자취를 따라 후진하기 시작한다 (시작했으면 True). 사람이 가까웠으면(뒤는 센서가 없다) 하지
        않고 천천히 돈다. 같은 자리에서 EXIT_TRIES번 후진해도 그대로면(실습실 Gazebo: 위치가 0.36 m 틀어져 가구에 붙은 채) 후진을 그만두고
        보통대로 계획한다 — 내리지는 않는다."""
        blocked, room, left, right = self.pocket(x, y, z, yaw)
        if not blocked:
            return False
        now = self.now()
        near = self.people_near(x, y)
        if near is not None:
            if now - self.reverse_logged > 5.0:
                self.reverse_logged = now
                self.get_logger().warn(f"막힌 주머니(둘레 {room:.2f} m)지만 {near:.1f} m 안에 사람이 있었습니다. 뒤가 안 보여 "
                                       "후진하지 않고 천천히 돕니다.")
            self.slow_turn_until = now + 1.0
            return False
        if (self.exit_spot is not None and now - self.exit_spot_t < 120.0
                and math.hypot(x - self.exit_spot[0], y - self.exit_spot[1]) < 0.7):
            self.exit_tries += 1
        else:
            self.exit_spot, self.exit_tries = (x, y), 1
        self.exit_spot_t = now
        if self.exit_tries > EXIT_TRIES:
            if self.exit_tries == EXIT_TRIES + 1:
                self.get_logger().warn(f"같은 자리에서 {EXIT_TRIES}번 후진해도 그대로입니다. 후진을 그만두고 보통대로 계획합니다.")
            return False
        return self.start_exit(x, y, yaw, f"돌 자리가 없고(둘레 {room:.2f} m) 정면 왼쪽 {left:.1f} m·오른쪽 {right:.1f} m 안이 막혀")

    def trim_trail(self) -> None:
        """물러나거나 되짚은 뒤 자취를 지금 자리까지 자른다. 뒤에 남은 점이 자취를 접어(U턴) 다음 되짚기를 흔들었다."""
        if self.raw_pose is None or not self.trail:
            return
        cut = trail_trim(self.trail, self.raw_pose[:2], 40, 0.5)
        if cut:
            del self.trail[-cut:]

    def occupied_near(self, p) -> bool:
        """카메라 지도(막힘 확정)에서 p 높이 ±0.1 m 안이 막혔는가 (기체 높이 10 cm, 가드 위아래)."""
        return any(self.known.occupied_at((p[0], p[1], p[2] + dz)) for dz in (-0.1, 0.0, 0.1))

    def people_near(self, x: float, y: float):
        """최근 PEOPLE_REAR_TIME 안에 잡힌 사람 중 PEOPLE_REAR 안의 가장 가까운 거리 (m). 없으면 None."""
        now = self.now()
        d = [math.hypot(p.x - x, p.y - y) for p in self.people.people
             if not p.gone and now - max(p.t_first, p.t_last) < PEOPLE_REAR_TIME]
        d = [v for v in d if v < PEOPLE_REAR]
        return min(d) if d else None

    def reverse_out(self, x: float, y: float, z: float, yaw: float) -> bool:
        """경로를 따라가다 제자리에서 돌려 하는데 다음 경유점이 뒤쪽(REVERSE_TURN 넘게)이고 막힌 주머니면, 돌지 않고 들어온 자취를
        따라 후진한다(EXIT). 시작했으면 True. 그 밖에는 (천천히) 돌아서 정면을 보고 간다."""
        if not self.waypoints:
            return False
        wp = self.waypoints[0]
        if abs(wrap(math.atan2(wp[1] - y, wp[0] - x) - yaw)) < REVERSE_TURN:
            return False
        return self.try_exit(x, y, z, yaw)

    def flip_check(self) -> bool:
        """기울기가 FLIP_TILT 넘게 FLIP_HOLD 이어지면 뒤집힌 것: 비상 정지(모터 끔)를 보내고 중단한다. 중단했으면 True."""
        if self.tilt < FLIP_TILT:
            self.flip_since = None
            return False
        now = self.now()
        if self.flip_since is None:
            self.flip_since = now
            return False
        if now - self.flip_since < FLIP_HOLD:
            return False
        self.send()
        self.crashed = True
        self.get_logger().error(f"기울기 {self.tilt:.0f}°가 {FLIP_HOLD}초 넘게 이어집니다. 뒤집혀 떨어진 것으로 보고 모터를 끕니다(비상 정지).")
        if self.driver:
            self.flight_pub.publish(String(data="emergency"))
        self.set_state("ABORT")   # 이유(바로 위 로그)를 잡도록 저장보다 먼저
        self.save_people()
        self.save_camera_map()
        return True

    def contact_check(self, x: float, y: float, z: float, yaw: float) -> bool:
        """모든 비행 단계에서 닿음을 본다 (autonomy/contact.py). 닿았으면 떼어 내기를 시작하고 True."""
        hit, self.contact_hit = self.contact_hit, None
        now = self.now()
        if self.state == "UNSTICK":
            u = self.unstick
            if (hit is not None and hit[1] is not None and now - hit[2] <= 0.5
                    and math.cos(wrap(yaw + hit[1]) - u["dir"]) < -0.3):
                # 미는 쪽에서 부딪혔다 (반력이 미는 방향의 반대쪽): 더 밀지 않고 그쪽도 막는다. 처음 닿은 쪽의 튐(같은 방향)은 흘려보낸다
                self.send()
                self.mark_contact(x, y, z, u["dir"], f"떼어 내는 쪽에서 {hit[0]}")
                self.get_logger().warn(f"떼어 내는 쪽({side_name(u['dir'] - yaw)})에서도 닿았습니다 ({hit[0]}). 더 밀지 않고 "
                                       f"입력을 끊어 {CONTACT_HOLD:.1f}초 머뭅니다.")
                self.contact_pushes += 1
                self.contact_episode["t"] = now
                self.contact_until, self.contact_armed = now + CONTACT_HOLD, now + CONTACT_SETTLE
                self.set_state("HOLD")
                return True
            self.watch.reset()
            self.push.reset()
            return False
        if now < self.contact_armed or (self.tof_down is not None and self.tof_down < CONTACT_GROUND):
            self.watch.reset()
            self.push.reset()
            return False
        # 가라는데 안 감: 지난 틱에 보낸 수평 명령을 보정 전 위치와 같은 좌표(지도 방향)로 넣는다
        rx, ry, _, ryaw = self.raw_pose or self.pose
        c, s = math.cos(ryaw), math.sin(ryaw)
        vx, vy = self.last_cmd[0], self.last_cmd[1]
        self.push.add(now, (c * vx - s * vy, s * vx + c * vy), (rx, ry), react_world(self.watch.last, ryaw))
        if hit is not None and now - hit[2] > 0.5:
            hit = None   # 다른 단계일 때 잡힌 오래된 것
        if hit is None:
            why = self.watch.turning(now, self.last_cmd[3], ryaw)
            if why is not None:
                hit = (why, self.watch.away(), now)
        if hit is None:
            stall = self.push.check(now)
            if stall is None:
                return False
            self.stall_count += 1
            hit = (stall[0], wrap(stall[1] + math.pi - ryaw), now)   # 떼어 낼 쪽은 가라 한 쪽의 반대 (기체 기준)
        self.start_unstick(x, y, z, yaw, hit[0], hit[1], self.push_hint() if hit[1] is None else None)
        return True

    def push_hint(self) -> float | None:
        """닿은 쪽을 모를 때 쓸 떼어 낼 방향 (기체 기준): 바로 앞에 가라 했는데 안 간 쪽의 반대 (autonomy/contact.py PushWatch.hint)."""
        d = self.push.hint(self.now())
        return None if d is None else wrap(d + math.pi - (self.raw_pose or self.pose)[3])

    def start_unstick(self, x: float, y: float, z: float, yaw: float, why: str, away: float | None,
                      hint: float | None = None) -> None:
        """닿았다: 닿은 쪽에 장애물을 세우고, 반대쪽으로 떼어 낸 뒤(UNSTICK) 입력 0으로 머물렀다가(HOLD) 다시 계획한다.
        away는 떼어 낼 방향(기체 기준 rad, 0 앞, + 왼쪽). 모르면 같은 곳에서 방금 닿은 쪽, 그것도 없으면 hint(바로 앞에 가라 했는데
        안 간 쪽의 반대, push_hint)를 쓰고, 다 없으면 떼어 내지 않고 머물기만 한다.
        CONTACT_EPISODE 안에 같은 곳에서 이어진 닿음은 한 사건이다: 더 멀리 밀고, CONTACT_PUSHES번 떼어 내도 또 닿으면 내린다.
        서로 다른 사건이 같은 곳에서 CONTACT_LAND_N번이어도 내린다."""
        now = self.now()
        self.send()
        self.watch.reset()
        self.push.reset()
        if self.door is not None:   # 문 지나기 중이었으면 그 문은 한 번 그만둔 것으로 센다
            self.door_fail(self.door["d"].center, add=True)
            self.door = None
        ep = self.contact_episode
        if ep is not None and now - ep["t"] < CONTACT_EPISODE and math.hypot(ep["x"] - x, ep["y"] - y) < CONTACT_LAND_R:
            ep["t"], ep["tries"] = now, ep["tries"] + 1
        else:
            ep = self.contact_episode = {"t": now, "x": x, "y": y, "tries": 1}
            self.contact_count += 1
            self.contacts = [c for c in self.contacts
                             if now - c[0] < CONTACT_LAND_WINDOW and math.hypot(c[1] - x, c[2] - y) < CONTACT_LAND_R] + [(now, x, y)]
        self.slow_turn_until = now + CONTACT_HOLD + SLOW_TURN_AFTER
        reused = False
        if away is None and self.contact_last is not None:
            # 닿은 쪽을 모르면(돌라는데 안 돎, 기울기·가속이 작음) 같은 곳에서 방금 닿은 물건이다: 그때 닿은 지도 방향을 쓴다. 장애물은
            # 움직이지 않는다 (Gazebo 11/13: 스친 기둥에서 첫 닿음 뒤 돌려다 세 번 '닿은 쪽 모름'으로 머물기만 하다 내렸다)
            t_l, x_l, y_l, toward_l = self.contact_last
            if now - t_l < CONTACT_LAND_WINDOW and math.hypot(x_l - x, y_l - y) < CONTACT_LAND_R:
                away, reused = wrap(toward_l + math.pi - yaw), True
        hinted = False
        if away is None and hint is not None:
            # 그래도 모르면 바로 앞에 가라 했는데 안 간 쪽이다 (11/14 Gazebo 반력 0 벽 판: 벽에 닿은 채 돌면서 나아가려다 '돌라는데
            # 안 돎'이 쪽을 모른 채 거듭 걸려 머물기만 하다 그 자리에 내렸다)
            away, hinted = hint, True
        side = ("모름" if away is None else f"기체 {side_name(away + math.pi)}" + (" (같은 곳에서 전에 닿은 쪽)" if reused else "")
                + (" (바로 앞에 가라 했는데 안 간 쪽)" if hinted else ""))
        if away is not None:
            toward = wrap(yaw + away + math.pi)
            self.contact_last = (now, x, y, toward)
            if not reused:   # 이미 세운 장애물이다
                self.mark_contact(x, y, z, toward, f"{why}, 닿은 쪽 {side}")
        if len(self.contacts) >= CONTACT_LAND_N:
            self.get_logger().error(f"{why}, 닿은 쪽 {side}. {CONTACT_LAND_WINDOW:.0f}초 안에 같은 곳({CONTACT_LAND_R:.0f} m 안)에서 "
                                    f"{len(self.contacts)}번 닿아 더 움직이지 않고 그 자리에 내립니다.")
            self.start_landing(x, y)
            return
        if ep["tries"] > CONTACT_PUSHES:
            self.get_logger().error(f"{why}, 닿은 쪽 {side}. {CONTACT_PUSHES}번 떼어 내도 또 닿아 더 움직이지 않고 그 자리에 내립니다.")
            self.start_landing(x, y)
            return
        self.contact_armed = now + CONTACT_SETTLE
        again = "" if ep["tries"] == 1 else f" (이어서 {ep['tries']}번째)"
        if away is None:
            self.get_logger().warn(f"{why}{again}: 무엇에 닿은 것으로 보고 입력을 끊고 {CONTACT_HOLD:.1f}초 머뭅니다 "
                                   "(닿은 쪽을 몰라 떼어 내지는 않음).")
            self.contact_until = now + CONTACT_HOLD
            self.set_state("HOLD")
            return
        direction = wrap(yaw + away)
        want = CONTACT_PUSH_DIST + 0.15 * (ep["tries"] - 1)
        dist = self.push_room(x, y, z, direction, want)
        tight = "" if dist >= want else f" — 그쪽 지도가 가까이 막혀 {dist:.2f} m만"
        self.get_logger().warn(f"{why}, 닿은 쪽 {side}{again}. 반대쪽({side_name(away)})으로 {want:.2f} m 떨어질 때까지(최대 "
                               f"{CONTACT_PUSH_MAX:.0f}초) 밀어 떼어 낸 뒤{tight} 입력을 끊고 {CONTACT_HOLD:.1f}초 머뭅니다 "
                               "(입력 0만 보내면 Tello가 닿은 쪽으로 계속 민다).")
        self.unstick = {"t": now, "dir": direction, "z": z, "x": x, "y": y, "dist": dist}
        self.set_state("UNSTICK")

    def peek_check(self, x: float, y: float, z: float, yaw: float, cmd) -> bool:
        """경로를 따라 나아가려는 참에 (PEEK_* 설명): 지금 기수로 찍은 장면이 아직 없으면 나아가지 않고 기다리고, 양옆이 가깝게
        읽히는데 계획 지도가 모르는 쪽이 있으면 그쪽을 정면 ToF로 겨눠 보기 시작한다. True면 이번 주기는 끝 (명령을 보냈다)."""
        now = self.now()
        sc = self.scene
        if sc is None or now - sc[0] > PEEK_SCENE_DEAD:
            return False   # 깊이 추정이 안 온다: 정면 ToF만으로 간다
        # 장면이 지금 기수와 어긋난 만큼 전진을 줄인다: PEEK_SCENE_YAW까지는 그대로, PEEK_SCENE_YAW_MAX부터는 0 (딱 끊으면 가다 서다 한다.
        # 영상 지연 0.25초면 24°/s까지는 그대로, 48°/s로 돌 때는 서서 돈다)
        lag = abs(wrap(sc[1] - yaw))
        k = (PEEK_SCENE_YAW_MAX - lag) / (PEEK_SCENE_YAW_MAX - PEEK_SCENE_YAW)
        if now - sc[0] > PEEK_SCENE_AGE or k <= 0.0:
            # 돌고 있거나 막 돌았다: 새로 보이는 쪽을 카메라가 본 다음에 나아간다 (회전·높이는 그대로)
            self.progress = None   # 일부러 선 것이다 ('제자리에 걸림'으로 세지 않게)
            self.send(0.0, 0.0, cmd.vz, cmd.yaw_rate)
            return True
        cmd.vx *= min(1.0, k)
        centers = np.asarray(sc[2], dtype=float)
        raw = np.asarray(sc[3], dtype=float) / max(self.scene_scale, 1e-3)
        for side in side_near(centers, raw, TOF_HALF, PEEK_RAW, PEEK_RUN):
            world = wrap(sc[1] + side)
            if self.peek_known(x, y, z, world, now):
                continue
            near = float(np.min(raw[centers * side > TOF_HALF * abs(side)]))   # 그쪽 옆 열(ToF 원뿔 밖) 가운데 가장 가까운 값
            name = "왼쪽" if side > 0 else "오른쪽"
            self.send()
            self.get_logger().info(f"{name} 앞이 카메라에 가깝게 보이는데(깊이 {near:.1f} m로 읽힘) 지도에는 그쪽 {PEEK_REACH} m 안에 "
                                   "아무것도 없습니다. 나아가기 전에 그쪽으로 기수를 돌려 정면 ToF로 재 봅니다.")
            self.peek = {"yaw": world, "t": now, "at": None, "z": z, "side": name}
            return True
        return False

    def peek_known(self, x: float, y: float, z: float, world: float, now: float) -> bool:
        """그쪽(지도 방향 world)은 볼 것 없다: 방금 이 근처에서 겨눠 봤거나, 계획 지도에 PEEK_REACH 안으로 막힌 칸(벽·가구·찾은
        장애물·사람)이 이미 있다."""
        r, a, keep = PEEK_MEMO
        self.peek_memo = [m for m in self.peek_memo if now - m[0] < keep]
        if any(math.hypot(m[1] - x, m[2] - y) < r and abs(wrap(m[3] - world)) < a for m in self.peek_memo):
            return True
        for da in (-0.09, 0.0, 0.09):   # 가운데와 ±5°
            c, s = math.cos(world + da), math.sin(world + da)
            for k in range(3, int(round(PEEK_REACH / 0.05)) + 1):
                if self.believed.occupied_at((x + c * 0.05 * k, y + s * 0.05 * k, z)):
                    return True
        return False

    def peek_step(self, x: float, y: float, z: float, yaw: float) -> None:
        """옆 겨눠 보기: 그쪽으로 천천히 돌아 PEEK_DWELL 동안 정면 ToF를 본다. 무엇이 있으면 그 폭을 단단한 칸으로 넣고 다시
        계획하고(가까우면 그다음 회전 전에 물러나기 규칙이 걸린다), 비어 있으면 가던 길을 간다."""
        P, now = self.peek, self.now()
        vz = max(-self.params.vz_max, min(self.params.vz_max, self.params.k_pos * (P["z"] - z)))
        err = wrap(P["yaw"] - yaw)
        if P["at"] is None and abs(err) > math.radians(4) and now - P["t"] < PEEK_TIMEOUT:
            rate = math.copysign(max(0.2, min(SLOW_TURN_RATE, 1.5 * abs(err))), err)   # 천천히 (rc 무시 구간 위)
            self.send(0.0, 0.0, vz, rate)
            return
        if P["at"] is None:
            P["at"] = now
        if now - P["at"] < PEEK_DWELL:
            self.send(0.0, 0.0, vz, 0.0)
            return
        front = self.front
        self.peek_memo.append((now, x, y, P["yaw"]))
        self.peek = None
        self.progress = None
        self.peek_count += 1
        if math.isfinite(front) and front < TOF_MAX - 0.05:
            self.peek_found += 1
            origin = self.sensor_origin(self.pose)
            self.cmap.mark_hard(front_points(origin, yaw, front, -TOF_HALF, TOF_HALF, z - 0.35, z + 0.35, self.cmap.p.res),
                                HARD_WEIGHT)
            self.update_camera_map(x, y, z)
            self.get_logger().warn(f"{P['side']} 앞 {front:.2f} m에 무엇이 있습니다 (카메라 지도에 없던 것). 지도에 넣고 다시 계획합니다.")
            self.slow_turn_until = now + SLOW_TURN_AFTER
            self.send()
            self.set_state("REPLAN")
            return
        self.send(0.0, 0.0, vz, 0.0)
        self.get_logger().info(f"{P['side']} 앞 {TOF_MAX} m 안은 비어 있습니다 (먼 면을 가깝게 읽은 것). 가던 길을 갑니다.")

    def push_room(self, x: float, y: float, z: float, direction: float, want: float) -> float:
        """떼어 낼 쪽(지도 방향)으로 밀 거리: 계획 지도에서 그쪽 want + 0.2 m(기체 가장자리 0.1 + 여유 0.1) 안이 막혔으면 그만큼
        줄인다. 옆은 보는 센서가 없어 지도만 믿는다. 바로 막혀 있어도 0.1 m는 민다 (닿은 채로 머물면 다시 닿는다)."""
        c, s = math.cos(direction), math.sin(direction)
        for k in range(2, int(round((want + 0.2) / 0.05)) + 1):   # 0.10 m부터 0.05 m씩
            d = 0.05 * k
            if self.believed.occupied_at((x + c * d, y + s * d, z)):
                return max(0.1, d - 0.2)
        return want

    def mark_contact(self, x: float, y: float, z: float, toward: float, why: str) -> None:
        """닿은 쪽(지도 방향 toward)에 장애물을 세운다. 카메라 지도가 닿은 자리에 아무것도 모르면 카메라가 못 보는 면(유리·흰 벽)에
        닿은 것이라 유리 규칙처럼 닿은 방향과 직각으로 양옆 GLASS_HALF를 단단한 칸으로 막는다: 0.3 m만 막았더니 Gazebo의 안 보이는
        폭 4.5 m 벽에서 막은 곳 사이 틈을 문으로 보고 같은 벽에 여섯 번 닿았다(11/13). 카메라가 아는 것(문틀·가구)에 닿았으면 그 자리만
        0.3 m 막는다 — 넓게 막으면 옆의 문까지 닫는다.

        넓게 막을 때도 카메라 지도에 찍지 않고 0.3 m 상자 여럿을 찾은 장애물로 줄지어 넣는다: 카메라 지도를 다시 만들 때마다 드론 둘레
        ±0.2 m를 비우므로(드론이 있는 자리) 0.12 m 앞에 찍은 칸은 바로 지워지고, 한 상자로 넣으면 비스듬한 면에서 축에 맞춘 상자가
        1 m 넘게 부푼다."""
        c, s = math.cos(toward), math.sin(toward)
        tip = (x + CONTACT_EDGE * c, y + CONTACT_EDGE * s)
        if self.mapping and not self.camera_knows(tip):
            reach = CONTACT_EDGE + CONTACT_BOX / 2   # 상자 가운데: 앞면이 닿은 자리에 오게
            n = 0
            for lat in np.arange(-GLASS_HALF, GLASS_HALF + 1e-6, CONTACT_BOX):
                bx, by = x + c * reach - s * lat, y + s * reach + c * lat
                n += self.memory.add({"x": round(bx, 3), "y": round(by, 3), "z": round(self.ceiling / 2, 3),
                                      "sx": CONTACT_BOX, "sy": CONTACT_BOX, "sz": round(self.ceiling, 3)}, "contact")
            if n:
                self.rebuild_maps()
                self.map_dirty = True
                self.save_discovered()
            self.get_logger().info(f"지도에 없는 장애물: 닿음 ({why}) — 닿은 곳 ({tip[0]:.2f}, {tip[1]:.2f})을 카메라 지도가 몰라 "
                                   f"(유리·흰 벽처럼 못 보는 면) 닿은 방향과 직각으로 양옆 {GLASS_HALF} m를 막습니다 (상자 {n}개).")
            return
        box = tof_box((x, y), toward, CONTACT_EDGE, math.radians(15), self.ceiling, depth=CONTACT_BOX)
        self.add_obstacle(box, "contact", f"닿음 ({why})")

    def unstick_step(self, x: float, y: float, z: float, yaw: float) -> None:
        """UNSTICK: 닿은 반대쪽(지도 방향으로 고정)으로, 추정 위치가 그쪽으로 u["dist"] 옮겨질 때까지(CONTACT_PUSH_MAX까지) 민다.
        기수는 그대로, 높이는 지킨다. 끝나면 입력 0으로 머문다: 그때 Tello가 지키는 자리가 떨어진 자리로 바뀐다."""
        u, now = self.unstick, self.now()
        moved = (x - u["x"]) * math.cos(u["dir"]) + (y - u["y"]) * math.sin(u["dir"])
        if moved >= u["dist"] or now - u["t"] >= CONTACT_PUSH_MAX:
            self.send()
            self.contact_pushes += 1
            short = "" if moved >= u["dist"] else f" — 목표 {u['dist']:.2f} m에 못 미쳤습니다"
            self.get_logger().info(f"닿은 곳에서 {max(moved, 0.0):.2f} m 떼어 냈습니다 ({now - u['t']:.1f}초){short}.")
            if self.contact_episode is not None:
                self.contact_episode["t"] = now   # 사건 시간은 떼어 낸 뒤부터 잰다
            self.contact_until, self.contact_armed = now + CONTACT_HOLD, now + CONTACT_SETTLE
            self.set_state("HOLD")
            return
        a = wrap(u["dir"] - yaw)
        vz = max(-self.params.vz_max, min(self.params.vz_max, self.params.k_pos * (u["z"] - z)))
        self.send(CONTACT_PUSH_SPEED * math.cos(a), CONTACT_PUSH_SPEED * math.sin(a), vz, 0.0)

    def start_exit(self, x: float, y: float, yaw: float, why: str) -> bool:
        """들어온 자취(최근 TRAIL_WINDOW 점)를 거꾸로 되짚기 시작한다. 근처에 자취가 없거나 EXIT_MIN 넘게 뒤가 없으면 False."""
        # 보정 전(raw) 좌표로 되짚는다: 자취도 raw로 남겼고, 상대 기하만 필요하므로 위치 맞추기 보정이 바뀌어도 흔들리지 않는다
        rx, ry, _, ryaw = self.raw_pose
        start = max(len(self.trail) - TRAIL_WINDOW, 0)
        pts = [tuple(p[:3]) for p in self.trail[start:]]
        cursor = trail_cursor(pts, (rx, ry), EXIT_LOOKBACK)
        fold = 0
        if cursor is not None:   # 접힌 점(U턴) 너머는 되짚지 않는다
            fold = trail_fold(pts, cursor, TRAIL_FOLD)
            pts, cursor = pts[fold:cursor + 1], cursor - fold
        if cursor is None or retreat_direction([p[:2] for p in pts], (rx, ry), EXIT_MIN) is None:
            self.get_logger().warn(f"{why} 되짚을 자취가 근처에 없어 그대로 다시 계획합니다.")
            return False
        first = trail_target(pts, cursor, (rx, ry), EXIT_LOOKBACK)[0]
        aim = math.hypot(first[0] - rx, first[1] - ry)
        if aim > 1e-6 and off_tail(((first[0] - rx) / aim, (first[1] - ry) / aim), ryaw) > TRAIL_AHEAD:
            self.get_logger().warn(f"{why} 되짚을 자취가 기체 앞쪽이라 후진하지 않고 그대로 다시 계획합니다.")
            return False
        self.send()
        self.exit = {"pts": pts, "base": start + fold, "cursor": cursor, "x": rx, "y": ry, "t": self.now(), "yaw": ryaw, "last": None,
                     "brake": self.moving_fast(), "check_t": -math.inf, "sideways": False}
        self.get_logger().warn(f"{why} 제자리에서 돌지 않고 들어온 자취를 따라 후진합니다 (돌 자리가 생길 때까지, 최대 {EXIT_MAX} m, "
                               f"{RETREAT_SPEED} m/s, 같은 자리 {self.exit_tries}/{EXIT_TRIES}).")
        self.set_state("EXIT")
        return True

    def tof_near_path(self, x: float, y: float, yaw: float) -> bool:
        """정면 ToF가 닿은 점이 남은 경로에서 지도 여유 + 한 칸 안인가. 멀리 비켜 있으면 지도를 다시 만들 일이 없다
        (16:12 비행: 앞 0.8 m 안에서 0.3 s마다 통째로 다시 만들어 틱의 27%를 썼다)."""
        origin = self.sensor_origin(self.pose)
        hit = (origin[0] + math.cos(yaw) * self.front, origin[1] + math.sin(yaw) * self.front)
        pts = [(x, y)] + [(float(w[0]), float(w[1])) for w in self.waypoints]
        return min(_seg_dist(hit, a, b) for a, b in zip(pts, pts[1:])) <= self.map_radius + self.cmap.p.res

    def clear_ahead(self, x: float, y: float, yaw: float):
        """앞이 확실히 비어 있는 거리(m): 카메라 장면 정면 ±15°의 가까운 거리, 정면 ToF, 경로가 기수 방향으로 곧게 이어지는
        길이 중 가장 짧은 것. 카메라 장면이 없거나 오래됐으면 None(빠르게 가지 않는다)."""
        if self.scene is None or self.now() - self.scene[0] > SHAPE_FRESH:
            return None
        _, scene_yaw, centers, ranges = self.scene
        ahead = ranges[np.abs(rotate_scene(centers, scene_yaw, yaw)) <= FAST_FOV]
        if not len(ahead):
            return None
        clear = float(ahead.min())
        if math.isfinite(self.front):
            clear = min(clear, self.front)
        return min(clear, straight_ahead(x, y, yaw, [(w[0], w[1]) for w in self.waypoints]))

    def scene_shape(self, yaw: float, dist: float):
        """최근 깊이 장면으로 정면 dist m에 있는 것의 모양(기둥/벽, autonomy/shape.py). 장면이 없거나 오래됐으면 None."""
        if self.scene is None or self.now() - self.scene[0] > SHAPE_FRESH:
            return None
        _, scene_yaw, centers, ranges = self.scene
        return front_shape(rotate_scene(centers, scene_yaw, yaw), ranges, dist)

    def soft_stop(self, x: float, y: float, z: float, yaw: float) -> bool:
        """정면 ToF가 TOF_SLOW_STOP 안에 머물고 그 자리가 남은 경로 위면 멈추고 다시 계획한다 (True면 이번 주기는 끝)."""
        if not (math.isfinite(self.front) and self.front < TOF_SLOW_STOP):
            self.soft_since = None
            return False
        now = self.now()
        if self.soft_since is None:
            self.soft_since = now
            return False
        if now - self.soft_since < TOF_SOFT_HOLD:
            return False
        origin = self.sensor_origin(self.pose)
        hit = (origin[0] + math.cos(yaw) * self.front, origin[1] + math.sin(yaw) * self.front)
        pts = [(x, y)] + [(float(w[0]), float(w[1])) for w in self.waypoints]
        if min(_seg_dist(hit, a, b) for a, b in zip(pts, pts[1:])) > TOF_SOFT_PATH:
            return False   # 경로 옆을 스치는 것: 경로는 그 옆으로 간다
        self.soft_since = None
        self.send()
        self.tof_pose = self.pose
        if self.mapping:   # ToF 원뿔 폭만큼 단단한 칸으로 찍는다 (카메라가 못 보는 것이면 카메라 광선이 지워 다시 들이받았다)
            self.cmap.mark_hard(front_points(origin, yaw, self.front, -TOF_HALF, TOF_HALF, z - 0.35, z + 0.35,
                                             self.cmap.p.res), HARD_WEIGHT)
            self.update_camera_map(x, y, z)
        else:
            self.check_tof()
            self.add_tof_obstacle()
        self.get_logger().warn(f"앞 {self.front:.2f} m가 길을 막습니다. 비상 정지 전에 멈추고 다시 계획합니다.")
        self.set_state("REPLAN")
        return True

    def tof_hit_point(self):
        if not math.isfinite(self.front):
            return None
        origin, yaw = self.sensor_origin(self.tof_pose), self.tof_pose[3]
        return (origin[0] + math.cos(yaw) * self.front, origin[1] + math.sin(yaw) * self.front)

    def camera_knows(self, p) -> bool:
        """p 둘레 0.2 m(비행 높이 ±0.35 m)에 카메라가 찍은 막힘·의심 칸이 있나. ToF가 찍은 점 0.12 m 안의 칸은 카메라 것이 아니다.
        실습실 Gazebo에서 카메라에 잘 보이는 가구끼리 ToF 점을 이어 가짜 벽을 만들었다: 잇기는 카메라가 못 보는 것에만 쓴다."""
        if p is None:
            return False
        c = self.cmap
        res = c.p.res
        r = int(math.ceil(0.2 / res))
        i0, j0 = int((p[0] - c.origin[0]) / res), int((p[1] - c.origin[1]) / res)
        k0, k1 = c.slice_index(self.height - 0.35), c.slice_index(self.height + 0.35) + 1
        sl = (slice(max(i0 - r, 0), min(i0 + r + 1, c.shape[0])), slice(max(j0 - r, 0), min(j0 + r + 1, c.shape[1])))
        top = c.logodds[sl[0], sl[1], k0:k1].max(axis=2)
        for di, dj in zip(*np.nonzero(top > c.p.suspect)):
            cx = c.origin[0] + (sl[0].start + di + 0.5) * res
            cy = c.origin[1] + (sl[1].start + dj + 0.5) * res
            if math.hypot(cx - p[0], cy - p[1]) <= 0.2 and not any(math.hypot(cx - q[0], cy - q[1]) < 0.12 for q, _, _ in self.tof_hits):
                return True
        return False

    def note_tof_shape(self, blind: bool = True) -> None:
        """4단계: 정면 ToF에 무엇이 있으면 기둥인지 벽인지 나눠 지도에 그 폭대로 찍는다.

        카메라 지도는 1 m 안 단안 깊이로 막힘을 찍지 않아(hit_min_range) 벽에 다가가면 ToF 한 점만 남았고, 경로가 그 점 옆으로
        비집다 다시 ToF에 걸려 재계획하기를 되풀이했다(10/30 복도). 깊이 장면에서 정면 덩어리 폭을 재서 벽이면 그 폭을 다 막고
        (방향을 바꾸게), 기둥이면 잰 폭만 막는다(옆으로 지나가게). 카메라가 못 보는 흰 벽·유리는 ToF 점이 옆으로 이어질 때
        벽으로 잇는다. ToF가 찍은 칸은 '단단한' 칸이라 카메라가 벽 너머를 본다고 해도 천천히만 지워진다.
        찍는 모양은 장면 열 점들에 맞춘 직선(shape_points, ToF 원뿔이 비스듬한 면의 가까운 쪽을 재는 것을 보정)이다: 기수에 수직인
        직선으로 찍으면 복도에서 벽을 비스듬히 볼 때 직선 끝이 복도 안으로 들어와 1.4 m 복도가 0.5 m로 막혔다 (11/03 Gazebo corridor_L).
        """
        front, now = self.front, self.now()
        if not math.isfinite(front) or front >= TOF_MAX - 0.05:
            return
        origin, yaw = self.sensor_origin(self.tof_pose), self.tof_pose[3]
        hit = (origin[0] + math.cos(yaw) * front, origin[1] + math.sin(yaw) * front)
        self.tof_hits = [(p, t, h) for p, t, h in self.tof_hits if now - t < TOF_HIT_KEEP]
        z0, z1, res = self.tof_pose[2] - 0.35, self.tof_pose[2] + 0.35, self.cmap.p.res   # ToF는 지금 높이를 본다
        shape = self.scene_shape(yaw, front)
        pts, what = None, None
        if shape is not None and shape.kind in ("wall", "pillar"):
            _, scene_yaw, centers, ranges = self.scene
            pts = shape_points(origin, yaw, rotate_scene(centers, scene_yaw, yaw), ranges, shape, front, z0, z1, res,
                               extra=0.3 if shape.kind == "wall" else 0.0, cone=TOF_HALF)   # 벽은 화면에 잘린 끝을 0.3 m 더
            what = f"앞 {front:.2f} m는 {shape.text()}: " + ("그 폭을 막고 방향을 바꿉니다" if shape.kind == "wall" else "옆으로 지나갑니다")
        else:   # 카메라가 못 보는 것(흰 벽·유리·얇은 기둥)이거나 모양이 애매: ToF 점이 옆으로 이어지면 벽으로 잇는다
            # 카메라가 못 보는 것만, 비슷한 기수로 본 점끼리, 사이가 빈 곳으로 보이지 않을 때만 잇는다 (실습실 Gazebo에서 가구끼리 이었다)
            near = [p for p, t, h in self.tof_hits if blind and abs(wrap(h - yaw)) < TOF_JOIN_YAW
                    and TOF_JOIN[0] <= math.hypot(p[0] - hit[0], p[1] - hit[1]) <= TOF_JOIN[1] and not self.free_between(p, hit)]
            if near:
                far = max(near, key=lambda p: math.hypot(p[0] - hit[0], p[1] - hit[1]))
                pts = segment_points(far, hit, z0, z1, res)
                what = (f"ToF 점이 옆으로 이어집니다(폭 {math.hypot(far[0] - hit[0], far[1] - hit[1]) + 0.6:.1f} m): "
                        "벽으로 보고 막습니다")
        self.tof_hits.append((hit, now, yaw))
        if pts is not None:
            self.cmap.mark_hard(pts, HARD_WEIGHT)
        key = None if not what else ("벽" if "벽" in what[:12] else "기둥")   # 로그는 종류마다 3초에 한 번
        if what and (key != self.shape_logged[0] or now - self.shape_logged[1] > 3.0):
            self.shape_logged = (key, now)
            self.get_logger().info(what)

    def free_between(self, a, b) -> bool:
        """두 점 사이(비행 높이 ±0.35 m)에 카메라가 본 빈 칸이 TOF_JOIN_FREE 비율 넘게 있으면 True (잇지 않는다)."""
        zc = self.tof_pose[2] if self.tof_pose is not None else self.height
        pts = segment_points(a, b, zc - 0.35, zc + 0.35, self.cmap.p.res, extra=-0.15)
        if not len(pts):
            return False
        idx = self.cmap._index(pts)
        idx = idx[np.all((idx >= 0) & (idx < np.array(self.cmap.shape)), axis=1)]
        if not len(idx):
            return False
        i, j, k = idx[:, 0], idx[:, 1], idx[:, 2]   # known_free()로 100만 칸을 만들지 않고 그 칸만 본다 (ToF 콜백마다 최대 50번)
        s, nz = self.cmap.seen, self.cmap.shape[2]
        seen = s[i, j, k] | s[i, j, np.maximum(k - 1, 0)] | s[i, j, np.minimum(k + 1, nz - 1)]
        free = seen & (self.cmap.logodds[i, j, k] <= self.cmap.p.occupied)
        return float(free.mean()) > TOF_JOIN_FREE

    def check_tof(self) -> bool:
        origin = self.sensor_origin(self.tof_pose)
        self.expected = expected_range(self.believed, origin, self.rays.directions(self.tof_pose[3]), TOF_MAX)
        # 위치가 불확실할수록 지도와 비교한 차이를 너그럽게 본다 (위치 오차로 벽을 장애물로 착각하지 않게)
        margin = TOF_MARGIN + min(1.5 * self.pose_sigma, 0.3)
        if not is_unexpected(self.front, self.expected, TOF_MAX, margin):
            return False
        # 닿은 자리 근처에 아는 벽·가구·문틀이 있으면, 위치가 조금 틀어진 채 그것을 본 것으로 본다
        radius = 0.2 + min(self.pose_sigma, 0.25)
        return not explained_by_map(self.known, origin, self.tof_pose[3], self.front, self.rays.h_angles,
                                    TOF_HALF, radius)

    def add_tof_obstacle(self) -> None:
        """1–3단계: ToF가 지도에 없는 것을 봤다. 깊이 장면이 있으면 기둥·벽을 나눠 잰 폭으로, 없으면 원뿔 폭의 기둥으로 넣는다.
        옆으로 이어진 ToF 장애물은 하나의 벽으로 잇는다: 한 점씩 넣으면 경로가 그 사이로 비집다 또 막힌다 (10/30 오후)."""
        origin, yaw = self.sensor_origin(self.tof_pose), self.tof_pose[3]
        box = tof_box(origin, yaw, self.front, TOF_HALF, self.ceiling)
        why = f"ToF {self.front:.2f} m (지도대로면 {self.expected:.2f} m)"
        shape = self.scene_shape(yaw, self.front)
        if shape is not None and shape.kind in ("wall", "pillar"):
            half = shape.width / 2 + (0.3 if shape.kind == "wall" else 0.0)
            box = tof_box(origin, yaw, self.front, math.atan2(half, self.front), self.ceiling)
            why += f", {shape.text()}" + (" → 그 폭을 막아 방향을 바꿈" if shape.kind == "wall" else " → 옆으로 지나감")
        near = [i for i in self.memory.items if "tof" in i.sources
                and TOF_JOIN[0] <= math.hypot(i.box["x"] - box["x"], i.box["y"] - box["y"]) <= TOF_JOIN[1]]
        if near:
            bs = [box] + [i.box for i in near]
            x0, x1 = min(b["x"] - b["sx"] / 2 for b in bs), max(b["x"] + b["sx"] / 2 for b in bs)
            y0, y1 = min(b["y"] - b["sy"] / 2 for b in bs), max(b["y"] + b["sy"] / 2 for b in bs)
            box = {"x": round((x0 + x1) / 2, 3), "y": round((y0 + y1) / 2, 3), "z": box["z"],
                   "sx": round(x1 - x0, 3), "sy": round(y1 - y0, 3), "sz": box["sz"]}
            self.memory.drop_where(lambda i: i in near)
            why += f" → 옆의 ToF 장애물 {len(near)}개와 벽으로 이음(폭 {max(box['sx'], box['sy']):.1f} m)"
        self.add_obstacle(box, "tof", why)

    def on_detections(self, msg: String) -> None:
        self.detect_at = self.now()
        if self.state not in AIRBORNE:
            return
        data = json.loads(msg.data)
        pose = self.pose_at(data["stamp"])
        if pose is None:
            return
        cam = self.sensor_origin(pose)
        now = self.now()
        w, h = int(data.get("width", IMG_W)), int(data.get("height", IMG_H))   # 실기체는 폭 480으로 줄여 낸다
        self.log_people(data["detections"], cam, pose[3], now, w, h)
        if self.search and self.avoid:
            self.check_people_gone(data["detections"], cam, pose[3], w)
        if not self.avoid:
            return
        self.candidates = [c for c in self.candidates if now - c[2] < 2.0]
        for d in data["detections"]:
            if d["name"] not in AVOID_CLASSES:
                continue
            bbox = (d["x1"], d["y1"], d["x2"], d["y2"])
            box = ground_box(bbox, cam, pose[3], w, h, self.hfov, self.ceiling, name=d["name"])
            close = None
            if box is None and d["name"] == "person" and d["conf"] >= 0.7:
                close = near_person_box(bbox, cam, pose[3], w, h, self.hfov, self.ceiling)
                if close is not None:   # 발이 잘릴 만큼 가까운 사람: ToF 원뿔 밖 옆에 있어도 피해야 한다
                    box = close[0]
            if box is None:
                continue   # 너무 멀거나 아래가 잘린 것
            if d["name"] == "person":   # 머리가 보이면 기둥 높이를 잰 키로 (낮은 사람은 위로 넘을 수 있게)
                foot = ground_point(bbox, cam, pose[3], w, h, self.hfov)
                dist = close[1] if close is not None else foot[2] if foot is not None else None
                top = person_top(bbox, cam[2], dist, w, h, self.hfov) if dist is not None else None
                if top is not None and top + PERSON_TOP_PAD < box["sz"]:
                    box = {**box, "z": round((top + PERSON_TOP_PAD) / 2, 3), "sz": round(top + PERSON_TOP_PAD, 3)}
            if d["name"] != "person" and is_known(self.known, box):
                continue   # 원래 지도에 있는 가구 (사람은 지도에 없으니, 벽·가구 옆에 서 있어도 거르지 않는다)
            if close is None and self.memory.suspected(box["x"], box["y"]):
                continue   # 카메라로만 봤다가 길을 막아 지운 자리 (가까이 가면 ToF·가까운 사람 인식이 확인한다)
            seen = self.memory.covering(box["x"], box["y"])
            if seen is not None:
                seen.sources.add(f"yolo:{d['name']}")
                continue   # 이미 찾은 장애물
            if close is not None:
                if in_sight(self.known, cam, (box["x"], box["y"])):   # 다시 볼 틈이 없으니 한 번에 넣는다
                    self.add_obstacle(box, "yolo:person", f"YOLO 가까운 사람 {d['conf']:.2f}, 약 {close[1]:.1f} m 앞")
                continue
            gx, gy, dist = ground_point(bbox, cam, pose[3], w, h, self.hfov)
            if not in_sight(self.known, cam, (gx, gy)):
                continue   # 벽 너머로 나온 추정은 틀린 것
            source = f"yolo:{d['name']}"
            match = next((c for c in self.candidates
                          if c[1] == source and math.hypot(c[0]["x"] - box["x"], c[0]["y"] - box["y"]) < 0.5), None)
            if match is None:
                self.candidates.append((box, source, now))
                continue
            self.candidates.remove(match)
            self.add_obstacle(box, source, f"YOLO {d['name']} {d['conf']:.2f}, 약 {dist:.1f} m 앞")

    def log_people(self, detections, cam, yaw: float, now: float, w: int = IMG_W, h: int = IMG_H) -> None:
        """YOLO가 잡은 사람의 위치를 기록한다. 1 m 안이면 같은 사람.

        분할 마스크 안의 깊이(depth_m, 탐지 노드가 /tt/depth에서 잼)가 있으면 그 거리로 잰다: 발이 안 보여도 되고
        바닥 가정이 필요 없다. 없으면 발 위치(ground_point)로.
        """
        for d in detections:
            if d["name"] != "person":
                continue
            g = None
            if d.get("depth_m") is not None:
                dist = float(d["depth_m"]) * self.depth_scale   # 깊이 영상 배율은 비행 노드가 바닥으로 맞춘 값
                if 0.3 <= dist <= 5.0:
                    f = (w / 2) / math.tan(self.hfov / 2)
                    left = dist * (w / 2 - (d["x1"] + d["x2"]) / 2) / f
                    g = (cam[0] + math.cos(yaw) * dist - math.sin(yaw) * left,
                         cam[1] + math.sin(yaw) * dist + math.cos(yaw) * left, dist)
                    if not in_sight(self.known, cam, g[:2]):
                        continue
            if g is None:
                # 발이 잘린 가까운 사람(near_person_box)은 피하는 데만 쓴다. 머리·폭으로 어림한 위치는 0.5 m쯤
                # 틀려 같은 사람이 따로 기록됐다 (3단계 seed 3). 찾은 사람 목록은 발 위치로 잰 것만
                g = ground_point((d["x1"], d["y1"], d["x2"], d["y2"]), cam, yaw, w, h, self.hfov)
                if g is not None and self.known.occupied_at((g[0], g[1], 0.3)):
                    continue   # 지도의 가구 속: 발이 가구에 가려 상자 아래 변을 발로 잘못 본 것 (위치가 멀게 나온다)
                if g is None or g[2] > 5.0 or not in_sight(self.known, cam, g[:2]):
                    continue   # 발이 안 보이거나 너무 멀거나 벽 너머로 나온 추정
            px, py = g[0] + 0.15 * math.cos(yaw), g[1] + 0.15 * math.sin(yaw)
            top = person_top((d["x1"], d["y1"], d["x2"], d["y2"]), cam[2], g[2], w, h, self.hfov) if g[2] <= TOP_MAX_DIST else None
            person = self.people.add(px, py, float(d["conf"]), now, top=top)
            if person is not None:
                self.get_logger().info(f"사람 발견 #{len(self.people.people)}: ({px:.2f}, {py:.2f}), "
                                       f"신뢰도 {d['conf']:.2f}, 약 {g[2]:.1f} m 앞"
                                       + (f", 키 약 {top:.2f} m" if top is not None else ""))
                self.save_people()
        key = self.blocking_key()
        if self.avoid and key != self.people_key:   # 길을 막는 사람이 늘었거나 자리가 옮겨졌으면 지도에 반영
            self.people_key = key
            self.rebuild_maps()
            self.map_dirty = True

    def blocking_key(self) -> tuple:
        return tuple((round(p.x, 1), round(p.y, 1)) for p in self.people.blocking())

    def check_people_gone(self, detections, cam, yaw: float, w: int = IMG_W) -> None:
        """찾은 사람 자리를 카메라가 똑바로 보는데 그쪽에 사람 상자가 없고 깊이로도 비어 있으면 세고, 이어지면 그 사람은
        길을 막지 않게 한다.

        YOLO가 못 잡은 것만으로는 지우지 않는다: Gazebo에서 가만히 선 사람을 정면 3.6 m에서 보면서 YOLO가 놓쳐 지운 적이 있다
        (기록한 자리는 0.6 m 앞이었다). 그래서 카메라 깊이 지도에서 그 방향 '기록한 거리 −0.6 ~ +1.0 m'에 막힌 칸이 없고,
        기록한 자리 둘레를 본 적이 있을 때만 센다. 깊이 지도가 없는 3단계(가구 지도)에서는 지우지 않는다.
        """
        if not self.mapping:
            return
        f = (w / 2) / math.tan(self.hfov / 2)
        bearings = [math.atan2(w / 2 - (d["x1"] + d["x2"]) / 2, f) for d in detections if d["name"] == "person"]
        gone = []
        for p in self.people.blocking():
            view = person_in_view(p.x, p.y, cam, yaw, self.hfov)
            if view is None:
                continue
            dist, bearing = view
            if person_seen_toward(bearing, dist, bearings):
                p.misses = 0
                continue
            # 사람 몸은 원래 비행 높이 단면에서 본다 (위로 넘느라 올라가 있으면 그 단면에는 낮은 사람이 없다)
            occluded, _ = self.cmap.sight_line(cam, (p.x, p.y), 0.3, dist - 0.6, self.base_height)
            if occluded:
                continue   # 앞에 가리는 것이 있어 그 자리가 안 보인다
            blocked, _ = self.cmap.sight_line(cam, (p.x, p.y), dist - 0.6, dist + 1.0, self.base_height)
            if blocked:   # 그 방향에 무언가 있다 (그 사람이거나 가리는 물건)
                p.misses = 0
                continue
            _, seen = self.cmap.sight_line(cam, (p.x, p.y), dist - 0.3, dist + 0.3, self.base_height)
            if seen < 0.5:
                continue   # 그 자리를 아직 깊이로 못 봤다
            if self.people.note_empty(p, PERSON_GONE_MISSES, self.now(), PERSON_GONE_TIME):
                gone.append(p)
        for p in gone:
            self.get_logger().info(f"찾은 사람 자리 ({p.x:.2f}, {p.y:.2f})를 다시 봤는데 아무도 없습니다. "
                                   "지나간 사람으로 보고 길을 막지 않습니다.")
        if gone:
            self.people_key = self.blocking_key()
            self.rebuild_maps()
            self.map_dirty = True
            self.save_people()

    # ------------------------------------------------------------ 지도 고치기

    def add_obstacle(self, box: dict, source: str, why: str) -> None:
        if not self.memory.add(box, source):
            return   # 이미 아는 장애물
        self.rebuild_maps()
        self.map_dirty = True
        self.get_logger().info(f"지도에 없는 장애물: {why} → 중심 ({box['x']:.2f}, {box['y']:.2f}), "
                               f"{box['sx']:.2f}×{box['sy']:.2f} m")
        self.save_discovered()

    def rebuild_maps(self) -> None:
        boxes = self.memory.boxes()
        # 찾은 사람(발 위치로 여러 번 잰 확정 목록)은 지우지 않는 장애물로 넣는다. 카메라 장애물은 길이 막히면
        # 빼는데, 그때 사람까지 빠져 돌아오는 길에 사람 옆을 스친 일이 있었다 (3단계 seed 1)
        # 몸 높이는 잰 키(+여유), 못 쟀으면 PERSON_TALL. 위험이 낮아 사람 둘레를 뺐으면(people_soft) 몸 둘레 하한만 남긴다 (11/10)
        people = [{"x": p.x, "y": p.y, "z": top / 2, "sx": PERSON_BOX, "sy": PERSON_BOX, "sz": top}
                  for p in self.blocking_people()
                  for top in [PERSON_TALL if p.top is None else min(PERSON_TALL, p.top + PERSON_TOP_PAD)]
                  ] if self.search and self.avoid else []
        self.believed = with_boxes(self.known, boxes + people)
        self.inflated = with_boxes(self.known_inflated, [inflate_box(b, self.found_radius) for b in boxes])
        if self.mapping:   # 지나온 길은 찾은 장애물(ToF·닿음·밑)로도 막지 않는다: 유리 사이에서 들어온 길이 막혀 '길이 없음'이 됐다 (11/02 17:33)
            self.inflated.occ &= ~self.cmap.flown
        if people:
            margin = PERSON_MARGIN_SOFT if self.people_soft else self.person_margin
            self.inflated = with_boxes(self.inflated, [inflate_box(b, margin) for b in people])

    def blocking_people(self) -> list:
        """지도에 넣는 찾은 사람 (몇 번만 잡힌 사람을 뺐으면 그만큼 뺀다)."""
        return [p for p in self.people.blocking() if not (self.drop_weak_people and p.hits < PERSON_WEAK_HITS)]

    def save_discovered(self) -> None:
        if not self.discovered_out:
            return
        items = [{**i.box, "sources": sorted(i.sources), "hits": i.hits} for i in self.memory.items]
        Path(self.discovered_out).write_text(json.dumps({"obstacles": items, "replans": self.replans},
                                                        ensure_ascii=False, indent=1), encoding="utf-8")

    # ------------------------------------------------------------ 상태 기계

    def set_state(self, state: str) -> None:
        if state != self.state:
            # 내리거나 중단하는 이유: 바로 앞에 남긴 로그 (land_at_goal처럼 미리 정해 두었으면 그대로). 중단은 늘 새로 잡는다
            if state == "ABORT" or (state == "LAND" and self.why_end is None):
                self.why_end = self.end_reason()
            self.state = state
            self.peek = None   # 옆 겨눠 보기는 경로를 따라가는 동안의 일이다
            self.state_pub.publish(String(data=state))
            self.publish_note()
            self.get_logger().info(f"상태: {state}")
            if state in ("LAND", "DONE", "ABORT") and (self.goto_active is not None or self.goto_pending is not None):
                self.goto_pending, self.goto_drop = None, False
                self.goto_end("내리게 되어 찍은 자리로는 가지 않습니다.")
            if state in ("DONE", "ABORT"):
                self.save_discovered()

    def send(self, vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0) -> None:
        if self.state in FRESH_STATES:
            vx, vy, vz, yaw_rate = self.fresh_guard(vx, vy, vz, yaw_rate)
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z = vx, vy, vz, yaw_rate
        self.cmd_pub.publish(msg)
        self.last_cmd = (vx, vy, vz, yaw_rate)

    def input_ages(self) -> tuple[float, float | None]:
        """(위치 나이, 전방 ToF 나이 — 받은 적 없으면 None) (s)."""
        now = self.now()
        return now - self.pose_t, (now - self.front_t if math.isfinite(self.front_t) else None)

    def fresh_guard(self, vx: float, vy: float, vz: float, yaw_rate: float):
        """오래된 위치면 멈추고, 오래된 전방 ToF면 앞으로 가지 않는다 (freshness.guard). 더 길어지면 step()이 freshness.lost로 내린다."""
        pose_age, tof_age = self.input_ages()
        cmd, why = freshness.guard((vx, vy, vz, yaw_rate), pose_age, tof_age)
        if why and self.now() - self.stale_logged >= 2.0:
            self.stale_logged = self.now()
            if "위치" in why and self.plan_s >= 0.3 and self.now() - self.plan_done_t < pose_age:
                # 위치가 끊긴 것이 아니라 길 계획이 오래 걸려 그동안 들어온 위치를 아직 못 읽은 것 (11/06 14:35: 0.8–1.4초)
                why = f"길 계획에 {self.plan_s:.1f}초 걸려 위치 처리가 {pose_age:.1f}초 밀렸습니다 (위치가 끊긴 것은 아님). 제자리에 멈춥니다."
            self.get_logger().warn(why)
        return cmd

    def step(self) -> None:
        if self.pose is None:
            return
        x, y, z, yaw = self.pose
        if self.log:
            t = self.now()
            c = self.correction
            self.log.writerow([f"{t:.3f}", f"{x:.3f}", f"{y:.3f}", f"{z:.3f}", f"{yaw:.3f}", self.state,
                               f"{self.pose_sigma:.3f}" if self.pose_sigma_known else "", f"{c.tx:.3f}", f"{c.ty:.3f}", f"{c.yaw:.4f}",
                               f"{self.front:.3f}" if math.isfinite(self.front) else "",
                               f"{self.last_cmd[0]:.3f}", f"{self.last_cmd[1]:.3f}", f"{self.last_cmd[3]:.3f}"])

        self.ticks += 1
        self.pose_steps += 1
        self.front_steps += 1
        if self.state in AIRBORNE and self.flip_check():
            return
        if self.state in AIRBORNE and self.avoid and self.contact_check(x, y, z, yaw):
            return
        if self.state in AIRBORNE and self.avoid and self.under_check(x, y, z):
            return
        if self.mapping:
            if self.state in ("PLAN",) + AIRBORNE and self.now() - self.map_built_at >= MAP_REBUILD:
                self.update_camera_map(x, y, z)
        elif self.search and self.state in AIRBORNE and abs(z - self.height) < 0.3 and self.ticks % 4 == 0:
            self.coverage.observe(x, y, yaw)   # 카메라로 본 곳 기록 (초당 5번)

        # 위치 다시 잡기 중에는 일부러 후보를 흩어 σ가 커지므로 보지 않는다
        checking = self.state in ("PLAN",) + AIRBORNE and self.state != "RELOC"
        if checking and self.pose_sigma_known and self.pose_sigma > self.max_pose_sigma and not self.lost:
            self.lost = True
            self.get_logger().error(f"위치가 불확실합니다 (σ {self.pose_sigma:.2f} m > {self.max_pose_sigma} m). "
                                    "그 자리에 착륙합니다.")
            self.start_landing(x, y)
            return
        pose_age, tof_age = self.input_ages()
        lost = freshness.lost(pose_age, self.pose_steps, tof_age, self.front_steps) if checking and not self.lost else None
        if lost:
            self.lost = True
            self.get_logger().error(f"{lost}. 그 자리에 착륙합니다.")
            self.start_landing(x, y)
            return

        if self.forced_landing and self.state not in ("DONE", "ABORT"):
            self.get_logger().error(f"드라이버가 기체를 내렸습니다 ({describe_landing(self.forced_landing)}). 중단합니다.")
            self.forced_landing = None
            self.set_state("ABORT")
            return

        if self.land_requested:
            self.land_requested = False
            if self.state in ("WAIT_POSE", "READY"):
                self.get_logger().info("아직 이륙하지 않았으므로 끝냅니다.")
                self.set_state("ABORT")
                return
            if self.state == "TAKEOFF" and self.driver and not self.flying:
                self.get_logger().info("이륙 중에 착륙 요청: 이륙을 마치면 바로 내립니다.")
                self.set_state("ABORT")   # ABORT는 떠 있으면 착륙을 요청한다
                return
            if self.state not in ("LAND", "DONE", "ABORT"):
                self.get_logger().info("요청에 따라 그 자리에 착륙합니다.")
                self.start_landing(x, y)
                return

        if self.state in AIRBORNE and self.return_check(x, y):
            return

        if self.state == "WAIT_POSE":
            self.hold = (x, y)
            self.set_state("READY" if self.wait_start else "TAKEOFF")

        elif self.state == "READY":
            self.send()
            self.hold = (x, y)
            if self.start_requested:
                self.start_requested = False
                if not (self.driver and self.start_blocked()):
                    self.takeoff_seen = self.takeoff_result().get("sent_at")
                    self.set_state("TAKEOFF")

        elif self.state == "TAKEOFF":
            if self.driver and not self.flying:
                # Tello takeoff는 약 0.8 m까지 스스로 오른 뒤 응답한다. 그다음 비행 높이까지는 아래처럼 속도로 오른다
                self.send()
                if self.wait_start and self.takeoff_refused():
                    return
                tries = self.flight_req[2] if self.flight_req and self.flight_req[0] == "takeoff" else 0
                if tries >= TAKEOFF_TRIES and self.now() - self.flight_req[1] > TAKEOFF_RETRY:
                    failed = (self.flight_info.get("last_result") or {}).get("failed")
                    self.get_logger().error(f"이륙 요청 {tries}번에도 뜨지 않았습니다 (점검 실패: {failed}). 중단합니다.")
                    self.set_state("ABORT")
                else:
                    self.request_flight("takeoff", TAKEOFF_RETRY)
                return
            # 경로는 지금 높이에서 시작하므로 비행 높이에 거의(TAKEOFF_TOL) 닿은 뒤 계획한다.
            # follow()는 0.15 m 안이면 도착으로 보고 멈추므로 이륙 높이는 따로 맞춘다.
            dz = self.height - z
            vz = self.params.k_pos * dz
            if abs(dz) >= TAKEOFF_TOL:
                vz = math.copysign(max(abs(vz), CLIMB_MIN), dz)
            self.send(0.0, 0.0, max(-self.params.vz_max, min(self.params.vz_max, vz)), 0.0)
            if abs(dz) < TAKEOFF_TOL:
                self.send()
                if self.mapping:   # 아무것도 모르므로 먼저 제자리에서 둘러보며 지도를 채운다
                    self.update_camera_map(x, y, z)
                    self.look_target, self.looks = None, 0
                    self.set_state("LOOK")
                else:
                    self.set_state("PLAN")

        elif self.state == "PLAN":
            self.send()   # 볼 자리를 여러 번 고르는 동안에도 멈춤 명령을 보낸다 (안 보내면 2초 뒤 드라이버가 '명령 끊김'으로 내린다)
            if self.search and not self.returning:
                self.choose_viewpoint(x, y)
                if self.state != "PLAN":   # 사람과 부딪힐 위험이 높아 착륙을 시작했다
                    return
            self.make_plan()

        elif self.state == "REPLAN":
            self.send()
            if self.avoid and self.mapping and self.try_exit(x, y, z, yaw):   # 막힌 주머니에서 멈춘 뒤라면 계획보다 먼저 후진한다
                return
            if self.goto_take():   # 운용자가 찍은 자리가 기다린다: 볼 자리를 그리로 바꾼다
                return
            self.replans += 1
            limit = MAX_REPLANS_MAPPING if self.mapping else MAX_REPLANS
            if self.replans > limit:
                self.get_logger().error(f"재계획이 {limit}번을 넘었습니다. 중단합니다.")
                self.set_state("ABORT")
                return
            if self.vp_budget is not None and self.search and not self.returning:
                self.vp_budget.note_replan(self.now())
                why = self.vp_budget.give_up(self.now())
                if why:
                    self.give_up_viewpoint(why)
                    return
            self.make_plan()

        elif self.state == "FOLLOW":
            if self.goto_take():   # 운용자가 찍은 자리가 기다린다: 가던 길을 멈추고 볼 자리를 그리로 바꾼다
                return
            if self.map_dirty:
                self.map_dirty = False
                here = np.array([x, y, z])
                points = ([here] if not (self.mapping and self.inflated.occupied_at(here)) else []) + self.waypoints
                if not path_clear(self.inflated, points):
                    self.send()
                    self.get_logger().warn("남은 경로가 새 장애물에 막혔습니다. 다시 계획합니다.")
                    self.set_state("REPLAN")
                    return
            if self.peek is not None:   # 옆을 정면 ToF로 겨눠 보는 중
                self.peek_step(x, y, z, yaw)
                return
            final = len(self.waypoints) == 1
            cmd, reached = follow(self.pose, self.waypoints[0], self.params, final)
            if self.vp_budget is not None and self.search and not self.returning:
                # 볼 자리 한도: 앞·옆으로 움직이는 동안만 센다 (제자리 회전은 빼고)
                now = self.now()
                self.vp_budget.remaining(self.remaining_path(x, y, z), now)
                self.vp_budget.tick(now, moving=abs(cmd.vx) > 0.02 or abs(cmd.vy) > 0.02)
                why = self.vp_budget.give_up(now)
                if why:
                    self.give_up_viewpoint(why)
                    return
            if cmd.vx == 0.0 and cmd.vy == 0.0 and abs(cmd.yaw_rate) > 0.05:
                if self.avoid and self.mapping and self.reverse_out(x, y, z, yaw):
                    return
                if self.avoid and self.near_turn(x, y, yaw, "FOLLOW"):
                    return
                arc = self.can_arc(x, y, z, yaw)
                if not arc and self.brake_before_turn(cmd.vz):   # 제자리에서 돌 때는 앞으로 미끄러지던 관성부터 끊는다
                    return
                self.turned_at = self.now()   # 제자리에서 도는 중
                if arc:
                    cmd.vx = self.arc_speed   # 돌면서 기수 쪽으로 조금씩 나아간다 (앞이 비었을 때만, 의도한 전진)
            if self.avoid:   # 가까이 무엇이 있으면(또는 닿은 직후) 천천히 돈다
                cmd.yaw_rate = limit_turn(cmd.yaw_rate, self.front, SLOW_TURN_RANGE, SLOW_TURN_RATE,
                                          force=self.now() < self.slow_turn_until)
            elif self.search and self.detecting() and not reached and self.now() - self.turned_at < TURN_LOOK:
                # 모퉁이에서 돌자마자 출발하면, 시야 밖에 있다가 막 들어온 사람을 영상 지연(약 0.25 s) 때문에
                # 잡기 전에 옆을 스친다 (3단계 seed 1). 잠깐 멈춰 YOLO가 새로 보이는 쪽을 보게 한다
                self.send(vz=cmd.vz)
                return
            if self.stuck(cmd):
                self.send()
                self.get_logger().warn(f"움직이라고 했는데 {STUCK_TIME:.1f}초째 제자리입니다. 바로 앞을 막힌 곳으로 봅니다.")
                if not self.avoid:
                    self.set_state("ABORT")
                    return
                box = tof_box(self.sensor_origin(self.pose), yaw, 0.05, TOF_HALF, self.ceiling, depth=0.3)   # 앞면만 안다: 깊이 0.3
                self.add_obstacle(box, "contact", "제자리에 걸림")
                self.slow_turn_until = self.now() + SLOW_TURN_AFTER
                self.start_backoff(x, y, yaw)
                return
            if cmd.vx > 0 and self.front < self.stop_distance:
                self.send()
                if not self.avoid:
                    self.get_logger().warn(f"전방 {self.front:.2f} m에 장애물. 멈추고 중단합니다.")
                    self.set_state("ABORT")
                    return
                self.get_logger().warn(f"전방 {self.front:.2f} m에 장애물. 비상 정지 후 조금 물러납니다.")
                self.stops += 1
                self.tof_pose = self.pose
                # 지도의 벽으로 설명되더라도 넣는다. 위치가 틀어져 벽을 더 멀리 믿고 있으면, 안 넣을 때
                # 같은 경로로 다시 가서 멈추기만 되풀이한다 (3단계 seed 2에서 30번). 길을 막으면 계획 실패 때 뺀다
                self.check_tof()
                self.add_tof_obstacle()
                self.start_backoff(x, y, yaw)
                return
            if reached:
                self.waypoints.pop(0)
                self.stops = 0
                if not self.waypoints:
                    if self.door is not None and self.door["phase"] == "approach":   # 문 앞 자리에 닿았다
                        self.send()
                        self.start_door()
                        return
                    if self.search and not self.returning:
                        self.send()
                        self.look_target, self.looks = None, 0
                        self.set_state("LOOK")
                        return
                    if self.mapping and self.localize_on and self.spin is None:
                        self.send()
                        self.spin = {"last": yaw, "turned": 0.0}
                        self.localizer.home_mode = True   # 이륙 직후 지도에 넓게(±0.6 m) 대어 본다
                        self.get_logger().info("출발점 위에서 한 바퀴 돌며 카메라 지도로 위치를 맞춘 뒤 내립니다.")
                        self.set_state("HOMESPIN")
                        return
                    self.land_at_goal(x, y)   # 목적지 바로 위로 맞춰 내린다 (목적지가 막혀 경로가 앞에서 끝났으면 여기에)
                    return
            # 나아가기 전에: 지금 기수로 본 장면이 있는지, 양옆이 가깝게 보이는데 지도가 모르는 쪽이 있는지 (옆 겨눠 보기)
            if cmd.vx > 0 and self.avoid and self.mapping and self.peek_check(x, y, z, yaw, cmd):
                return
            # 앞이 확실히 비어 있으면(카메라 정면·ToF·곧은 경로가 다 멀 때) speed 대신 fast까지 올린다 (10/30 저녁 요청)
            if self.fast > self.params.speed and cmd.vx > 0 and abs(cmd.vy) < 0.05 and abs(cmd.yaw_rate) < 0.2:
                clear = self.clear_ahead(x, y, yaw)
                k = fast_factor(clear)
                if k > 0:
                    cmd.vx = boosted(cmd.vx, self.params.speed, self.fast, k)
                    if self.now() - self.fast_logged > 5.0:
                        self.fast_logged = self.now()
                        self.get_logger().info(f"앞 {clear:.1f} m가 비어 {cmd.vx:.2f} m/s로 갑니다")
            if cmd.vx > PERSON_SLOW_SPEED and self.search and self.avoid:
                # 찾은 사람 옆은 천천히: 정면 ToF는 옆 사람을 못 보고, 사람 둘레를 뺀 뒤 위치 오차만큼 붙을 수 있다 (Gazebo 사람 둘레 11/10)
                near_p = [math.hypot(p.x - x, p.y - y) for p in self.blocking_people()]
                near_p = [d for d in near_p if d < PERSON_SLOW_R]
                if near_p:
                    cmd.vx = PERSON_SLOW_SPEED
                    if self.now() - self.person_slow_logged > 5.0:
                        self.person_slow_logged = self.now()
                        self.get_logger().info(f"찾은 사람이 {min(near_p):.1f} m 옆에 있어 {PERSON_SLOW_SPEED} m/s로 갑니다.")
            # 정면 ToF가 무엇을 보면 거리에 맞춰 줄인다: 0.35 m 비상 정지만 믿으면 실기체는 멈추기 전에 닿았다
            # (2026-10-29: rc 42로 0.4 m/s, ToF 0.62 → 0.22 m를 0.9초에 지나 모니터에 닿음. ToF는 0.1초마다 새 값)
            if cmd.vx > 0 and self.avoid and self.soft_stop(x, y, z, yaw):
                return
            if cmd.vx > 0 and math.isfinite(self.front) and self.front < TOF_MAX:
                cmd.vx = min(cmd.vx, max(TOF_SLOW_MIN, TOF_SLOW_GAIN * (self.front - max(self.stop_distance + 0.1, TOF_SLOW_STOP))))
                if self.mapping and self.front < TOF_RECHECK and self.now() - self.map_built_at >= 0.3 and self.tof_near_path(x, y, yaw):
                    self.update_camera_map(x, y, z)   # ToF가 넣은 막힘이 남은 경로 근처일 때만 바로 다시 만들어 본다
            self.send(cmd.vx, cmd.vy, cmd.vz, cmd.yaw_rate)

        elif self.state == "DOOR":
            self.door_step(x, y, z, yaw)

        elif self.state == "LOOK":
            # 탐색 자리에 도착: 안 본 곳이 많은 방향으로 기수를 돌려 둘러본다 (볼 게 없을 때까지, 최대 6번)
            vz = max(-self.params.vz_max, min(self.params.vz_max, self.params.k_pos * (self.height - z)))
            if self.goto_active is not None and not self.goto_active["arrived"]:
                self.goto_active["arrived"] = True
                self.goto_note("찍은 자리에 도착해 둘러봅니다.")
            if self.look_target is None:
                if self.mapping and self.look_at is not None:   # 막힌 경계(또는 운용자가 찍은 곳) 쪽부터 오래 본다
                    look_yaw, enough = math.atan2(self.look_at[1] - y, self.look_at[0] - x), True
                    self.look_at, self.look_dwell, self.approach_pending = None, APPROACH_DWELL, None
                elif self.mapping:
                    look_yaw, gain = self.cmap.best_look(x, y, self.height, self.hfov, now_yaw=yaw)
                    # 안 본 쪽이 많이 남았을 때만, 그리고 지금 보는 쪽이 아닐 때만 돈다 (같은 곳 다시 찍지 않기, 10/30 저녁)
                    fresh = self.cmap.explored_area(self.base_height) < LOOK_FRESH_AREA
                    enough = gain >= LOOK_MIN_UNKNOWN and (fresh or abs(wrap(look_yaw - yaw)) >= LOOK_MIN_TURN)
                    self.look_dwell = LOOK_DWELL
                else:
                    look_yaw, gain = self.coverage.best_look(x, y)
                    enough = gain >= self.coverage.p.min_gain_cells
                if not enough or self.looks >= (LOOKS_MAX_MAPPING if self.mapping else
                                                LOOKS_MAX if self.detecting() else LOOKS_MAX_BLIND):
                    self.send(vz=vz)
                    if self.goal is not None:
                        self.visited.append((float(self.goal[0]), float(self.goal[1])))   # 다녀온 자리는 다시 고르지 않는다
                    # 실제로 둘러본 자리도 (이륙 직후 출발점 포함). 11/09 13:05: 출발점에서 둘러본 것이 남지 않아, 문 지나기를
                    # 그만둔 뒤 0.4 m 옆의 출발점을 '새로 보일 곳 8.4 m²' 볼 자리로 다시 골라 또 둘러봤다
                    self.visited.append((float(x), float(y)))
                    if self.goto_active is not None:
                        self.goto_count[1] += 1
                        self.goto_end(f"찍은 자리를 둘러봤습니다. {self.goto_after()}.")
                    self.set_state("PLAN")
                    return
                self.look_target, self.looks, self.look_reached = look_yaw, self.looks + 1, None
            err = wrap(self.look_target - yaw)
            if abs(err) >= 0.1 and self.avoid and self.near_turn(x, y, yaw, "LOOK"):
                return
            if abs(err) < 0.1:
                # 4단계는 그 방향 깊이 영상이 지도에 들어올 때까지 잠깐 본다 (영상 지연 + 추정 시간)
                if self.look_reached is None:
                    self.look_reached = self.now()
                if not self.mapping or self.now() - self.look_reached >= self.look_dwell:
                    self.look_target = None
                self.send(vz=vz)
                return
            if self.brake_before_turn(vz):   # 오던 관성으로 앞으로 밀리며 돌지 않게 (문을 지난 직후 둘러볼 때 0.15 m/s)
                return
            rate = max(-self.params.yaw_rate_max, min(self.params.yaw_rate_max, 1.5 * err))
            if self.avoid:
                rate = limit_turn(rate, self.front, SLOW_TURN_RANGE, SLOW_TURN_RATE, force=self.now() < self.slow_turn_until)
            self.send(0.0, 0.0, vz, rate)

        elif self.state == "BACKOFF":
            # 들어온 길로 물러난다. 회전 명령은 0이므로 기수가 저절로 돌거나 크게 기울면 무엇에 닿은 것: 바로 멈춘다
            b, now = self.backoff, self.now()
            touched = contact(wrap(yaw - b["yaw"]), self.tilt, CONTACT_YAW, CONTACT_TILT)
            if touched:   # 떼어 낼 방향은 그때 기울기·가속으로 (모르면 입력 0으로 머물기만)
                self.start_unstick(x, y, z, yaw, f"물러나는 중 {touched}", self.watch.away(), self.push_hint())
                return
            if math.hypot(x - b["x"], y - b["y"]) >= RETREAT_DIST or now - b["t"] > RETREAT_TIME:
                self.send()
                if self.stops >= RELOC_STOPS and self.start_relocalize(f"같은 곳에서 비상 정지를 {self.stops}번 했습니다."):
                    return
                self.trim_trail()
                if self.avoid and self.try_exit(x, y, z, yaw):   # 막힌 주머니면 돌거나 계획하기 전에 들어온 자취로 후진한다
                    return
                self.set_state(b["then"])
                return
            speed = BRAKE_SPEED if b["brake"] and now - b["t"] < BRAKE_TIME else RETREAT_SPEED
            # 기체 뒤쪽으로 곧게만 (11/05 13:04: 옆 0.05 m/s를 섞어 들어온 쪽으로 비스듬히 물러나다 왼쪽 나무 문짝에 닿아 23° 돌았다.
            # 0.3 m 물러나는 동안 옆으로 몇 cm 가는 것도 옆 센서가 없어 위험하다). 돌 자리가 없을 때 자취 방향으로 가는 것은 그대로
            vx, vy = self.retreat_cmd(b["dir"], yaw, speed, b["room"], lateral=0.0)
            vz = max(-self.params.vz_max, min(self.params.vz_max, self.params.k_pos * (b["z"] - z)))   # 지금 높이 유지
            self.send(vx, vy, vz, 0.0)

        elif self.state == "UNSTICK":
            self.unstick_step(x, y, z, yaw)

        elif self.state == "HOLD":
            self.send()   # 입력 0: Tello가 제자리·자세를 스스로 잡는다
            if self.now() >= self.contact_until:
                self.trim_trail()
                self.set_state("REPLAN")

        elif self.state == "EXIT":
            # 들어온 자취를 따라 기체 뒤쪽으로 곧게 후진한다: 기수는 그대로(회전 0), 자취에서 EXIT_LOOKBACK 뒤의 점을 겨눠 선 위로
            # 돌아오며 간다(옆 보정은 LATERAL_MAX까지). 회전 명령이 0이므로 기수가 저절로 돌거나 기울면 닿은 것 (BACKOFF와 같다)
            e, now = self.exit, self.now()
            rx, ry, rz, ryaw = self.raw_pose   # 자취와 같은 보정 전 좌표
            touched = contact(wrap(ryaw - e["yaw"]), self.tilt, CONTACT_YAW, CONTACT_TILT)
            if touched:
                self.start_unstick(x, y, z, yaw, f"후진하는 중 {touched}", self.watch.away(), self.push_hint())
                return
            prog = e.get("prog")   # 밀어도 STUCK_TIME 넘게 제자리면 옆이 막힌 것 (Gazebo 안 보이는 상자: 35초 동안 0.1 m)
            if prog is None or math.hypot(rx - prog[0], ry - prog[1]) > 0.05:
                e["prog"] = (rx, ry, now)
            elif now - prog[2] > STUCK_TIME:
                self.send()
                self.get_logger().warn(f"후진하는데 {STUCK_TIME:.1f}초째 제자리입니다. 무엇에 막힌 것으로 보고 입력을 끊고 "
                                       f"{CONTACT_HOLD:.1f}초 머뭅니다.")
                self.contact_until = now + CONTACT_HOLD
                self.slow_turn_until = now + CONTACT_HOLD + SLOW_TURN_AFTER
                self.set_state("HOLD")
                return
            target, e["cursor"], done = trail_target(e["pts"], e["cursor"], (rx, ry), EXIT_LOOKBACK)
            moved = math.hypot(rx - e["x"], ry - e["y"])
            room = e.get("room", 0.0)
            if now - e["check_t"] >= 0.2:   # 돌 자리가 생겼는지 0.2초마다 본다 (지도 광선 수백 개)
                e["check_t"] = now
                blocked, room, left, right = self.pocket(x, y, z, yaw)
                e["room"], e["free"] = room, (not blocked) and room >= TURN_ROOM and max(left, right) >= POCKET_RANGE
            if done or (moved >= EXIT_MIN and e.get("free")) or moved >= EXIT_MAX or now - e["t"] > EXIT_TIME:
                self.send()
                del self.trail[e["base"] + e["cursor"] + 1:]   # 후진한 만큼 자취를 지운다 (다음에 또 안쪽으로 가지 않게)
                self.trim_trail()
                how = ("자취 끝까지" if done else "돌 자리가 생겨" if e.get("free")
                       else f"{EXIT_MAX} m" if moved >= EXIT_MAX else f"{EXIT_TIME:.0f}초")
                self.get_logger().info(f"들어온 자취를 따라 {moved:.2f} m 후진했습니다 ({how}, 둘레 {room:.2f} m). 돌아서 다시 계획합니다.")
                self.set_state("REPLAN")
                return
            dx, dy = target[0] - rx, target[1] - ry
            n = math.hypot(dx, dy)
            dx, dy = (dx / n, dy / n) if n > 1e-6 else (-math.cos(ryaw), -math.sin(ryaw))
            if off_tail((dx, dy), ryaw) > TRAIL_AHEAD:   # 겨눌 자취가 기수 쪽으로 넘어갔다: 후진이 아니므로 끝낸다 (앞뒤로 흔들리지 않게)
                self.send()
                del self.trail[e["base"] + e["cursor"] + 1:]
                self.trim_trail()
                self.get_logger().warn(f"되짚을 자취가 기체 앞쪽으로 넘어가 후진을 끝냅니다 ({moved:.2f} m 후진). 천천히 돌아 다시 계획합니다.")
                self.slow_turn_until = now + SLOW_TURN_AFTER
                self.set_state("REPLAN")
                return
            if (math.isfinite(self.front) and self.front < self.stop_distance
                    and abs(wrap(math.atan2(dy, dx) - ryaw)) < EXIT_FRONT):
                self.send()
                self.get_logger().warn(f"후진하는 쪽 앞 {self.front:.2f} m에 무엇이 있습니다. 멈추고 {CONTACT_HOLD:.1f}초 머뭅니다.")
                if self.mapping:
                    self.cmap.mark_hard(front_points(self.sensor_origin(self.pose), yaw, self.front, -TOF_HALF, TOF_HALF,
                                                     z - 0.35, z + 0.35, self.cmap.p.res), HARD_WEIGHT)
                else:
                    self.tof_pose = self.pose
                    self.check_tof()
                    self.add_tof_obstacle()
                self.contact_until = now + CONTACT_HOLD
                self.set_state("HOLD")
                return
            speed = BRAKE_SPEED if e["brake"] and now - e["t"] < BRAKE_TIME else RETREAT_SPEED   # 다가가던 관성을 먼저 끊는다
            vx, vy = self.retreat_cmd((dx, dy), ryaw, speed, room)   # 기체 뒤쪽으로 곧게, 옆은 LATERAL_MAX까지만
            if abs(vy) > LATERAL_MAX + 1e-6 and not e["sideways"]:
                e["sideways"] = True
                self.get_logger().warn("들어온 자취가 기체 옆쪽인데 돌 자리가 없어 그 자취 방향으로 갑니다 (옆은 보는 센서가 없음).")
            tz = min(max(float(target[2]), 0.3), self.max_height)
            vz = max(-self.params.vz_max, min(self.params.vz_max, self.params.k_pos * (tz - rz)))
            self.send(vx, vy, vz, 0.0)

        elif self.state == "RELOC":
            # 위치 다시 잡기: 후보를 흩은 위치 추정 노드가 ToF로 주변을 고루 재도록 제자리에서 천천히 한 바퀴
            vz = max(-self.params.vz_max, min(self.params.vz_max, self.params.k_pos * (self.height - z)))
            self.reloc["turned"] += abs(wrap(yaw - self.reloc["last"]))
            self.reloc["last"] = yaw
            if self.reloc["turned"] < 2 * math.pi:
                self.send(0.0, 0.0, vz, RELOC_RATE)
                return
            self.send(vz=vz)
            near = self.memory.drop_where(self.tof_near_known)
            self.rebuild_maps()
            self.save_discovered()
            self.stops = 0
            self.get_logger().info(f"위치를 다시 잡았습니다 (σ {self.pose_sigma:.2f} m). "
                                   f"아는 벽에 붙은 ToF 장애물 {near}개를 빼고 다시 계획합니다.")
            self.set_state("REPLAN")

        elif self.state == "HOMESPIN":
            # 4단계: 출발점 위에서 한 바퀴. 그동안 맞추기 스레드가 보정을 고치고, 다 돌면 고친 위치로 출발점에 다시 간다
            vz = max(-self.params.vz_max, min(self.params.vz_max, self.params.k_pos * (self.height - z)))
            self.spin["turned"] += abs(wrap(yaw - self.spin["last"]))
            self.spin["last"] = yaw
            if self.spin["turned"] < 2 * math.pi:
                self.send(0.0, 0.0, vz, HOME_SPIN_RATE)
                return
            self.send(vz=vz)
            self.localizer.home_mode = False
            c = self.correction
            self.get_logger().info(f"한 바퀴 돌았습니다 (출발점 맞춤 {self.localizer.home_fixes}번). 보정 "
                                   f"({c.tx:+.2f}, {c.ty:+.2f}) m, {math.degrees(c.yaw):+.1f}°로 출발점에 다시 맞춰 내립니다.")
            self.set_state("REPLAN")

        elif self.state == "LAND":
            hx, hy = self.hold
            if math.hypot(hx - x, hy - y) > LAND_SETTLE and self.now() - self.land_t < LAND_SETTLE_TIME:
                # 도착 판정(0.15 m) 뒤 남은 수평 오차를 높이를 지킨 채 줄인다. 짧은 거리라 기수는 그대로 둔다
                k = self.params.k_pos
                wx, wy = (max(-0.2, min(0.2, k * d)) for d in (hx - x, hy - y))
                vz = max(-self.params.vz_max, min(self.params.vz_max, k * (self.height - z)))
                self.send(math.cos(yaw) * wx + math.sin(yaw) * wy, -math.sin(yaw) * wx + math.cos(yaw) * wy, vz)
                return
            if self.driver:
                self.send()
                if self.flying:
                    self.request_flight("land", LAND_RETRY)   # Tello land: 제자리에서 스스로 내려 끄고 응답한다
                    return
            else:
                cmd, _ = follow(self.pose, (*self.hold, 0.0), self.params, final=True)
                self.send(cmd.vx, cmd.vy, max(cmd.vz, -0.25), 0.0)
                if z >= 0.15:
                    return
            self.send()
            err = f"{math.hypot(x - self.goal[0], y - self.goal[1]):.2f} m" if self.goal is not None else "없음"
            self.get_logger().info(f"착륙 완료. 목적지와 수평 거리 {err}(추정 위치 기준), "
                                   f"재계획 {self.replans}번, 찾은 장애물 {len(self.memory.items)}개")
            if self.search:
                self.get_logger().info(f"탐색 결과: {self.coverage_text()}, "
                                       f"찾은 사람 {len(self.people.confirmed())}명, 돌기 전 제동 {self.turn_brakes}번, "
                                       f"닿음 {self.contact_count}번(떼어 냄 {self.contact_pushes}번, "
                                       f"가라는데 안 감으로 잡음 {self.stall_count}번), "
                                       f"기수 값이 흘러 시작을 안 받음 {self.takeoff_holds}번·이륙 거절 {self.takeoff_refusals}번, "
                                       f"옆 겨눠 보기 {self.peek_count}번(무엇이 있던 것 {self.peek_found}번), "
                                       f"문 지남 {self.doors_passed}번(지난 틈으로 되나감 {self.doors_reopened}번), "
                                       f"뒤를 몰라 안 물러남 {self.rear_refused}번(들어온 길 쪽이 아니라서 {self.off_trail_refused}번), "
                                       f"운용자가 찍은 자리 {self.goto_count[0]}곳(도착해 둘러봄 {self.goto_count[1]}곳)")
                if self.mapping:
                    c, loc = self.correction, self.localizer
                    if self.tof_scale is not None:
                        self.get_logger().info(f"깊이 영상 {self.depth_frames}장, 깊이 배율은 정면 ToF로 맞춤: 표본 "
                                               f"{self.tof_scale.used}개, 마지막 배율 {self.depth_scale:.2f}. "
                                               f"지도의 가구 위에 있다고 알린 상태값 {self.under_frames}개")
                    else:
                        self.get_logger().info(f"깊이 영상 {self.depth_frames}장 중 {self.floor_frames}장을 바닥으로 "
                                               f"배율 맞춤 (마지막 배율 {self.depth_scale:.2f})")
                    if self.localize_on:
                        self.get_logger().info(f"카메라 지도로 위치 맞춤 {loc.fixes}번 (시도 {loc.tries}번, "
                                               f"옮긴 거리 합 {loc.moved:.2f} m), 최종 보정 "
                                               f"({c.tx:+.2f}, {c.ty:+.2f}) m, {math.degrees(c.yaw):+.1f}°")
                    if self.recorder:
                        self.recorder.flush()
                        self.get_logger().info(f"비행 기록: 깊이 영상 {self.recorder.count}장 → {self.recorder.dir}")
                self.save_people()
                self.save_camera_map()
            self.set_state("ABORT" if self.lost else "DONE")

        else:   # DONE, ABORT
            self.send()
            if self.driver and self.flying and not self.crashed:
                self.request_flight("land", LAND_RETRY)   # 중단하면 공중에 머물지 않고 그 자리에 내린다

    def stuck(self, cmd) -> bool:
        """움직이라는(가거나 돌라는) 명령을 내는데 STUCK_TIME 넘게 5 cm도, 6°도 못 움직였으면 True."""
        x, y, _, yaw = self.pose
        now = self.now()
        pushing = math.hypot(cmd.vx, cmd.vy) > 0.1 or abs(cmd.yaw_rate) > 0.2
        if (not pushing or self.progress is None or math.hypot(x - self.progress[0], y - self.progress[1]) > 0.05
                or abs(math.atan2(math.sin(yaw - self.progress[3]), math.cos(yaw - self.progress[3]))) > 0.1):
            self.progress = (x, y, now, yaw)
            return False
        return now - self.progress[2] > STUCK_TIME

    def detecting(self) -> bool:
        """YOLO 탐지 결과가 최근에 왔는지. 안 오면 사람을 볼 일이 없으므로 멈춰 보거나 오래 둘러보지 않는다."""
        return self.now() - self.detect_at < DETECT_FRESH

    def can_arc(self, x: float, y: float, z: float, yaw: float) -> bool:
        """제자리 회전 대신 돌면서 나아가도 되는지: 경유점이 멀고, 앞이 비었고, 바로 앞이 지도에서 막히지 않았다."""
        if self.arc_speed <= 0.0 or not self.waypoints:
            return False
        wx, wy = self.waypoints[0][0], self.waypoints[0][1]
        if math.hypot(wx - x, wy - y) < ARC_MIN_DIST or self.front < ARC_CLEAR:
            return False
        ahead = np.array([x + ARC_AHEAD * math.cos(yaw), y + ARC_AHEAD * math.sin(yaw), z])
        return not self.inflated.occupied_at(ahead)

    def choose_viewpoint(self, x: float, y: float) -> None:
        """3단계: 다음에 둘러볼 자리를 목적지로 정한다. 없으면(또는 시간이 다 되면) 출발점으로."""
        now = self.now()
        if self.search_start is None:
            self.search_start = now
        elapsed = now - self.search_start
        vp = None
        self.look_at = None
        self.restore_after_people(x, y)
        if self.mapping and self.map_radius != self.map_radius_full and self.height == self.base_height:
            self.set_map_radius(self.map_radius_full)   # 줄인 여유는 그 자리까지만 쓴다 (사람 위로 넘는 중이면 그대로)
        self.goto_drop = False
        self.approach_pending = None
        if self.goto_active is not None and self.goto_pending is None:
            # 찍은 자리로 가다 볼 자리를 다시 고르게 됐다 (보통은 그 전에 이유와 함께 끝냈다. 웹 화면에 가는 중으로 남지 않게)
            self.goto_end("찍은 자리로 가던 것을 그만두고 볼 자리를 다시 고릅니다.", warn=True)
        if self.goto_pending is not None and self.goto_choose(x, y, elapsed):
            return   # 운용자가 웹 지도에서 찍은 자리가 다음 볼 자리다
        if elapsed < self.search_time and len(self.bad_viewpoints) < 20:
            vp = self.find_viewpoint(x, y)
            if vp is None and self.mapping and self.map_radius_tight < self.map_radius_full:
                # 멀리서 본 헛 장애물이 문·통로를 막았을 수 있다. 가까이 가면 깊이 영상이 다시 비운다
                self.set_map_radius(self.map_radius_tight)
                vp = self.find_viewpoint(x, y)
                if vp is None:
                    self.set_map_radius(self.map_radius_full)
                else:
                    self.get_logger().warn(f"갈 수 있는 경계가 없어 지도 여유를 {self.map_radius_tight} m로 줄였습니다.")
            # 찾은 장애물(어림한 크기 + 여유)이 문 앞을 막아 못 가는 곳이 남았을 수 있다.
            # 경로 계획과 같은 순서로 여유를 줄여 보고, 그래도 없으면 카메라로만 본 장애물을 빼 본다
            if vp is None and self.memory.items and self.found_radius > FOUND_RADIUS_TIGHT:
                self.found_radius = FOUND_RADIUS_TIGHT
                self.rebuild_maps()
                vp = self.find_viewpoint(x, y)
                if vp is not None:
                    self.get_logger().warn(f"찾은 장애물에 막혀 더 볼 곳이 없었습니다. 여유를 {FOUND_RADIUS_TIGHT} m로 줄였습니다.")
            if vp is None:
                dropped = self.memory.drop_camera_only()
                if dropped:
                    self.rebuild_maps()
                    self.save_discovered()
                    vp = self.find_viewpoint(x, y)
                    self.get_logger().warn(f"더 볼 곳이 없어 카메라로만 본 장애물 {dropped}개를 빼고 다시 골랐습니다 "
                                           "(가까이 가면 ToF·가까운 사람 인식이 다시 잡는다).")
            if vp is None and self.people_blocked_viewpoints(x, y):
                vp = self.people_viewpoint(x, y)
                if self.state == "LAND":
                    return
            if vp is None and self.door_goals and not self.door_retried:
                # 문 너머라 포기했던 자리를 한 번 더 본다 (11/05 10:51: 문 둘레를 금지한 채 '닫힌 공간'으로 끝냈다)
                self.door_retried = True
                goals = set(self.door_goals)
                self.bad_viewpoints = [b for b in self.bad_viewpoints if not (len(b) > 2 and (b[0], b[1]) in goals)]
                self.door_fails = []
                vp = self.find_viewpoint(x, y)
                if vp is None and self.mapping and self.map_radius_tight < self.map_radius_full:
                    self.set_map_radius(self.map_radius_tight)
                    vp = self.find_viewpoint(x, y)
                    if vp is None:
                        self.set_map_radius(self.map_radius_full)
                self.get_logger().warn(f"더 볼 곳이 없어 문 너머라 포기했던 볼 자리 {len(goals)}곳의 금지를 풀고 다시 골랐습니다"
                                       + (f" → ({vp[0]:.1f}, {vp[1]:.1f})." if vp is not None else " (그래도 없음)."))
            if vp is None and self.mapping:
                ap = self.cmap.blocked_frontier_approach(x, y, self.inflated, self.height, exclude=self.approached,
                                                         shadow=FRONTIER_SHADOW, z_top=self.max_height)   # 닫힌 창 너머는 빼고
                if ap is not None:
                    ax, ay, tx, ty, size, cx, cy = ap
                    self.approached.append((cx, cy))
                    self.approach_pending = (cx, cy)   # 가는 길에 찍은 자리가 끼어들면 되돌린다 (goto_take)
                    self.look_at = (tx, ty)
                    self.goal = np.array([ax, ay, self.height])
                    self.get_logger().warn(f"갈 수 있는 경계가 없습니다. 막힌 경계({size}칸, 중심 {cx:.1f}, {cy:.1f})를 "
                                           f"({ax:.1f}, {ay:.1f})에서 {math.hypot(tx - ax, ty - ay):.1f} m 거리로 다시 봅니다.")
                    return
        if self.mapping and elapsed < self.search_time:
            self.note_space(x, y, vp)
        if vp is None:
            why = f"탐색 시간({self.search_time:.0f}초)이 다 돼" if elapsed >= self.search_time else "더 볼 곳이 없어"
            if self.mapping and elapsed < self.search_time and self.vp_reach < VP_REACH_NOTE:
                why = f"남은 {self.search_time - elapsed:.0f}초 안에 갈 수 있는 {self.vp_reach:.1f} m 안에 더 볼 곳이 없어"
            self.why_return = why   # 웹 화면에 보인다 (publish_note)
            if not self.return_home:
                far = math.hypot(x - self.start_xy[0], y - self.start_xy[1])
                self.get_logger().info(f"{why} 돌아가지 않고 여기(출발점에서 {far:.1f} m)에 내립니다 (return_home:=false). "
                                       f"{self.coverage_text()}, 찾은 사람 {len(self.people.confirmed())}명")
                self.save_people()
                self.start_landing(x, y)
                return
            self.get_logger().info(f"{why} 출발점으로 돌아갑니다. {self.coverage_text()}, "
                                   f"찾은 사람 {len(self.people.confirmed())}명")
            self.returning = True
            self.return_t0 = self.return_best_t = self.now()
            self.return_best = math.inf
            self.goal = np.array([*self.start_xy, self.height])
            self.save_people()
            self.publish_note()   # 돌아가기 시작을 바로 알린다 (단계는 PLAN 그대로라 set_state가 내지 않는다)
            return
        gx, gy, gain, dist = vp[:4]
        area = vp[4] if len(vp) > 4 else None
        self.goal = np.array([gx, gy, self.height])
        self.goal_area = area or 0.0
        self.vp_budget = None   # 경로가 나오면 make_plan이 새로 잡는다
        if self.mapping:
            heading = self.travel_heading(x, y)
            turn = f", 가던 방향에서 {math.degrees(wrap(math.atan2(gy - y, gx - x) - heading)):+.0f}°" if heading is not None else ""
            self.leg_from = (x, y)
            what = f"새로 보일 곳 약 {area:.1f} m²(경계 {gain}칸)" if area is not None else f"모르는 곳과의 경계 {gain}칸"
            self.get_logger().info(f"다음 볼 자리 ({gx:.1f}, {gy:.1f}): {what}, "
                                   f"{dist:.1f} m 거리{turn}. {self.coverage_text()}")
            return
        area = gain * self.coverage.p.cell ** 2
        self.get_logger().info(f"다음 볼 자리 ({gx:.1f}, {gy:.1f}): 새로 보일 곳 약 {area:.1f} m², "
                               f"{dist:.1f} m 거리. 지금까지 {self.coverage_text()}")

    def coverage_text(self) -> str:
        if self.mapping:
            return f"지도에 담은 빈 곳 {self.cmap.explored_area(self.base_height):.0f} m²"
        return f"본 곳 {self.coverage.coverage():.0%}"

    def update_camera_map(self, x: float, y: float, z: float) -> None:
        """4단계: 드론이 있는 자리를 비우고, 카메라 지도로 계획용 지도를 다시 만든다."""
        self.map_built_at = self.now()
        self.cmap.mark_free_sphere((x, y, z))
        self.known = self.cmap.voxel_map()
        self.known_inflated = self.cmap.planning_map(self.map_radius)
        self.rebuild_maps()
        self.map_dirty = True
        self.publish_obstacle_grid()
        if self.now() - self.map_saved_at >= 30.0:
            self.save_camera_map(background=True)

    def publish_obstacle_grid(self) -> None:
        """비행 높이 ±0.35 m 단면의 카메라 지도: 100 막힘(확정), 50 의심(경로는 피함), 0 본 빈 곳, -1 모름."""
        if self.grid_pub is None:
            return
        c = self.cmap
        k0, k1 = c.slice_index(self.height - 0.35), c.slice_index(self.height + 0.35) + 1
        lo = c.logodds[:, :, k0:k1].max(axis=2)
        grid = np.full(lo.shape, -1, dtype=np.int8)
        grid[c.seen[:, :, k0:k1].any(axis=2)] = 0
        grid[lo > c.p.suspect] = 50
        grid[lo > c.p.occupied] = 100
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.info.resolution = float(c.p.res)
        msg.info.width, msg.info.height = int(lo.shape[0]), int(lo.shape[1])
        msg.info.origin.position.x, msg.info.origin.position.y = float(c.origin[0]), float(c.origin[1])
        msg.data = grid.T.ravel().tolist()   # OccupancyGrid는 y 줄마다 x가 이어진다
        self.grid_pub.publish(msg)

    def set_map_radius(self, radius: float) -> None:
        self.map_radius = radius
        self.known_inflated = self.cmap.planning_map(radius)
        self.rebuild_maps()

    def save_camera_map(self, background: bool = False) -> None:
        """카메라 지도를 저장한다. background면 배열을 복사해 다른 스레드에서 쓴다 (압축 저장 42 ms가 제어 주기를 막지 않게)."""
        if not (self.mapping and self.map_out):
            return
        self.map_saved_at = self.now()
        if not background:
            self.cmap.save(self.map_out)
            return
        c = self.cmap
        copies = dict(logodds=c.logodds.copy(), seen=c.seen.copy(), hard=c.hard.copy(), origin=c.origin.copy(), res=c.p.res)
        threading.Thread(target=np.savez_compressed, args=(self.map_out,), kwargs=copies, daemon=True).start()

    def travel_heading(self, x: float, y: float):
        """가던 방향: 지난 볼 자리를 고른 자리에서 0.5 m 넘게 움직였으면 그 방향. 아니면 None(방향 계수 없음).

        둘러본 직후의 기수는 마지막으로 본 쪽이라 가던 방향이 아니다 (Gazebo 복도에서 출발 직후 뒤쪽 경계를 골랐다)."""
        if self.leg_from is not None and math.hypot(x - self.leg_from[0], y - self.leg_from[1]) > 0.5:
            return math.atan2(y - self.leg_from[1], x - self.leg_from[0])
        return None

    def note_space(self, x: float, y: float, vp) -> None:
        """지금 공간(SPACE_REACH 안)에 30×30 cm 넘는 통로 경계가 없으면 '닫힌 공간'으로 보고, 그 공간을 마치고 어디로 가는지 남긴다.
        고르기 자체는 frontier_viewpoint가 한다(닫힌 공간 안의 작은 구멍·창 너머는 이미 경계가 아니다)."""
        try:
            c = self.cmap.frontier_census(x, y, self.inflated, self.height, shadow=FRONTIER_SHADOW, passage=PASSAGE_MIN,
                                          passage_area=PASSAGE_AREA, z_top=self.max_height, reach=SPACE_REACH)
        except Exception as err:   # 로그용 판단이라 실패해도 비행은 계속한다
            self.get_logger().warn(f"공간 판단 오류: {err}")
            return
        if c["passages"] > 0:
            return
        if vp is not None and vp[3] <= SPACE_REACH:
            return
        key = (round(x), round(y), vp is None)
        if key == self.space_logged:
            return
        self.space_logged = key
        people = sum(1 for p in self.people.confirmed() if math.hypot(p.x - x, p.y - y) <= SPACE_REACH)
        where = (f"이 공간 탐색을 마치고 ({vp[0]:.1f}, {vp[1]:.1f}) 쪽 통로로 갑니다({vp[3]:.1f} m)" if vp is not None
                 else "다른 곳으로 갈 통로도 없습니다")
        self.get_logger().info(f"지금 공간(둘레 {SPACE_REACH:.0f} m, 본 빈 곳 {c['local_m2']:.0f} m²)은 30×30 cm 넘는 통로가 없는 닫힌 "
                               f"공간입니다(창·가구 너머 {c['beyond']}곳, 작은 구멍 {c['small']}곳, 이 안에서 찾은 사람 {people}명). {where}.")

    def remaining_path(self, x: float, y: float, z: float) -> float:
        """지금 자리에서 남은 경유점을 따라 목적지까지의 길이 (m)."""
        pts = [np.array([x, y, z])] + [np.asarray(w, dtype=float) for w in self.waypoints]
        return float(sum(np.linalg.norm(b - a) for a, b in zip(pts, pts[1:])))

    def give_up_viewpoint(self, why: str) -> None:
        """볼 자리 한도(ViewpointBudget)를 넘었다: 그 자리 둘레는 빼고 다른 곳을 고른다.
        문 너머 자리는 둘레를 작게(VP_GIVEUP_DOOR) 빼고 기억해 두었다가, 더 볼 곳이 없을 때 한 번 더 본다 (11/05)."""
        gx, gy = float(self.goal[0]), float(self.goal[1])
        b = self.vp_budget
        # 문을 못 알아봤어도, 좁은 길로만 가는데 너머에 새로 보일 곳이 넓으면 문·통로 너머로 본다
        door = b is not None and (b.door or (b.tight and self.goal_area >= DOOR_GAIN))
        radius = VP_GIVEUP_DOOR if door else VP_GIVEUP_RADIUS
        self.bad_viewpoints.append((gx, gy, radius))
        if door:
            self.door_goals.append((gx, gy))
        self.vp_budget = None
        self.door = None
        self.send()
        self.get_logger().warn(f"볼 자리 ({gx:.1f}, {gy:.1f})를 포기합니다: {why}. "
                               f"둘레 {radius} m는 다시 고르지 않고 다른 곳을 봅니다"
                               + (" (문 너머라 문은 막지 않고, 더 볼 곳이 없으면 다시 봅니다)." if door else "."))
        if self.goto_active is not None:
            self.goto_end(f"찍은 자리 ({gx:.1f}, {gy:.1f})로 가다 그만둡니다: {why}. {self.goto_after()}.", warn=True)
        self.set_state("PLAN")

    # ------------------------------------------------------------ 문 지나기 (11/05)

    def door_blocked(self) -> np.ndarray:
        """문 재기용: 비행 높이 ±0.1 m에 실제로 있는 막힌 칸(계획이 피하는 점수부터, 부풀리기 전)과 모르는 칸, 찾은 장애물.
        지나온 칸은 뺀다. 계획용 지도의 '가구 기둥 막기'는 넣지 않는다 — 탁자 사이가 문처럼 좁아 보인다(Gazebo 11/05)."""
        c = self.cmap
        k = c.slice_index(self.height)
        band = (c.logodds[:, :, max(k - 1, 0):k + 2] > c.p.suspect).any(axis=2)
        boxes = with_boxes(VoxelMap(c.origin.copy(), c.p.res, np.zeros(c.shape, dtype=bool)), self.memory.boxes())
        return (band | ~c.seen_slice(k) | boxes.occ[:, :, k]) & ~c.flown[:, :, k]

    def door_fail(self, center, add: bool = False) -> int:
        """그 문(가운데 0.5 m 안)에서 곧게 지나기를 그만둔 횟수. add면 하나 더한다."""
        for f in self.door_fails:
            if math.hypot(f[0] - center[0], f[1] - center[1]) < 0.5:
                f[2] += int(add)
                return f[2]
        if add:
            self.door_fails.append([float(center[0]), float(center[1]), 1])
            return 1
        return 0

    def door_approach(self, d, hz: float):
        """문 가운데 선 위 문 앞 자리(DOOR_PRE, 막혔으면 0.6·0.5 m, 높이 hz)까지의 경로와 그 거리. 못 가면 (None, None).
        드론보다 문에서 먼 자리(뒤로 가야 하는 자리)는 고르지 않는다."""
        x, y, z, _ = self.pose
        ahead = -d.along(x, y)
        pres = [p for p in (DOOR_PRE, 0.6, 0.5) if p <= ahead + 0.05] or [ahead]
        for pre in pres:
            e = d.point(-pre)
            goal = np.array([e[0], e[1], hz])
            if self.inflated.occupied_at(goal):
                continue
            try:
                return plan(self.inflated, (x, y, z), goal, z_max=self.max_height), pre
            except PlanError:
                continue
        return None, None

    def door_path(self, path):
        """경로가 좁은 틈(autonomy/doorway.py, 폭 약 1.2 m 안)을 지나면 문 앞 자리까지의 경로로 바꾸고 self.door를 잡는다.
        아니면 그대로. 평소 여유로 길이 났어도 본다: Gazebo 문 월드(11/05)에서 지도가 0.9 m 문을 넓게 봐 평소 여유 경로로 지나다
        문틀 모서리를 스쳤다(−0.02 m). 문 앞에서 멈춰 정면으로 다시 재면 문틀이 지도에 잡힌다."""
        self.door = None
        if not (self.mapping and self.avoid):
            return path
        x, y, z, _ = self.pose
        blocked = self.door_blocked()
        d = find_doorway(blocked, self.cmap.origin, self.cmap.p.res, path, narrow=DOOR_NARROW)
        if d is None or -d.along(x, y) < DOOR_NEAR or self.door_fail(d.center) >= DOOR_TRIES:
            return path
        if is_corridor(blocked, self.cmap.origin, self.cmap.p.res, d):
            # 넓은(1.05 m~) 틈인데 앞뒤 1 m까지 양옆 벽이 같은 간격으로 이어진다: 문이 아니라 좁은 복도의 한 자리 (11/11)
            key = (round(float(d.center[0]) * 2) / 2, round(float(d.center[1]) * 2) / 2)
            if key != self.corridor_logged:
                self.corridor_logged = key
                self.get_logger().info(f"경로의 폭 {d.width:.2f} m 틈(가운데 ({d.center[0]:.1f}, {d.center[1]:.1f}))은 앞뒤로 같은 폭이 "
                                       "이어지는 복도라 문 지나기를 쓰지 않고 보통 경로로 갑니다.")
            return path
        # 경로가 문을 지나던 높이로 지난다 (Gazebo 11/05: 탁자 위로 넘은 1.35 m 그대로 문을 지났다 — 그 높이에 막힘이 없다)
        hz = min(max(d.z if d.z is not None else self.height, self.height), self.max_height)
        sub, pre = self.door_approach(d, hz)
        if sub is None:
            return path
        self.door = {"d": d, "phase": "approach", "pre": pre, "realigned": 0, "z": hz}
        c = d.center
        key = (round(c[0], 1), round(c[1], 1))
        if key != self.door_logged:
            self.door_logged = key
            self.get_logger().info(f"경로가 폭 {d.width:.2f} m 틈(문, 가운데 ({c[0]:.1f}, {c[1]:.1f}), 방향 "
                                   f"{math.degrees(math.atan2(d.normal[1], d.normal[0])):.0f}°)을 지납니다. 문 앞 {pre:.1f} m까지 "
                                   "가서 정면으로 본 뒤 가운데로 곧게 지납니다.")
        return sub

    def reopen_door(self, x: float, y: float, z: float, err) -> bool:
        """돌아갈 길이 없을 때: 문 지나기로 들어온 틈(최근 것부터)을 같은 문 지나기로 되나간다. 시작했으면 True.

        좁은 틈(0.6–0.9 m)은 지나온 뒤 양쪽 문틀에 ToF가 찍히면 지도에서 닫힌다 — 찍는 자리가 문틀보다 0.1–0.2 m 안쪽이고, 지나온 칸에
        다시 찍힌 막힘은 둘레 여유까지 막기 때문이다(11/04 외부 리뷰). 11/11 Gazebo 실습실: 2.4 m 사물함과 상자 사이 0.75 m 틈으로
        들어가 둘러본 뒤 그 틈이 지도에서 0.2–0.3 m가 돼 출발점 3.4 m 앞에 내렸다. 지도만으로는 지날 수 있는지 알 수 없으니
        들어올 때와 같은 절차에 맡긴다: 문 앞 자리(평소 지도로 갈 수 있어야 한다)에 멈춰 정면으로 다시 재고, 정면 ToF가 문 너머까지
        비었을 때만 0.15 m/s로 곧게 지난다. 문 가운데 선만 연 지도로 문 너머에서 목적지까지 길이 있는지도 먼저 본다."""
        if not (self.mapping and self.avoid):
            return False
        # 후보: ① 문 지나기로 지난 틈(잰 가운데·방향을 뒤집어) ② 보통 경로로 지나온 자취 가운데 지금은 막힌 구간 (끝에서부터 처음 것)
        cands = [Doorway(d.center.copy(), -d.normal, d.width, d.z) for d in reversed(self.doors_done)]
        gap = trail_gap([self.correction.apply(p) for p in self.trail],
                        lambda p: self.inflated.occupied_at((p[0], p[1], self.height)), DOOR_PRE + DOOR_POST)
        if gap is not None and all(float(np.linalg.norm(c.center - gap.center)) >= 0.5 for c in cands):
            cands.append(gap)
        for back in cands:
            key = (round(float(back.center[0]), 1), round(float(back.center[1]), 1))
            if (-back.along(x, y) < DOOR_NEAR or self.door_fail(back.center) >= DOOR_TRIES
                    or self.reopen_count.get(key, 0) >= REOPEN_TRIES):
                continue
            hz = min(max(back.z if back.z is not None else self.height, self.height), self.max_height)
            sub, pre = self.door_approach(back, hz)
            if sub is None:
                continue
            out = back.point(DOOR_POST)
            try:   # 문 너머에서 목적지까지 (문 가운데 선만 연 지도로)
                plan(open_axis(self.inflated, back, DOOR_PRE, DOOR_POST, hz), (float(out[0]), float(out[1]), hz), self.goal,
                     z_max=self.max_height)
            except PlanError:
                continue
            self.door = {"d": back, "phase": "approach", "pre": pre, "realigned": 0, "z": hz}
            self.reopen_count[key] = self.reopen_count.get(key, 0) + 1   # 문 앞까지 가는 길이 자꾸 막히면 그만둔다 (그 자리 착륙으로)
            self.doors_reopened += 1
            c = back.center
            what = f"폭 {back.width:.2f} m" if back.width > 0 else "보통 경로로 지나온 자취"
            self.get_logger().warn(f"경로 계획 실패({err}). 돌아갈 길이 지도에서 닫혀, 들어올 때 지난 틈(가운데 ({c[0]:.1f}, {c[1]:.1f}), "
                                   f"{what})으로 되나갑니다: 문 앞 {pre:.1f} m에서 정면으로 다시 재고 정면 ToF가 너머까지 비었을 때만 곧게 "
                                   "지납니다.")
            self.publish_path(sub)
            self.waypoints = sub[1:]
            self.vp_budget = None
            self.progress = None
            if self.waypoints and path_length(sub) >= REOPEN_DIRECT:
                self.set_state("FOLLOW")
            else:
                # 이미 문 앞 자리(또는 그 REOPEN_DIRECT 안): 가지 않고 그 자리에서 바로 문을 정면으로 본다. Gazebo 틈 월드(실기체 설정):
                # 틈 옆에서 벽을 0.37 m 앞에 두고 서 있었는데, 0.13 m 옆 문 앞 자리로 가는 경로가 '앞이 길을 막습니다'에 걸려
                # 0.7초 동안 다시 계획 → 되나가기 시작을 두 번 되풀이하고 그 자리에 내렸다
                self.waypoints = []
                self.send()
                self.start_door()
            return True
        return False

    def start_door(self) -> None:
        self.door.update(phase="turn", t0=self.now(), aligned=None)
        self.set_state("DOOR")

    def door_step(self, x: float, y: float, z: float, yaw: float) -> None:
        """DOOR: 문 정면 보기(turn) → 다시 재고 정면 ToF 확인 → 가운데 선을 따라 곧게(through)."""
        D, now = self.door, self.now()
        if D is None:
            self.set_state("REPLAN")
            return
        d, hz = D["d"], D["z"]
        vz = max(-self.params.vz_max, min(self.params.vz_max, self.params.k_pos * (hz - z)))
        looking = now - D["look_start"] if D["phase"] == "look" else 0.0   # 돌아보는 시간은 시간 한도에 넣지 않는다
        if now - D["t0"] - looking > DOOR_TIMEOUT:
            self.door_abort(x, y, yaw, f"{DOOR_TIMEOUT:.0f}초 안에 못 지남")
            return
        if D["phase"] == "look":
            self.door_look_step(x, y, yaw, vz)
            return
        if D["phase"] == "turn":
            err = wrap(math.atan2(d.normal[1], d.normal[0]) - yaw)
            if abs(err) > DOOR_ALIGN:
                if self.brake_before_turn(vz):   # 문 앞 자리에 닿은 관성으로 문 쪽으로 밀리며 돌지 않게
                    D["aligned"] = None
                    return
                rate = max(-self.params.yaw_rate_max, min(self.params.yaw_rate_max, 1.5 * err))
                rate = limit_turn(rate, self.front, SLOW_TURN_RANGE, SLOW_TURN_RATE, force=now < self.slow_turn_until)
                D["aligned"] = None
                self.send(0.0, 0.0, vz, rate)
                return
            if D["aligned"] is None:
                D["aligned"] = now
            if now - D["aligned"] < DOOR_SETTLE:
                self.send(0.0, 0.0, vz, 1.5 * err)
                return
            # 문을 정면으로 본 지도로 다시 잰다 (멀리서 비스듬히 잰 가운데·방향을 고친다)
            new = measure_door(self.door_blocked(), self.cmap.origin, self.cmap.p.res, d.center, d.normal)
            prev = D.get("last_center")
            D["last_center"] = None if new is None else new.center.copy()
            shaky = new is None or (prev is not None and float(np.linalg.norm(new.center - prev)) > DOOR_SHAKE)
            people = self.door_people(d, D.get("pre", DOOR_PRE))
            if people and D.get("waited", 0.0) < DOOR_WAIT:
                look = self.door_unseen(people, (x, y, z, yaw), D.setdefault("looked", set()))
                if look is not None:   # 그 사람 자리가 화면 밖: 돌아서 떠났는지 본다
                    D["looked"].add(id(look))
                    D.update(phase="look", look=look, look_t=None, look_start=now)
                    self.get_logger().info(f"문 지나는 길 옆 찾은 사람 ({look.x:.2f}, {look.y:.2f}) 자리가 화면 밖이라, "
                                           "돌아보고 아직 있는지 봅니다.")
                    return
            if (shaky or people) and D.get("waited", 0.0) < DOOR_WAIT:
                # 사람이 오가면 잴 때마다 문 가운데가 옮겨진다 (11/09 13:05: 0.3 m씩). 제자리에서 기다렸다 다시 잰다
                if not D.get("wait_logged"):
                    D["wait_logged"] = True
                    why = (f"문 지나는 길 {PASS_PEOPLE_CLEAR} m 안에 사람이 {len(people)}명 있어" if people
                           else "다시 잰 문 가운데가 흔들려(사람이 오감)")
                    self.get_logger().info(f"{why} 문 앞에서 {DOOR_WAIT:.0f}초까지 기다렸다 다시 잽니다.")
                D["waited"] = D.get("waited", 0.0) + DOOR_SETTLE
                D["t0"] += DOOR_SETTLE   # 기다린 만큼 시간 한도를 미룬다
                D["aligned"] = now       # DOOR_SETTLE 뒤 다시 잰다
                self.send(0.0, 0.0, vz, 1.5 * err)
                return
            if people:
                # 기다려도 사람이 문 둘레에 그대로다: 사람 사이 틈을 문으로 본 것이거나 문간에 사람이 서 있다. 곧게 지나지 않고
                # 보통 경로(사람 둘레 여유가 걸린다)로 돌린다 (Gazebo people_pair 11/10: 1.2 m 간격으로 선 두 사람 사이 0.7 m를 문으로
                # 두 번 지나며 사람과 −0.03 m까지 갔다. 11/02 같은 월드에서는 문 지나기가 없어 0.38 m)
                self.door_abort(x, y, yaw, f"{DOOR_WAIT:.0f}초를 기다려도 문 지나는 길 {PASS_PEOPLE_CLEAR} m 안에 사람 {len(people)}명이 있음")
                return
            if (new is not None and DOOR_WIDTH[0] <= new.width <= DOOR_WIDTH[1] and np.linalg.norm(new.center - d.center) < 0.35
                    and abs(wrap(math.atan2(new.normal[1], new.normal[0]) - math.atan2(d.normal[1], d.normal[0]))) < math.radians(25)):
                new.z = d.z
                D["d"] = d = new
            along, across = d.along(x, y), d.across(x, y)
            if abs(across) > DOOR_ACROSS or abs(wrap(math.atan2(d.normal[1], d.normal[0]) - yaw)) > 2 * DOOR_ALIGN:
                if D["realigned"] < 1:
                    sub, pre = self.door_approach(d, hz)
                    D["realigned"] += 1
                    if sub is not None and len(sub) >= 2:
                        D.update(phase="approach", pre=pre)
                        self.get_logger().info(f"다시 잰 문 가운데 선에서 {across:+.2f} m 벗어나 문 앞 {pre:.1f} m 자리로 다시 갑니다.")
                        self.publish_path(sub)
                        self.waypoints = sub[1:]
                        self.progress = None
                        self.set_state("FOLLOW")
                        return
                    D["aligned"] = None   # 문 앞 자리 그대로: 다시 맞춘 방향으로 돌아 본다
                    return
                if abs(across) > 1.5 * DOOR_ACROSS:
                    self.door_abort(x, y, yaw, f"문 가운데 선에서 {across:+.2f} m 벗어남")
                    return
            room = -along
            if math.isfinite(self.front) and self.front < min(room + DOOR_BEYOND, TOF_MAX - 0.05):
                # 문 너머 DOOR_BEYOND까지 비어야 한다 (Gazebo 실습실 11/05: 가구 사이 틈을 문으로 보고 들어가 너머 상자에 0.22 m까지)
                self.door_abort(x, y, yaw, f"문 너머가 막힘 (정면 ToF {self.front:.2f} m, 문까지 {room:.2f} m)")
                return
            D["phase"] = "through"
            self.progress = None
            tof = f"{self.front:.2f} m" if math.isfinite(self.front) else "비어 있음"
            self.get_logger().info(f"문 {room:.2f} m 앞, 폭 {d.width:.2f} m, 가운데 선에서 {across:+.2f} m, 정면 ToF {tof}: "
                                   f"{DOOR_SPEED} m/s로 곧게 지납니다.")
            return
        along = d.along(x, y)
        if along >= DOOR_POST - 0.2:
            self.door_done(x, y)
            return
        left = DOOR_POST - along   # 곧게 갈 남은 거리
        if math.isfinite(self.front) and self.front < min(left + DOOR_BEYOND_RUN, TOF_MAX - 0.05):
            # 빠져나갈 자리보다 앞에 무엇이 있다. 문 가운데를 넉넉히 지났으면 지난 것으로, 아니면 그만둔다
            # (11/05 13:04 실기체: 0.48 m 틈 너머 1.17 m에 문짝이 보였는데 0.21 m까지 곧게 가서야 그만뒀다)
            if along >= DOOR_THROUGH:
                self.door_done(x, y)
            else:
                self.door_abort(x, y, yaw, f"빠져나갈 자리 앞에 무엇이 있음 (정면 ToF {self.front:.2f} m, 남은 거리 {left:.2f} m)")
            return
        t = d.point(min(along + DOOR_LOOKAHEAD, DOOR_POST))
        cmd, _ = follow(self.pose, (float(t[0]), float(t[1]), hz), self.door_params, final=False)
        if self.stuck(cmd):
            self.door_abort(x, y, yaw, f"{STUCK_TIME:.1f}초째 제자리")
            return
        self.send(cmd.vx, cmd.vy, cmd.vz, cmd.yaw_rate)

    def door_people(self, d, pre: float) -> list:
        """곧게 지나는 길(가운데 선, 문 앞 pre부터 너머 DOOR_POST까지)에 몸 가장자리가 PASS_PEOPLE_CLEAR 안으로 닿는 찾은 사람.
        문틀 뒤처럼 길에서 비켜 선 사람은 세지 않는다 (11/12 17:19)."""
        return [p for p in self.people.blocking() if d.on_pass(p.x, p.y, pre, DOOR_POST, PERSON_BOX)]

    def door_unseen(self, people, pose, looked: set):
        """그 가운데 지금 화면의 '떠났나' 확인 범위 밖이고 아직 돌아보지 않은 사람 하나 (없으면 None).
        너무 가깝거나 멀어 돌아봐도 못 세는 사람은 뺀다."""
        cam = self.sensor_origin(pose)
        for p in people:
            if id(p) in looked or not PERSON_VIEW[0] <= math.hypot(p.x - cam[0], p.y - cam[1]) <= PERSON_VIEW[1]:
                continue
            if person_in_view(p.x, p.y, cam, pose[3], self.hfov) is None:
                return p
        return None

    def door_look_step(self, x: float, y: float, yaw: float, vz: float) -> None:
        """DOOR 'look': 화면 밖에 있던 찾은 사람 자리로 돌아 DOOR_LOOK초 본다. 그동안 탐지 콜백(check_people_gone)이 그 자리가
        비었는지 센다. 떠났다고 지워지거나 다 봤으면 다시 문 정면(turn)으로."""
        D, now = self.door, self.now()
        p = D["look"]
        still = any(q is p for q in self.people.blocking())
        if not still or now - D["look_start"] > DOOR_LOOK_MAX or (D["look_t"] is not None and now - D["look_t"] >= DOOR_LOOK):
            if still:
                self.get_logger().info(f"돌아보니 찾은 사람 ({p.x:.2f}, {p.y:.2f}) 자리를 지울 수 없어 문 앞으로 돌아가 기다립니다.")
            D["t0"] += now - D["look_start"]
            D.update(phase="turn", aligned=None)
            return
        err = wrap(math.atan2(p.y - y, p.x - x) - yaw)
        if abs(err) > DOOR_ALIGN:
            if self.brake_before_turn(vz):
                D["look_t"] = None
                return
            rate = max(-self.params.yaw_rate_max, min(self.params.yaw_rate_max, 1.5 * err))
            rate = limit_turn(rate, self.front, SLOW_TURN_RANGE, SLOW_TURN_RATE, force=now < self.slow_turn_until)
            D["look_t"] = None
            self.send(0.0, 0.0, vz, rate)
            return
        if D["look_t"] is None:
            D["look_t"] = now
        self.send(0.0, 0.0, vz, 1.5 * err)

    def door_done(self, x: float, y: float) -> None:
        d = self.door["d"]
        self.doors_passed += 1
        self.door = None
        self.send()
        self.get_logger().info(f"문을 지났습니다 (가운데 ({d.center[0]:.1f}, {d.center[1]:.1f}), 폭 {d.width:.2f} m).")
        # 지난 틈을 지난 방향과 함께 적어 둔다 (같은 틈은 최신 것 하나만): 돌아갈 길이 닫히면 여기로 되나간다
        self.doors_done = [p for p in self.doors_done if float(np.linalg.norm(p.center - d.center)) >= 0.5] + [d]
        if self.vp_budget is not None:   # 문을 지난 것은 나아간 것이다 (남은 길 기준을 새로 잡는다)
            self.vp_budget.best, self.vp_budget.stalled = math.inf, 0.0
        if (self.search and not self.returning and self.goal is not None
                and d.covers((x, y), self.goal, DOOR_PRE, DOOR_NARROW)):
            # 볼 자리가 문 바로 너머였거나 방금 곧게 지나온 틈 안이었다: 도착한 셈으로 둘러본다 (되돌아가 같은 틈을 다시 지나지 않는다)
            self.look_target, self.looks = None, 0
            self.set_state("LOOK")
            return
        self.make_plan()

    def door_abort(self, x: float, y: float, yaw: float, why: str) -> None:
        d = self.door["d"]
        n = self.door_fail(d.center, add=True)
        self.door = None
        self.send()
        more = " 이제 이 문은 보통 경로로만 갑니다." if n >= DOOR_TRIES else ""
        self.get_logger().warn(f"문 지나기를 그만둡니다 ({why}, 이 문에서 {n}번째).{more} 들어온 길로 물러난 뒤 다시 계획합니다.")
        self.start_backoff(x, y, yaw)

    def update_tof_scale(self) -> None:
        """depth_scale_mode tof: 정면 ToF가 1.2 m 안에서 면을 재면, 같은 기수로 찍은 깊이 장면의 원뿔 안 가장 가까운 거리
        (배율 곱하기 전)와의 비를 TofDepthScale에 넣는다."""
        if self.tof_scale is None or self.scene is None:
            return
        front, now = self.front, self.now()
        if not math.isfinite(front) or front >= TOF_MAX - 0.05 or now - self.scene[0] > SHAPE_FRESH:
            return
        _, scene_yaw, centers, ranges = self.scene
        yaw = self.tof_pose[3]
        if abs(wrap(scene_yaw - yaw)) > TOF_SCALE_YAW:
            return
        cone = np.asarray(ranges)[np.abs(rotate_scene(centers, scene_yaw, yaw)) <= TOF_HALF]
        if cone.size < 2 or float(np.min(cone)) >= SCENE_MAX - 0.05:   # 깊이가 그 방향을 못 봤다 (4 m로 채운 열)
            return
        self.tof_scale.add(front, float(np.min(cone)) / self.scene_scale, now)

    def log_depth_scale(self) -> None:
        old, t = self.scale_logged
        if abs(self.depth_scale - old) >= 0.05 and self.now() - t > 3.0:
            self.scale_logged = (self.depth_scale, self.now())
            self.get_logger().info(f"깊이 배율 {old:.2f} → {self.depth_scale:.2f} (정면 ToF와 비교한 최근 "
                                   f"{len(self.tof_scale.samples)}개 중앙값)")

    def check_see_through(self) -> None:
        """정면 ToF가 면을 재는데 깊이 장면은 그 너머를 보면(흰 벽·유리를 꿰뚫어 봄), 그 뒤에 칠한 '본 빈 칸'을 지우고
        SEE_THROUGH_HOLD 동안 그 방향 깊이 영상은 ToF 거리까지만 넣는다(on_depth). 두 번 이어질 때만 믿는다."""
        front, now = self.front, self.now()
        if (not math.isfinite(front) or front >= TOF_MAX - 0.05 or self.scene is None
                or now - self.scene[0] > SHAPE_FRESH):
            self.see_through_streak = 0
            return
        _, scene_yaw, centers, ranges = self.scene
        yaw = self.tof_pose[3]
        far = see_through(rotate_scene(centers, scene_yaw, yaw), ranges, front, TOF_HALF)
        if far is None:
            self.see_through_streak = 0
            return
        self.see_through_streak += 1
        if self.see_through_streak < 2:
            return
        z = self.tof_pose[2]
        n = self.cmap.forget_behind(self.sensor_origin(self.tof_pose), yaw, front + SEE_THROUGH_BEHIND, self.cmap.p.max_range,
                                    TOF_HALF + SEE_THROUGH_WIDEN, z - 0.5, z + 0.5)
        self.see_through = (now, yaw, front)
        if now - self.see_through_logged > 3.0:
            self.see_through_logged = now
            self.get_logger().info(f"앞 {front:.2f} m를 ToF가 재는데 깊이는 {far:.1f} m를 봅니다(흰 벽·유리를 꿰뚫어 봄). "
                                   f"그 뒤에 칠한 빈 칸 {n}개를 지우고 이 방향 깊이는 ToF 거리까지만 넣습니다.")

    def find_viewpoint(self, x: float, y: float):
        if self.mapping:   # 가구에 붙은 그늘은 경계가 아니고, 둘레가 빈 자리에서만 본다 (11/03 14:32)
            elapsed = self.now() - self.search_start if self.search_start is not None else 0.0
            self.vp_reach = max(VP_MIN_REACH, (self.search_time - elapsed) * self.params.speed * VP_TIME_SPEED)   # 남은 시간 안 (11/04)
            # 경계는 닿는 곳만(가구 위로는 넘되 창턱 위로는 안 넘음 — 닫힌 창 너머는 빼고), 30×30 cm 넘는 통로의 입구는 경계로 (11/04)
            common = dict(exclude=self.bad_viewpoints, heading=self.travel_heading(x, y), visited=self.visited, visited_radius=1.0,
                          z_top=self.max_height, reach_frontier=True, passage=PASSAGE_MIN, passage_area=PASSAGE_AREA,
                          shadow=FRONTIER_SHADOW, standoff=VIEW_STANDOFF, room_prefer=VIEW_ROOM, room_weight=VIEW_ROOM_WEIGHT,
                          view_range=VIEW_RANGE, min_gain=VIEW_GAIN_MIN, cost_distance=VIEW_COST_DISTANCE,   # 새로 보일 넓이로 (11/04)
                          max_dist=self.vp_reach)
            vp = self.cmap.frontier_viewpoint(x, y, self.inflated, self.height, clearance=VIEW_CLEAR, **common)
            if vp is None:   # 좁은 복도(1.4 m, Gazebo corridor_L)에서는 둘레 0.45 m가 빈 자리가 없어 탐색이 바로 끝났다 → 0.2 m로 한 번 더
                vp = self.cmap.frontier_viewpoint(x, y, self.inflated, self.height, clearance=VIEW_CLEAR_MIN, **common)
                if vp is not None and self.now() - self.narrow_vp_logged > 10.0:
                    self.narrow_vp_logged = self.now()
                    self.get_logger().info(f"둘레 {VIEW_CLEAR} m가 빈 볼 자리가 없어 {VIEW_CLEAR_MIN} m인 자리에서 봅니다 (좁은 복도).")
            return vp
        k = int(math.floor((self.height - self.inflated.origin[2]) / self.inflated.res))   # 사람 위로 넘는 중이면 올린 높이
        free2d = ~self.inflated.occ[:, :, k]
        return self.coverage.next_viewpoint(x, y, free2d, self.inflated.origin[:2], self.inflated.res,
                                            exclude=[b[:2] for b in self.bad_viewpoints] + self.visited)

    def save_people(self) -> None:
        if not self.people_out:
            return
        data = {"coverage": round(self.coverage.coverage(), 3) if self.search and not self.mapping else None,
                "explored_m2": round(self.cmap.explored_area(self.base_height), 1) if self.mapping else None,
                "t_start": round(self.search_start, 2) if self.search_start else None,
                "search_time": round(self.now() - self.search_start, 1) if self.search_start else 0.0,
                "people": [_person(p) for p in self.people.confirmed()],
                "unconfirmed": [_person(p) for p in self.people.people if p not in self.people.confirmed()]}
        Path(self.people_out).write_text(json.dumps(data, ensure_ascii=False, indent=1), encoding="utf-8")

    def return_check(self, x: float, y: float) -> bool:
        """돌아오는 중 출발점 가까이에서 더 못 가거나 너무 오래 걸리면 그 자리에 내린다 (True: 착륙 시작). RETURN_* 설명."""
        if not self.returning or self.return_t0 is None or self.state in ("LAND", "DONE", "ABORT"):
            return False
        now = self.now()
        if self.state in ("HOMESPIN", "RELOC"):
            # 일부러 멈춰 도는 동안(출발점 위 한 바퀴·위치 다시 잡기)은 정체로 세지 않는다 (Gazebo 사람 둘레 11/10: 한 바퀴 31초 뒤
            # 다시 다가가기 전에 0.6 m 앞에서 내렸다)
            self.return_best_t = now
            return False
        d = math.hypot(x - self.start_xy[0], y - self.start_xy[1])
        if d < self.return_best - 0.3:
            self.return_best, self.return_best_t = d, now
        if d <= RETURN_NEAR and now - self.return_best_t > RETURN_STALL:
            why = f"출발점 {d:.1f} m 앞에서 {RETURN_STALL:.0f}초 넘게 더 가까워지지 못해"
        elif now - self.return_t0 > RETURN_MAX:
            why = f"돌아오기 시작 {RETURN_MAX:.0f}초가 지나 출발점에서 {d:.1f} m 떨어진"
        else:
            return False
        self.get_logger().warn(f"{why} 여기에 내립니다.")
        self.start_landing(x, y)
        return True

    def free_near_start(self, radius: float = RETURN_RETARGET):
        """출발점 둘레 radius 안에서 계획 지도(비행 높이)가 빈 칸 중 출발점에 가장 가까운 자리 (없으면 None)."""
        m = self.inflated
        k = m.index((self.start_xy[0], self.start_xy[1], self.height))[2]
        n = int(math.ceil(radius / m.res))
        ci, cj, _ = m.index((self.start_xy[0], self.start_xy[1], self.height))
        best, best_d = None, math.inf
        for i in range(ci - n, ci + n + 1):
            for j in range(cj - n, cj + n + 1):
                if not m.inside((i, j, k)) or m.occ[i, j, k]:
                    continue
                p = m.center((i, j, k))
                dd = math.hypot(p[0] - self.start_xy[0], p[1] - self.start_xy[1])
                if dd <= radius and dd < best_d:
                    best, best_d = p, dd
        return None if best is None else (float(best[0]), float(best[1]), best_d)

    def make_plan(self) -> None:
        t0 = self.now()
        try:
            self._make_plan()
        finally:
            self.plan_s, self.plan_done_t = self.now() - t0, self.now()
            if self.plan_s > 0.3:
                self.get_logger().warn(f"길 계획에 {self.plan_s:.1f}초 걸렸습니다 (그동안 위치·ToF 처리가 밀립니다).")

    def _make_plan(self) -> None:
        x, y, z, _ = self.pose
        self.door = None
        if self.mapping and self.map_radius != self.map_radius_full and self.height == self.base_height:
            self.set_map_radius(self.map_radius_full)   # 줄인 여유는 지난 경로까지만: 늘 평소 여유로 먼저 짠다 (사람 위로 넘는 중이면 그대로)
        try:
            try:
                path = plan(self.inflated, (x, y, z), self.goal, z_max=self.max_height)
            except PlanError as err:
                if not (self.mapping and self.map_radius > self.map_radius_tight):
                    raise
                # 바로 이 자리에서 줄인 여유로 한 번 더 (REPLAN으로 돌려보내면 다음 계획이 다시 평소 여유로 되돌려 끝없이 돌았다, Gazebo 실습실)
                self.get_logger().warn(f"경로 계획 실패({err}). 지도 여유를 {self.map_radius_tight} m로 줄여 다시 시도합니다.")
                self.set_map_radius(self.map_radius_tight)
                path = plan(self.inflated, (x, y, z), self.goal, z_max=self.max_height)
        except PlanError as err:
            if self.search and not self.returning and self.goto_active is not None:
                if self.goto_fallback(x, y, err):   # 찍은 칸까지는 길이 없다: 갈 수 있는 가장 가까운 자리로 한 번 옮겨 본다
                    return
                # 운용자가 찍은 자리로 가는 길이 없다: 알리고 하던 탐색을 잇는다. 못 간 자리 목록에는 넣지 않는다
                # (그 목록이 20곳이면 탐색을 끝내고, 찍은 자리는 운용자가 다시 찍을 수 있어야 한다)
                self.goto_end(f"찍은 자리 ({self.goal[0]:.1f}, {self.goal[1]:.1f})로 가는 길이 없습니다({err}). "
                              f"{self.goto_after()}.", warn=True)
                self.set_state("PLAN")
                return
            if self.search and not self.returning:
                # 탐색 자리로 가는 길이 없다: 그 자리는 빼고 다른 자리를 고른다
                self.bad_viewpoints.append((float(self.goal[0]), float(self.goal[1])))
                self.get_logger().warn(f"그 자리로 가는 길이 없습니다({err}). 다른 자리를 고릅니다.")
                self.set_state("PLAN")
                return
            if self.found_radius > FOUND_RADIUS_TIGHT and self.memory.items:
                self.get_logger().warn(f"경로 계획 실패({err}). 찾은 장애물 여유를 {FOUND_RADIUS_TIGHT} m로 줄여 다시 시도합니다.")
                self.found_radius = FOUND_RADIUS_TIGHT
                self.rebuild_maps()
                self.set_state("REPLAN")
                return
            suspicious = sum(1 for i in self.memory.items if self.tof_near_known(i))
            if suspicious:   # 위치 오차로 아는 벽·문틀을 새 장애물로 잘못 넣었을 수 있다
                # 빼기만 하면 위치가 틀어진 채라 같은 문틀을 다시 넣고 막히기를 되풀이한다 (3단계 seed 1).
                # 먼저 위치를 다시 잡고, 그 뒤 뺀다
                if self.start_relocalize(f"경로 계획 실패({err}). 아는 벽에 붙은 ToF 장애물이 {suspicious}개 있습니다."):
                    return
                near = self.memory.drop_where(self.tof_near_known)
                self.get_logger().warn(f"경로 계획 실패({err}). 아는 벽에 붙은 ToF 장애물 {near}개를 빼고 다시 시도합니다.")
                self.rebuild_maps()
                self.save_discovered()
                self.set_state("REPLAN")
                return
            dropped = self.memory.drop_camera_only()
            if dropped:   # 카메라 오탐이 길을 막았을 수 있다. 다음 주기에 REPLAN으로 다시 시도
                self.get_logger().warn(f"경로 계획 실패({err}). 카메라로만 본 장애물 {dropped}개를 빼고 다시 시도합니다.")
                self.rebuild_maps()
                self.save_discovered()
                self.set_state("REPLAN")
                return
            if self.people.blocking() and self.person_margin > PERSON_MARGIN_TIGHT:
                # 찾은 사람(같은 사람이 위치가 조금 다르게 여러 번 잡히기도 한다)이 통로를 막았을 수 있다
                self.get_logger().warn(f"경로 계획 실패({err}). 찾은 사람 둘레 여유를 {PERSON_MARGIN_TIGHT} m로 줄여 다시 시도합니다.")
                self.person_margin = PERSON_MARGIN_TIGHT
                self.people_relaxed_at = self.people_relaxed_at or (x, y)
                self.rebuild_maps()
                self.set_state("REPLAN")
                return
            weak = [p for p in self.people.blocking() if p.hits < PERSON_WEAK_HITS]
            if weak and not self.drop_weak_people:
                # 몇 번만 잡힌 '사람'은 같은 사람을 위치가 틀어진 채 다시 잡았거나 멀리서 잘못 잡은 것이 많다. 그것이
                # 출발점으로 돌아가는 통로를 막아 중단한 적이 있다 (4단계 seed 3). 가까이 가면 YOLO·ToF가 다시 잡는다
                self.get_logger().warn(f"경로 계획 실패({err}). 몇 번만 잡힌 사람 {len(weak)}명을 장애물에서 빼고 다시 시도합니다.")
                self.drop_weak_people = True
                self.rebuild_maps()
                self.set_state("REPLAN")
                return
            act = self.people_block(x, y, f"경로 계획 실패({err}).")
            if act == "land":
                return
            if act == "retry":
                self.set_state("REPLAN")
                return
            if self.start_relocalize(f"경로 계획 실패({err})."):
                return
            if self.returning:
                if self.reopen_door(x, y, z, err):   # 들어올 때 지난 틈이 지도에서 닫혔으면 같은 문 지나기로 되나간다 (11/11)
                    return
                d = math.hypot(x - self.start_xy[0], y - self.start_xy[1])
                if not self.return_retargeted:
                    # 위치가 밀려 지도에서 출발점이 벽 뒤에 갇혔을 수 있다 (11/06 14:35): 둘레 빈 자리로 한 번 다시 겨눈다
                    self.return_retargeted = True
                    near = self.free_near_start()
                    if near is not None and math.hypot(near[0] - x, near[1] - y) > 0.3:
                        self.goal = np.array([near[0], near[1], self.height])
                        self.get_logger().warn(f"경로 계획 실패({err}). 지도에서 출발점 둘레가 막혀 출발점에서 {near[2]:.1f} m 떨어진 "
                                               f"빈 자리 ({near[0]:.1f}, {near[1]:.1f})로 갑니다.")
                        self.set_state("REPLAN")
                        return
                self.get_logger().warn(f"경로 계획 실패({err}). 출발점까지 길이 없어 출발점에서 {d:.1f} m 떨어진 여기에 내립니다.")
                self.start_landing(x, y)
                return
            self.get_logger().error(f"경로 계획 실패: {err}")
            self.set_state("ABORT")
            return
        path = self.door_path(path)   # 좁은 틈(문)을 지나면 문 앞까지의 경로로 바꾸고 self.door를 잡는다
        self.publish_path(path)
        self.waypoints = path[1:]
        if self.search and not self.returning:   # 볼 자리마다 재계획·시간 한도 (11/04)
            if self.state == "PLAN" or self.vp_budget is None:
                self.vp_budget = ViewpointBudget(self.now())
            if self.door is not None:
                self.vp_budget.door = True    # 문 너머: 문은 원래 좁으므로 '좁은 길이면 빨리 포기'를 쓰지 않는다 (11/05)
            elif self.mapping and self.map_radius < self.map_radius_full:
                self.vp_budget.tight = True   # 줄인 여유로만 길이 나왔다: 좁은 곳이라 더 빨리 포기한다
            self.vp_budget.remaining(path_length(path), self.now())
        else:
            self.vp_budget = None
        what = "경로" if self.state == "PLAN" else f"새 경로 (재계획 {self.replans}번째)"
        self.progress = None
        self.get_logger().info(f"{what} {path_length(path):.2f} m, 경유점 {len(self.waypoints)}개")
        if not self.waypoints and self.door is not None:   # 이미 문 앞 자리: 바로 문을 정면으로 본다
            self.send()
            self.start_door()
            return
        if not self.waypoints:   # 출발점과 목적지가 같은 칸: 이미 목적지 근처
            if self.search and not self.returning:
                # 탐색 중이면 도착한 셈으로 둘러본다. 예전에는 여기서 그 자리에 착륙해 탐색이 끝났다
                self.send()
                self.look_target, self.looks = None, 0
                self.set_state("LOOK")
                return
            self.land_at_goal(*self.pose[:2])
            return
        self.set_state("FOLLOW")

    def tof_near_known(self, item) -> bool:
        """ToF로 찾은 장애물이 아는 벽·가구에 붙어 있는가 (위치 오차로 아는 것을 새 장애물로 넣었을 수 있음)."""
        return "tof" in item.sources and near_known(self.known, item.box, 0.3, self.height)

    def reloc_useful(self) -> bool:
        """지금 자리에서 한 바퀴 돌면 ToF(1.2 m)에 아는 벽·가구가 두 방향(사분면) 넘게 잡히는가.

        넓은 곳에서 후보만 흩고 돌면 잡히는 게 없어 오히려 σ가 커지고 위치를 잃는다 (3단계 seed 3).
        """
        x, y, _, _ = self.pose
        quadrants = set()
        for k in range(24):
            a = 2 * math.pi * k / 24
            for r in np.arange(0.1, 1.1, 0.05):
                if self.known.occupied_at((x + r * math.cos(a), y + r * math.sin(a), self.height)):
                    quadrants.add(k // 6)
                    break
        return len(quadrants) >= 2

    def start_relocalize(self, why: str) -> bool:
        """제자리에서 천천히 한 바퀴 돌며 위치를 다시 잡기 시작한다. 이미 RELOC_MAX번 했으면 False."""
        if self.relocs >= RELOC_MAX or self.pose is None or self.mapping:
            return False   # 4단계는 위치 추정에 쓸 지도가 없어 다시 잡을 수 없다
        if not self.reloc_useful():
            self.get_logger().warn(f"{why} 위치를 다시 잡아 보려 했지만, 1.1 m 안에 벽·가구가 한 방향뿐이라 "
                                   "돌아도 바로잡히지 않아 하지 않습니다.")
            return False
        self.relocs += 1
        self.send()
        self.reloc = {"last": self.pose[3], "turned": 0.0}
        self.reloc_pub.publish(String(data=json.dumps({"xy": RELOC_SPREAD, "yaw": math.radians(3.0)})))
        self.get_logger().warn(f"{why} 위치가 틀어졌을 수 있어 제자리에서 한 바퀴 돌며 다시 잡습니다 "
                               f"({self.relocs}/{RELOC_MAX}).")
        self.set_state("RELOC")
        return True

    # ------------------------------------------------------------ 사람이 길을 막을 때 (2026-11-02)

    def people_blocked_viewpoints(self, x: float, y: float) -> bool:
        """볼 자리가 없는 것이 찾은 사람 때문인가: 사람 둘레를 빼면 볼 자리가 생기는가."""
        if self.people_soft or not (self.search and self.avoid) or not self.blocking_people():
            return False
        self.people_soft = True
        self.rebuild_maps()
        found = self.find_viewpoint(x, y) is not None
        self.people_soft = False
        self.rebuild_maps()
        return found

    def people_viewpoint(self, x: float, y: float):
        """사람에 막혀 볼 자리가 없을 때: people_block 차례대로 해 보며 볼 자리를 찾는다 (착륙하면 None)."""
        while True:
            act = self.people_block(x, y, "찾은 사람이 길을 막아 갈 수 있는 볼 자리가 없습니다.")
            if act != "retry":
                return None
            vp = self.find_viewpoint(x, y)
            if vp is not None:
                return vp

    def people_block(self, x: float, y: float, why: str) -> str:
        """찾은 사람이 길을 막았다. 한 번에 한 가지씩 바꾸고 'retry'(다시 찾는다), 그 자리 착륙을 시작했으면 'land',
        사람이 없거나 더 바꿀 것이 없으면 ''을 돌려준다.

        차례: 사람 둘레 여유 줄이기 → 키를 잰 낮은 사람이면 높이를 올려 위로 넘기 → 부딪힐 위험이 높으면 그 자리에 착륙,
        낮으면 찾은 사람 둘레를 지도에서 빼고 이어 간다 (카메라가 본 몸·ToF·비상 정지는 그대로 피한다).
        """
        blocking = self.blocking_people()
        if not (self.search and self.avoid) or not blocking or self.people_soft:
            return ""   # 사람은 탐색 중에만 지도에 넣는다
        self.people_relaxed_at = self.people_relaxed_at or (x, y)
        if self.person_margin > PERSON_MARGIN_TIGHT:
            self.person_margin = PERSON_MARGIN_TIGHT
            self.rebuild_maps()
            self.get_logger().warn(f"{why} 찾은 사람 둘레 여유를 {PERSON_MARGIN_TIGHT} m로 줄여 다시 찾습니다.")
            return "retry"
        if self.try_climb(x, y, blocking):
            return "retry"
        risk = people_risk(blocking, x, y)
        if risk is not None:
            self.get_logger().warn(f"{why} {risk}. 사람과 부딪힐 위험이 높아 그 자리에 착륙합니다.")
            self.save_people()
            self.start_landing(x, y)
            return "land"
        self.people_soft = True
        self.rebuild_maps()
        self.get_logger().warn(f"{why} 가까이(0.6 m 안)나 사방에 사람이 있지 않아 부딪힐 위험은 낮습니다. 찾은 사람 둘레 여유를 "
                               f"몸 둘레 {PERSON_MARGIN_SOFT} m만 남기고 탐색을 이어 갑니다 (사람 {PERSON_SLOW_R:g} m 안에서는 "
                               f"{PERSON_SLOW_SPEED} m/s, 카메라가 본 몸·ToF·비상 정지는 그대로 피합니다).")
        return "retry"

    def try_climb(self, x: float, y: float, blocking) -> bool:
        """막은 사람(CLIMB_NEAR 안)의 키를 모두 쟀고 max_height 아래로 넘을 수 있으면 비행 높이를 올린다."""
        near = [p for p in blocking if math.hypot(p.x - x, p.y - y) < CLIMB_NEAR]
        if not near or self.climbs >= CLIMB_MAX:
            return False
        if any(p.top is None or p.top_n < CLIMB_TOP_N for p in near):
            self.get_logger().info(f"길을 막은 사람 중 키를 {CLIMB_TOP_N}번 넘게 재지 못한 사람(머리가 화면 위로 잘리거나 멀리서만 봄)이 "
                                   "있어 높이를 올려 넘지 않습니다.")
            return False
        top = max(p.top for p in near)
        radius = self.map_radius_tight if self.mapping else 0.0   # 깊이 지도의 몸도 여유만큼 부푼다
        need = round(top + max(PERSON_TOP_PAD + self.person_margin, radius) + CLIMB_CLEAR, 2)
        if need > self.max_height:
            self.get_logger().info(f"길을 막은 사람 키가 약 {top:.1f} m라 경로 높이 상한 {self.max_height:.2f} m 아래로는 "
                                   "위로 넘지 못합니다.")
            return False
        if self.height >= need - 0.01:
            return False   # 이미 그 높이에서 찾았다
        self.climbs += 1
        self.height = max(self.base_height, need)
        if self.goal is not None:
            self.goal[2] = self.height
        if self.mapping and self.map_radius != self.map_radius_tight:
            self.set_map_radius(self.map_radius_tight)
        self.get_logger().warn(f"사람(키 약 {top:.1f} m)이 길을 막아 높이를 {self.height:.2f} m로 올려 위로 넘어갈 길을 찾습니다 "
                               f"({self.climbs}/{CLIMB_MAX}).")
        return True

    def restore_after_people(self, x: float, y: float) -> None:
        """사람에 막혀 바꾼 높이·여유·사람 둘레를 그 자리에서 PEOPLE_RELAX_LEAVE m 넘게 벗어나면 되돌린다."""
        if self.people_relaxed_at is None:
            return
        far = math.hypot(x - self.people_relaxed_at[0], y - self.people_relaxed_at[1])
        if far < PEOPLE_RELAX_LEAVE:
            return
        undo = []
        if self.height != self.base_height:
            self.height = self.base_height
            undo.append(f"비행 높이 {self.base_height:.2f} m")
        if self.person_margin != PERSON_MARGIN:
            self.person_margin = PERSON_MARGIN
            undo.append(f"사람 둘레 여유 {PERSON_MARGIN} m")
        if self.people_soft:
            self.people_soft = False
            undo.append("찾은 사람 둘레")
        if self.goto_weak_dropped:   # 운용자가 찍은 자리로 가느라 뺀 몇 번만 잡힌 사람 (goto_relax)
            self.goto_weak_dropped = self.drop_weak_people = False
            undo.append("몇 번만 잡힌 사람")
        self.people_relaxed_at = None
        self.rebuild_maps()
        if undo:
            self.get_logger().info(f"사람이 막았던 자리에서 {far:.1f} m 벗어나 {', '.join(undo)}를 되돌립니다.")

    def under_check(self, x: float, y: float, z: float) -> bool:
        """밑 감시: 하방 ToF가 UNDER_MIN보다 짧으면 수평을 멈추고 UNDER_CLEAR까지 오르며 그 자리를 막는다. 오르는 동안 True."""
        now = self.now()
        if self.under_since is None:
            if self.tof_down is None or self.tof_down >= UNDER_MIN:
                return False
            self.under_since = now
            top = round(max(0.3, z - self.tof_down + 0.05), 2)
            if self.mapping:
                r = self.cmap.p.res
                xs, ys = np.arange(x - UNDER_FOOT, x + UNDER_FOOT + 1e-9, r / 2), np.arange(y - UNDER_FOOT, y + UNDER_FOOT + 1e-9, r / 2)
                zs = np.arange(0.3, top + 1e-9, r)
                self.cmap.mark_hard(np.array([(px, py, pz) for px in xs for py in ys for pz in zs]), HARD_WEIGHT)
                self.update_camera_map(x, y, z)
            else:
                self.add_obstacle({"x": round(x, 3), "y": round(y, 3), "z": round(top / 2, 3), "sx": 2 * UNDER_FOOT,
                                   "sy": 2 * UNDER_FOOT, "sz": top}, "under", f"하방 ToF {self.tof_down:.2f} m: 바로 밑에 무엇이 있음")
            self.get_logger().warn(f"바로 밑 {self.tof_down:.2f} m에 무엇이 있습니다(꼭대기 약 {top:.2f} m). 멈추고 {UNDER_CLEAR} m 떨어질 때까지 "
                                   "오른 뒤 다시 계획합니다.")
        done = self.tof_down is None or self.tof_down >= UNDER_CLEAR or z >= self.max_height
        if done or now - self.under_since > 4.0:
            self.under_since = None
            self.send()
            if z >= self.max_height and self.tof_down is not None and self.tof_down < UNDER_CLEAR:
                self.get_logger().warn(f"경로 높이 상한 {self.max_height:.2f} m라 더 오르지 못합니다. 그 자리를 막고 다시 계획합니다.")
            self.set_state("REPLAN")
            return True
        self.send(0.0, 0.0, UNDER_CLIMB, 0.0)
        return True

    def near_turn(self, x: float, y: float, yaw: float, then: str) -> bool:
        """정면 NEAR_TURN 안이 막혀 있는데 제자리에서 돌려 하면, 먼저 들어온 길로 물러난다 (같은 자리에서 NEAR_TURN_TRIES번까지).
        물러나기를 시작했으면 True."""
        if not (math.isfinite(self.front) and self.front < NEAR_TURN):
            return False
        if self.near_spot is None or math.hypot(x - self.near_spot[0], y - self.near_spot[1]) > 0.5:
            self.near_spot, self.near_tries = (x, y), 0
        if self.near_tries >= NEAR_TURN_TRIES:
            self.slow_turn_until = self.now() + 1.0   # 더 물러나도 그대로면 천천히 돈다
            return False
        rear, off, _, _ = self.retreat_check(x, y, yaw)
        if rear is not None or off is not None:
            # 뒤를 모르거나(11/09 13:03: 이륙 직후 둘러보다 본 적 없는 뒤로 후진해 떨어짐) 뒤가 들어온 길 쪽이 아니면(11/16 Gazebo:
            # 문틀 사이에서 돌다 반대편 문틀 쪽으로 물러남) 물러나지 않고 천천히 돈다
            self.slow_turn_until = self.now() + 1.0
            self.note_rear(rear, f"앞 {self.front:.2f} m가 막혔지만", off)
            return False
        self.near_tries += 1
        self.send()
        self.get_logger().warn(f"앞 {self.front:.2f} m가 막혀 있어 제자리에서 돌지 않고, 들어온 길로 {RETREAT_DIST} m 물러난 뒤 "
                               f"천천히 돕니다 ({self.near_tries}/{NEAR_TURN_TRIES}).")
        self.slow_turn_until = self.now() + SLOW_TURN_AFTER
        self.start_backoff(x, y, yaw, then=then)
        return True

    def land_at_goal(self, x: float, y: float) -> None:
        """경로 끝 (x, y)에서 내린다. 목적지가 LAND_NEAR 안이면 그 바로 위로 맞춰 내리고(도착 판정 0.15 m의 남은 오차), 더 멀면 — 목적지
        칸이 막혀 경로가 가장 가까운 빈 칸에서 끝난 것 — 맞추러 가지 않고 여기에 내린다. 맞추는 동안(LAND)은 장애물을 보지 않는다.
        11/11 Gazebo 사람 둘레: 출발점 자리에 사람이 잡혀 경로가 0.5 m 앞에서 끝났는데, 맞추기가 0.2 m/s로 0.47 m를 더 가 사람과 겹쳤다."""
        gx, gy = float(self.goal[0]), float(self.goal[1])
        d = math.hypot(gx - x, gy - y)
        if d > LAND_NEAR:
            self.get_logger().warn(f"목적지 ({gx:.1f}, {gy:.1f}) 둘레가 막혀 경로가 {d:.2f} m 앞에서 끝났습니다. 더 다가가지 않고 여기에 내립니다.")
            gx, gy = x, y
        else:   # 평소 끝: 바로 앞에 남긴 로그가 없으므로 이유를 여기서 정한다 (웹 화면에 보인다)
            self.why_end = (f"{self.why_return} 출발점으로 돌아와 내립니다." if self.returning and self.why_return
                            else "목적지에 도착해 내립니다.")
        self.start_landing(gx, gy)

    def start_landing(self, x: float, y: float) -> None:
        self.send()
        self.hold = (x, y)
        self.land_t = self.now()
        self.set_state("LAND")

    def publish_path(self, path) -> None:
        msg = PathMsg()
        msg.header.frame_id = "world"
        msg.header.stamp = self.get_clock().now().to_msg()
        for p in path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, p)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)


def _seg_dist(p, a, b) -> float:
    """점 p에서 선분 a–b까지 거리 (2D)."""
    ax, ay, bx, by = a[0], a[1], b[0], b[1]
    dx, dy = bx - ax, by - ay
    n = dx * dx + dy * dy
    t = 0.0 if n < 1e-12 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / n))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def _person(p) -> dict:
    return {"x": round(p.x, 2), "y": round(p.y, 2), "conf": round(p.conf, 2), "t_first": round(p.t_first, 2),
            "hits": p.hits, "gone": p.gone, "top": None if p.top is None else round(p.top, 2), "top_n": p.top_n}   # 잰 키 (m)·잰 횟수


def main() -> None:
    rclpy.init()
    node = Mission()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():   # Ctrl+C로 이미 꺼진 뒤에는 보낼 수 없다
            node.send()
        if node.recorder:   # 중간에 꺼져도 남은 기록을 쓴다
            node.recorder.flush()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
