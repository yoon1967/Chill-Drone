# 시뮬레이션

ROS 2·Gazebo 모델·월드·센서·브리지·실행 구성을 둔다. 문의: 허병우. 기존 Jira 기준 환경은 Ubuntu 24.04·ROS 2 Jazzy·Gazebo Harmonic이며 실제 패키지와 실행 파일은 추후 가져와 검증한다.

최초 구현 시 지원 버전·설치·실행·테스트·설정 이름과 입출력 계약 링크를 이 문서에 추가한다.
팀 작업 절차는 저장소 루트의 docs/team/START_HERE.md를 따른다.

## 현재 들어 있는 것

- [tt-sim](tt-sim/README.md): RoboMaster TT 사양 실내 시뮬레이션과 그 위에서 검증한 자율비행 코드.
  seed로 실내 맵을 만들고(방·문·가구·사람), 단안 카메라·전방 ToF·하방 거리 센서를 단 TT 드론 모델로
  경로 계획 비행, YOLO 사람 탐지, 지도에 없는 장애물 피하기, 정답 위치 없이 위치 추정, 사람 찾기 탐색,
  지도 없이 카메라 깊이 추정으로 지도를 만들며 탐색하는 단계까지 시험한다. 단계별 결과는 tt-sim README의 진행 단계 표에 있다.
  기본으로 실기체와 같은 Tello 드라이버(`modules/autonomy/tello_driver`)를 거쳐 UDP SDK 명령(rc·takeoff·land)으로 날고,
  `tt-sim/nodes/gz_tello.py`가 Gazebo 드론을 Tello처럼 흉내 낸다(`--direct`면 예전처럼 속도를 Gazebo에 바로 넣는다).

| 항목 | 내용 |
| --- | --- |
| 확인한 환경 | Ubuntu 24.04(WSL2) · ROS 2 Jazzy · Gazebo Sim 8.15(Harmonic) |
| 설치 | `sudo bash tt-sim/scripts/setup_jazzy.sh`, YOLO·깊이 추정은 `tt-sim/scripts/setup_yolo.sh`가 만드는 `~/venvs/tt` (`TT_VENV`로 바꿀 수 있음) |
| 실행 | tt-sim 폴더에서 `python3 generate_world.py --seed 7 --stage 1`로 맵을 만들고 `bash scripts/run_mission.sh worlds/s7_stage1`. 카메라·지도를 보며 버튼으로 시작하려면 `ros2 launch launch/mission.launch.py world_dir:=$PWD/worlds/s7_stage1 web:=true` 뒤 http://localhost:8080 |
| 테스트 | tt-sim 폴더에서 `python3 -m unittest discover -s tests` (단위 테스트 121개, numpy·scipy 필요) |
| 생성물 | `worlds/`(맵·비행 기록), `weights/`, 비행 기록 `record/`는 tt-sim/.gitignore로 올리지 않는다 |

배치 메모: 저장소 구조상 비행 코드는 `modules/autonomy/`에 둘 자리지만, 지금은 tt-sim 안의 `autonomy/`·`perception/`·`nodes/`에 함께 있다.
launch 파일과 실행 스크립트가 tt-sim 폴더를 기준으로 이 코드를 불러 쓰기 때문이다. 실기체 드라이버에 연결할 때 `modules/autonomy/`로 옮기고 경로를 맞춘다.
맵 파일 형식(`known_map.json`, `ground_truth.json`)은 아직 tt-sim 내부 형식이며 `contracts/`에 올린 계약은 없다.
