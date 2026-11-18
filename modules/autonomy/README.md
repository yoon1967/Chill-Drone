# 자율비행·경로·기체 어댑터

경로 계획·경유점 추종·중단/복구·실제 TT 제어 어댑터 코드를 둔다. 문의: 허병우. Gazebo 모델·월드·실행 구성은 simulation에 둔다.

최초 구현 시 지원 버전·설치·실행·테스트·설정 이름과 입출력 계약 링크를 이 문서에 추가한다.
팀 작업 절차는 저장소 루트의 docs/team/START_HERE.md를 따른다.

## 현재 들어 있는 것

- [tello_link](tello_link/README.md): Tello EDU SDK 연결 모듈과 연결 점검·수동 조작 화면(S15P31B205-156). 아래 tello_driver가 import해 쓴다.
- [tello_driver](tello_driver/README.md): 자율비행용 ROS 2 드라이버. 비행 노드의 속도 명령·이륙·착륙 요청을 rc·takeoff·land로 보내고 Tello 상태를 `/tt/state`로 낸다. Gazebo 시뮬레이션도 같은 드라이버를 거쳐 난다(`simulation/tt-sim/nodes/gz_tello.py`가 Tello를 흉내).
- [autonomy_web](autonomy_web/README.md): 자율비행 웹 화면. 카메라 영상·지도·비행 단계·기체 상태를 보며 시작·착륙·비상 정지. tello_link 점검 화면의 모양과 안전 확인 흐름을 가져왔다.
- 시뮬레이션에서 검증한 경로 계획·위치 추정·탐색 코드는 지금 [simulation/tt-sim](../../simulation/tt-sim/README.md) 안에 함께 있다. 실기체 드라이버에 연결할 때 이 폴더로 옮긴다.
- [legacy](legacy/README.md): 지금은 쓰지 않는 자율비행 코드 보관(MoNA-Bench 장애물 회피). 실행기와 연결돼 있지 않다.
