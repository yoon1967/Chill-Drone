# 허병우 팀원 자율비행 웹 서버 실행

`scripts\start.cmd team-autonomy docker`와 `scripts\start.cmd team-autonomy wsl`(실행기 메뉴 5)은
`infra/autonomy-web/launcher.py`를 호출한다. 두 명령 모두 저장소 `.venv` Python과 Git을 사용하며
추가 Python 패키지는 필요 없다.
`origin/feature/tello-ros-driver`의 `modules/autonomy/autonomy_web` 코드를 임시 폴더에서 실행한다.
실행 전에 `git fetch origin feature/tello-ros-driver`로 팀원 코드를 갱신한다.
다른 브랜치로 전환하거나 팀원의 전체 코드를 병합하지 않는다. 종료 시 임시 소스는 정리한다.

## Docker

Docker Desktop의 Linux 엔진을 켜고 `scripts\start.cmd team-autonomy docker`를 실행한다.
최초 실행은 ROS 2 Jazzy·OpenCV 이미지 다운로드와 빌드에 인터넷과 시간이 필요하다.
이미지 `ssafy-autonomy-web:jazzy`는 다음 실행에도 사용한다. 컨테이너는 종료 시 제거된다.
기본 주소는 `http://localhost:8080`이며 서버 응답 후 브라우저가 열린다.

기본 Docker 브리지 네트워크는 다른 WSL/호스트의 ROS 노드를 자동으로 연결하지 않는다.
호스트의 ROS 노드와 연결하려면 Docker Desktop에서 host networking을 활성화한 뒤
`scripts\start.cmd team-autonomy docker --host-network --domain-id 0`을 사용한다.
실제 ROS 토픽 검색·통신은 네트워크 환경에 따라 별도 확인이 필요하다.

## WSL ROS 2

`Ubuntu-24.04` WSL 배포판에 ROS 2 Jazzy가 설치되어 있어야 한다.
환경이 없다면 먼저 PowerShell에서 `wsl --install -d Ubuntu-24.04`를 실행하고,
팀원 브랜치의 `simulation/tt-sim/scripts/setup_jazzy.sh` 설치 절차를 따른다.
WSL Ubuntu에서 웹 노드 의존성은 다음으로 설치할 수 있다.

```bash
sudo apt-get update
sudo apt-get install python3-opencv python3-numpy ros-jazzy-nav-msgs ros-jazzy-sensor-msgs
```

`scripts\start.cmd team-autonomy wsl`을 실행한다. 다른 배포판은 `--distro Ubuntu`로 지정한다.
Windows localhost 전달이 켜져 있어야 브라우저에서 접속할 수 있다.

## 공통 옵션과 동작

- `--port 8081`: 포트 변경
- `--domain-id 10`: 기존 ROS 노드와 동일한 ROS_DOMAIN_ID 사용
- `--no-browser`: 브라우저 없이 서버 실행
- `--ref <Git 참조>`: 실행할 로컬/원격 추적 브랜치 또는 커밋 지정
- `--help`: 옵션 안내

이 실행기는 **웹 ROS 노드만 실행**한다. 비행 드라이버·미션·Gazebo·깊이 추정은 별도로 실행한다.
이들 노드가 없으면 화면은 열리지만 드론 상태·영상·지도는 없다. CMD 실행만으로 이륙하지 않는다.
별도 비행 launch가 이미 웹 서버를 띄웠다면 같은 포트로 중복 실행하지 말고 그 서버 주소를 연다.
`Ctrl+C`는 웹 서버를 종료하며, 별도로 실행한 비행 노드를 종료하지 않는다.
