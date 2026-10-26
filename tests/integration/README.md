# 파트 간 연결 검증

계약 호환성·회차 격리·입력 누락/상실·재시작 복구 등의 실제 연결을 검증한다. 합성 예제와 모의/실제 환경을 구분한다.

최초 구현 시 지원 버전·설치·실행·테스트·설정 이름과 입출력 계약 링크를 이 문서에 추가한다.
팀 작업 절차는 저장소 루트의 docs/team/START_HERE.md를 따른다.

## 녹화 영상 → 실시간 SLAM

Python 3.12와 `modules/mapping/localization/requirements-online.txt` 의존성을 사용한다.
합성 MP4의 보정값·촬영 방향·HTTP 설정 응답 검사는 Docker 없이 실행한다.

```powershell
.\.venv\Scripts\python.exe -B -m unittest discover -s tests/integration -p test_live_demo.py -v
```

`replay_live_slam.py`는 명시적으로 실행하는 실제 Docker 종단 검사다. 자동 단위 테스트에서
실제 영상이나 Docker를 요구하지 않는다. 서버 준비와 실행 명령, 출력 보고서는
`modules/mapping/localization/ORB_SLAM3.md`의 **저장 결과에서 녹화 영상 전송 재현**을 따른다.
입출력 지도는 `contracts/mapping/ORB_SLAM3_V1.md` 계약을 사용한다.
