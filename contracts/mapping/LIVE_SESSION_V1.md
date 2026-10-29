# 실시간 녹화·세션 저장 1.0

생산자: `modules/mapping/data/live_recording.py`, 실시간 입력 어댑터.
소비자: `apps/console/`, 후속 세션 불러오기 구현.
영상·프레임·지도·전체 로그는 로컬 `output/phone_mapping/live_<id>/`에만 저장한다.

## 세션 묶음

루트의 `session.json`은 `schema_version=mapping-session/1.0`이며 다음 필드를 제공한다.

| 필드 | 의미 |
| --- | --- |
| `session_id`, `started_at`, `ended_at` | 회차 ID·UTC 시작/종료 시각. 종료 전 ended_at은 null |
| `input` | `tello_camera` 또는 `browser_camera` |
| `mapping_timestamp_zero_monotonic` | Tello 매핑 PTS 0에 대응하는 PC 단조 시각. 브라우저 입력은 null |
| `recording` | 저장 상태·큐 크기·바이트 수·구간 수·원본 H.264 여부·재생 파일 생성 오류 수 |
| `recordings` | 녹화 구간 목록. 재연결로 수신기가 다시 열리면 새 구간을 만든다 |
| `mapping` | 매핑 종료 상태·최초 종료 사유·오류·최신 지도 revision·보존된 지도 구간 |
| `assets` | 보정값·설정·최종 상태·native 지도·revision·키프레임·추적·진단의 상대 경로 |
| `asset_sha256` | 정상적으로 닫힌 뒤 읽을 수 있는 camera.json·settings.json·state.json의 SHA-256 |
| `scale`, `privacy` | `arbitrary`, `local_only` |

`assets`의 지도 관련 경로는 지도 초기화 전 종료 시 존재하지 않을 수 있다.
세션 정보만 저장된 정상 종료와 지도 생성 성공은 별개다.
manifest의 파일 경로는 세션 루트 기준이다. 기존 `local_assets.json` 및 state의 로컬 경로 힌트는
이전 PC의 절대 경로일 수 있으므로 후속 불러오기에서는 manifest의 키프레임 루트로 다시 연결해야 한다.
다른 map ID는 [ORB 계약](ORB_SLAM3_V1.md)에 따라 독립 좌표계로 유지한다.

## 원본과 수신 영상

- Tello 직접 연결: UDP 소켓에서 받은 H.264 payload를 디코딩·JPEG 변환 전에
  `recordings/part_0000.h264`에 순서대로 보존한다. 소켓에 도착하기 전의 무선 패킷 손실까지 복구한다는 뜻은 아니다.
- 게이트웨이·브라우저: 서버가 받은 JPEG를 `part_0000.mjpeg`에 그대로 이어 쓴다.
  이 입력은 `mode=received_jpeg`, `original_drone_bitstream=false`이며 원래 장치의 H.264 원본이 아니다.
- 구간마다 `.jsonl`에 `offset`, `size`, `timestamp_s`를 기록한다.
  `time_basis`는 PC UDP 수신 단조 시각, 게이트웨이 수신 단조 시각 또는 브라우저 media PTS다.
  원본은 SLAM 대기 큐에서 프레임을 생략하기 전에 저장한다.
- 구간 메타데이터는 원본 경로·인덱스 경로·실제 저장 바이트·SHA-256·첫/마지막 수신 시각을 담는다.
  직접 수신에서는 `receiver_bytes`, `receiver_packets`, `capture_complete`로 소켓 수신량과 저장량을 대조한다.
- 종료 시 인코딩된 패킷을 다시 인코딩하지 않고 H.264는 MP4, JPEG는 Matroska(MKV)로 묶는다.
  `playback`에는 경로·SHA-256·프레임 수·해상도·시각 기준·재인코딩 여부가 있다.
  H.264에는 원래 촬영 PTS가 없으므로 MP4 시각은 PC 패킷 수신 시각의 근사치다.
  키프레임 전 패킷은 재생 파일에서 생략할 수 있으며 원본에는 남는다.
  B-frame 순서에 필요한 촬영 PTS가 없거나 변환이 실패하면 원본을 보존하고 `playback_error`를 기록한다.

## 부하·오류·종료

수신 스레드는 파일을 쓰지 않고 전용 작업 큐에 넣는다. 8MiB 또는 대기 작업 8,192개에 도달하면
영상 데이터 수락을 중단한다. 구간 닫기·종료를 위한 소수의 제어 작업은 계속 수락한다.
세션당 녹화 구간 한도는 64개다. 큐 초과 시 이후 녹화를 중단하고 이미 받은 연속된 앞부분을 보존한다.
저장 장치 오류도 매핑 중단 사유로 전달하지 않으며 웹의 녹화 상태에 별도로 표시한다.
녹화 중단 뒤의 전체 수신량은 `bytes_received`에 계속 누적되지 않는다.
직접 연결의 구간별 `receiver_bytes`가 닫힌 소켓의 최종 수신량이다.

`recording.status`는 `waiting`, `recording`, `finalizing`, `complete`, `partial`이다.
오류가 있으면 정리 중에도 partial일 수 있으므로 내려받기 준비 여부는 `finalized`도 확인한다.
Tello 입력 종료와 매핑 종료가 모두 확인된 후 파일을 닫고 재생 파일·최종 manifest를 만든다.
두 단계의 종료 순서가 바뀌어도 아직 도착하는 원본을 미리 닫지 않는다.
강제 프로세스 종료 시에는 남은 원본 앞부분과 `finalized=false` manifest가 보존되며,
완전하게 저장된 회차로 간주하지 않는다. 주기적인 버퍼 flush는 전원 장애에 대한 완전한 보장을 뜻하지 않는다.

## 로컬 HTTP

| 경로 | 동작 |
| --- | --- |
| `GET /api/live/session?session=ID` | 현재 또는 명시적으로 서버에 연결한 세션의 manifest |
| `GET /api/live/session-download?session=ID` | 매핑 종료·저장 finalized 후 세션 전체 ZIP 스트리밍 |

세션 ID 불일치는 409, 세션/녹화 정보 없음은 404, 저장 진행 중 ZIP 요청은 409다.
입력의 임의 파일 경로를 받지 않는다. ZIP은 프레임·영상·지도·진단을 포함하고
미완료 `.tmp` 파일과 심볼릭 링크/정션을 제외한다. 별도 ZIP 사본을 디스크에 만들지 않는다.
manifest 파일 쓰기가 실패해도 살아 있는 서버는 메모리의 최신 진단 manifest를 ZIP에 넣을 수 있다.
반출 ZIP도 촬영 자료를 포함하는 비공개 자료이며 자동 업로드하지 않는다.

이번 버전은 저장·내려받기만 추가한다. ZIP/폴더 불러오기 화면·보정 프로필·초기화 튜토리얼은 후속 단계다.
