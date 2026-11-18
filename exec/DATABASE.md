# DB 덤프 제출 여부와 기록 복원

[빌드·배포 포팅 매뉴얼](BUILD_AND_DEPLOY.md)의 최종 통합 커밋 기준입니다.

## 운영 DB 덤프

**제출 대상 없음.** 최종 관제는 Windows·WSL의 회차 폴더에 기록하고, EC2의 `infra/app.compose.yml`은 `status` 컨테이너만 실행합니다. `services/api/`에는 구현 공간을 설명하는 README가 있으며, 최종 실행에 필요한 DB 서버, 접속 계정, 접속 문자열, 운영 ERD 또는 마이그레이션은 정의되어 있지 않습니다. EC2 상태 페이지는 `/run/status/runtime.json`의 상태 스냅샷을 읽습니다.

[EC2 Compose](https://lab.ssafy.com/s15-final/S15P31B205/-/blob/411e36e85cf6144ffab8573bd34f326d7a5de1fd/infra/app.compose.yml) · [서버 API 현황](https://lab.ssafy.com/s15-final/S15P31B205/-/blob/411e36e85cf6144ffab8573bd34f326d7a5de1fd/services/api/README.md) · [상태 서비스 구현](https://lab.ssafy.com/s15-final/S15P31B205/-/blob/411e36e85cf6144ffab8573bd34f326d7a5de1fd/infra/status/server.py)

## 관제·지도 결과를 옮기는 단위

| 자료 | 저장 위치와 구성 | 확인·재현 방법 |
| --- | --- | --- |
| 관제 비행 회차 | Windows `data/outputs/tello/<회차>/`. `manifest.json`, `events.jsonl`, `sensor.jsonl`, `detections.jsonl`, 발견 사진, 저장된 지도 및 자율 비행 기록 | 공유가 허용된 회차 폴더 전체를 원래 구조로 배치하고 `--replay`로 엽니다. 영상은 회차에 저장된 경우 재생됩니다. |
| 3D 지도 촬영 세션 | Windows `output/phone_mapping/live_<id>/`. `session.json`, `settings.json`, 센서·프레임 기록, 영상·지도 산출물 | 지도 세션을 정상 종료해 `finalized` 상태를 확인하고 해당 회차를 묶음 ZIP으로 내보냅니다. 지원하는 드론 영상·센서 세션 ZIP은 **지도 콘솔 → 지도 복원**의 상단 드롭 영역으로 가져옵니다. |
| WSL 자율 비행 회차 | `~/ttrepo/data/outputs/flights/<시각>/`. `people.json`, `discovered.json`, `camera_map.npz`, `viewer.html` 등 | 종료 후 생성된 폴더의 구조를 유지합니다. 실기체 자율 비행 영상은 관제의 `ros_…` 회차와 별도로 WSL 쪽 기록을 확인합니다. |
| 공개 가능한 예제 지도 | `data/samples/poc-v0.1/result.json`, `points.ply` | 소스 클론으로 내려받습니다. 비행 회차 대신 예제 점군·메타데이터로 데이터 구조를 확인할 수 있습니다. |

게이트웨이를 종료해 포트를 비운 뒤 저장소 루트에서 회차 폴더명을 실제 값으로 바꾸어 실행합니다.

```powershell
$runDir = 'data/outputs/tello/회차_폴더명'
.\.venv\Scripts\python.exe -B modules\autonomy\tello_link\gateway_server.py --replay $runDir --replay-loop
```

`http://127.0.0.1:8010/#/live`에 **기록 재생**이 표시됩니다. 원본 회차 폴더는 읽기용으로 사용하며, 기록 재생에서는 기체 명령을 실행할 수 없습니다.

[관제 기록 재생](https://lab.ssafy.com/s15-final/S15P31B205/-/blob/411e36e85cf6144ffab8573bd34f326d7a5de1fd/modules/autonomy/tello_link/README.md#기록-재생---replay) · [지도 세션 파일 계약](https://lab.ssafy.com/s15-final/S15P31B205/-/blob/411e36e85cf6144ffab8573bd34f326d7a5de1fd/contracts/mapping/LIVE_SESSION_V1.md)

## 선택형 COLMAP SQLite 파일

휴대폰 영상의 오프라인 SfM 경로에서 PyCOLMAP **4.2.1**이 특징점·SIFT 기술자·매칭·카메라 정보를 `database.db`에 저장합니다. 관제나 EC2 실행에는 필요하지 않습니다. 저장소에 기록된 전달본은 `2f1_20261021_handoff_v1.zip`이며, 내부 DB 위치는 `output/phone_mapping/2f1_20261021_features_v1/full/database.db`입니다. 이 SQLite 파일과 원본 촬영 영상은 팀 내부 인계 자산이며 제출 브랜치에는 파일 식별 정보와 절차를 기록했습니다.

1. 이 오프라인 경로를 검증하는 경우 공유 권한이 있는 팀 전달본을 받습니다. 아래 매니페스트의 ZIP 파일명과 SHA-256을 `Get-FileHash -Algorithm SHA256` 결과에 대조합니다.
2. ZIP을 풀어 `handoff_manifest.json`이 있는 패키지 루트에서 설치·수신 검사를 진행합니다. 기본 검사는 파일과 체크섬·SQLite 무결성을 확인하고, `--native`는 PyCOLMAP 4.2.1로 복사본의 네이티브 읽기를 검증합니다.
3. SfM을 이어서 실행할 때는 DB와 동일한 패키지의 이미지 묶음을 함께 사용합니다. 특징점 인덱스는 생산 회차의 DB와 이미지에 종속됩니다.

[팀 전달본 파일명과 SHA-256](https://lab.ssafy.com/s15-final/S15P31B205/-/blob/411e36e85cf6144ffab8573bd34f326d7a5de1fd/data/mapping/phone_video/2f1_20261021/external_artifacts.json) · [수신·검증 절차](https://lab.ssafy.com/s15-final/S15P31B205/-/blob/411e36e85cf6144ffab8573bd34f326d7a5de1fd/data/mapping/phone_video/2f1_20261021/README.md) · [특징점 DB 생성 코드](https://lab.ssafy.com/s15-final/S15P31B205/-/blob/411e36e85cf6144ffab8573bd34f326d7a5de1fd/modules/mapping/data/phone_mapping/full_features.py)
