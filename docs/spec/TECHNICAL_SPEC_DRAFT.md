# 통합 기술명세서 초안

작업: [S15P31B205-30](https://ssafy.atlassian.net/browse/S15P31B205-30).
버전: 0.2 / 2026-10-22. 사용자 요청에 따라 Jira를 바탕으로 초기 기본값을 먼저 정한 통합 초안이다.
인프라는 현행 설정·실측, 나머지는 현재 요구와 선행 설계다. 아래 기본값으로 병렬 개발을 시작한다.
이 문서의 완료는 초안 작성·검증·develop 반영이며, 파트별 최종 합의나 전체 앱 구현을 완료했다는 의미가 아니다.

## 0. 범위와 문서 원본

- 이번 주: 휴대폰 녹화 RGB의 로컬 희소 3D 복원, 카메라 경로, 스케일/품질 기록과 미리보기.
- 후속 웹: 로컬에서 만든 작은 결과를 인증 후 저장·조회·시각화. EC2에서 영상 복원·학습을 실행하지 않는다.
- 병렬 트랙: 사람 탐지·이벤트, ROS 2/Gazebo·제어를 각각 독립 개발하고 동일 회차/시각/출처로 연결한다.
- 실시간 스트리밍·실기체·밀집 메시·공개 가입·외부 대용량 저장소는 후속 확장으로 둔다.

흐름·책임은 [아키텍처 초안](../architecture/SYSTEM_DRAFT.md), 필드·파일·API 원본은
[PoC 연결 초안 0.1.0](../../contracts/POC_V0_1.md)이다. 중복 정의가 달라지면 계약을 우선 갱신한다.
[합성 결과 묶음](../../data/samples/poc-v0.1/README.md)으로 화면/수신기를 먼저 만들 수 있다.

## 인프라 명세

| 항목 | 현재 구성·근거 |
| --- | --- |
| 제공 자원 | EC2 t3.xlarge, ap-northeast-2, Ubuntu 24.04.4 x86_64, 4 논리 CPU, OS 메모리 15.41 GiB, 루트 약 308.93 GiB |
| 실행 도구 | 조사 시 Docker 29.9.2·Compose 5.6.1·Runner 19.5.0. 앱/CI Python 3.13 이미지 digest는 저장소 파일 기준 |
| 현재 서비스 | Caddy + 상태 웹 2개. 실제 BE/FE·DB는 아직 미배포 |
| 외부 경로 | HTTPS 443 상태 화면/API, 승인된 키의 SSH 22 관리 접근 |
| 내부 경로 | Caddy → 127.0.0.1:18065 → 웹 18066, 상태 폴더 읽기 전용 |
| CI/CD | MR 검증, 보호된 develop의 SHA 이미지 빌드·Compose 배포·HTTPS/건강 검사 |
| 실패 복구 | 이전 앱 이미지·Compose 복구, 배포는 실패로 기록. DB/서버 전체 복원은 별도 |
| 데이터 | 합성/공개 가능한 작은 예제만 Git, 원본·모델·큰 결과는 로컬/비공개 회차 저장 |
| 운영 한계 | 단일 인스턴스·CPU 크레딧·서버 종료일/예산·실제 백업 목표 미정 |

상태 화면의 정상·degraded는 서버 운영 상태이고 3D 복원·탐지 정확도를 뜻하지 않는다.
Jira 완료는 develop 머지, 실제 배포 성공은 파이프라인으로 별도 판단한다.
상태 데이터는 3초 간격으로 수집/조회하고 15초 넘는 지연은 정상으로 표시하지 않는다.
선택된 최근 이벤트 20건을 유지하며 재부팅 때 초기화된다.

## 1. 분야별 요구와 초기 구성

| 분야 | Jira에서 확인한 흐름 | 먼저 사용할 설계 기본값·구현 인계 |
| --- | --- | --- |
| 지도/좌표 | 휴대폰 RGB → SfM 점군·카메라 경로 → 스케일·품질 평가 | COLMAP 희소 복원, local CLI, JSON pose·PLY 결과. 임의 단위와 실측 m 분리 |
| 지도 데이터 | 원본 PTS·frame_id·run_id, 매칭/실측 관측, 회차별 저장·미리보기 | JSON manifest/결과 + CSV 2D 관측/매칭, COLMAP DB는 파생물. 계약 0.1.0 기준 |
| 탐지 | YOLO 사람 탐지·조건별 평가, 추론/학습 분리 | 기존 탐지 트랙의 기준 모델을 로컬 실행, 모델 버전/가중치 해시 기록. events.jsonl로 전달 |
| 시뮬레이션/제어 | WSL2 Ubuntu 24.04·ROS 2 Jazzy·Gazebo Harmonic, TT 어댑터 | 해당 환경과 별도 ROS 패키지 유지. 정답 pose/추정 pose 분리, 실기체 어댑터는 후속 |
| BE | API·오류 정책·분야 통합 검토 이슈 존재 | Python 3.13 + FastAPI/Pydantic 2 + SQLite, 단일 프로세스의 결과 수신·조회 |
| FE/화면 | 결과 확인 화면 폴더 준비 | React 19 + TypeScript + Vite 8 + Three.js PLYLoader. 목록/상세/점군·경로·단위·이벤트 표시 |
| Data 기술명세 | 완료 이슈 존재, 최종 원문은 미확보 | 현재 휴대폰 데이터 계약을 본 초안으로 취합. 실제 원문이 확보되면 차이를 갱신 |
| AI 기술명세 | 작성·검토 이슈 존재, 최종 원문은 미확보 | 추론·학습·평가 분리와 이벤트 경계를 취합. 특정 성능/모델 라이선스 허용을 가정하지 않음 |

완료 상태만으로 실제 문서·코드·버전이 확보됐다고 보지 않는다.
Confluence 통합 검색은 접근 오류가 있었고, 대상 검색도 사이트 이용 불가 응답이었다.
이번에 읽은 BE·Data 이슈에는 산출물 본문/첨부 링크가 없었다. 접근 제한을 우회하지 않았다.

## 2. 공통 데이터·좌표·시간

- 회차 식별자·원본 체크섬·버전과 프레임 식별자/원본 PTS.
- 원본 해상도/회전·카메라 정보 출처와 좌표 변환 방향.
- 임의 스케일/실측 보정 단위, 정답/추정/미제공 위치의 구분.
- 탐지 클래스·신뢰도·촬영 시각·중복 이벤트·객체/촬영 위치 구분.
- 지도·카메라 경로·PNG·메타데이터 묶음과 부분/손상/재시도 처리.
- BE 업로드/조회·인증·용량 제한·오류와 FE 표시 방식.

필드명·타입·API 경로·오류는 연결 초안 0.1.0에 먼저 정했다.
각 파트는 합성 예제로 독립 개발하고 실제 예제의 상호 수신 결과는 구현 작업에 남긴다.
원본 촬영·얼굴·실내 구조·키·토큰은 통합 명세의 예제에 복사하지 않는다.

카메라 변환은 world→camera 행 우선 4×4, 점군은 COLMAP 임의 world를 사용한다.
카메라 중심과 사람 위치를 구분하며 미제공 pose/위치는 null로 보존한다.
회차를 재처리하거나 축척을 바꾸면 새 run_id로 저장한다. 다른 회차의 프레임·pose·이벤트를 섞지 않는다.

## 3. 환경·의존성·폴더

| 실행 단위 | 초기 목표 환경 | 구현 시 고정할 원본 |
| --- | --- | --- |
| 지도·데이터 로컬 | 기존 파트 환경 우선, 신규 기준안 WSL2 Ubuntu 24.04/Python 3.12 + COLMAP CLI | 파트 README, COLMAP 실제 버전/빌드·옵션·Python lockfile |
| 탐지·학습 로컬 | 현재 기준 모델 환경 유지, GPU가 필요한 학습은 별도 PC | 모델/패키지/CUDA 실제 버전, weights 해시·평가 입력/지표 |
| API | Linux 컨테이너/Python 3.13, FastAPI/Pydantic 2, Uvicorn 1 worker, Python 제공 SQLite | services/api/ lockfile·이미지 digest·실제 sqlite_version |
| 화면 개발/빌드 | Node 22 계열(22.12 이상), React 19, TypeScript, Vite 8, Three.js | apps/console/package-lock.json, Node 이미지 digest |
| 화면 운영 | 빌드한 정적 파일을 Caddy가 /console/에서 제공 | 정적 파일 이미지/빌드 SHA, 같은 origin의 /api/v1/ |
| 시뮬레이션 | ROS 2 Jazzy/Gazebo Harmonic, 지도·API와 별도 환경 | simulation/README, 패키지·토픽·어댑터 버전 |

표의 신규 앱 환경은 설치·실행을 확인한 버전 목록이 아니다. 현재 확인된 인프라 버전은 위 인프라 표를 따른다.
FastAPI/Three.js 등 정확한 패치·전이 의존성은 최초 구현에서 호환 검증 후 lockfile로 고정하고
이 문서에 기록한다. 문서 작업을 위해 전 팀원의 도구를 설치하거나 최신 버전을 무조건 적용하지 않는다.
Python 3.12 지도 환경과 Python 3.13 API/CI 환경을 독립 유지하며 양쪽 호환 검증을 했다고 가정하지 않는다.

파트 경계는 `modules/mapping/`, `modules/perception/`, `modules/autonomy/`, `simulation/`,
`services/api/`, `apps/console/`, `contracts/`를 따른다. 실제 담당은 Jira의 현재 배분을 유지한다.
화면 구현 담당은 아직 배정되지 않았으며 기존 팀원에게 자동으로 UI 업무를 추가하지 않는다.

## 4. BE 저장 구조·API

SQLite는 메타데이터를 저장하고 PLY/JSON 파일은 별도 전용 볼륨에 둔다. 초기에 단일 API writer로 운영한다.
복원 작업 큐·Redis·별도 DB 컨테이너는 초기 구성에서 요구하지 않는다.

| 테이블 | 초기 키·주요 필드 |
| --- | --- |
| runs | run_id PK, parent_run_id nullable, schema_version, input_kind, source_sha256 nullable, manifest_sha256, status, coordinate_frame, scale_status/unit/factor, result_json, received_at_utc |
| artifacts | (run_id,artifact_id) PK/FK, 내부 상대 저장 키, media_type, sha256, bytes |
| detection_events | (run_id,event_id) PK/FK, frame_id, pts_us, event_json, received_at_utc |

`result_json`에 프레임·카메라 변환·quality·producer를 보존해 초기 대량 pose 정규화를 피한다.
이는 초안 ERD이며 기존 BE의 실제 ERD를 확보하면 영향·마이그레이션을 기록해 맞춘다.
manifest_sha256은 업로드한 manifest 파일 바이트의 해시다. 다운로드 href는 조회 응답에서 서버가 생성한다.

API는 연결 초안의 `/api/v1/runs` 생성/목록/상세, pointcloud 다운로드, events 입력/목록을 따른다.
기본 제한은 요청 50 MiB·JSON 5 MiB·100,000점·저장 총량 1 GiB, 중복 run_id 동일 내용은 멱등 수용,
다른 내용은 409다. 형식·해시 오류는 422, 저장 한도는 507로 응답한다.
파일·DB 간 확정 순서와 실패 정리로 미완성 회차를 목록에 정상 결과로 노출하지 않는다.
잘못된 입력은 기존 회차를 덮어쓰지 않으며 자동 재복원을 시작하지 않는다.

## 5. 화면·인증·향후 배포

화면 첫 구현은 합성 파일 입력으로 회차 목록·상세·점군·카메라 경로·상태·단위를 표시한다.
Three.js PLYLoader로 점군 geometry를 읽고 Points로 표현하며, pose의 역변환으로 카메라 중심을 구한다.
world 방향을 실제 중력/북쪽으로 가정하지 않는다. 보정 전 m 표시를 막고 partial/unregistered를 명시한다.
이후 같은 결과를 API에서 읽도록 교체한다. 사용자의 수동 갱신과 5초 조회를 기본으로 둔다.
대상은 최신 데스크톱 Chrome/Edge이며 모바일·큰 점군 성능은 후속 실제 측정 대상으로 둔다.

후속 배포 초안: Caddy 443 → `/console/` 읽기 전용 정적 파일, `/api/v1/` →
호스트 loopback 18067 → API 컨테이너 18167, 기존 상태 서비스 → 호스트 loopback 18065.
현행 Caddy의 host network를 유지할 수 있도록 API도 127.0.0.1에만 publish하고 외부 직접 연결을 막는다.
API·console은 HTTPS/Caddy의 개인별 Basic 인증을 적용하는 설계다. 계정은 배포 구현 때 관리자가 공급한다.
SQLite와 파일 볼륨은 API만 쓰고 Caddy에서 직접 파일 디렉터리를 공개하지 않는다.
현재 Caddy/Compose/UFW·계정·공개 경로는 이번 문서 작업에서 변경하지 않는다.

후속 앱 CI는 해당 lockfile의 검사·빌드와 계약 예제 수신을 검증한 뒤 SHA 이미지로 배포한다.
API/DB 스키마가 생기면 앱 이미지 롤백만으로 DB가 복구되지 않으므로 변경 전 백업·호환 마이그레이션을 준비한다.
현행 상태 서비스의 파이프라인·실패 복구는 그대로 운영 문서를 따른다.

## 6. 보존·백업·공개 기본안

- 원본 영상·전체 점군·모델은 생산자 로컬 비공개 보관, Git에는 작은 합성 예제와 코드·명세만 둔다.
- 웹 회차 저장 상한은 1 GiB. 상한 초과는 신규 수신을 거부하고 기존 데이터를 자동 삭제하지 않는다.
- 초기 보존 목표는 프로젝트 종료까지다. 정리가 필요하면 소유자가 대상·백업을 확인하고 별도 수행한다.
- 웹 운영을 시작할 때 SQLite backup API로 일관된 DB 사본과 같은 시점의 artifact 목록/해시를 보관한다.
  목표 RPO 24시간·RTO 4시간은 초안 목표이며 실제 보장/복구 시험 결과가 아니다.
- EC2 밖의 승인된 비공개 백업 목적지는 아직 확보하지 않았다. 같은 서버 내 복사만으로 서버 상실에 대비했다고 보지 않는다.
- 오픈소스는 코드·문서·직접 작성한 합성 예제부터 준비한다. 라이선스 선택과 모델/데이터 이용 조건 확인 전에는
  원본·가중치·기존 결과를 GitHub로 공개하지 않는다. 공개·미러 연결은 기존 공개 전환 절차를 따른다.

## 7. 검증과 구현 순서

| 단계 | 확인할 결과 | 이번 문서 작업의 상태 |
| --- | --- | --- |
| 초안 | 출처·실측/제안 구분, 파일 경로·계약·합성 해시/좌표·로컬 검사 | 수행 결과를 MR/Jira에 기록 |
| 지도 연결 | 실제 영상 1회차 manifest 수신→복원→결과 재로딩, 회전/PTS/축척·누락 pose | 기존 지도 구현 작업에서 수행 |
| 탐지 연결 | 같은 run/frame/PTS 이벤트, 중복·잘못된 bbox·위치 null | 탐지/연결 구현에서 수행 |
| 웹 연결 | 합성 결과 업로드→목록/상세/점군, 401/409/413/422/507, 경로 격리·회차 혼입 차단 | API/화면 구현에서 수행 |
| 운영 | 배포·앱/DB 호환 롤백, 백업 복구·목적지, 실제 부하/브라우저 측정 | 해당 기능 배포 때 수행 |

사용자 요청으로 초안 기본값·근거·미확인 부분을 먼저 작성하고 검증 후 develop에 반영한다.
파트별 합의·최종 원문 확보·실제 종단 구현을 이 초안의 선행 승인 단계로 두지 않는다.
이후 확보한 원문/실행 결과와 차이가 있으면 버전·변경 근거를 갱신한다.
초안 완료를 실제 모델 성능·전체 앱 통합·실기체 비행·DB/서버 복원 완료로 보고하지 않는다.

## 근거와 관련 문서

- [자원·제약](../infra/RESOURCE_CONSTRAINTS.md), [환경](../infra/ENVIRONMENTS.md),
  [스택 비교](../infra/STACK_DECISIONS.md), [접근 정책](../infra/NETWORK_ACCESS.md)
- [Vite 8·Node 요구사항](https://vite.dev/blog/announcing-vite8), [React 버전](https://react.dev/versions),
  [Three.js PLYLoader](https://threejs.org/docs/pages/PLYLoader.html)
- [FastAPI 버전 고정](https://fastapi.tiangolo.com/deployment/versions/),
  [SQLite 용도·동시성 고려](https://www.sqlite.org/whentouse.html)

- [운영·배포·실패 복구](../EC2_CICD.md), [폴더/계약 경계](../team/REPOSITORY_STRUCTURE.md)
- [현행 아키텍처 초안 작업](https://ssafy.atlassian.net/browse/S15P31B205-9)
- [지도 트랙 A](https://ssafy.atlassian.net/browse/S15P31B205-213), [트랙 B](https://ssafy.atlassian.net/browse/S15P31B205-214)
- [입력·좌표·스케일 계약](https://ssafy.atlassian.net/browse/S15P31B205-215)
- [2D 관측](https://ssafy.atlassian.net/browse/S15P31B205-253), [실측 전달 검증](https://ssafy.atlassian.net/browse/S15P31B205-254)
- [탐지 평가](https://ssafy.atlassian.net/browse/S15P31B205-250), [시뮬레이션 환경](https://ssafy.atlassian.net/browse/S15P31B205-201)
- [BE API](https://ssafy.atlassian.net/browse/S15P31B205-42), [BE 통합 검토](https://ssafy.atlassian.net/browse/S15P31B205-55)
- [AI 기술명세](https://ssafy.atlassian.net/browse/S15P31B205-105), [Data 기술명세](https://ssafy.atlassian.net/browse/S15P31B205-130)
