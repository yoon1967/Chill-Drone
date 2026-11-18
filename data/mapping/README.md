# 지도 작업의 확정 공유 입력

사용자가 Git 기반 협업을 위해 공유를 승인한 실제 **수치 입력·설정·검증 기록**을 보관한다.
영상·사진·그림·COLMAP DB·ZIP·전체 로그는 이 폴더에 추가하지 않는다.

- 분야/입력 종류/촬영 회차/처리 버전으로 구분한다. 현재 `phone_video/2f1_20261021/`이 첫 확정 입력이다.
- 확정 버전은 불변이다. 재처리 결과는 새 버전/새 snapshot으로 제안하고 기존 ID와 좌표를 덮어쓰지 않는다.
- 생산자 입력은 mapping/data, 복원 코드는 mapping/localization이 소유한다. 새로운 복원 결과의 메타데이터는 회차별 `reconstruction/<버전>/`에 별도 MR로 추가한다. 아직 실제 복원 결과는 없다.
- 원본·DB가 필요한 작업은 `external_artifacts.json`으로 비공개 자료를 식별한다. Git에 인증 정보나 비밀 다운로드 URL을 적지 않는다.
- 숫자 입력도 실제 촬영에서 유래한다. GitHub 공개 전 자료와 이력을 별도 점검한다.

형식: `contracts/mapping/PHONE_INPUT_SNAPSHOT_V1.md`. 현재 회차는 기존 phone 형식이며 공통 PoC 0.1.0과 같지 않다.
