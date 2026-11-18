# 휴대폰 확정 입력 snapshot 1.0

생산자: 지도 데이터 담당. 소비자: 지도 복원 담당 및 좌표 입력 기능 개발자.
이는 기존 휴대폰 실험을 보존하는 형식이다. `POC_V0_1.md`의 공통 교환 형식으로 변환됐다는 뜻은 아니다.

## 경로와 버전

- `run_manifest.json`의 `schema_version=phone-mapping-snapshot/1.0`.
- 기존 `input_run_id`, `feature_run_id`, `annotation_run_id`를 유지한다. UUID나 `frame-000001`로 자동 치환하지 않는다.
- `files`는 논리 이름 → snapshot 내부 POSIX 상대 경로·SHA-256·크기다. `package_path`는 기존 외부 패키지 루트 기준 경로다.
- `external_artifacts.json`은 기존 패키지 ID·ZIP/manifest 해시·외부 경로를 명시한다. Drive 위치 미상은 null로 기록한다.
- `load_run(run_dir, package_root=None)`의 `paths`로 현재 Git 파일을, `package_paths`로 현재 외부 파일을 읽는다.
- 복사한 생산자 JSON 내부의 상대 경로와 오래된 manifest 해시는 실행 당시 출처다. 이 snapshot 폴더 기준 경로로 사용하지 않는다. 원본 JSON은 바이트/해시 보존을 위해 고치지 않았다.

## 좌표와 대응

- `keypoints.csv`: run_id, image_id, frame_id, source_pts_s, keypoint_id, x_px, y_px.
- frame_id는 기존 `frame_0035.jpg` 같은 원본 이름이며 image_id는 DB ID다. 서로 대체하지 않는다.
- keypoint_id는 해당 이미지 DB 특징점/기술자의 0 기반 행 번호다. 여러 이미지의 같은 번호가 같은 점을 뜻하지 않는다.
- 좌표는 원본 1920×1080, 왼쪽 위 기준 x 오른쪽/y 아래, 첫 픽셀 중심 `(0.5,0.5)`다. source_pts_s는 초 단위 원본 영상 PTS다.
- `matches.csv`: frame_id_a, keypoint_id_a, frame_id_b, keypoint_id_b, geometric_inlier. 플래그는 0/1이다.
- 기하 검증 대응도 물리적 정답이나 복원 성공을 보장하지 않는다. 후보 트랙 ID는 3D 점 ID가 아니다.
- 측정 JSON/CSV의 A/B/C는 자동 SIFT ID와 별개다. 수동 원본은 정수 픽셀 중심, 내보낸 관측은 +0.5 표기다.
- 44cm는 스케일 기준이다. 136cm는 바닥 C 동일점 미확인으로 `correspondence_ready=false`이며 모든 C의 metric 사용은 비활성 상태다.

## 검사와 외부 의존

`shared_inputs.py`의 기본 검사는 표준 라이브러리만 사용한다. 해시, 회차/프레임/점 참조,
좌표·PTS·매칭 수, 수동 관측·상태/사용 제한을 검사한다. 사진 없이 입력 소비 코드를 개발할 수 있다.

`--package-root`는 외부 manifest, DB·영상·참조 사진, 선택 프레임과 Git 파일의 바이트 일치를 검사한다.
`--native`는 PyCOLMAP 4.2.1 등의 기존 의존성을 추가로 사용해 **DB 사본**으로 기존 수신 검사를 수행한다.
김수호 환경의 검사나 실제 복원 성공을 대신하지 않는다.

`--workspace <새 경로>`는 외부 패키지를 검증한 뒤 새 작업 사본을 만든다. 기존 대상 경로를 덮어쓰지 않는다.
입력 snapshot과 전달 패키지는 보존하고 기존 추출/주석 도구는 작업 사본에서 실행한다.
CSV에는 SIFT 기술자와 전체 두 시점 기하 정보가 없어 기존 COLMAP DB의 대체물이 아니다.

누락/해시 불일치, 잘못된 점 참조, 경로 이탈, 다른 패키지/회차/버전, C의 임의 활성화는 오류로 종료한다.
공통 PoC 형식 연결 시에는 별도 adapter에서 ID·초→마이크로초·좌표 관례·카메라 모델을 명시적으로 변환해야 한다.
