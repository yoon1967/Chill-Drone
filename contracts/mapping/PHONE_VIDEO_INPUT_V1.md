# 새 동영상 프레임 입력 1.0

생산자: `modules/mapping/data/video_input.py`. 준비 중인 파일 기반 매핑 실행 흐름의 입력이다.
`schema_version=phone-video-input/1.0`. 기존 31장 확정 snapshot과 별도 형식이며,
특징점 추출·SfM 소비자의 연결은 이후 구현 단계다.

## 파일

| 파일 | 내용 |
| --- | --- |
| `manifest.json` | 입력 영상 해시·스트림 정보·설정·추출 프레임·처리 상태 |
| `frames/*.jpg` | 표시 회전을 적용하고 선택적으로 축소한 특징점 입력 이미지 |
| `frame_decisions.jsonl` | 디코딩한 모든 프레임의 시각·선택 여부·선택/제외 이유 |
| `status.json` | `extracting`, `frames_ready`, `failed`. `stage=video_input` |
| `timing.json` | 이번 입력 준비 단계의 시간과 측정 범위 |
| `execution.log` | 최종 상태·실패 단계·경과 시간 |
| `local_source.json` | 원본 영상의 로컬 절대 경로. 로컬 전용이며 공유/HTTP 제공 제외 |

영상·프레임·DB·로그 등 실제 자료와 생성물은 Git에 올리지 않는다. 저장소 내부 출력은
ignore된 `output/`만 허용한다. 다른 위치의 새 출력 폴더도 인자로 지정할 수 있다.
기존 출력은 빈 폴더여도 덮어쓰지 않는다. 원본 영상은 읽기만 하고 전후 해시를 확인한다.

## 시각과 선택

- `run_id`는 회차 ID다. 자동 ID의 시각과 `created_utc`는 처리 시작 시각이며 촬영 시각이 아니다.
- 첫 비디오 스트림이 유일할 때 사용한다. 오디오가 함께 있는 것은 허용한다.
- 모든 프레임을 순서대로 디코딩한다. 처리 지연에 따른 프레임/작업 교체는 없다.
- 기본 2fps의 시간 격자에서 해당 경계 이후 첫 프레임을 선택한다. 같은 격자 내 나머지는
  `sampling_interval`로 기록한다. 시차나 흐림을 평가한 품질 선택을 의미하지 않는다.
- `source_pts`와 `source_time_base`는 디코더의 원본 표현이다.
  `source_pts_s = source_pts * source_time_base`, 초 단위이며 음수 시작도 보존한다.
- `timestamp_s = source_pts_s - 첫 디코딩 프레임의 source_pts_s`로 샘플링한다.
  `timestamp_source=source_pts`. 프레임 번호/명목 FPS로 촬영 시각을 만들어내지 않는다.
- PTS 누락·역전·중복은 실패로 기록한다. VFR도 실제 PTS로 처리하며 `nominal_fps`는 참고 정보다.
- `decoded_frame_index`는 0부터 시작한다. `frame_id=frame_0000001.jpg` 등은 이 번호에 1을 더한
  디코딩 순서이며 DB 이미지/특징점 ID가 아니다. 파일 이름에는 선택하지 않은 프레임의 번호가 빠진다.

## 이미지와 좌표

각 선택 프레임에 상대 `path`, JPEG 바이트의 `sha256`·크기, `width/height`, 원본 coded 크기,
표시 크기와 `rotation_applied_ccw_degrees`, `coded_to_processed_pixel_centers`를 기록한다.
좌표 공간은 저장한 이미지(`processed_image`)다. 첫 픽셀 중심은 `(0.5,0.5)`, x 오른쪽/y 아래다.

디코더의 표시 행렬에서 0/90/180/270도 반시계 회전을 적용한다. `max_width=0`이면 회전 후 크기를
유지하고, 양수이면 그 가로 폭을 넘는 이미지만 비율을 유지해 축소한다. 반올림한 실제 가로·세로
비율을 행렬에 각각 반영한다. 행렬은 `[x_processed,y_processed,1] = M @ [x_coded,y_coded,1]`이다.
소비자는 저장 이미지 크기에 맞는 카메라를 사용하며 회전/리사이즈를 중복 적용하지 않는다.

미러링·표시 행렬의 추가 스케일/이동·직각 외 회전·비정방 픽셀은 전용 어댑터가 필요하므로 실패로
기록한다. 입력 도중 크기/회전이 바뀌면 같은 카메라 입력으로 섞지 않는다.
픽셀 종횡비 미상(None 또는 FFmpeg의 0/1)은 경고로 남기고 디코딩 픽셀을 정방으로 취급한다.
정규화한 `sample_aspect_ratio`는 null, 원래 표기는 `raw_sample_aspect_ratio`에 보존한다.

## 완료와 실패

`frames_ready`는 영상 디코딩과 최소 3장 선택·파일 저장·원본 해시 재확인 완료만 뜻한다.
특징점·3D 복원 성공 여부는 이후 단계에서 관리한다. 실측은 요구하지 않으며 `metric_reference=null`이다.

디코더 오류·손상 플래그·선언 프레임 수와 실제 디코딩 수 불일치·시각/회전 오류·선택 3장 미만은
`failed`다. 생성된 프레임/부분 manifest/로그는 남긴다. 잘못된 CLI 인자·없는 파일·기존 출력 등
작업 폴더 생성 전 오류는 stderr와 종료 코드로 보고한다. CLI 성공은 0, 실패는 2, 사용자 중단은 130이다.
디코더와 컨테이너 정보로 발견되지 않는 모든 손상을 검출한다는 의미는 아니다.

`timing.json`의 `elapsed_seconds`는 함수 진입부터 추출/원본 재확인까지이며 마지막 보고서 쓰기는
제외한다. `decode_transform_write_seconds`는 프레임 디코딩·변환·저장·프레임 해시 계산 구간이다.
전체 영상→3D 파이프라인 시간은 후속 실행 진입점이 별도로 기록한다.

PyAV의 [PTS 정의](https://pyav.basswood.io/docs/stable/api/frame.html)와
[표시 회전 정의](https://pyav.basswood.io/docs/stable/api/video.html)를 따른다. 실행 버전은 requirements-video.txt로 고정한다.
