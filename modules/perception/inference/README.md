# 영상 입력·사람 세그멘테이션

Tello/RoboMaster TT의 실시간 영상을 받아 YOLO26 인스턴스 세그멘테이션으로 사람만 추론한다.
학습 코드는 옆 `training`, 공용 형식은 `contracts/perception`에서 관리한다.
문의: 송승우·이수진.

## 설치와 모델

검증 환경은 Python 3.12, OpenCV 5.0.0, Ultralytics 8.4.189다.

```powershell
.\.venv\Scripts\python.exe -m pip install -r modules/perception/inference/requirements.txt
```

기본 모델 이름은 `yolo26n-seg.pt`다. 로컬에 없으면 Ultralytics가 최초 실행 시 공식 가중치를
내려받는다. `*.pt`는 Git에서 제외하며 가중치를 저장소에 커밋하지 않는다. 오프라인 장비에서는
미리 받은 파일을 `--tello-model C:\models\yolo26n-seg.pt`로 지정한다.

## 실행

저장소 루트에서 드론 탐지 화면만 실행한다.

```powershell
.\.venv\Scripts\python.exe apps/console/serve.py --tello-only --port 8790
```

안전 조종 패널을 함께 켜려면 `--tello-flight`를 추가한다. 실행 옵션 `--tello-confidence`는
0~1 신뢰도 기준이며 기본값은 0.25다. 모델은 COCO의 클래스 0 `person`만 요청한다.

`TelloPersonStream.stats()`의 `detection_event`에는 공통 계약
`person-detection-event/1.0`에 따른 최신 프레임 결과가 들어간다. 입력 세션·프레임·탐지 시각,
이미지 크기, 모델 정보와 모든 사람의 bbox·복수 polygon을 포함한다. 자세한 형식과 합성 예시는
`contracts/perception`을 참고한다.

기존 `detections`에는 사람별 `class_id`, `label`, `confidence`,
호환용 `xyxy`, 그리고 이미지 픽셀 좌표의 `polygons`가 들어간다. 현재 화면은 네모 박스를
그리지 않고 polygon 외곽선과 신뢰도만 표시한다. 가려진 신체를 복원하지 않으며 모델이
실제로 분할한 보이는 영역만 윤곽선으로 사용한다.

## 테스트

실기체 없이 가상 영상·탐지기·Tello 링크로 회귀 검사를 실행한다.

```powershell
.\.venv\Scripts\python.exe -B -m unittest modules.perception.inference.tests.test_detection_event modules.perception.inference.tests.test_tello_person_stream -v
```

실제 드론의 자세·가림·원거리·어두움·흔들림 조건과 처리 지연은 별도 실기 검증 대상이다.
팀 작업 절차는 저장소 루트의 `docs/team/START_HERE.md`를 따른다.

## IP Webcam 실내 시맨틱 세그멘테이션 실험

사람 인스턴스 탐지와 별개로, SegFormer-B0 ADE20K가 휴대폰 영상의 벽·바닥·천장·테이블·의자를
어떻게 픽셀 단위로 분류하는지 확인하는 실험 도구다. Android IP Webcam 앱에서 서버를 시작하고
휴대폰과 PC를 같은 네트워크에 연결한다. 앱 화면의 IPv4 주소가 `192.168.0.20:8080`이라면:

```powershell
.\.venv\Scripts\python.exe -m pip install -r modules/perception/inference/requirements.txt
.\.venv\Scripts\python.exe -m modules.perception.inference.ip_webcam_semantic `
  --source http://192.168.0.20:8080/video
```

처음 실행할 때 `nvidia/segformer-b0-finetuned-ade-512-512` 가중치를 내려받는다. 창에는 관심 영역의
반투명 마스크, 화면 점유율, 추론 시간과 처리 FPS가 표시된다. `q` 또는 `Esc`로 종료한다.

```powershell
# 표시 대상과 기준 조정
.\.venv\Scripts\python.exe -m modules.perception.inference.ip_webcam_semantic `
  --source http://192.168.0.20:8080/video `
  --classes wall,floor,ceiling,table,chair,door,window `
  --confidence 0.40 --device auto

# 드론·휴대폰 없이 순수 마스크 로직 검증
.\.venv\Scripts\python.exe -B -m unittest modules.perception.inference.tests.test_semantic_overlay -v
```

IP·포트·비밀번호를 코드나 문서에 커밋하지 않는다. 이 모델은 영역의 종류를 분류하지만 벽까지의
실제 거리를 제공하지 않으므로 충돌 방지 검증에는 깊이 추정이나 거리 센서가 별도로 필요하다.
