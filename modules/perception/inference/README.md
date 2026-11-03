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

사람 인스턴스 탐지와 별개로, SegFormer-B0 ADE20K가 휴대폰 영상의 벽·바닥·천장·테이블·의자·문·창문을
어떻게 픽셀 단위로 분류하는지 확인하는 실험 도구다. Android IP Webcam 앱에서 서버를 시작하고
휴대폰과 PC를 같은 네트워크에 연결한다. 앱 화면의 IPv4 주소가 `192.168.0.20:8080`이라면:

```powershell
.\.venv\Scripts\python.exe -m pip install -r modules/perception/inference/requirements.txt
.\.venv\Scripts\python.exe -m modules.perception.inference.ip_webcam_semantic `
  --source http://192.168.0.20:8080/video
```

처음 실행할 때 `nvidia/segformer-b0-finetuned-ade-512-512` 가중치를 내려받는다. 카메라 수신과 추론은
별도 스레드에서 동작하며 오래된 입력은 쌓지 않고 최신 프레임으로 교체한다. 창에는 관심 영역의 반투명
마스크, 화면 점유율, 추론 시간, 표시 FPS와 `mask Nf/Nms` 형식의 마스크 지연이 표시된다. `q` 또는
`Esc`로 종료한다. 경계는 표시 대상 7개 클래스와 나머지 배경의 확률맵만 bilinear 보간한 뒤 결정해,
150개 전체 확률맵을 원본 크기로 확대하지 않고 계단 현상을 줄인다.

```powershell
# 표시 대상과 기준 조정
.\.venv\Scripts\python.exe -m modules.perception.inference.ip_webcam_semantic `
  --source http://192.168.0.20:8080/video `
  --classes wall,floor,ceiling,table,chair,door,windowpane `
  --confidence 0.40 --device auto

# GPU 실시간 처리 최적화
.\.venv\Scripts\python.exe -m modules.perception.inference.ip_webcam_semantic `
  --source http://192.168.0.20:8080/video `
  --device cuda `
  --confidence 0.45 `
  --input-size 384 `
  --display-width 960 `
  --inference-every 2

# 드론·휴대폰 없이 순수 마스크 로직 검증
.\.venv\Scripts\python.exe -B -m unittest modules.perception.inference.tests.test_semantic_overlay -v
```

IP·포트·비밀번호를 코드나 문서에 커밋하지 않는다. 이 모델은 영역의 종류를 분류하지만 벽까지의
실제 거리를 제공하지 않으므로 충돌 방지 검증에는 깊이 추정이나 거리 센서가 별도로 필요하다.

## Tello에서 YOLO와 SegFormer 동시 실행

Tello의 UDP 11111 영상 소켓은 한 프로세스만 열고, 같은 최신 프레임을 YOLO26 사람 인스턴스
세그멘테이션과 SegFormer 공간 세그멘테이션에 순서대로 전달한다. 초록 윤곽선은 사람, 반투명 색상은
벽·바닥·천장·테이블·의자·문·창문이다. 오래된 프레임을 쌓지 않으며 기본값은 YOLO를 매 처리 프레임,
SegFormer를 3 처리 프레임마다 갱신한다.

```powershell
# PC를 TELLO-/RMTT- Wi-Fi에 연결한 뒤 저장소 루트에서 실행
.\.venv\Scripts\python.exe -m modules.perception.inference.tello_dual_segmentation `
  --device cuda `
  --input-size 384 `
  --semantic-every 3

# 실기체 없이 순수 합성 결과 검증
.\.venv\Scripts\python.exe -B -m unittest `
  modules.perception.inference.tests.test_tello_dual_segmentation -v
```

이 실행기는 `command`와 `streamon`만 보내며 이륙·이동 명령을 보내지 않는다. 다른 Tello 영상 서버나
탐지 프로세스와 동시에 실행하면 UDP 11111 포트가 충돌하므로 먼저 종료한다. 두 모델을 한 GPU에서
순차 실행하므로 처리율이 부족하면 `--semantic-every`를 늘리거나 `--input-size`를 낮춘다.

# 웹에서 Tello YOLO + SegFormer 확인

기존 Tello 점검 사이트에서 합성 결과를 보려면 저장소 루트에서 실행한다.

```powershell
.\.venv\Scripts\python.exe modules/autonomy/tello_link/check_server.py --ai
```

브라우저에서 `http://127.0.0.1:8010`을 열고 **점검 시작 → 영상 켜기**를 누른다.
카메라 제목 옆 선택 상자에서 `AI 합성`과 `원본` 영상을 전환할 수 있다.
