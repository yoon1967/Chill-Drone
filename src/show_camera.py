import argparse
import time

import cv2
from ultralytics import YOLO


def parse_args():
    parser = argparse.ArgumentParser(
        description="휴대폰 IP 카메라 영상을 화면에 표시합니다."
    )
    parser.add_argument(
        "--source",
        required=True,
        help="IP 카메라 영상 주소 또는 카메라 번호",
    )
    parser.add_argument(
        "--model",
        default="yolo26n.pt",
        help="사용할 YOLO 모델 파일 또는 모델 이름",
    )
    parser.add_argument(
        "--conf",
        type=float,
        default=0.25,
        help="탐지 결과에 표시할 최소 신뢰도(0~1)",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=640,
        help="YOLO 추론 이미지 크기",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    if not 0.0 <= args.conf <= 1.0:
        raise ValueError("--conf 값은 0 이상 1 이하여야 합니다.")

    # "0", "1"처럼 숫자만 입력하면 카메라 번호로 처리
    # 그 외에는 IP 카메라 주소로 처리
    source = int(args.source) if args.source.isdigit() else args.source

    capture = cv2.VideoCapture(source)

    if not capture.isOpened():
        raise RuntimeError(f"영상에 연결할 수 없습니다: {args.source}")

    print(f"YOLO 모델을 불러오는 중입니다: {args.model}")
    model = YOLO(args.model)

    if model.names.get(0) != "person":
        capture.release()
        raise RuntimeError("선택한 모델에서 클래스 0이 person이 아닙니다.")

    print("영상 연결 성공")
    print(f"person 탐지 시작 (신뢰도 임계값: {args.conf})")
    print("종료하려면 영상 창을 클릭한 뒤 q를 누르세요.")

    previous_time = time.perf_counter()
    display_fps = 0.0

    try:
        while True:
            success, frame = capture.read()

            if not success:
                print("프레임을 읽을 수 없어 종료합니다.")
                break

            result = model.predict(
                source=frame,
                classes=[0],
                conf=args.conf,
                imgsz=args.imgsz,
                verbose=False,
            )[0]

            output_frame = result.plot()

            current_time = time.perf_counter()
            elapsed = current_time - previous_time
            previous_time = current_time

            if elapsed > 0:
                current_fps = 1.0 / elapsed
                display_fps = (
                    current_fps
                    if display_fps == 0.0
                    else display_fps * 0.9 + current_fps * 0.1
                )

            cv2.putText(
                output_frame,
                f"FPS: {display_fps:.1f}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

            cv2.imshow("Phone Camera - Person Detection", output_frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                print("사용자 요청으로 종료합니다.")
                break
    finally:
        capture.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
