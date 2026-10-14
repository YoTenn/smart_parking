import argparse
from pathlib import Path

import cv2
from ultralytics import YOLO


DEFAULT_WEIGHTS = (
    Path(__file__).resolve().parent
    / "runs"
    / "detect"
    / "bien_so_yolov8n"
    / "weights"
    / "best.pt"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the trained license-plate character detector on a webcam."
    )
    parser.add_argument(
        "--source",
        default="1",
        help="Webcam index (default: 1) or a video file path.",
    )
    parser.add_argument(
        "--backend",
        choices=("dshow", "msmf", "default"),
        default="dshow",
        help="Windows webcam capture backend (default: dshow).",
    )
    parser.add_argument(
        "--weights",
        type=Path,
        default=DEFAULT_WEIGHTS,
        help=f"Model weights (default: {DEFAULT_WEIGHTS})",
    )
    parser.add_argument(
        "--conf",
        type=float,
        default=0.4,
        help="Detection confidence threshold (default: 0.25).",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=416,
        help="Inference image size (default: 416).",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not 0.0 <= args.conf <= 1.0:
        raise SystemExit("--conf must be between 0 and 1.")
    if args.imgsz <= 0:
        raise SystemExit("--imgsz must be greater than 0.")
    if not args.weights.is_file():
        raise SystemExit(f"Model weights not found: {args.weights}")

    source: int | str = int(args.source) if args.source.isdecimal() else args.source
    model = YOLO(str(args.weights))
    if isinstance(source, int):
        backends = {
            "dshow": cv2.CAP_DSHOW,
            "msmf": cv2.CAP_MSMF,
            "default": cv2.CAP_ANY,
        }
        camera = cv2.VideoCapture(source, backends[args.backend])
    else:
        camera = cv2.VideoCapture(source)
    if not camera.isOpened():
        camera.release()
        raise SystemExit(
            f"Could not open video source {args.source!r}. "
            "Check the webcam connection or source path."
        )

    print("Webcam is running. Press Q or Esc to quit.")
    try:
        while True:
            success, frame = camera.read()
            if not success:
                raise SystemExit(
                    "Could not read a frame from the video source. "
                    "Close other apps using the webcam, or try "
                    "--backend msmf / --backend default."
                )

            result = model.predict(
                frame,
                conf=args.conf,
                imgsz=args.imgsz,
                verbose=False,
            )[0]
            cv2.imshow("YOLOv8 License Plate Detection", result.plot())

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
