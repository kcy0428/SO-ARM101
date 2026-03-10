"""
YOLOv8 쓰레기 실시간 감지 스크립트 (탑 카메라 테스트용)
ROS2 없이 단독으로 카메라 연결 후 동작 확인할 때 사용
"""

import argparse
import cv2
import time
from pathlib import Path
from ultralytics import YOLO


MODEL_DIR   = Path(__file__).parent.parent / "models"
BEST_MODEL  = MODEL_DIR / "garbage_detect" / "weights" / "best.pt"


def detect_realtime(
    model_path: str = None,
    camera_id: int = 0,
    conf: float = 0.5,
    show: bool = True,
):
    """
    실시간 쓰레기 감지

    Args:
        model_path: 학습된 모델 경로 (없으면 best.pt 자동 사용)
        camera_id:  카메라 번호 (탑 카메라 = 0 또는 1)
        conf:       감지 신뢰도 임계값 (0.0 ~ 1.0)
        show:       화면 출력 여부
    """
    # 모델 로드
    if model_path is None:
        if BEST_MODEL.exists():
            model_path = str(BEST_MODEL)
            print(f"[INFO] 학습된 모델 사용: {BEST_MODEL}")
        else:
            model_path = "yolov8n.pt"
            print("[WARN] 학습된 모델 없음. COCO 기본 모델 사용")
            print("       쓰레기 전용 감지를 위해 먼저 train.py를 실행하세요.")

    model = YOLO(model_path)

    # 카메라 연결
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"[ERROR] 카메라 {camera_id}번을 열 수 없습니다.")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    print(f"[INFO] 카메라 {camera_id}번 연결 완료")
    print("[INFO] 'q' 키를 누르면 종료합니다.")
    print()

    fps_list = []

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[ERROR] 프레임을 읽을 수 없습니다.")
            break

        t_start = time.time()

        # 쓰레기 감지 실행
        results = model(frame, conf=conf, verbose=False)

        # FPS 계산
        fps = 1.0 / (time.time() - t_start)
        fps_list.append(fps)

        # 감지 결과 파싱
        detections = []
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                confidence       = float(box.conf[0])
                class_id         = int(box.cls[0])
                class_name       = model.names[class_id]
                cx               = (x1 + x2) // 2
                cy               = (y1 + y2) // 2

                detections.append({
                    "class":      class_name,
                    "confidence": confidence,
                    "bbox":       (x1, y1, x2, y2),
                    "center":     (cx, cy),
                })

                # 터미널 출력
                print(f"  감지: {class_name} ({confidence:.2f})  중심: ({cx}, {cy})")

        if show:
            # 결과 시각화
            annotated = results[0].plot()
            cv2.putText(annotated, f"FPS: {fps:.1f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(annotated, f"감지: {len(detections)}개", (10, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("Garbage Detection", annotated)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cap.release()
    cv2.destroyAllWindows()

    avg_fps = sum(fps_list) / len(fps_list) if fps_list else 0
    print(f"\n[INFO] 평균 FPS: {avg_fps:.1f}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="YOLOv8 실시간 쓰레기 감지")
    parser.add_argument("--model",    type=str,   default=None,
                        help="모델 경로 (기본: models/garbage_detect/weights/best.pt)")
    parser.add_argument("--camera",   type=int,   default=0,
                        help="카메라 번호 (기본: 0)")
    parser.add_argument("--conf",     type=float, default=0.5,
                        help="감지 신뢰도 임계값 (기본: 0.5)")
    parser.add_argument("--no-show",  action="store_true",
                        help="화면 출력 비활성화")
    args = parser.parse_args()

    detect_realtime(
        model_path=args.model,
        camera_id=args.camera,
        conf=args.conf,
        show=not args.no_show,
    )
