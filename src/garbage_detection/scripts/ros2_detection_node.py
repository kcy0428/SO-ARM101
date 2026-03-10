"""
YOLOv8 쓰레기 감지 ROS2 노드
탑 카메라(USB RGB)로 쓰레기를 감지하고 결과를 ROS2 토픽으로 퍼블리시

퍼블리시 토픽:
  /garbage_detection/image        - 감지 결과 시각화 이미지
  /garbage_detection/detections   - 감지된 쓰레기 정보 (BBox, 클래스, 신뢰도)

구독 토픽:
  /image_raw                      - 탑 카메라 RGB 영상
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
from pathlib import Path
import json
import cv2


MODEL_DIR  = Path(__file__).parent.parent / "models"
BEST_MODEL = MODEL_DIR / "garbage_detect" / "weights" / "best.pt"


class GarbageDetectionNode(Node):

    def __init__(self):
        super().__init__("garbage_detection_node")

        # 파라미터 선언
        self.declare_parameter("model_path", str(BEST_MODEL))
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("camera_topic", "/image_raw")

        model_path   = self.get_parameter("model_path").get_parameter_value().string_value
        self.conf    = self.get_parameter("conf_threshold").get_parameter_value().double_value
        camera_topic = self.get_parameter("camera_topic").get_parameter_value().string_value

        # YOLOv8 모델 로드
        if Path(model_path).exists():
            self.get_logger().info(f"학습된 모델 로드: {model_path}")
        else:
            model_path = "yolov8n.pt"
            self.get_logger().warn("학습된 모델 없음. COCO 기본 모델 사용")

        self.model  = YOLO(model_path)
        self.bridge = CvBridge()

        # 구독자: 탑 카메라 영상
        self.sub_image = self.create_subscription(
            Image, camera_topic, self.image_callback, 10
        )

        # 퍼블리셔: 감지 결과
        self.pub_image      = self.create_publisher(Image,  "/garbage_detection/image",      10)
        self.pub_detections = self.create_publisher(String, "/garbage_detection/detections",  10)

        self.get_logger().info("GarbageDetectionNode 시작")
        self.get_logger().info(f"카메라 토픽 구독: {camera_topic}")
        self.get_logger().info(f"신뢰도 임계값: {self.conf}")

    def image_callback(self, msg: Image):
        # ROS2 Image → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # 쓰레기 감지
        results = self.model(frame, conf=self.conf, verbose=False)

        # 감지 결과 파싱
        detections = []
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                confidence       = float(box.conf[0])
                class_id         = int(box.cls[0])
                class_name       = self.model.names[class_id]
                cx               = (x1 + x2) // 2
                cy               = (y1 + y2) // 2

                detections.append({
                    "class":      class_name,
                    "confidence": round(confidence, 3),
                    "bbox":       [x1, y1, x2, y2],
                    "center_px":  [cx, cy],   # 픽셀 좌표 (D405로 깊이 조회에 사용)
                })

        # 감지 결과 퍼블리시 (JSON)
        det_msg = String()
        det_msg.data = json.dumps(detections, ensure_ascii=False)
        self.pub_detections.publish(det_msg)

        # 시각화 이미지 퍼블리시
        annotated = results[0].plot()
        img_msg   = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        img_msg.header = msg.header
        self.pub_image.publish(img_msg)

        if detections:
            self.get_logger().info(f"감지: {[d['class'] for d in detections]}")


def main(args=None):
    rclpy.init(args=args)
    node = GarbageDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
