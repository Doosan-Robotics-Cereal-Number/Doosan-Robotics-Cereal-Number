#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import message_filters
import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO


class CupDetectionCVNode(Node):
    def __init__(self):
        super().__init__("cup_detection_cv_node")

        # --- YOLO 관련 파라미터 ---
        self.declare_parameter('model', 'yolov8m.pt')
        self.declare_parameter('conf', 0.086)
        self.declare_parameter('imgsz', 640)

        model_path = self.get_parameter('model').value
        self.conf = float(self.get_parameter('conf').value)
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        # --- 카메라 토픽 설정 ---
        color_topic = '/camera/camera/color/image_raw'
        depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'
        info_topic  = '/camera/camera/aligned_depth_to_color/camera_info'

        self.color_sub = message_filters.Subscriber(self, Image, color_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        self.info_sub  = message_filters.Subscriber(self, CameraInfo, info_topic)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.info_sub],
            queue_size=10, slop=0.1
        )
        self.ts.registerCallback(self.callback)

        self.pub = self.create_publisher(Float32MultiArray, "/cup_detections", 10)
        self.intrinsics = None
        self.get_logger().info("✅ Cup detection (with CV view) node started.")

    def callback(self, color_msg, depth_msg, info_msg):
        try:
            color_img = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        # --- 카메라 내부 파라미터 세팅 ---
        if self.intrinsics is None:
            intr = rs.intrinsics()
            intr.width = info_msg.width
            intr.height = info_msg.height
            intr.ppx, intr.ppy = info_msg.k[2], info_msg.k[5]
            intr.fx, intr.fy = info_msg.k[0], info_msg.k[4]
            intr.model = rs.distortion.brown_conrady
            intr.coeffs = list(info_msg.d)
            self.intrinsics = intr
            self.get_logger().info("Camera intrinsics set.")

        # --- YOLO 감지 ---
        results = self.model.predict(source=color_img, imgsz=self.imgsz, conf=self.conf, device='cpu', verbose=False)[0]
        if results.boxes is None or len(results.boxes) == 0:
            cv2.imshow("Cup Detection", color_img)
            cv2.waitKey(1)
            return

        boxes = results.boxes
        names = self.model.names
        cup_data = []

        for i in range(len(boxes)):
            cls_id = int(boxes.cls[i])
            label = names.get(cls_id, "")
            if label != "cup":
                continue

            x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy().astype(int)
            cx, cy = (x1 + x2)//2, (y1 + y2)//2
            d = depth_img[cy, cx]
            if d == 0:
                continue

            depth_m = d / 1000.0
            X, Y, Z = rs.rs2_deproject_pixel_to_point(self.intrinsics, [cx, cy], depth_m)
            cup_data.extend([X, Y, Z, depth_m])

            # --- 시각화 ---
            cv2.rectangle(color_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(color_img, (cx, cy), 4, (0, 0, 255), -1)
            cv2.putText(color_img, f"Cup {depth_m:.2f}m", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        # --- 퍼블리시 ---
        if cup_data:
            msg = Float32MultiArray()
            msg.data = cup_data
            self.pub.publish(msg)
            self.get_logger().info(f"Cup(s) detected → {cup_data}")

        # --- OpenCV 윈도우 표시 ---
        cv2.imshow("Cup Detection", color_img)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CupDetectionCVNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
