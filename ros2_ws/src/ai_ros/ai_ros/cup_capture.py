#!/usr/bin/env python3
"""Simple RGB capture node: press SPACE to save current frame."""

import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

DEFAULT_SAVE_DIR = Path("/home/gwon_ho/ros2_ws/captures_rgb")


class SimpleRGBCaptureNode(Node):
    def __init__(self) -> None:
        super().__init__("simple_rgb_capture_node")

        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("save_dir", str(DEFAULT_SAVE_DIR))
        self.declare_parameter("window_title", "RGB Capture")

        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        save_dir_param = self.get_parameter("save_dir").get_parameter_value().string_value
        self.save_dir = Path(save_dir_param).expanduser().resolve()
        self.save_dir.mkdir(parents=True, exist_ok=True)
        self.window_title = self.get_parameter("window_title").get_parameter_value().string_value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.bridge = CvBridge()
        self.latest_bgr: Optional[np.ndarray] = None

        self.create_subscription(Image, self.image_topic, self._on_image, qos)

        self.get_logger().info("=" * 50)
        self.get_logger().info("Simple RGB capture node started")
        self.get_logger().info(f" image_topic: {self.image_topic}")
        self.get_logger().info(f" save_dir   : {self.save_dir}")
        self.get_logger().info(" Keys -> SPACE: save frame, q/ESC: exit")
        self.get_logger().info("=" * 50)

    def _on_image(self, msg: Image) -> None:
        try:
            self.latest_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as err:
            self.get_logger().error(f"CvBridge error: {err}")

    def save_snapshot(self) -> None:
        if self.latest_bgr is None:
            self.get_logger().warn("No frame available yet; skipping save.")
            return

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = self.save_dir / f"{timestamp}.jpg"
        ok = cv2.imwrite(str(filename), self.latest_bgr, [cv2.IMWRITE_JPEG_QUALITY, 95])
        if ok:
            self.get_logger().info(f"Saved {filename}")
        else:
            self.get_logger().error(f"Failed to save {filename}")

    def show_frame(self) -> None:
        if self.latest_bgr is None:
            return
        frame = self.latest_bgr.copy()
        cv2.putText(
            frame,
            "SPACE=Save, q/ESC=Quit",
            (10, frame.shape[0] - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 200, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.imshow(self.window_title, frame)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimpleRGBCaptureNode()

    try:
        cv2.namedWindow(node.window_title, cv2.WINDOW_AUTOSIZE)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.001)
            node.show_frame()
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if key == 32:
                node.save_snapshot()
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
