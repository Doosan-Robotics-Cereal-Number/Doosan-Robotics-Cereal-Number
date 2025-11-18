#!/usr/bin/env python3
"""
cup_detection_clip_node.py

YOLO로 기본 감지 후, 픽업 구역에서 False 판정이 일정 프레임 누적되면
CLIP으로 ROI 이미지를 분류해 컵 여부를 재검증합니다.
"""

from collections import deque
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32MultiArray, Bool
from transformers import CLIPModel, CLIPProcessor
import torch
import message_filters
from ultralytics import YOLO


def point_in_rect(x: float, y: float, rect: Dict[str, float]) -> bool:
    """점이 사각형 안에 있는지 확인"""
    return (rect["xmin"] <= x <= rect["xmax"]) and (rect["ymin"] <= y <= rect["ymax"])


class CupDetectionCLIPNode(Node):
    """픽업 구역 False 케이스를 CLIP으로 재검증하는 노드"""

    def __init__(self):
        super().__init__("cup_detection_clip_node")

        # =====================================================
        # 파라미터 선언
        # =====================================================
        self.declare_parameter("model", "yolov8m.pt")
        self.declare_parameter("conf", 0.04)
        self.declare_parameter("imgsz", 1280)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("class_label", "cup")

        self.declare_parameter("detect_delay_sec", 0.8)
        self.declare_parameter("window_size", 20)
        self.declare_parameter("stable_radius", 0.01)

        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("info_topic", "/camera/camera/aligned_depth_to_color/camera_info")

        self.declare_parameter("visualize", True)
        self.declare_parameter("min_box_area", 200)
        self.declare_parameter("pickup_st1_roi_px", "")
        self.declare_parameter("pickup_st2_roi_px", "")
        self.declare_parameter("verify_false_frames", 10)

        # CLIP 관련
        self.declare_parameter("clip_model", "openai/clip-vit-base-patch32")
        self.declare_parameter("clip_device", "")
        self.declare_parameter("clip_positive_text", "a cup is on the pickup station")
        self.declare_parameter("clip_negative_text", "an empty pickup station with no cup")
        self.declare_parameter("clip_positive_threshold", 0.55)
        self.declare_parameter("clip_negative_threshold", 0.45)

        # 파라미터 로드
        self.model_path = self.get_parameter("model").value
        self.conf = float(self.get_parameter("conf").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.device = str(self.get_parameter("device").value)
        self.class_label = str(self.get_parameter("class_label").value)

        self.detect_delay_sec = float(self.get_parameter("detect_delay_sec").value)
        self.window_size = int(self.get_parameter("window_size").value)
        self.stable_radius = float(self.get_parameter("stable_radius").value)

        self.color_topic = str(self.get_parameter("color_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.info_topic = str(self.get_parameter("info_topic").value)

        self.visualize = bool(self.get_parameter("visualize").value)
        self.min_box_area = int(self.get_parameter("min_box_area").value)
        self.verify_false_frames = int(self.get_parameter("verify_false_frames").value)
        if self.verify_false_frames < 1:
            self.verify_false_frames = 1

        roi1_str = self.get_parameter("pickup_st1_roi_px").get_parameter_value().string_value
        roi2_str = self.get_parameter("pickup_st2_roi_px").get_parameter_value().string_value
        self.zone_rois_px: Dict[str, Optional[Tuple[int, int, int, int]]] = {
            "pickup_st1": self._parse_roi_string(roi1_str),
            "pickup_st2": self._parse_roi_string(roi2_str),
        }
        self._pickup_false_counts = {
            "pickup_st1": 0,
            "pickup_st2": 0,
        }
        self._pickup_states = {
            "pickup_st1": True,
            "pickup_st2": True,
        }

        # CLIP 준비
        clip_model_name = self.get_parameter("clip_model").value
        clip_device_param = self.get_parameter("clip_device").get_parameter_value().string_value
        if clip_device_param:
            self.clip_device = clip_device_param
        else:
            self.clip_device = "cuda" if torch.cuda.is_available() else "cpu"
        self.clip_positive_text = self.get_parameter("clip_positive_text").value
        self.clip_negative_text = self.get_parameter("clip_negative_text").value
        self.clip_texts = [self.clip_positive_text, self.clip_negative_text]
        self.clip_positive_threshold = float(self.get_parameter("clip_positive_threshold").value)
        self.clip_negative_threshold = float(self.get_parameter("clip_negative_threshold").value)

        try:
            self.get_logger().info(f"🧠 Loading CLIP model: {clip_model_name} on {self.clip_device}")
            self.clip_model = CLIPModel.from_pretrained(clip_model_name)
            self.clip_processor = CLIPProcessor.from_pretrained(clip_model_name)
            self.clip_model.to(self.clip_device)
            self.clip_model.eval()
        except Exception as err:
            self.get_logger().error(f"CLIP model load failed: {err}")
            raise

        # =====================================================
        # 내부 변수
        # =====================================================
        self.bridge = CvBridge()
        self.intrinsics: Optional[rs.intrinsics] = None
        self.latest_cv_color = None
        self.latest_cv_depth_mm = None

        # 구역 플래그
        self._detect_personal = False
        self._detect_pickup = False
        self._publish_coords_active = False

        # 안정화 버퍼
        self.detect_buf = deque(maxlen=self.window_size)
        self.first_detect_time = None

        # YOLO 모델 로드
        self.model = YOLO(self.model_path)
        self.get_logger().info(f"✅ YOLO model loaded: {self.model_path}")

        # =====================================================
        # 구역 정의
        # =====================================================
        self.zones: Dict[str, Dict[str, float]] = {
            "personal_cup_st": {
                "xmin": 0.400,
                "xmax": 0.625,
                "ymin": -0.344,
                "ymax": -0.166,
            },
            "pickup_st1": {
                "xmin": 0.393,
                "xmax": 0.520,
                "ymin": -0.010,
                "ymax": 0.152,
            },
            "pickup_st2": {
                "xmin": 0.520,
                "xmax": 0.640,
                "ymin": -0.010,
                "ymax": 0.152,
            },
        }

        # =====================================================
        # 카메라 토픽 구독
        # =====================================================
        self.color_sub = message_filters.Subscriber(self, RosImage, self.color_topic)
        self.depth_sub = message_filters.Subscriber(self, RosImage, self.depth_topic)
        self.info_sub = message_filters.Subscriber(self, CameraInfo, self.info_topic)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.info_sub], queue_size=10, slop=0.1
        )
        self.ts.registerCallback(self._camera_callback)

        # =====================================================
        # 제어 토픽 구독
        # =====================================================
        self.create_subscription(Bool, "/check_cup", self._on_check_cup, 10)
        self.create_subscription(Bool, "/check_cup_done", self._on_check_cup_done, 10)
        self.create_subscription(Bool, "/call_cup_stable_coordinates", self._on_call_coords, 10)
        self.create_subscription(Bool, "/arrive_cup_stable_coordinates", self._on_arrive_coords, 10)

        # =====================================================
        # 발행자
        # =====================================================
        self.pub_personal = self.create_publisher(Bool, "/cup_detections/personal_cup_st", 10)
        self.pub_pick1 = self.create_publisher(Bool, "/cup_detections/pickup_st1", 10)
        self.pub_pick2 = self.create_publisher(Bool, "/cup_detections/pickup_st2", 10)
        self.cup_coord_pub = self.create_publisher(Float32MultiArray, "/cup_stable_coordinates", 10)

        # =====================================================
        # 시작 메시지
        # =====================================================
        self.get_logger().info("=" * 60)
        self.get_logger().info("✅ Cup Detection CLIP Node Started")
        self.get_logger().info(f"   YOLO: {self.model_path}")
        self.get_logger().info(f"   CLIP: {clip_model_name} ({self.clip_device})")
        self.get_logger().info("=" * 60)

    # =====================================================
    # 제어 콜백
    # =====================================================
    def _on_check_cup(self, msg: Bool):
        if msg.data and not self._detect_pickup:
            self._detect_pickup = True
            self.get_logger().info("🟢 Pickup detection STARTED")
        elif not msg.data and self._detect_pickup:
            self._detect_pickup = False
            self._publish_pickup_false()
            self.get_logger().info("🔴 Pickup detection STOPPED")

    def _on_check_cup_done(self, msg: Bool):
        if msg.data and self._detect_pickup:
            self._detect_pickup = False
            self._publish_pickup_false()
            self.get_logger().info("🔴 Pickup detection STOPPED (done)")

    def _on_call_coords(self, msg: Bool):
        if msg.data and not self._detect_personal:
            self._detect_personal = True
            self._publish_coords_active = True
            self._reset_buffers()
            self.get_logger().info("🟢 Personal cup detection + coords STARTED")
        elif not msg.data and self._detect_personal:
            self._detect_personal = False
            self._publish_coords_active = False
            self._reset_buffers()
            self.pub_personal.publish(Bool(data=False))
            self.get_logger().info("🔴 Personal cup detection + coords STOPPED")

    def _on_arrive_coords(self, msg: Bool):
        if msg.data and self._detect_personal:
            self._detect_personal = False
            self._publish_coords_active = False
            self._reset_buffers()
            self.pub_personal.publish(Bool(data=False))
            self.get_logger().info("🔴 Personal cup detection + coords STOPPED (arrived)")

    def _publish_pickup_false(self):
        self.pub_pick1.publish(Bool(data=False))
        self.pub_pick2.publish(Bool(data=False))

    def _reset_buffers(self):
        self.detect_buf.clear()
        self.first_detect_time = None

    # =====================================================
    # 유틸리티
    # =====================================================
    def _which_zone(self, x: float, y: float) -> Optional[str]:
        for name, rect in self.zones.items():
            if point_in_rect(x, y, rect):
                return name
        return None

    def _parse_roi_string(self, raw: str) -> Optional[Tuple[int, int, int, int]]:
        if not raw:
            return None
        parts = [p.strip() for p in raw.replace(";", ",").split(",") if p.strip()]
        if len(parts) != 4:
            self.get_logger().warn(f"Invalid ROI string '{raw}'. Expected 4 comma-separated values.")
            return None
        try:
            x1, y1, x2, y2 = [int(float(p)) for p in parts]
        except ValueError:
            self.get_logger().warn(f"Invalid ROI numbers in '{raw}'.")
            return None
        if x2 <= x1 or y2 <= y1:
            self.get_logger().warn(f"ROI has non-positive size: '{raw}'.")
            return None
        return (x1, y1, x2, y2)

    def _transform_to_robot_coords(self, X: float, Y: float, Z: float) -> list:
        x_mm = Y * 1000
        y_mm = X * 1000
        z_mm = Z * 1000

        final_x = 777 + x_mm - 249 + 262
        final_y = y_mm - 265
        final_z = 970 - z_mm - 200

        if final_z <= 55:
            final_z = 55

        return [float(final_x), float(final_y), float(final_z)]

    def _crop_zone_image(self, zone_name: str) -> Optional[np.ndarray]:
        roi = self.zone_rois_px.get(zone_name)
        if roi is None:
            return None
        if self.latest_cv_color is None:
            return None
        x1, y1, x2, y2 = roi
        h, w = self.latest_cv_color.shape[:2]
        x1 = max(0, min(w - 1, x1))
        x2 = max(0, min(w, x2))
        y1 = max(0, min(h - 1, y1))
        y2 = max(0, min(h, y2))
        if x2 <= x1 or y2 <= y1:
            return None
        roi_img = self.latest_cv_color[y1:y2, x1:x2].copy()
        try:
            cv2.imshow(f"ROI {zone_name}", roi_img)
            cv2.waitKey(1)
        except cv2.error:
            pass
        return roi_img

    def _np_to_pil(self, roi_img: np.ndarray) -> Image.Image:
        rgb = cv2.cvtColor(roi_img, cv2.COLOR_BGR2RGB)
        return Image.fromarray(rgb)

    def _clip_score_roi(self, roi_img: np.ndarray) -> Optional[float]:
        """CLIP으로 컵 존재 확률 반환"""
        if roi_img is None:
            return None
        pil_img = self._np_to_pil(roi_img)
        inputs = self.clip_processor(
            text=self.clip_texts,
            images=pil_img,
            return_tensors="pt",
            padding=True,
        )
        inputs = {k: v.to(self.clip_device) if hasattr(v, "to") else v for k, v in inputs.items()}
        with torch.inference_mode():
            out = self.clip_model(**inputs)
            probs = out.logits_per_image.softmax(dim=-1).cpu().numpy()[0]
        return float(probs[0])  # positive text 확률

    def _resolve_pickup_state(self, zone_name: str, detected: bool) -> bool:
        if detected:
            self._pickup_false_counts[zone_name] = 0
            self._pickup_states[zone_name] = True
            return True

        self._pickup_false_counts.setdefault(zone_name, 0)
        self._pickup_false_counts[zone_name] += 1
        if self._pickup_false_counts[zone_name] < self.verify_false_frames:
            return self._pickup_states[zone_name]

        self._pickup_false_counts[zone_name] = 0
        roi_img = self._crop_zone_image(zone_name)
        if roi_img is None:
            self.get_logger().warn(f"{zone_name}: ROI unavailable; keeping previous state.")
            return self._pickup_states[zone_name]

        cup_prob = self._clip_score_roi(roi_img)
        if cup_prob is None:
            self.get_logger().warn(f"{zone_name}: CLIP scoring failed; keeping previous state.")
            return self._pickup_states[zone_name]

        if cup_prob >= self.clip_positive_threshold:
            self.get_logger().info(f"CLIP override: {zone_name} cup_prob={cup_prob:.3f} → True")
            self._pickup_states[zone_name] = True
        elif cup_prob <= self.clip_negative_threshold:
            self.get_logger().info(f"CLIP confirm empty: {zone_name} cup_prob={cup_prob:.3f} → False")
            self._pickup_states[zone_name] = False
        else:
            self.get_logger().warn(
                f"{zone_name}: CLIP ambiguous cup_prob={cup_prob:.3f}; keeping previous state."
            )

        return self._pickup_states[zone_name]

    # =====================================================
    # 메인 카메라 콜백
    # =====================================================
    def _camera_callback(self, color_msg: RosImage, depth_msg: RosImage, info_msg: CameraInfo):
        try:
            self.latest_cv_color = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            self.latest_cv_depth_mm = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        if self.intrinsics is None:
            self.intrinsics = rs.intrinsics()
            self.intrinsics.width = info_msg.width
            self.intrinsics.height = info_msg.height
            self.intrinsics.ppx = info_msg.k[2]
            self.intrinsics.ppy = info_msg.k[5]
            self.intrinsics.fx = info_msg.k[0]
            self.intrinsics.fy = info_msg.k[4]
            if info_msg.distortion_model in ["plumb_bob", "rational_polynomial"]:
                self.intrinsics.model = rs.distortion.brown_conrady
            else:
                self.intrinsics.model = rs.distortion.none
            self.intrinsics.coeffs = list(info_msg.d)
            self.get_logger().info("📷 Camera intrinsics configured")

        detected_personal = False
        detected_pick1 = False
        detected_pick2 = False
        personal_cup_coords = None

        if self._detect_personal or self._detect_pickup:
            results = self.model.predict(
                source=self.latest_cv_color,
                imgsz=self.imgsz,
                conf=self.conf,
                device=self.device,
                verbose=False,
            )[0]

            boxes = getattr(results, "boxes", None)
            names = self.model.names

            if boxes is not None and len(boxes) > 0:
                xyxy = boxes.xyxy.cpu().numpy().astype(int)
                cls = boxes.cls.cpu().numpy().astype(int)

                for i in range(len(xyxy)):
                    cls_id = int(cls[i])
                    label = names.get(cls_id, str(cls_id))
                    if label != self.class_label:
                        continue

                    x1, y1, x2, y2 = xyxy[i]
                    if (x2 - x1) * (y2 - y1) < self.min_box_area:
                        continue

                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                    if (
                        cy < 0
                        or cy >= self.latest_cv_depth_mm.shape[0]
                        or cx < 0
                        or cx >= self.latest_cv_depth_mm.shape[1]
                    ):
                        continue

                    d_mm = int(self.latest_cv_depth_mm[cy, cx])
                    if d_mm == 0:
                        continue

                    depth_m = d_mm / 1000.0
                    X, Y, Z = rs.rs2_deproject_pixel_to_point(self.intrinsics, [cx, cy], depth_m)

                    zone = self._which_zone(float(X), float(Y))
                    if zone is None:
                        continue

                    if zone == "personal_cup_st" and self._detect_personal:
                        detected_personal = True
                        personal_cup_coords = (float(X), float(Y), float(Z))
                    elif zone == "pickup_st1" and self._detect_pickup:
                        detected_pick1 = True
                    elif zone == "pickup_st2" and self._detect_pickup:
                        detected_pick2 = True

                    if self.visualize:
                        cv2.rectangle(self.latest_cv_color, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.circle(self.latest_cv_color, (cx, cy), 4, (0, 0, 255), -1)
                        cv2.putText(
                            self.latest_cv_color,
                            f"{zone} {depth_m:.2f}m",
                            (x1, max(0, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (255, 255, 0),
                            2,
                        )

            if self._detect_personal:
                self.pub_personal.publish(Bool(data=detected_personal))

            state_pick1 = self._pickup_states["pickup_st1"]
            state_pick2 = self._pickup_states["pickup_st2"]
            if self._detect_pickup:
                state_pick1 = self._resolve_pickup_state("pickup_st1", detected_pick1)
                state_pick2 = self._resolve_pickup_state("pickup_st2", detected_pick2)
                self.pub_pick1.publish(Bool(data=state_pick1))
                self.pub_pick2.publish(Bool(data=state_pick2))

            detected_zones = []
            if detected_personal and self._detect_personal:
                detected_zones.append("personal_cup_st")
            if state_pick1 and self._detect_pickup:
                detected_zones.append("pickup_st1")
            if state_pick2 and self._detect_pickup:
                detected_zones.append("pickup_st2")
            if detected_zones:
                self.get_logger().info(f"🔍 Detected: {', '.join(detected_zones)}")

        if self._publish_coords_active and personal_cup_coords is not None:
            self._process_personal_cup_coords(personal_cup_coords)

        if self.visualize and self.latest_cv_color is not None:
            display = self.latest_cv_color.copy()
            personal_status = "ACTIVE" if self._detect_personal else "IDLE"
            personal_color = (0, 255, 0) if self._detect_personal else (128, 128, 128)
            cv2.putText(
                display,
                f"Personal: {personal_status}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                personal_color,
                2,
            )
            pickup_status = "ACTIVE" if self._detect_pickup else "IDLE"
            pickup_color = (0, 255, 0) if self._detect_pickup else (128, 128, 128)
            cv2.putText(
                display,
                f"Pickup: {pickup_status}",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                pickup_color,
                2,
            )
            buf_text = f"Buffer: {len(self.detect_buf)}/{self.window_size}"
            cv2.putText(
                display,
                buf_text,
                (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 0),
                2,
            )
            cv2.imshow("Cup Detection CLIP", display)
            cv2.waitKey(1)

    # =====================================================
    # 개인컵 좌표 안정화
    # =====================================================
    def _process_personal_cup_coords(self, coords: tuple):
        X, Y, Z = coords
        now = self.get_clock().now()
        if self.first_detect_time is None:
            self.first_detect_time = now
            self.detect_buf.clear()
            self.detect_buf.append((X, Y, Z))
            self.get_logger().info("🔍 Personal cup detected - collecting samples...")
            return

        if len(self.detect_buf) > 0:
            lastX, lastY, lastZ = self.detect_buf[-1]
            if (abs(X - lastX) > 0.05) or (abs(Y - lastY) > 0.05) or (abs(Z - lastZ) > 0.05):
                self.get_logger().warn("⚠️  Large jump detected - resetting buffer")
                self.first_detect_time = now
                self.detect_buf.clear()

        self.detect_buf.append((X, Y, Z))

        elapsed = (now - self.first_detect_time).nanoseconds / 1e9
        if elapsed < self.detect_delay_sec:
            return

        xs = [p[0] for p in self.detect_buf]
        ys = [p[1] for p in self.detect_buf]
        zs = [p[2] for p in self.detect_buf]

        Xavg = sum(xs) / len(xs)
        Yavg = sum(ys) / len(ys)
        Zavg = sum(zs) / len(zs)

        if any(abs(px - Xavg) > self.stable_radius for px in xs) or any(
            abs(py - Yavg) > self.stable_radius for py in ys
        ) or any(abs(pz - Zavg) > self.stable_radius for pz in zs):
            self.get_logger().debug("   Stabilizing...")
            self.first_detect_time = now
            return

        self.get_logger().info(f"✅ Position stabilized ({len(self.detect_buf)} samples)")
        self.get_logger().info(f"   Camera: X={Xavg:.3f}, Y={Yavg:.3f}, Z={Zavg:.3f}")

        robot_coords = self._transform_to_robot_coords(Xavg, Yavg, Zavg)
        self.get_logger().info(
            f"   Robot: X={robot_coords[0]:.1f}, Y={robot_coords[1]:.1f}, Z={robot_coords[2]:.1f}"
        )

        coord_msg = Float32MultiArray()
        coord_msg.data = robot_coords
        self.cup_coord_pub.publish(coord_msg)
        self.get_logger().info("📢 Cup coordinates published!")

        self.detect_buf.clear()
        self.first_detect_time = None

    # =====================================================
    # 마우스 클릭 콜백
    # =====================================================
    def mouse_callback(self, event, u, v, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if self.latest_cv_depth_mm is None or self.intrinsics is None:
            return
        try:
            depth_mm = self.latest_cv_depth_mm[v, u]
        except IndexError:
            return
        if depth_mm == 0:
            return

        depth_m = float(depth_mm) / 1000.0
        point_3d = rs.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], depth_m)
        X, Y, Z = float(point_3d[0]), float(point_3d[1]), float(point_3d[2])
        robot_coords = self._transform_to_robot_coords(X, Y, Z)

        self.get_logger().info("=" * 60)
        self.get_logger().info(f"🖱️  Manual click at pixel ({u}, {v})")
        self.get_logger().info(f"   Camera: X={X:.3f}, Y={Y:.3f}, Z={Z:.3f}")
        self.get_logger().info(
            f"   Robot: X={robot_coords[0]:.1f}, Y={robot_coords[1]:.1f}, Z={robot_coords[2]:.1f}"
        )
        self.get_logger().info("=" * 60)

        coord_msg = Float32MultiArray()
        coord_msg.data = robot_coords
        self.cup_coord_pub.publish(coord_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CupDetectionCLIPNode()

    if node.visualize:
        cv2.namedWindow("Cup Detection CLIP")
        cv2.setMouseCallback("Cup Detection CLIP", node.mouse_callback)

    print("=" * 60)
    print("Cup Detection CLIP Node")
    print("  - check_cup → pickup_st1/st2 detection (CLIP fallback when empty)")
    print("  - call_coords → personal_cup_st detection + coords")
    print("📤 Publishing:")
    print("  - /cup_detections/personal_cup_st")
    print("  - /cup_detections/pickup_st1")
    print("  - /cup_detections/pickup_st2")
    print("  - /cup_stable_coordinates")
    print("=" * 60)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.001)
            if cv2.waitKey(1) & 0xFF == 27:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if node.visualize:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
