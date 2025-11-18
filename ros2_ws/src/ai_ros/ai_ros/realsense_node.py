#!/usr/bin/env python3
"""
realsense_node.py

RealSense 깊이 카메라를 직접 구동해 워크스페이스에 새로 등장한 물체를 감지하고
간단한 ID를 부여해 publish 합니다.

출력 토픽
  - /workspace_objects (Float32MultiArray):
        [id, X(mm), Y(mm), Z(mm), u(px), v(px)] 반복
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import threading
from typing import Dict, List, Optional, Tuple, Any

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray, Header, String


@dataclass
class TrackedObject:
    """추적 대상 1개에 대한 상태 저장 구조체."""

    state: np.ndarray  # [X, Y, Z, Vx, Vy, Vz]
    covariance: np.ndarray  # 6x6 Kalman covariance
    pixel: Tuple[float, float] = (0.0, 0.0)
    missed: int = 0
    last_seen: Time = field(default_factory=Time)
    filter_time: Time = field(default_factory=Time)
    shape_feature: Optional[np.ndarray] = None

    @property
    def position(self) -> np.ndarray:
        return self.state[:3]

    @position.setter
    def position(self, value: np.ndarray):
        arr = np.asarray(value, dtype=np.float32)
        self.state[:3] = arr[:3]


class RealSenseWorkspaceNode(Node):
    """새 물체 감지 + ID 부여"""

    def __init__(self):
        super().__init__("realsense_workspace_node")

        # ===============================
        # Parameters
        # ===============================
        self.declare_parameter("depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("info_topic", "/camera/aligned_depth_to_color/camera_info")
        self.declare_parameter("update_rate_hz", 15.0)  # max processing rate (Hz)
        self.declare_parameter("bg_init_frames", 30)  # frames to warm up background
        self.declare_parameter("bg_alpha", 0.02)  # EMA blending factor
        self.declare_parameter("bg_freeze_foreground", True)
        self.declare_parameter("depth_threshold_m", 0.02)  # min delta depth to accept as foreground
        self.declare_parameter("min_region_area", 800)  # min connected component area in px
        self.declare_parameter("max_match_distance_m", 0.08)  # nearest-neighbor gating distance
        self.declare_parameter("max_missed_frames", 5)  # drop track after N misses
        self.declare_parameter("workspace_roi_px", "")  # optional px ROI
        self.declare_parameter("depth_scale", 0.001)  # multiply raw depth units to meters
        self.declare_parameter("depth_anchor_percentile", 55.0)  # focus deeper pixels (0-100)
        self.declare_parameter("depth_anchor_padding_m", 0.01)
        self.declare_parameter("stable_event_topic", "/workspace_stable_object")
        self.declare_parameter("stable_min_frames", 50)
        self.declare_parameter("stable_min_duration_sec", 5)
        self.declare_parameter("stable_event_cooldown_sec", 30.0)
        self.declare_parameter("visualize", False)
        self.declare_parameter("color_topic", "")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("debug_image_topic", "/workspace_debug")
        self.declare_parameter("motion_model_enabled", True)
        self.declare_parameter("motion_process_std", 0.3)  # m/s^2 process noise
        self.declare_parameter("motion_measurement_std", 0.01)  # m position noise
        self.declare_parameter("motion_initial_velocity_std", 0.05)  # m/s initial velocity noise
        self.declare_parameter("reid_enable", True)
        self.declare_parameter("reid_max_age_sec", 4.0)
        self.declare_parameter("reid_feature_threshold", 0.35)

        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.info_topic = str(self.get_parameter("info_topic").value)
        self.update_rate_hz = float(self.get_parameter("update_rate_hz").value)
        self.bg_init_frames = int(self.get_parameter("bg_init_frames").value)
        self.bg_alpha = float(self.get_parameter("bg_alpha").value)
        self.bg_freeze_foreground = bool(self.get_parameter("bg_freeze_foreground").value)
        self.depth_threshold_m = float(self.get_parameter("depth_threshold_m").value)
        self.min_region_area = int(self.get_parameter("min_region_area").value)
        self.max_match_distance_m = float(self.get_parameter("max_match_distance_m").value)
        self.max_missed_frames = int(self.get_parameter("max_missed_frames").value)
        self.depth_scale = float(self.get_parameter("depth_scale").value)
        self.depth_anchor_percentile = float(self.get_parameter("depth_anchor_percentile").value)
        self.depth_anchor_padding_m = max(0.0, float(self.get_parameter("depth_anchor_padding_m").value))
        self.stable_event_topic = str(self.get_parameter("stable_event_topic").value)
        self.stable_min_frames = max(1, int(self.get_parameter("stable_min_frames").value))
        self.stable_min_duration_sec = max(0.0, float(self.get_parameter("stable_min_duration_sec").value))
        self.stable_event_cooldown_sec = max(0.0, float(self.get_parameter("stable_event_cooldown_sec").value))
        self.visualize = bool(self.get_parameter("visualize").value)
        self.color_topic = str(self.get_parameter("color_topic").value).strip()
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value).strip()
        self.motion_model_enabled = bool(self.get_parameter("motion_model_enabled").value)
        self.motion_process_std = max(
            1e-4, float(self.get_parameter("motion_process_std").value)
        )
        self.motion_measurement_std = max(
            1e-4, float(self.get_parameter("motion_measurement_std").value)
        )
        self.motion_initial_velocity_std = max(
            1e-4, float(self.get_parameter("motion_initial_velocity_std").value)
        )
        self.reid_enable = bool(self.get_parameter("reid_enable").value)
        self.reid_max_age_sec = max(0.0, float(self.get_parameter("reid_max_age_sec").value))
        self.reid_feature_threshold = max(
            1e-4, float(self.get_parameter("reid_feature_threshold").value)
        )

        self._measurement_matrix = np.hstack(
            [np.eye(3, dtype=np.float32), np.zeros((3, 3), dtype=np.float32)]
        )
        self._measurement_cov = (
            np.eye(3, dtype=np.float32) * (self.motion_measurement_std ** 2)
        )
        self._identity6 = np.eye(6, dtype=np.float32)
        self.lost_tracks: Dict[int, Dict] = {}
        self.stable_event_pub = self.create_publisher(String, self.stable_event_topic, 10)
        self.stable_states: Dict[int, Dict[str, Any]] = {}

        # 관심 영역 문자열을 px 좌표로 파싱 (사각형 또는 다각형)
        self.workspace_roi = self._parse_roi_string(
            self.get_parameter("workspace_roi_px").get_parameter_value().string_value
        )

        # ===============================
        # Internal buffers
        # ===============================
        self.bg_frame: Optional[np.ndarray] = None  # EMA background (meters)
        self.bg_frames_seen = 0
        self.intrinsics: Optional[rs.intrinsics] = None
        self.last_process_time: Optional[Time] = None
        self.update_period = 1.0 / self.update_rate_hz if self.update_rate_hz > 0 else 0.0
        self.latest_debug_frame: Optional[np.ndarray] = None
        self.latest_debug_stamp: Optional[Time] = None

        self.tracked: Dict[int, TrackedObject] = {}
        self.next_id = 1

        self.color_lock = threading.Lock()
        self.latest_color_frame: Optional[np.ndarray] = None
        self.latest_color_stamp: Optional[Time] = None

        # Visualization / debug
        self.window_name = "Workspace Depth"
        self.window_ready = False
        self.debug_image_pub = None
        if self.publish_debug_image and self.debug_image_topic:
            self.debug_image_pub = self.create_publisher(Image, self.debug_image_topic, 2)
            self.get_logger().info(f"🖼️ Publishing debug view to {self.debug_image_topic}")
        if self.visualize:
            self._ensure_window()

        # Publisher
        self.objects_pub = self.create_publisher(Float32MultiArray, "/workspace_objects", 10)

        # Subscribers (depth + intrinsics)
        self.bridge = CvBridge()
        self.depth_sub = message_filters.Subscriber(self, Image, self.depth_topic)
        self.info_sub = message_filters.Subscriber(self, CameraInfo, self.info_topic)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.depth_sub, self.info_sub], queue_size=10, slop=0.05
        )
        self.ts.registerCallback(self._sync_callback)

        if self.color_topic:
            self.color_sub = self.create_subscription(Image, self.color_topic, self._color_callback, 10)
            self.get_logger().info(f"🎨 Subscribing to color stream: {self.color_topic}")
        else:
            self.color_sub = None

        self.get_logger().info("✅ RealSense workspace node listening to depth stream")

    # -------------------------------------------------------
    # Utility
    # -------------------------------------------------------
    def _parse_roi_string(self, raw: str) -> Optional[Dict]:
        """Parse ROI parameter into rectangle or polygon definition."""
        if not raw:
            return None
        raw = raw.strip()
        entries = [entry.strip() for entry in raw.split("|") if entry.strip()]
        if not entries:
            return None
        if len(entries) == 1:
            return self._parse_roi_entry(entries[0])

        shapes = []
        for entry in entries:
            shape = self._parse_roi_entry(entry)
            if shape is None:
                return None
            shapes.append(shape)
        return {"type": "multi", "shapes": shapes}

    def _parse_roi_entry(self, raw: str) -> Optional[Dict]:
        if raw.lower().startswith("poly:"):
            poly_raw = raw[5:]
            points = self._parse_roi_points(poly_raw)
            return self._validate_polygon(points, raw)

        numbers = self._parse_roi_numbers(raw)
        if len(numbers) == 4:
            x1, y1, x2, y2 = numbers
            if x2 <= x1 or y2 <= y1:
                self.get_logger().warn(f"ROI has non-positive size: '{raw}'.")
                return None
            return {"type": "rect", "coords": (x1, y1, x2, y2)}

        if len(numbers) >= 6 and len(numbers) % 2 == 0:
            points = [(numbers[i], numbers[i + 1]) for i in range(0, len(numbers), 2)]
            return self._validate_polygon(points, raw)

        self.get_logger().warn(
            f"Invalid ROI entry '{raw}'. Use 'x1,y1,x2,y2' for rectangle or 'poly:x1,y1;x2,y2;...' for polygon."
        )
        return None

    def _parse_roi_numbers(self, raw: str) -> List[int]:
        parts = [p.strip() for p in raw.replace(";", ",").replace("|", ",").split(",") if p.strip()]
        numbers = []
        for part in parts:
            try:
                numbers.append(int(float(part)))
            except ValueError:
                self.get_logger().warn(f"ROI parse error: '{part}' is not a number.")
                return []
        return numbers

    def _parse_roi_points(self, raw: str) -> List[Tuple[int, int]]:
        tokens = [tok.strip() for tok in raw.replace("|", ";").split(";") if tok.strip()]
        points: List[Tuple[int, int]] = []
        for tok in tokens:
            coords = [c.strip() for c in tok.replace(",", " ").split() if c.strip()]
            if len(coords) != 2:
                self.get_logger().warn(f"ROI polygon point '{tok}' is invalid. Use x,y.")
                return []
            try:
                x = int(float(coords[0]))
                y = int(float(coords[1]))
            except ValueError:
                self.get_logger().warn(f"ROI polygon point '{tok}' has invalid numbers.")
                return []
            points.append((x, y))
        return points

    def _validate_polygon(self, points: List[Tuple[int, int]], raw: str) -> Optional[Dict]:
        if len(points) < 3:
            self.get_logger().warn(f"Polygon ROI needs >=3 points: '{raw}'.")
            return None
        return {"type": "poly", "points": points}

    def _apply_roi(self, mask: np.ndarray) -> np.ndarray:
        """Zero-out mask pixels outside the configured ROI."""
        if self.workspace_roi is None:
            return mask
        roi_mask = np.zeros_like(mask, dtype=np.uint8)
        self._fill_roi_mask(roi_mask, self.workspace_roi)
        return mask * roi_mask

    def _draw_roi(self, image: np.ndarray, color: Tuple[int, int, int]):
        if self.workspace_roi is None:
            return
        self._draw_roi_shape(image, self.workspace_roi, color)

    def _fill_roi_mask(self, mask: np.ndarray, roi_def: Dict):
        if roi_def["type"] == "rect":
            x1, y1, x2, y2 = roi_def["coords"]
            y1c = max(0, min(mask.shape[0], y1))
            y2c = max(0, min(mask.shape[0], y2))
            x1c = max(0, min(mask.shape[1], x1))
            x2c = max(0, min(mask.shape[1], x2))
            if y2c > y1c and x2c > x1c:
                mask[y1c:y2c, x1c:x2c] = 1
        elif roi_def["type"] == "poly":
            pts = np.array([roi_def["points"]], dtype=np.int32)
            cv2.fillPoly(mask, pts, 1)
        elif roi_def["type"] == "multi":
            for shape in roi_def["shapes"]:
                self._fill_roi_mask(mask, shape)

    def _draw_roi_shape(self, image: np.ndarray, roi_def: Dict, color: Tuple[int, int, int]):
        if roi_def["type"] == "rect":
            x1, y1, x2, y2 = roi_def["coords"]
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 1)
        elif roi_def["type"] == "poly":
            pts = np.array(roi_def["points"], dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(image, [pts], isClosed=True, color=color, thickness=1)
        elif roi_def["type"] == "multi":
            for shape in roi_def["shapes"]:
                self._draw_roi_shape(image, shape, color)

    def _select_anchor_mask(
        self, label_mask: np.ndarray, depth_image: np.ndarray, depth_vals: np.ndarray
    ) -> np.ndarray:
        """Hand contact 시 가까운 픽셀을 억제하고 하부(깊은) 영역으로 중심을 재계산."""
        percentile = float(np.clip(self.depth_anchor_percentile, 0.0, 100.0))
        if percentile <= 0.0 or depth_vals.size == 0:
            return label_mask
        threshold = np.percentile(depth_vals, percentile)
        anchor_mask = label_mask & (depth_image >= (threshold - self.depth_anchor_padding_m))
        if np.count_nonzero(anchor_mask) < 10:
            return label_mask
        return anchor_mask

    def _compute_shape_feature(
        self,
        anchor_mask: np.ndarray,
        bbox: Tuple[int, int, int, int],
        anchor_depths: np.ndarray,
    ) -> Optional[np.ndarray]:
        if anchor_depths.size == 0:
            return None
        binary = anchor_mask.astype(np.uint8)
        moments = cv2.moments(binary)
        if moments["m00"] == 0:
            return None
        hu = cv2.HuMoments(moments).flatten()
        hu_log = np.sign(hu) * np.log10(np.abs(hu) + 1e-9)
        area = float(np.count_nonzero(anchor_mask))
        w = max(1, bbox[2])
        h = max(1, bbox[3])
        aspect = float(w) / float(h)
        depth_median = float(np.median(anchor_depths))
        depth_std = float(np.std(anchor_depths))
        feature = np.concatenate(
            [
                hu_log.astype(np.float32),
                np.array(
                    [
                        np.log1p(area),
                        aspect,
                        depth_median,
                        depth_std,
                    ],
                    dtype=np.float32,
                ),
            ]
        )
        return feature

    def _deproject(self, u: float, v: float, depth_m: float) -> Optional[np.ndarray]:
        """Pixel→3D point using current intrinsics."""
        if self.intrinsics is None or depth_m <= 0:
            return None
        point = rs.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], depth_m)
        return np.array(point, dtype=np.float32)

    # -------------------------------------------------------
    def _sync_callback(self, depth_msg: Image, info_msg: CameraInfo):
        """Depth + camera info callback."""
        now = self.get_clock().now()
        if self.last_process_time is not None and self.update_period > 0:
            elapsed = (now - self.last_process_time).nanoseconds / 1e9
            if elapsed < self.update_period:
                return
        self.last_process_time = now

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
            self.get_logger().info("📷 Intrinsics populated from CameraInfo")

        try:
            depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
        except CvBridgeError as err:
            self.get_logger().error(f"CvBridge error: {err}")
            return

        depth_image = depth_raw.astype(np.float32) * self.depth_scale  # meters
        fg_mask = np.zeros_like(depth_image, dtype=np.uint8)

        should_render = self.visualize or self.debug_image_pub is not None

        if self.bg_frame is None:
            self.bg_frame = depth_image.copy()
            self.bg_frames_seen = 1
            self.get_logger().info("Initializing background model...")
            if should_render:
                self._render_debug(depth_image, fg_mask, [], depth_msg.header)
            return

        if self.bg_frames_seen < self.bg_init_frames:
            alpha = 1.0 / float(self.bg_frames_seen + 1)
            self.bg_frame = (1 - alpha) * self.bg_frame + alpha * depth_image
            self.bg_frames_seen += 1
            if should_render:
                self._render_debug(depth_image, fg_mask, [], depth_msg.header)
            return

        # 새 물체 후보 마스크 (배경보다 가까워진 영역)
        diff = self.bg_frame - depth_image  # positive if new object is closer
        fg_mask = (diff > self.depth_threshold_m) & (depth_image > 0)
        fg_mask = fg_mask.astype(np.uint8)
        fg_mask = self._apply_roi(fg_mask)

        if not np.any(fg_mask):
            if should_render:
                self._render_debug(depth_image, fg_mask, [], depth_msg.header)
            self._update_background(depth_image, fg_mask)
            self._handle_no_detections()
            return

        kernel = np.ones((3, 3), np.uint8)
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, kernel, iterations=1)
        fg_mask = cv2.dilate(fg_mask, kernel, iterations=2)  # fill small gaps

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(fg_mask, connectivity=8)

        detections: List[Dict] = []
        for label in range(1, num_labels):
            area = stats[label, cv2.CC_STAT_AREA]
            if area < self.min_region_area:
                continue

            label_mask = labels == label
            depth_vals = depth_image[label_mask]
            depth_vals = depth_vals[depth_vals > 0]
            if depth_vals.size == 0:
                continue

            anchor_mask = self._select_anchor_mask(label_mask, depth_image, depth_vals)
            ys, xs = np.nonzero(anchor_mask)
            if xs.size == 0:
                continue
            anchor_depths = depth_image[anchor_mask]
            anchor_depths = anchor_depths[anchor_depths > 0]
            if anchor_depths.size == 0:
                continue

            cx = float(xs.mean())
            cy = float(ys.mean())
            depth_m = float(np.median(anchor_depths))
            point = self._deproject(cx, cy, depth_m)
            if point is None:
                continue
            bbox = (
                int(stats[label, cv2.CC_STAT_LEFT]),
                int(stats[label, cv2.CC_STAT_TOP]),
                int(stats[label, cv2.CC_STAT_WIDTH]),
                int(stats[label, cv2.CC_STAT_HEIGHT]),
            )
            feature = self._compute_shape_feature(anchor_mask, bbox, anchor_depths)
            detections.append(
                {
                    "pixel": (float(cx), float(cy)),
                    "position": point,
                    "bbox": bbox,
                    "feature": feature,
                }
            )

        self._update_tracks(detections)

        if should_render:
            self._render_debug(depth_image, fg_mask, detections, depth_msg.header)

        self._update_background(depth_image, fg_mask)

    def _handle_no_detections(self):
        """Foreground가 없을 때 기존 트랙의 missed 카운트를 늘리고 필요시 제거."""
        to_delete = []
        for obj_id, obj in self.tracked.items():
            obj.missed += 1
            if obj.missed > self.max_missed_frames:
                to_delete.append(obj_id)
        for obj_id in to_delete:
            track = self.tracked.pop(obj_id, None)
            if track is not None:
                self._store_lost_track(obj_id, track, self.get_clock().now())

        if to_delete:
            self.get_logger().debug(f"Removed stale IDs: {to_delete}")

        if self.tracked:
            self._publish_current_objects([])  # notify subscribers no objects seen this frame
            self._update_stable_objects([], self.get_clock().now())

    def _update_tracks(self, detections: List[Dict]):
        """새 detection 목록을 기존 트랙과 최근접 매칭."""
        now = self.get_clock().now()
        self._predict_tracks(now)
        self._prune_lost_tracks(now)
        matched = set()
        current_objects = []

        for det in detections:
            # 각 detection을 기존 트랙과 최근접 매칭
            det_pos = det["position"]
            best_id = None
            best_dist = self.max_match_distance_m

            for obj_id, obj in self.tracked.items():
                if obj_id in matched:
                    continue
                dist = np.linalg.norm(det_pos - obj.position)
                if dist < best_dist:
                    best_dist = dist
                    best_id = obj_id

            if best_id is not None:
                # 기존 트랙 업데이트
                tracked_obj = self.tracked[best_id]
                self._apply_measurement(tracked_obj, det_pos, now, det.get("feature"))
                tracked_obj.pixel = det["pixel"]
                matched.add(best_id)
                current_objects.append((best_id, tracked_obj))
            else:
                # 새 detection → 새 ID 할당
                reused_id = self._reidentify_track(det, now)
                if reused_id is not None:
                    self.tracked[reused_id] = self._create_track(det, now)
                    matched.add(reused_id)
                    current_objects.append((reused_id, self.tracked[reused_id]))
                else:
                    obj_id = self.next_id
                    self.next_id += 1
                    self.tracked[obj_id] = self._create_track(det, now)
                    matched.add(obj_id)
                    current_objects.append((obj_id, self.tracked[obj_id]))

        for obj_id, obj in self.tracked.items():
            if obj_id in matched:
                continue
            obj.missed += 1
        # 오래 관측되지 않은 트랙 제거
        stale = [obj_id for obj_id, obj in self.tracked.items() if obj.missed > self.max_missed_frames]
        for obj_id in stale:
            track = self.tracked.pop(obj_id, None)
            if track is not None:
                self._store_lost_track(obj_id, track, now)

        self._publish_current_objects(current_objects)
        self._update_stable_objects(current_objects, now)

    def _publish_current_objects(self, objects: List[Tuple[int, TrackedObject]]):
        """현재 관측 중인 객체들을 Float32MultiArray로 패킹."""
        msg = Float32MultiArray()
        for obj_id, obj in objects:
            X, Y, Z = obj.position
            u, v = obj.pixel
            msg.data.extend([float(obj_id), X * 1000.0, Y * 1000.0, Z * 1000.0, u, v])
        self.objects_pub.publish(msg)

    def _update_stable_objects(self, objects: List[Tuple[int, TrackedObject]], timestamp: Time):
        active_ids = set()
        for obj_id, obj in objects:
            active_ids.add(obj_id)
            state = self.stable_states.get(obj_id)
            if state is None:
                self.stable_states[obj_id] = {
                    "frames": 1,
                    "first_seen": timestamp,
                    "last_seen": timestamp,
                    "last_event": None,
                }
            else:
                state["frames"] += 1
                state["last_seen"] = timestamp
            state = self.stable_states[obj_id]
            if self._stable_ready(state, timestamp):
                if self._emit_stable_event(obj_id, obj, timestamp):
                    state["last_event"] = timestamp
                    state["frames"] = 0  # reset to avoid immediate re-fire
                    state["first_seen"] = timestamp

        stale_keys = [obj_id for obj_id in self.stable_states.keys() if obj_id not in active_ids]
        for obj_id in stale_keys:
            self.stable_states.pop(obj_id, None)

    def _stable_ready(self, state: Dict[str, Any], timestamp: Time) -> bool:
        if state["frames"] < self.stable_min_frames:
            return False
        duration = self._dt_seconds(state["first_seen"], timestamp)
        if duration < self.stable_min_duration_sec:
            return False
        last_event = state.get("last_event")
        if last_event is not None:
            since_event = self._dt_seconds(last_event, timestamp)
            if since_event < self.stable_event_cooldown_sec:
                return False
        return True

    def _emit_stable_event(self, obj_id: int, obj: TrackedObject, timestamp: Time) -> bool:
        if self.stable_event_pub is None:
            return False
        payload = {
            "id": int(obj_id),
            "position_mm": [float(obj.position[0]) * 1000.0, float(obj.position[1]) * 1000.0, float(obj.position[2]) * 1000.0],
            "pixel": [float(obj.pixel[0]), float(obj.pixel[1])],
            "timestamp": timestamp.nanoseconds / 1e9,
        }
        if self.latest_debug_stamp is not None:
            payload["debug_stamp"] = self.latest_debug_stamp.nanoseconds / 1e9
        msg = String()
        msg.data = json.dumps(payload)
        self.stable_event_pub.publish(msg)
        self.get_logger().info(f"📦 Stable object detected → ID {obj_id}")
        return True

    def _create_track(self, detection: Dict, timestamp: Time) -> TrackedObject:
        """Detection으로부터 새로운 트랙 생성을 위한 초기 상태 구성."""
        state = np.zeros(6, dtype=np.float32)
        state[:3] = detection["position"]
        covariance = np.zeros((6, 6), dtype=np.float32)
        pos_var = self.motion_measurement_std ** 2
        vel_var = self.motion_initial_velocity_std ** 2
        np.fill_diagonal(covariance[:3, :3], pos_var)
        np.fill_diagonal(covariance[3:, 3:], vel_var)
        return TrackedObject(
            state=state,
            covariance=covariance,
            pixel=detection["pixel"],
            missed=0,
            last_seen=timestamp,
            filter_time=timestamp,
            shape_feature=detection.get("feature"),
        )

    def _store_lost_track(self, obj_id: int, track: TrackedObject, timestamp: Time):
        if not self.reid_enable or track.shape_feature is None:
            return
        self.lost_tracks[obj_id] = {
            "feature": track.shape_feature.copy(),
            "timestamp": timestamp,
        }
        # 메모리 폭주 방지를 위한 최대 보관 수
        if len(self.lost_tracks) > 32:
            oldest_id = min(
                self.lost_tracks.keys(),
                key=lambda tid: self.lost_tracks[tid]["timestamp"].nanoseconds,
            )
            self.lost_tracks.pop(oldest_id, None)

    def _prune_lost_tracks(self, current_time: Time):
        if not self.reid_enable:
            self.lost_tracks.clear()
            return
        to_remove = []
        for obj_id, record in self.lost_tracks.items():
            age = self._dt_seconds(record["timestamp"], current_time)
            if age > self.reid_max_age_sec:
                to_remove.append(obj_id)
        for obj_id in to_remove:
            self.lost_tracks.pop(obj_id, None)

    def _reidentify_track(self, detection: Dict, timestamp: Time) -> Optional[int]:
        if not self.reid_enable:
            return None
        feature = detection.get("feature")
        if feature is None:
            return None
        best_id = None
        best_dist = self.reid_feature_threshold
        for obj_id, record in self.lost_tracks.items():
            age = self._dt_seconds(record["timestamp"], timestamp)
            if age > self.reid_max_age_sec:
                continue
            dist = self._shape_distance(feature, record["feature"])
            if dist < best_dist:
                best_dist = dist
                best_id = obj_id
        if best_id is not None:
            self.lost_tracks.pop(best_id, None)
        return best_id

    @staticmethod
    def _shape_distance(feature_a: np.ndarray, feature_b: np.ndarray) -> float:
        if feature_a is None or feature_b is None:
            return np.inf
        if feature_a.shape != feature_b.shape:
            return np.inf
        return float(np.linalg.norm(feature_a - feature_b))

    def _predict_tracks(self, current_time: Time):
        """현재 시각까지 Kalman 예측 단계를 수행."""
        if not self.motion_model_enabled:
            return
        for track in self.tracked.values():
            self._kalman_predict(track, current_time)

    def _apply_measurement(
        self, track: TrackedObject, measurement: np.ndarray, timestamp: Time, feature: Optional[np.ndarray]
    ):
        """Detection 값을 이용해 트랙을 보정."""
        measurement = np.asarray(measurement, dtype=np.float32)
        if self.motion_model_enabled:
            self._kalman_correct(track, measurement)
        else:
            track.position = measurement
        track.last_seen = timestamp
        track.filter_time = timestamp
        track.missed = 0
        if feature is not None:
            track.shape_feature = feature

    def _kalman_predict(self, track: TrackedObject, current_time: Time):
        dt = self._dt_seconds(track.filter_time, current_time)
        if dt <= 0.0:
            track.filter_time = current_time
            return
        F = self._motion_transition(dt)
        Q = self._process_noise_matrix(dt)
        track.state = F @ track.state
        track.covariance = F @ track.covariance @ F.T + Q
        track.filter_time = current_time

    def _kalman_correct(self, track: TrackedObject, measurement: np.ndarray):
        H = self._measurement_matrix
        P = track.covariance
        innovation = measurement - H @ track.state
        S = H @ P @ H.T + self._measurement_cov
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            S_inv = np.linalg.pinv(S)
        K = P @ H.T @ S_inv
        track.state = track.state + K @ innovation
        track.covariance = (self._identity6 - K @ H) @ P
        # 수치적 비대칭 방지
        track.covariance = 0.5 * (track.covariance + track.covariance.T)

    def _process_noise_matrix(self, dt: float) -> np.ndarray:
        q = self.motion_process_std ** 2
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2
        Q = np.zeros((6, 6), dtype=np.float32)
        pos_var = 0.25 * dt4 * q
        cross = 0.5 * dt3 * q
        vel_var = dt2 * q
        for axis in range(3):
            i = axis
            v = axis + 3
            Q[i, i] = pos_var
            Q[i, v] = cross
            Q[v, i] = cross
            Q[v, v] = vel_var
        return Q

    def _motion_transition(self, dt: float) -> np.ndarray:
        F = self._identity6.copy()
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        return F

    @staticmethod
    def _dt_seconds(previous: Time, current: Time) -> float:
        if previous is None:
            return 0.0
        delta = (current - previous).nanoseconds / 1e9
        return max(float(delta), 0.0)

    def _update_background(self, depth_image: np.ndarray, fg_mask: np.ndarray):
        """배경 EMA 업데이트. 옵션에 따라 전경 영역을 제외."""
        if self.bg_frame is None:
            return
        valid = depth_image > 0
        if self.bg_freeze_foreground:
            update_mask = (fg_mask == 0) & valid
            if not np.any(update_mask):
                return
            self.bg_frame[update_mask] = (
                (1 - self.bg_alpha) * self.bg_frame[update_mask] + self.bg_alpha * depth_image[update_mask]
            )
        else:
            blended = (1 - self.bg_alpha) * self.bg_frame + self.bg_alpha * depth_image
            # 깊이 0인 구간은 그대로 유지
            self.bg_frame = np.where(valid, blended, self.bg_frame)

    def _render_debug(
        self,
        depth_image: np.ndarray,
        fg_mask: np.ndarray,
        detections: List[Dict],
        header: Optional[Header] = None,
    ):
        """Depth heatmap + mask + bounding boxes 시각화."""
        valid = depth_image[depth_image > 0]
        if valid.size == 0:
            return
        vmin = float(np.percentile(valid, 5))
        vmax = float(np.percentile(valid, 95))
        if vmax - vmin < 1e-4:
            return
        norm = (depth_image - vmin) / (vmax - vmin)
        norm = np.clip(norm, 0.0, 1.0)
        depth_vis = (norm * 255).astype(np.uint8)
        depth_vis[depth_image <= 0] = 0
        depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_TURBO)

        mask_vis = cv2.cvtColor((fg_mask * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
        display = cv2.addWeighted(depth_vis, 0.85, mask_vis, 0.15, 0)

        self._draw_roi(display, (255, 255, 255))

        # 중심점 표시는 생략 (물체 형상을 가리는 요소 제거)

        for obj_id, obj in self.tracked.items():
            u, v = obj.pixel
            if u <= 0 or v <= 0:
                continue
            text_x = int(u)
            text_y = int(max(0, v - 20))
            cv2.putText(
                display,
                f"ID{obj_id}",
                (text_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 0),
                2,
            )

        color_panel = self._make_color_panel(detections)

        if color_panel is not None:
            if color_panel.shape[0] != display.shape[0]:
                scale = display.shape[0] / float(color_panel.shape[0])
                new_width = max(1, int(color_panel.shape[1] * scale))
                color_panel = cv2.resize(color_panel, (new_width, display.shape[0]))
            combined = np.hstack([color_panel, display])
        else:
            combined = display

        self.latest_debug_frame = combined.copy()
        if header is not None:
            self.latest_debug_stamp = Time.from_msg(header.stamp)
        else:
            self.latest_debug_stamp = self.get_clock().now()

        self._publish_debug_image(combined, header)

        if not self.visualize:
            return

        self._ensure_window()
        if not self.window_ready:
            return

        cv2.imshow(self.window_name, combined)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            cv2.destroyWindow(self.window_name)
            self.window_ready = False
            self.visualize = False
            self.get_logger().info("Visualization disabled via key press (q).")

    def _publish_debug_image(self, display: np.ndarray, header: Optional[Header]):
        if self.debug_image_pub is None:
            return
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(display, encoding="bgr8")
        except CvBridgeError as err:
            self.get_logger().warn(f"Failed to convert debug image: {err}")
            return
        if header is not None:
            debug_msg.header = header
        else:
            debug_msg.header.stamp = self.get_clock().now().to_msg()
        self.debug_image_pub.publish(debug_msg)

    def _ensure_window(self):
        if not self.visualize or self.window_ready:
            return
        try:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 960, 540)
            if hasattr(cv2, "startWindowThread"):
                cv2.startWindowThread()
            self.window_ready = True
            self.get_logger().info(
                f"OpenCV window '{self.window_name}' initialized (close with 'q' or Ctrl+C)."
            )
        except cv2.error as err:
            self.window_ready = False
            self.visualize = False
            self.get_logger().error(f"Failed to create OpenCV window: {err}")

    # -------------------------------------------------------
    def destroy_node(self):
        if self.window_ready:
            try:
                cv2.destroyWindow(self.window_name)
            except cv2.error:
                pass
        super().destroy_node()

    # -------------------------------------------------------
    def _color_callback(self, color_msg: Image):
        try:
            color_frame = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        except CvBridgeError as err:
            self.get_logger().warn(f"Failed to convert color image: {err}")
            return
        with self.color_lock:
            self.latest_color_frame = color_frame
            self.latest_color_stamp = Time.from_msg(color_msg.header.stamp)

    def _make_color_panel(self, detections: List[Dict]) -> Optional[np.ndarray]:
        color_frame = self._get_latest_color_frame()
        if color_frame is None:
            return None

        panel = color_frame.copy()
        self._draw_roi(panel, (255, 255, 255))

        # 컬러 패널에서도 중심점 표시는 생략

        for obj_id, obj in self.tracked.items():
            u, v = obj.pixel
            if u <= 0 or v <= 0:
                continue
            text_x = int(u)
            text_y = int(max(0, v - 20))
            cv2.putText(
                panel,
                f"ID{obj_id}",
                (text_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
            )

        return panel

    def _get_latest_color_frame(self) -> Optional[np.ndarray]:
        if self.latest_color_frame is None:
            return None
        with self.color_lock:
            if self.latest_color_frame is None:
                return None
            return self.latest_color_frame.copy()


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseWorkspaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
