#!/usr/bin/env python3
import os, json, base64, asyncio, threading, queue, time
from io import BytesIO
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import pyrealsense2 as rs

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray, Bool
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from PIL import Image as PILImage
import websockets
from websockets.exceptions import InvalidStatusCode, InvalidMessage
import sys, signal
import inspect

# ------------------------------
# Realtime ROI Client (WS)
# ------------------------------
class RealtimeROIClient:
    """
    gpt-realtime-mini 로 ROI 이미지에서 컵 존재여부만 JSON으로 판단.
    반환 형식: {"cup_in_roi": bool, "confidence": float}
    - 단일 WebSocket 세션 유지
    - ROI 이미지는 JPEG(base64 data URL)로 전송
    - detail="low" 권장(저비용)
    """
    def __init__(self, api_key: Optional[str] = None, model: str = "gpt-realtime-mini", detail: str = "low"):
        self.api_key = api_key or os.getenv("OPENAI_API_KEY")
        if not self.api_key:
            raise RuntimeError("OPENAI_API_KEY env var not set")
        self.model = model
        self.detail = detail

        self._loop = asyncio.new_event_loop()
        self._ws = None
        self._req_q: "queue.Queue[Dict]" = queue.Queue(maxsize=3)
        self._res_q: "queue.Queue[Dict]" = queue.Queue(maxsize=3)
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    
        # Ready wait
        try:
            ready = self._res_q.get(timeout=15.0)  # 5 → 15초
        except queue.Empty:
            raise RuntimeError("Realtime client init timeout (no response from thread)")

        if ready != "__READY__":
            print("Realtime init detail:", ready)   # ← 실제 에러 내용을 보자
            raise RuntimeError("Realtime client failed to initialize")


    def close(self):
        try:
            self._req_q.put({"type": "__STOP__"})
            self._thread.join(timeout=2.0)
        except Exception:
            pass

    def classify_roi_bgr(self, roi_bgr) -> Dict:
        """OpenCV BGR ndarray → JSON 결과(동기 호출)"""
        rgb = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2RGB)
        img = PILImage.fromarray(rgb)
        buf = BytesIO()
        img.save(buf, format="JPEG", quality=80)
        data_url = "data:image/jpeg;base64," + base64.b64encode(buf.getvalue()).decode("ascii")

        self._req_q.put({"type": "classify", "data_url": data_url})
        return self._res_q.get()  # blocking

    # ---- internal ----
    def _run_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._main())

    async def _main(self):
        url = f"wss://api.openai.com/v1/realtime?model={self.model}"
        headers = [
            ("Authorization", f"Bearer {self.api_key}"),
            ("OpenAI-Beta", "realtime=v1"),
        ]

        # 디버깅: 실제 로드된 websockets 모듈/버전과 connect 시그니처 출력
        try:
            import websockets as _ws_mod
            print("websockets module:", getattr(_ws_mod, "__file__", "<unknown>"),
                "version:", getattr(_ws_mod, "__version__", "<unknown>"))
            import inspect
            print("connect signature:", inspect.signature(_ws_mod.connect))
        except Exception as _e:
            print("websockets introspection failed:", _e)

        try:
            import websockets
            ws = None
            try:
                # 기본 경로 (>=10 버전)
                ws = await websockets.connect(url, extra_headers=headers, ping_interval=20)
            except TypeError as te:
                # 구버전/변형에서 additional_headers 키를 요구하는 경우
                if "extra_headers" in str(te):
                    print("[compat] retry with additional_headers due to:", te)
                    ws = await websockets.connect(url, additional_headers=headers, ping_interval=20)
                else:
                    raise

            async with ws:
                self._ws = ws
                # 세션 설정
                await ws.send(json.dumps({
                    "type": "session.update",
                    "session": {
                        "modalities": ["text", "vision"],
                        "instructions": (
                            "You will receive ROI images. "
                            'Respond STRICTLY as JSON: {"cup_in_roi": true|false}. '
                            "Interpret cup as mug/paper cup/tumbler used for drinking. "
                            "If uncertain, return false."
                        )
                    }
                }))
                # 준비 완료 알림
                if self._res_q.empty():
                    self._res_q.put("__READY__")

                recv_task = asyncio.create_task(self._recv_loop(ws))

                while True:
                    # 블로킹 큐 → 스레드 풀로 넘겨서 non-blocking
                    req = await asyncio.get_event_loop().run_in_executor(None, self._req_q.get)
                    if req.get("type") == "__STOP__":
                        break

                    await ws.send(json.dumps({
                        "type": "conversation.item.create",
                        "item": {
                            "type": "message",
                            "role": "user",
                            "content": [
                                {"type": "input_text", "text": "ROI image for cup presence check."},
                                {"type": "input_image", "image_url": req["data_url"]},
                            ],
                        },
                    }))

                    await ws.send(json.dumps({
                        "type": "response.create",
                        "response": {"modalities": ["text"], "instructions": "JSON only"},
                    }))

                recv_task.cancel()

        except InvalidStatusCode as e:
            # HTTP 핸드셰이크 실패 (401/403/404/426 등)
            err = {"error": "InvalidStatusCode", "status": getattr(e, "status_code", None)}
            try:
                err["headers"] = dict(getattr(e, "headers", {}) or {})
            except Exception:
                pass
            if self._res_q.empty():
                self._res_q.put(err)
            return

        except InvalidMessage as e:
            if self._res_q.empty():
                self._res_q.put({"error": "InvalidMessage", "detail": str(e)})
            return

        except Exception as e:
            if self._res_q.empty():
                self._res_q.put({"error": "Exception", "detail": str(e)})
            return


    async def _recv_loop(self, ws):
        partial = ""
        async for msg in ws:
            evt = json.loads(msg)
            t = evt.get("type")
            if t == "response.output_text.delta":
                partial += evt.get("delta","")
            elif t in ("response.completed","response.done"):
                text = partial.strip()
                partial = ""
                try:
                    l = text.find("{"); r = text.rfind("}")
                    if l>=0 and r>l:
                        j = json.loads(text[l:r+1])
                    else:
                        j = {"cup_in_roi": False, "confidence": 0.0}
                except Exception:
                    j = {"cup_in_roi": False, "confidence": 0.0}
                self._res_q.put(j)

# ------------------------------
# Utilities
# ------------------------------
def clamp_roi(x, y, w, h, W, H):
    x = max(0, min(x, W-1)); y = max(0, min(y, H-1))
    w = max(1, min(w, W-x)); h = max(1, min(h, H-y))
    return x, y, w, h

# ------------------------------
# ROS2 Node (YOLO → Realtime 대체)
# ------------------------------
class RealtimeCupDetectionNode(Node):
    """
    - 픽셀 ROI만 잘라 Realtime API로 컵 존재여부 판단
    - True면 ROI '깊이 기반 대표 픽셀' (u,v)을 선택 → (personal만) 3D 복원 → 로봇 좌표로 변환
    - personal/pickup 모두 화면 시각화(대표 픽셀, 깊이 기반 근사 bbox)
    - 기존 토픽 인터페이스 유지
      * /cup_detections/personal_cup_st, /pickup_st1, /pickup_st2 (Bool)
      * /cup_stable_coordinates (Float32MultiArray)
    """
    def __init__(self):
        super().__init__("realtime_cup_detection_node")
        self.bridge = CvBridge()
        self.intrinsics: Optional[rs.intrinsics] = None
        self.latest_cv_color = None
        self.latest_cv_depth_mm = None

        # 파라미터
        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('info_topic',  '/camera/camera/aligned_depth_to_color/camera_info')

        self.declare_parameter('visualize', True)
        self.declare_parameter('verify_fps', 1.0)           # Realtime 호출 FPS (1~2 추천)
        self.declare_parameter('min_confidence', 0.7)       # 컵 판정 최소 신뢰도
        self.declare_parameter('detail', 'low')             # low 권장
        
        self.declare_parameter('roi_personal_cup_st', [820, 260, 240, 240])
        self.declare_parameter('roi_pickup_st1',      [600, 520, 220, 220])
        self.declare_parameter('roi_pickup_st2',      [860, 520, 220, 220])

        # 깊이 기반 대표 픽셀/박스 선택 파라미터
        self.declare_parameter('depth_top_percent', 0.10)   # 가까운 상위 비율
        self.declare_parameter('depth_minpix', 80)          # 최소 유효 픽셀 수

        self.color_topic = self.get_parameter('color_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.info_topic  = self.get_parameter('info_topic').value
        self.visualize = bool(self.get_parameter('visualize').value)
        self.verify_fps = float(self.get_parameter('verify_fps').value)
        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.detail = str(self.get_parameter('detail').value)
        
        def _read_roi_param(param_name: str):
            arr = list(self.get_parameter(param_name).value)
            if len(arr) != 4:
                raise ValueError(f"{param_name} must be [x,y,w,h], got {arr}")
            return [int(arr[0]), int(arr[1]), int(arr[2]), int(arr[3])]

        p_roi  = _read_roi_param('roi_personal_cup_st')
        s1_roi = _read_roi_param('roi_pickup_st1')
        s2_roi = _read_roi_param('roi_pickup_st2')

        self.pixel_rois: Dict[str, list] = {
            "personal_cup_st": p_roi,
            "pickup_st1":      s1_roi,
            "pickup_st2":      s2_roi,
        }


        self.depth_top_percent = float(self.get_parameter('depth_top_percent').value)
        self.depth_minpix = int(self.get_parameter('depth_minpix').value)

        # 제어 플래그
        self._detect_personal = False
        self._detect_pickup = False
        self._publish_coords_active = False

        # 안정화 버퍼
        self.detect_buf = []
        self.detect_delay_sec = 0.8
        self.stable_radius = 0.01
        self.first_detect_time = None

        # --- visualization state ---
        self.last_personal_uv = None
        self.last_personal_robot = None
        self.last_personal_bbox = None

        self.last_pick1_uv = None
        self.last_pick1_bbox = None

        self.last_pick2_uv = None
        self.last_pick2_bbox = None

        # 구독
        self.color_sub = message_filters.Subscriber(self, Image, self.color_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, self.depth_topic)
        self.info_sub  = message_filters.Subscriber(self, CameraInfo, self.info_topic)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.info_sub], queue_size=10, slop=0.1
        )
        self.ts.registerCallback(self._camera_callback)

        # 제어 토픽(기존과 동일)
        self.create_subscription(Bool, '/check_cup', self._on_check_cup, 10)
        self.create_subscription(Bool, '/check_cup_done', self._on_check_cup_done, 10)
        self.create_subscription(Bool, '/call_cup_stable_coordinates', self._on_call_coords, 10)
        self.create_subscription(Bool, '/arrive_cup_stable_coordinates', self._on_arrive_coords, 10)

        # 발행
        self.pub_personal = self.create_publisher(Bool, '/cup_detections/personal_cup_st', 10)
        self.pub_pick1    = self.create_publisher(Bool, '/cup_detections/pickup_st1', 10)
        self.pub_pick2    = self.create_publisher(Bool, '/cup_detections/pickup_st2', 10)
        self.cup_coord_pub = self.create_publisher(Float32MultiArray, '/cup_stable_coordinates', 10)

        # Realtime 클라이언트
        try:
            self.rt = RealtimeROIClient(detail=self.detail)
            self.get_logger().info(f"🔌 Realtime ready (detail={self.detail})")
        except Exception as e:
            self.get_logger().error(f"Realtime init failed: {e}")
            raise

        self._next_rt_time = 0.0

        self.get_logger().info("="*60)
        self.get_logger().info("✅ Realtime Cup Detection Node Started (YOLO-free, depth-based rep. pixel)")
        self.get_logger().info(f"   verify_fps={self.verify_fps}  min_conf={self.min_confidence}")
        self.get_logger().info("="*60)

    # ----------- controls (same API) -----------
    def _on_check_cup(self, msg: Bool):
        if msg.data and not self._detect_pickup:
            self._detect_pickup = True
            self.get_logger().info("🟢 Pickup detection STARTED")
        elif (not msg.data) and self._detect_pickup:
            self._detect_pickup = False
            self._publish_pickup(False, False)
            self.get_logger().info("🔴 Pickup detection STOPPED")

    def _on_check_cup_done(self, msg: Bool):
        if msg.data and self._detect_pickup:
            self._detect_pickup = False
            self._publish_pickup(False, False)
            self.get_logger().info("🔴 Pickup detection STOPPED (done)")

    def _on_call_coords(self, msg: Bool):
        if msg.data and not self._detect_personal:
            self._detect_personal = True
            self._publish_coords_active = True
            self._reset_buf()
            self.get_logger().info("🟢 Personal cup detection + coords STARTED")
        elif (not msg.data) and self._detect_personal:
            self._detect_personal = False
            self._publish_coords_active = False
            self._reset_buf()
            self.pub_personal.publish(Bool(data=False))
            self.get_logger().info("🔴 Personal cup detection + coords STOPPED")

    def _on_arrive_coords(self, msg: Bool):
        if msg.data and self._detect_personal:
            self._detect_personal = False
            self._publish_coords_active = False
            self._reset_buf()
            self.pub_personal.publish(Bool(data=False))
            self.get_logger().info("🔴 Personal cup detection + coords STOPPED (arrived)")

    def _publish_pickup(self, st1: bool, st2: bool):
        self.pub_pick1.publish(Bool(data=st1))
        self.pub_pick2.publish(Bool(data=st2))

    def _reset_buf(self):
        self.detect_buf.clear()
        self.first_detect_time = None

    # ----------- coordinates transform -----------
    def _transform_to_robot_coords(self, X: float, Y: float, Z: float) -> list:
        # 기존 네 좌표식 그대로 유지
        x_mm = Y * 1000
        y_mm = X * 1000
        z_mm = Z * 1000
        final_x = 777 + x_mm - 249 + 262
        final_y = y_mm - 265
        final_z = 970 - z_mm - 200
        if final_z <= 55:
            final_z = 55
        return [float(final_x), float(final_y), float(final_z)]

    # ----------- depth-based representative pixel in ROI -----------
    def _pick_uv_by_depth_roi(self, depth_mm, roi, top_percent=0.10, min_valid=50):
        """
        ROI 안에서 '가까운(작은 깊이)' 상위 top_percent 픽셀을 골라
        그들의 무게중심(평균 위치)을 대표 픽셀로 선택. 실패 시 None.
        """
        x, y, w, h = roi
        patch = depth_mm[y:y+h, x:x+w].astype(np.int32)
        valid = (patch > 0) & (patch < 10000)
        if valid.sum() < min_valid:
            return None

        vals = patch[valid]
        k = max(1, int(len(vals) * top_percent))
        # k번째로 작은 값(상위 k개 가까운 픽셀의 임계)
        thresh = np.partition(vals, k-1)[k-1]
        mask = valid & (patch <= thresh + 5)  # 여유 5mm

        m = (mask.astype(np.uint8) * 255)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3,3), np.uint8), iterations=1)

        ys, xs = np.where(m > 0)
        if len(xs) < min_valid:
            return None

        u = int(x + xs.mean())
        v = int(y + ys.mean())
        return (u, v)

    def _read_depth_with_fallback(self, depth_mm, u, v):
        """
        (u,v)에서 깊이가 0이면 주변 윈도우(홀수 크기)를 늘려가며
        유효 픽셀의 중앙값을 사용. 그래도 없으면 None.
        """
        H, W = depth_mm.shape[:2]
        if 0 <= v < H and 0 <= u < W and depth_mm[v, u] > 0:
            return int(depth_mm[v, u])
        for win in (3, 5, 7):
            r = win // 2
            x0, x1 = max(0, u-r), min(W, u+r+1)
            y0, y1 = max(0, v-r), min(H, v+r+1)
            patch = depth_mm[y0:y1, x0:x1]
            vals = patch[(patch > 0) & (patch < 10000)].astype(np.int32)
            if len(vals) >= 10:
                return int(np.median(vals))
        return None

    # ----------- depth-based bbox in ROI -----------
    def _depth_bbox_in_roi(self, depth_mm, roi, top_percent=0.10, minpix=80):
        """
        ROI 내에서 깊이가 가까운(작은) 상위 top_percent 픽셀로 마스크 만들고
        가장 큰 연결 성분의 외접 사각형을 반환.
        반환: (x,y,w,h)  *원본 이미지 좌표*, 실패 시 None
        """
        x, y, w, h = roi
        patch = depth_mm[y:y+h, x:x+w].astype(np.int32)
        valid = (patch > 0) & (patch < 10000)
        if valid.sum() < minpix:
            return None

        vals = patch[valid]
        k = max(1, int(len(vals) * top_percent))
        # 가까운 상위 k개 임계값
        thresh = np.partition(vals, k-1)[k-1]
        mask = valid & (patch <= thresh + 5)

        m = (mask.astype(np.uint8) * 255)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3,3), np.uint8), iterations=1)

        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None
        c = max(cnts, key=cv2.contourArea)
        rx, ry, rw, rh = cv2.boundingRect(c)
        return (x + rx, y + ry, rw, rh)

    # ----------- main camera callback -----------
    def _camera_callback(self, color_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        try:
            self.latest_cv_color = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            self.latest_cv_depth_mm = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # intrinsics 1회 설정
        if self.intrinsics is None:
            self.intrinsics = rs.intrinsics()
            self.intrinsics.width = info_msg.width
            self.intrinsics.height = info_msg.height
            self.intrinsics.ppx = info_msg.k[2]
            self.intrinsics.ppy = info_msg.k[5]
            self.intrinsics.fx = info_msg.k[0]
            self.intrinsics.fy = info_msg.k[4]
            self.intrinsics.model = rs.distortion.brown_conrady if info_msg.distortion_model in ['plumb_bob','rational_polynomial'] else rs.distortion.none
            self.intrinsics.coeffs = list(info_msg.d)
            self.get_logger().info("📷 Camera intrinsics configured")

        H, W = self.latest_cv_color.shape[:2]
        now = time.time()
        can_call = now >= self._next_rt_time

        detected_personal = False
        detected_pick1 = False
        detected_pick2 = False
        personal_coords_cam = None

        # 초기화(표시 리셋; 매 프레임 덮어씀)
        self.last_personal_uv = None
        self.last_personal_robot = None
        self.last_personal_bbox = None
        self.last_pick1_uv = None
        self.last_pick1_bbox = None
        self.last_pick2_uv = None
        self.last_pick2_bbox = None

        
        # Realtime 호출(스로틀): 지정된 FPS로만
        if can_call:
            self._next_rt_time = now + 1.0 / max(1e-6, self.verify_fps)

            try:
                if self._detect_pickup:
                    # pickup_st1
                    if "pickup_st1" in self.pixel_rois:
                        x, y, w, h = clamp_roi(*self.pixel_rois["pickup_st1"], W, H)
                        crop = self.latest_cv_color[y:y+h, x:x+w]
                        r = self.rt.classify_roi_bgr(crop)

                        # ✅ confidence 완전 제거: cup_in_roi만 사용
                        if bool(r.get("cup_in_roi", False)):
                            detected_pick1 = True
                            # 깊이 기반 대표 픽셀/박스
                            uv = self._pick_uv_by_depth_roi(
                                self.latest_cv_depth_mm, (x, y, w, h),
                                top_percent=self.depth_top_percent,
                                min_valid=self.depth_minpix
                            )
                            if uv is None:
                                uv = (x + w // 2, y + h // 2)  # 폴백: 중심
                            self.last_pick1_uv = tuple(map(int, uv))
                            self.last_pick1_bbox = self._depth_bbox_in_roi(
                                self.latest_cv_depth_mm, (x, y, w, h),
                                top_percent=self.depth_top_percent,
                                minpix=self.depth_minpix
                            )

                    # pickup_st2
                    if "pickup_st2" in self.pixel_rois:
                        x, y, w, h = clamp_roi(*self.pixel_rois["pickup_st2"], W, H)
                        crop = self.latest_cv_color[y:y+h, x:x+w]
                        r = self.rt.classify_roi_bgr(crop)

                        # ✅ confidence 완전 제거
                        if bool(r.get("cup_in_roi", False)):
                            detected_pick2 = True
                            uv = self._pick_uv_by_depth_roi(
                                self.latest_cv_depth_mm, (x, y, w, h),
                                top_percent=self.depth_top_percent,
                                min_valid=self.depth_minpix
                            )
                            if uv is None:
                                uv = (x + w // 2, y + h // 2)
                            self.last_pick2_uv = tuple(map(int, uv))
                            self.last_pick2_bbox = self._depth_bbox_in_roi(
                                self.latest_cv_depth_mm, (x, y, w, h),
                                top_percent=self.depth_top_percent,
                                minpix=self.depth_minpix
                            )

                if self._detect_personal:
                    if "personal_cup_st" in self.pixel_rois:
                        x, y, w, h = clamp_roi(*self.pixel_rois["personal_cup_st"], W, H)
                        crop = self.latest_cv_color[y:y+h, x:x+w]
                        r = self.rt.classify_roi_bgr(crop)

                        # ✅ confidence 완전 제거
                        if bool(r.get("cup_in_roi", False)):
                            detected_personal = True
                            # 깊기 기반 대표 픽셀 선택
                            uv = self._pick_uv_by_depth_roi(
                                self.latest_cv_depth_mm, (x, y, w, h),
                                top_percent=self.depth_top_percent,
                                min_valid=self.depth_minpix
                            )
                            if uv is None:
                                uv = (x + w // 2, y + h // 2)  # 폴백

                            u, v = map(int, uv)
                            d_mm = self._read_depth_with_fallback(self.latest_cv_depth_mm, u, v)
                            if d_mm is not None and d_mm > 0:
                                depth_m = d_mm / 1000.0
                                X, Y, Z = rs.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], depth_m)
                                personal_coords_cam = (float(X), float(Y), float(Z))

                                # 시각화용 저장
                                self.last_personal_uv = (u, v)
                                self.last_personal_robot = self._transform_to_robot_coords(X, Y, Z)
                                self.last_personal_bbox = self._depth_bbox_in_roi(
                                    self.latest_cv_depth_mm, (x, y, w, h),
                                    top_percent=self.depth_top_percent,
                                    minpix=self.depth_minpix
                                )

            except Exception as e:
                self.get_logger().warn(f"Realtime error: {e}")


        # 토픽 퍼블리시
        if self._detect_personal:
            self.pub_personal.publish(Bool(data=detected_personal))
        if self._detect_pickup:
            self._publish_pickup(detected_pick1, detected_pick2)

        # 개인컵 좌표 안정화 & 퍼블리시
        if self._publish_coords_active and personal_coords_cam is not None:
            self._process_personal_coords(personal_coords_cam)

        # 시각화
        if self.visualize:
            disp = self.latest_cv_color.copy()
            # ROI 박스 표시
            for name, rect in self.pixel_rois.items():
                x,y,w,h = clamp_roi(*rect, W, H)
                color = (0,255,0) if (
                    (name=="personal_cup_st" and detected_personal) or
                    (name=="pickup_st1" and detected_pick1) or
                    (name=="pickup_st2" and detected_pick2)
                ) else (128,128,128)
                cv2.rectangle(disp, (x,y), (x+w, y+h), color, 2)
                cv2.putText(disp, name, (x, max(0,y-6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # --- personal overlay ---
            if self.last_personal_uv is not None:
                u, v = self.last_personal_uv
                cv2.drawMarker(disp, (u, v), (0, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=16, thickness=2)
                cv2.circle(disp, (u, v), 4, (0, 255, 255), -1)
            if self.last_personal_bbox is not None:
                bx, by, bw, bh = self.last_personal_bbox
                cv2.rectangle(disp, (bx, by), (bx+bw, by+bh), (0, 255, 255), 2)
            if self.last_personal_robot is not None and self.last_personal_uv is not None:
                rx, ry, rz = self.last_personal_robot
                u, v = self.last_personal_uv
                info = f"uv=({u},{v})  robot=[{rx:.1f},{ry:.1f},{rz:.1f}]"
                cv2.putText(disp, info, (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
                cv2.putText(disp, "personal target", (u+8, v-8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)

            # --- pickup overlays ---
            if self.last_pick1_uv is not None:
                u, v = self.last_pick1_uv
                cv2.drawMarker(disp, (u, v), (255, 200, 0), markerType=cv2.MARKER_CROSS, markerSize=16, thickness=2)
                cv2.circle(disp, (u, v), 4, (255, 200, 0), -1)
            if self.last_pick1_bbox is not None:
                bx, by, bw, bh = self.last_pick1_bbox
                cv2.rectangle(disp, (bx, by), (bx+bw, by+bh), (255, 200, 0), 2)
                cv2.putText(disp, "pickup_st1 target", (bx, max(0, by-8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,200,0), 2)

            if self.last_pick2_uv is not None:
                u, v = self.last_pick2_uv
                cv2.drawMarker(disp, (u, v), (255, 0, 200), markerType=cv2.MARKER_CROSS, markerSize=16, thickness=2)
                cv2.circle(disp, (u, v), 4, (255, 0, 200), -1)
            if self.last_pick2_bbox is not None:
                bx, by, bw, bh = self.last_pick2_bbox
                cv2.rectangle(disp, (bx, by), (bx+bw, by+bh), (255, 0, 200), 2)
                cv2.putText(disp, "pickup_st2 target", (bx, max(0, by-8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,200), 2)

            cv2.putText(disp, f"Personal: {'ACTIVE' if self._detect_personal else 'IDLE'}", (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0) if self._detect_personal else (150,150,150), 2)
            cv2.putText(disp, f"Pickup:   {'ACTIVE' if self._detect_pickup else 'IDLE'}", (10,60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0) if self._detect_pickup else (150,150,150), 2)
            cv2.imshow("Realtime Cup Detection", disp)
            cv2.waitKey(1)

    # 좌표 안정화 (기존 로직 간소화 버전)
    def _process_personal_coords(self, coords):
        X, Y, Z = coords
        now = self.get_clock().now()

        if self.first_detect_time is None:
            self.first_detect_time = now
            self.detect_buf = [(X,Y,Z)]
            self.get_logger().info("🔍 Personal cup detected - collecting samples...")
            return

        self.detect_buf.append((X,Y,Z))
        elapsed = (now - self.first_detect_time).nanoseconds / 1e9
        if elapsed < self.detect_delay_sec:
            return

        xs = [p[0] for p in self.detect_buf]
        ys = [p[1] for p in self.detect_buf]
        zs = [p[2] for p in self.detect_buf]
        Xavg, Yavg, Zavg = sum(xs)/len(xs), sum(ys)/len(ys), sum(zs)/len(zs)

        # 간단한 안정성 체크
        if any(abs(px - Xavg) > self.stable_radius for px in xs) \
           or any(abs(py - Yavg) > self.stable_radius for py in ys) \
           or any(abs(pz - Zavg) > self.stable_radius for pz in zs):
            self.first_detect_time = now
            return

        robot_xyz = self._transform_to_robot_coords(Xavg, Yavg, Zavg)
        msg = Float32MultiArray(); msg.data = robot_xyz
        self.cup_coord_pub.publish(msg)
        self.get_logger().info(f"📢 Cup coordinates published (robot): {robot_xyz}")
        self.detect_buf.clear()
        self.first_detect_time = None

# ------------------------------
# main
# ------------------------------
def main(args=None):
    rclpy.init(args=args)

    # SIGINT/SIGTERM도 깔끔히 처리 (Ctrl+C 외 종료 신호 대비)
    def _handle_signal(signum, frame):
        raise KeyboardInterrupt
    for s in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(s, _handle_signal)
        except Exception:
            pass

    node = None
    try:
        # 💡 여기서 RealtimeROIClient 초기화가 실패하면 예외가 터짐
        node = RealtimeCupDetectionNode()
    except Exception as e:
        # 초기화 실패 원인 바로 출력
        print("\n[FATAL] Node initialization failed.")
        # 디버깅 도움: 환경변수 유무도 함께 출력
        print("OPENAI_API_KEY set?:", bool(os.getenv("OPENAI_API_KEY")))
        print("Error:", repr(e))
        # (만약 RealtimeROIClient에서 큐로 넘긴 상세 에러를 main에서 받도록 했다면 여기서도 출력)
        rclpy.shutdown()
        sys.exit(1)

    try:
        # 필요하면 spin_once 루프 대신 간단히 spin 사용
        rclpy.spin(node)
        # 만약 spin_once를 유지하고 싶다면 아래처럼:
        # while rclpy.ok():
        #     rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        # Realtime WS 안전 종료
        try:
            if hasattr(node, "rt") and node.rt:
                node.rt.close()
        except Exception:
            pass
        # OpenCV 창 정리
        try:
            if getattr(node, "visualize", False):
                cv2.destroyAllWindows()
        except Exception:
            pass
        # 노드 파괴 + rclpy 종료
        try:
            if node is not None:
                node.destroy_node()
        finally:
            rclpy.shutdown()

if __name__ == "__main__":
    main()
