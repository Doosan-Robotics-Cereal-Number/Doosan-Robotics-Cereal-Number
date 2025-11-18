# gwonho_orchestrator/network_manager.py (v7 - go_home added)
import json
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String, Float32, Bool, Float32MultiArray
from gwonho_interfaces.action import RunMotion


# 한글/영문 모두 수용
PORTION_MAP = {
    "적게": 100.0, "보통": 200.0, "많이": 300.0,
    "small": 100.0, "medium": 200.0, "large": 300.0,
}

# 컵 타입
CUP_TYPE_MAP = {
    "개인컵": "personal",
    "매장컵": "store",
    "personal": "personal",
    "store": "store",
}


class NetworkManager(Node):
    """
    v7: go_home 추가
    - 모든 시퀀스 마지막에 go_home 실행
    - check_cup 토픽 발행
    - personal_cup_st, pickup_st1, pickup_st2 구독
    - 조건에 따라 pickup 위치 결정
    """

    def __init__(self):
        super().__init__('network_manager')

        # 파라미터
        self.declare_parameter('robot_id', 'dsr01')
        self.robot_id = self.get_parameter('robot_id').value

        # 상태
        self._busy_lock = threading.Lock()  # 중복 주문 방지
        self._current_weight_a = 0.0  # 저울 A 현재 무게
        self._current_weight_b = 0.0  # 저울 B 현재 무게
        self._target_weight = 0.0     # 목표 무게
        self._monitoring = False       # 무게 모니터링 활성화 플래그
        self._active_scale = None      # 'a' or 'b' - 현재 사용 중인 저울
        
        # 🔥 컵 감지 상태
        self._personal_cup_detected = False
        self._pickup_st1_detected = False
        self._pickup_st2_detected = False
        
        # 🔥 개인컵 좌표 수신 상태
        self._personal_cup_coords = None
        self._personal_cup_coords_event = threading.Event()

        # ActionClient (절대 경로)
        action_name = f'/{self.robot_id}/motion_control/run'
        self.get_logger().info(f"[NetworkManager] Creating ActionClient for: '{action_name}'")
        self._ac = ActionClient(self, RunMotion, action_name)

        # 주문 구독 (상대 경로)
        self.create_subscription(String, 'kiosk/order', self._on_order, 10)
        
        # 🔥 각 저울 독립 구독 (절대 경로)
        weight_topic_a = f'/{self.robot_id}/weight/scale_a/current'
        weight_topic_b = f'/{self.robot_id}/weight/scale_b/current'
        self.create_subscription(Float32, weight_topic_a, self._on_weight_value_a, 10)
        self.create_subscription(Float32, weight_topic_b, self._on_weight_value_b, 10)
        
        # 🔥 컵 감지 토픽 구독
        self.create_subscription(Bool, '/cup_detections/personal_cup_st', self._on_personal_cup, 10)
        self.create_subscription(Bool, '/cup_detections/pickup_st1', self._on_pickup_st1, 10)
        self.create_subscription(Bool, '/cup_detections/pickup_st2', self._on_pickup_st2, 10)
        
        # 🔥 개인컵 좌표 구독
        self.create_subscription(Float32MultiArray, '/cup_stable_coordinates', self._on_cup_coords, 10)
        
        # 🔥 토픽 발행자들
        weight_ok_topic = f'/{self.robot_id}/weight_ok'
        self._weight_ok_pub = self.create_publisher(Bool, weight_ok_topic, 10)
        
        self._check_cup_pub = self.create_publisher(Bool, '/check_cup', 10)
        self._check_cup_done_pub = self.create_publisher(Bool, '/check_cup_done', 10)
        
        # 🔥 개인컵 감지 제어 토픽
        self._call_cup_coords_pub = self.create_publisher(Bool, '/call_cup_stable_coordinates', 10)
        self._arrive_cup_coords_pub = self.create_publisher(Bool, '/arrive_cup_stable_coordinates', 10)
        
        # 🔥 Motion Control에게 좌표 전달용 Publisher (같은 토픽 재사용)
        self._cup_coords_to_motion_pub = self.create_publisher(Float32MultiArray, '/cup_stable_coordinates', 10)

        self.get_logger().info("=" * 60)
        self.get_logger().info(f"[NetworkManager] namespace: {self.get_namespace()}")
        self.get_logger().info(f"[NetworkManager] Subscribed to:")
        self.get_logger().info(f"  - kiosk/order (String)")
        self.get_logger().info(f"  - {weight_topic_a} (Float32)")
        self.get_logger().info(f"  - {weight_topic_b} (Float32)")
        self.get_logger().info(f"  - /cup_detections/personal_cup_st (Bool)")
        self.get_logger().info(f"  - /cup_detections/pickup_st1 (Bool)")
        self.get_logger().info(f"  - /cup_detections/pickup_st2 (Bool)")
        self.get_logger().info(f"[NetworkManager] Publishing:")
        self.get_logger().info(f"  - {weight_ok_topic} (Bool)")
        self.get_logger().info(f"  - /check_cup (Bool)")
        self.get_logger().info(f"  - /check_cup_done (Bool)")
        self.get_logger().info("=" * 60)

    # -------------------------
    # 🔥 컵 감지 콜백 함수들
    # -------------------------
    def _on_personal_cup(self, msg: Bool):
        """개인컵 감지"""
        self._personal_cup_detected = msg.data
        if msg.data:
            self.get_logger().info("🔍 개인컵 탐지")

    def _on_pickup_st1(self, msg: Bool):
        """Pickup 위치 1 감지"""
        self._pickup_st1_detected = msg.data
        if msg.data:
            self.get_logger().debug("📍 Pickup position 1 detected")

    def _on_pickup_st2(self, msg: Bool):
        """Pickup 위치 2 감지"""
        self._pickup_st2_detected = msg.data
        if msg.data:
            self.get_logger().debug("📍 Pickup position 2 detected")
    
    def _on_cup_coords(self, msg: Float32MultiArray):
        """개인컵 좌표 수신"""
        if len(msg.data) >= 3:
            self._personal_cup_coords = list(msg.data[:3])
            self._personal_cup_coords_event.set()
            self.get_logger().info(f"✅ Personal cup coordinates received: {self._personal_cup_coords}")

    # -------------------------
    # 주문 수신 및 파싱
    # -------------------------
    def _on_order(self, msg: String):
        """
        주문 메시지 파싱
        - JSON: {"seat":"A", "portion":"많이", "cup_type":"개인컵"}
        - 쉼표: "start_sequence_a,많이,개인컵"
        """
        raw = (msg.data or "").strip()
        self.get_logger().info(f"🧾 order msg: {raw!r}")
        if not raw:
            self.get_logger().error("❌ 빈 주문 메시지 수신")
            return

        seat_cmd, target_g, cup_type = None, None, "store"  # 기본값: 매장컵
        
        # 1) JSON 파싱 시도
        try:
            payload = json.loads(raw)
            seat = payload.get("seat")
            if seat:
                seat = str(seat).strip().upper()
                if seat in ['A', 'B']:
                    seat_cmd = f"start_sequence_{seat.lower()}"
                else:
                    self.get_logger().error(f"❌ 잘못된 자리: '{seat}' (A 또는 B만 가능)")
                    return
            else:
                seat_cmd = payload.get("cmd") or payload.get("command")

            portion = str(payload.get("portion", "")).strip()
            target_g = PORTION_MAP.get(portion)
            
            # 🔥 컵 타입 파싱
            cup_type_raw = str(payload.get("cup_type", "매장컵")).strip()
            cup_type = CUP_TYPE_MAP.get(cup_type_raw, "store")
            
        except Exception:
            # 2) 쉼표 구분 파싱
            try:
                parts = [x.strip() for x in raw.split(',')]
                if len(parts) < 2:
                    raise ValueError("Invalid format")
                
                seat_cmd = parts[0]
                target_g = PORTION_MAP.get(parts[1])
                
                # 🔥 컵 타입 파싱 (선택적)
                if len(parts) >= 3:
                    cup_type = CUP_TYPE_MAP.get(parts[2], "store")
                else:
                    cup_type = "store"  # 기본값
                
            except Exception:
                self.get_logger().error(
                    f"❌ 잘못된 주문 형식: '{raw}'\n"
                    f"   예시 1: 'start_sequence_a,많이,개인컵'\n"
                    f"   예시 2: '{{\"seat\":\"A\",\"portion\":\"많이\",\"cup_type\":\"개인컵\"}}'"
                )
                return

        if not seat_cmd:
            self.get_logger().error(f"❌ 자리/명령을 해석할 수 없음: '{raw}'")
            return
        
        if target_g is None:
            self.get_logger().error(
                f"❌ 용량을 해석할 수 없음: '{raw}'\n"
                f"   가능: 적게/보통/많이 또는 small/medium/large"
            )
            return

        # 바쁠 때 무시
        if not self._busy_lock.acquire(blocking=False):
            self.get_logger().warn("⚠️  Busy 상태: 새 주문 무시")
            return

        # 백그라운드 스레드로 시퀀스 실행
        try:
            if seat_cmd == "start_sequence_a":
                t = threading.Thread(target=self._sequence_a, args=(target_g, cup_type), daemon=True)
                t.start()
            elif seat_cmd == "start_sequence_b":
                t = threading.Thread(target=self._sequence_b, args=(target_g, cup_type), daemon=True)
                t.start()
            else:
                self.get_logger().error(f"❌ 알 수 없는 명령: {seat_cmd}")
                self._busy_lock.release()
        except Exception as e:
            self.get_logger().error(f"❌ 시퀀스 시작 오류: {e}")
            self._busy_lock.release()

    # -------------------------
    # 공통: 모션 실행
    # -------------------------
    def run_motion(self, name: str, params: dict, timeout_s: float):
        """ActionClient를 이용해 RunMotion 실행"""
        self.get_logger().info(f"🎯 Running motion: '{name}'  params={params}")

        goal = RunMotion.Goal()
        goal.name = name
        goal.params_json = json.dumps(params or {})

        # 목표 전송
        send_future = self._ac.send_goal_async(goal, feedback_callback=self._on_feedback)
        start = time.time()
        while not send_future.done():
            if timeout_s and (time.time() - start) > timeout_s:
                self.get_logger().error(f"❌ send_goal timeout after {time.time() - start:.1f}s")
                return False, "send_timeout"
            time.sleep(0.01)

        try:
            gh = send_future.result()
        except Exception as e:
            self.get_logger().error(f"❌ Failed to send goal: {e}")
            return False, f"send_failed: {e}"

        if not gh or not gh.accepted:
            self.get_logger().error("❌ Goal rejected by server")
            return False, "rejected"

        self.get_logger().info(f"✅ Goal '{name}' accepted → waiting result...")

        # 결과 대기
        res_future = gh.get_result_async()
        start = time.time()
        while not res_future.done():
            if timeout_s and (time.time() - start) > timeout_s:
                self.get_logger().error("❌ get_result timeout → canceling goal...")
                try:
                    gh.cancel_goal_async()
                except Exception:
                    pass
                return False, "result_timeout"
            time.sleep(0.01)

        res = res_future.result().result
        ok = bool(res.success)
        self.get_logger().info(f"✅ Result: success={ok}, detail='{res.detail}'")
        return ok, res.detail

    def _on_feedback(self, fb_msg):
        """피드백 수신"""
        fb = fb_msg.feedback
        self.get_logger().info(f"  [{fb.progress*100:.0f}%] {fb.stage}")

    # -------------------------
    # 🔥 컵 감지 및 pickup 위치 결정
    # -------------------------
    def _wait_for_personal_cup(self, timeout_s: float = 30.0) -> bool:
        """
        개인컵 좌표 대기
        Returns:
            True: 개인컵 좌표 수신
            False: 타임아웃 (매장컵으로 전환)
        """
        # 이벤트 초기화
        self._personal_cup_coords = None
        self._personal_cup_coords_event.clear()
        
        # call 신호 발행
        self._call_cup_coords_pub.publish(Bool(data=True))
        self.get_logger().info("📸 개인컵 감지 요청 발행")
        
        # 대기 시작
        self.get_logger().info("⏳ 개인컵을 올려주세요...")
        start_time = time.time()
        
        while (time.time() - start_time) < timeout_s:
            # 5초마다 로그 출력
            elapsed = time.time() - start_time
            if int(elapsed) % 5 == 0 and int(elapsed) > 0:
                remaining = timeout_s - elapsed
                self.get_logger().info(f"⏳ 개인컵을 올려주세요... (남은 시간: {remaining:.0f}초)")
            
            # 좌표 수신 대기 (0.5초씩)
            if self._personal_cup_coords_event.wait(timeout=0.5):
                # 좌표 수신 성공
                self._arrive_cup_coords_pub.publish(Bool(data=True))
                self.get_logger().info("✅ 개인컵 좌표 수신 완료")
                return True
            
            time.sleep(0.5)
        
        # 타임아웃
        self._arrive_cup_coords_pub.publish(Bool(data=True))
        self.get_logger().warn("⚠️  타임아웃: 매장컵으로 진행합니다")
        return False
    
    def _handle_cup_detection_and_pickup(self):
        """
        컵 감지 및 pickup 위치 결정 로직
        1. check_cup 발행
        2. 개인컵 감지 시 대기
        3. pickup 위치 결정 (st1/st2)
        4. check_cup_done 발행
        """
        # 🔥 check_cup 발행
        self._check_cup_pub.publish(Bool(data=True))
        self.get_logger().info("📸 Check cup requested")
        
        time.sleep(5)

        # 🔥 개인컵 감지 대기
        while self._personal_cup_detected:
            self.get_logger().info("⏳ 개인컵 탐지됨 - 대기 중...")
            time.sleep(0.5)
        
        # 🔥 pickup 위치 결정
        if self._pickup_st1_detected and self._pickup_st2_detected:
            # 🚨 둘 다 감지됨 - 에러 상황
            self.get_logger().error("🚨 시리얼을 수거해 주세요")
            
            # 대기
            while self._pickup_st1_detected and self._pickup_st2_detected:
                self.get_logger().warn("⏳ 대기 중... 시리얼을 수거해 주세요")
                time.sleep(1.0)
            
            # 수거 후 다시 확인
            if self._pickup_st1_detected:
                self.get_logger().info("📍 Pickup position 1 detected → moving to pickup 2")
                ok, detail = self.run_motion("move_pickup_2", {"speed_scale": 0.7}, timeout_s=60.0)
                if not ok:
                    return self._fail("move_pickup_2", detail)
                    
            elif self._pickup_st2_detected:
                self.get_logger().info("📍 Pickup position 2 detected → moving to pickup 1")
                ok, detail = self.run_motion("move_pickup_1", {"speed_scale": 0.7}, timeout_s=60.0)
                if not ok:
                    return self._fail("move_pickup_1", detail)
            else:
                self.get_logger().info("📍 No pickup detected → moving to pickup 1 (default)")
                ok, detail = self.run_motion("move_pickup_1", {"speed_scale": 0.7}, timeout_s=60.0)
                if not ok:
                    return self._fail("move_pickup_1", detail)
        
        elif self._pickup_st1_detected:
            self.get_logger().info("📍 Pickup position 1 detected → moving to pickup 2")
            ok, detail = self.run_motion("move_pickup_2", {"speed_scale": 0.7}, timeout_s=60.0)
            if not ok:
                return self._fail("move_pickup_2", detail)
        
        elif self._pickup_st2_detected:
            self.get_logger().info("📍 Pickup position 2 detected → moving to pickup 1")
            ok, detail = self.run_motion("move_pickup_1", {"speed_scale": 0.7}, timeout_s=60.0)
            if not ok:
                return self._fail("move_pickup_1", detail)
        
        else:
            self.get_logger().info("📍 No pickup detected → moving to pickup 1 (default)")
            ok, detail = self.run_motion("move_pickup_1", {"speed_scale": 0.7}, timeout_s=60.0)
            if not ok:
                return self._fail("move_pickup_1", detail)
        
        # 🔥 check_cup_done 발행
        self._check_cup_done_pub.publish(Bool(data=True))
        self.get_logger().info("✅ Check cup done")
        
        return True

    # -------------------------
    # 시퀀스 A/B
    # -------------------------
    def _sequence_a(self, target_g: float, cup_type: str = "store"):
        """
        시퀀스 A: [개인컵 준비] → grab_cup → move_to_a → give_cereal_a → retrieve_cup_from_a → cup detection → pickup → go_home
        """
        try:
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"🚀 SEQUENCE A START (target={target_g}g, cup={cup_type})")
            self.get_logger().info("=" * 60)

            # 🔥 저울 A 활성화
            self._active_scale = 'a'

            # 🔥 개인컵인 경우 추가 동작
            if cup_type == "personal":
                self.get_logger().info("📍 [0/9] Detecting personal cup...")
                
                # 🔥 개인컵 좌표 대기 (30초)
                if self._wait_for_personal_cup(timeout_s=30.0):
                    # 개인컵 감지 성공
                    self.get_logger().info("📍 [1/9] Personal cup detected - preparing...")
                    
                    # 개인컵 준비
                    ok, detail = self.run_motion("ready_personal_cup", {"speed_scale": 0.7}, timeout_s=60.0)
                    if not ok:
                        return self._fail("ready_personal_cup", detail)
                    
                    # 🔥 Motion Control에 좌표 재발행 (grab_personal_cup에서 사용)
                    if self._personal_cup_coords:
                        coords_msg = Float32MultiArray()
                        coords_msg.data = self._personal_cup_coords
                        self._cup_coords_to_motion_pub.publish(coords_msg)
                        self.get_logger().info(f"📤 Sent cup coordinates to Motion Control: {self._personal_cup_coords}")
                        time.sleep(0.5)  # Motion Control이 수신할 시간
                    
                    # 개인컵 잡기 (이제 Motion Control이 좌표를 가지고 있음)
                    ok, detail = self.run_motion("grab_personal_cup", {"speed_scale": 0.7}, timeout_s=60.0)
                    if not ok:
                        return self._fail("grab_personal_cup", detail)
                    
                    step_offset = 2  # 단계 번호 조정
                    total_steps = 9  # 🔥 go_home 추가
                else:
                    # 타임아웃 → 매장컵으로 전환
                    self.get_logger().info("📍 [1/7] Timeout - using store cup...")
                    ok, detail = self.run_motion("grab_cup", {"speed_scale": 0.7}, timeout_s=60.0)
                    if not ok:
                        return self._fail("grab_cup", detail)
                    
                    step_offset = 1
                    total_steps = 7  # 🔥 go_home 추가
            else:
                # 매장컵인 경우
                # 1. 컵 잡기
                self.get_logger().info("📍 [1/7] Grabbing cup...")
                ok, detail = self.run_motion("grab_cup", {"speed_scale": 0.7}, timeout_s=60.0)
                if not ok:
                    return self._fail("grab_cup", detail)
                
                step_offset = 1
                total_steps = 7  # 🔥 go_home 추가

            # 2. A 위치로 이동
            self.get_logger().info(f"📍 [{step_offset+1}/{total_steps}] Moving to position A...")
            ok, detail = self.run_motion("move_to_a", {"speed_scale": 0.7}, timeout_s=60.0)
            if not ok:
                return self._fail("move_to_a", detail)

            # 3. 저울 A 안정화 대기
            self.get_logger().info(f"📍 [{step_offset+2}/{total_steps}] Waiting for cup placement on scale A...")
            if not self._wait_for_stable_weight(min_weight=10.0, timeout_s=15.0):
                return self._fail("cup_placement", "Cup not detected on scale A")

            # 4. 무게 모니터링 + 시리얼 주기
            self.get_logger().info(f"📍 [{step_offset+3}/{total_steps}] Dispensing cereal with weight monitoring (Scale A)...")
            self._target_weight = target_g
            self._monitoring = True
            
            monitor_thread = threading.Thread(target=self._monitor_weight, daemon=True)
            monitor_thread.start()
            
            params = {"speed_scale": 0.7, "target_g": float(target_g)}
            ok, detail = self.run_motion("give_cereal_a", params, timeout_s=180.0)
            
            self._monitoring = False
            
            if not ok:
                return self._fail("give_cereal_a", detail)

            # 5. 컵 회수
            self.get_logger().info(f"📍 [{step_offset+4}/{total_steps}] Retrieving cup from position A...")
            ok, detail = self.run_motion("retrieve_cup_from_a", {"speed_scale": 0.7}, timeout_s=60.0)
            if not ok:
                return self._fail("retrieve_cup_from_a", detail)

            # 6. 🔥 컵 감지 및 pickup 위치 결정
            self.get_logger().info(f"📍 [{step_offset+5}/{total_steps}] Cup detection and pickup...")
            if not self._handle_cup_detection_and_pickup():
                return  # _fail은 이미 호출됨
            
            # 7. 🔥 홈 위치로 복귀
            self.get_logger().info(f"📍 [{step_offset+6}/{total_steps}] Going to home position...")
            ok, detail = self.run_motion("go_home", {"speed_scale": 0.7}, timeout_s=60.0)
            if not ok:
                return self._fail("go_home", detail)

            self.get_logger().info("=" * 60)
            self.get_logger().info("✅ SEQUENCE A COMPLETED!")
            self.get_logger().info("=" * 60)
        finally:
            self._monitoring = False
            self._active_scale = None
            self._target_weight = 0.0
            
            # 🔥 컵 감지 플래그 초기화
            self._personal_cup_detected = False
            self._pickup_st1_detected = False
            self._pickup_st2_detected = False
            
            self.get_logger().info("🔄 Sequence A state reset")
            
            if self._busy_lock.locked():
                self._busy_lock.release()

    def _sequence_b(self, target_g: float, cup_type: str = "store"):
        """
        시퀀스 B: [개인컵 준비] → grab_cup → move_to_b → give_cereal_b → retrieve_cup_from_b → cup detection → pickup → go_home
        """
        try:
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"🚀 SEQUENCE B START (target={target_g}g, cup={cup_type})")
            self.get_logger().info("=" * 60)

            # 🔥 저울 B 활성화
            self._active_scale = 'b'

            # 🔥 개인컵인 경우 추가 동작
            if cup_type == "personal":
                self.get_logger().info("📍 [0/9] Detecting personal cup...")
                
                # 🔥 개인컵 좌표 대기 (30초)
                if self._wait_for_personal_cup(timeout_s=30.0):
                    # 개인컵 감지 성공
                    self.get_logger().info("📍 [1/9] Personal cup detected - preparing...")
                    
                    # 개인컵 준비
                    ok, detail = self.run_motion("ready_personal_cup", {"speed_scale": 0.7}, timeout_s=60.0)
                    if not ok:
                        return self._fail("ready_personal_cup", detail)
                    
                    # 🔥 Motion Control에 좌표 재발행 (grab_personal_cup에서 사용)
                    if self._personal_cup_coords:
                        coords_msg = Float32MultiArray()
                        coords_msg.data = self._personal_cup_coords
                        self._cup_coords_to_motion_pub.publish(coords_msg)
                        self.get_logger().info(f"📤 Sent cup coordinates to Motion Control: {self._personal_cup_coords}")
                        time.sleep(0.5)  # Motion Control이 수신할 시간
                    
                    # 개인컵 잡기 (이제 Motion Control이 좌표를 가지고 있음)
                    ok, detail = self.run_motion("grab_personal_cup", {"speed_scale": 0.7}, timeout_s=60.0)
                    if not ok:
                        return self._fail("grab_personal_cup", detail)
                    
                    step_offset = 2  # 단계 번호 조정
                    total_steps = 9  # 🔥 go_home 추가
                else:
                    # 타임아웃 → 매장컵으로 전환
                    self.get_logger().info("📍 [1/7] Timeout - using store cup...")
                    ok, detail = self.run_motion("grab_cup", {"speed_scale": 0.7}, timeout_s=60.0)
                    if not ok:
                        return self._fail("grab_cup", detail)
                    
                    step_offset = 1
                    total_steps = 7  # 🔥 go_home 추가
            else:
                # 매장컵인 경우
                # 1. 컵 잡기
                self.get_logger().info("📍 [1/7] Grabbing cup...")
                ok, detail = self.run_motion("grab_cup", {"speed_scale": 0.7}, timeout_s=60.0)
                if not ok:
                    return self._fail("grab_cup", detail)
                
                step_offset = 1
                total_steps = 7  # 🔥 go_home 추가

            # 2. B 위치로 이동
            self.get_logger().info(f"📍 [{step_offset+1}/{total_steps}] Moving to position B...")
            ok, detail = self.run_motion("move_to_b", {"speed_scale": 0.7}, timeout_s=60.0)
            if not ok:
                return self._fail("move_to_b", detail)

            # 3. 저울 B 안정화 대기
            self.get_logger().info(f"📍 [{step_offset+2}/{total_steps}] Waiting for cup placement on scale B...")
            if not self._wait_for_stable_weight(min_weight=10.0, timeout_s=15.0):
                return self._fail("cup_placement", "Cup not detected on scale B")

            # 4. 무게 모니터링 + 시리얼 주기
            self.get_logger().info(f"📍 [{step_offset+3}/{total_steps}] Dispensing cereal with weight monitoring (Scale B)...")
            self._target_weight = target_g
            self._monitoring = True
            
            monitor_thread = threading.Thread(target=self._monitor_weight, daemon=True)
            monitor_thread.start()
            
            params = {"speed_scale": 0.7, "target_g": float(target_g)}
            ok, detail = self.run_motion("give_cereal_b", params, timeout_s=180.0)
            
            self._monitoring = False
            
            if not ok:
                return self._fail("give_cereal_b", detail)

            # 5. 컵 회수
            self.get_logger().info(f"📍 [{step_offset+4}/{total_steps}] Retrieving cup from position B...")
            ok, detail = self.run_motion("retrieve_cup_from_b", {"speed_scale": 0.7}, timeout_s=60.0)
            if not ok:
                return self._fail("retrieve_cup_from_b", detail)

            # 6. 🔥 컵 감지 및 pickup 위치 결정
            self.get_logger().info(f"📍 [{step_offset+5}/{total_steps}] Cup detection and pickup...")
            if not self._handle_cup_detection_and_pickup():
                return  # _fail은 이미 호출됨
            
            # 7. 🔥 홈 위치로 복귀
            self.get_logger().info(f"📍 [{step_offset+6}/{total_steps}] Going to home position...")
            ok, detail = self.run_motion("go_home", {"speed_scale": 0.7}, timeout_s=60.0)
            if not ok:
                return self._fail("go_home", detail)

            self.get_logger().info("=" * 60)
            self.get_logger().info("✅ SEQUENCE B COMPLETED!")
            self.get_logger().info("=" * 60)
        finally:
            self._monitoring = False
            self._active_scale = None
            self._target_weight = 0.0
            
            # 🔥 컵 감지 플래그 초기화
            self._personal_cup_detected = False
            self._pickup_st1_detected = False
            self._pickup_st2_detected = False
            
            self.get_logger().info("🔄 Sequence B state reset")
            
            if self._busy_lock.locked():
                self._busy_lock.release()

    # -------------------------
    # 유틸리티 함수들
    # -------------------------
    def _on_weight_value_a(self, msg: Float32):
        """저울 A 무게값 수신"""
        self._current_weight_a = msg.data

    def _on_weight_value_b(self, msg: Float32):
        """저울 B 무게값 수신"""
        self._current_weight_b = msg.data

    def _monitor_weight(self):
        """
        🔥 무게 모니터링 스레드
        - 목표 무게 달성 시 weight_ok 발행
        """
        scale_name = self._active_scale.upper() if self._active_scale else 'UNKNOWN'
        self.get_logger().info(f"⏳ Weight monitoring started (Scale {scale_name}, target={self._target_weight}g)")
        
        while self._monitoring:
            if self._active_scale == 'a':
                current = self._current_weight_a
            elif self._active_scale == 'b':
                current = self._current_weight_b
            else:
                self.get_logger().error("❌ Invalid active scale!")
                break
            
            if current >= self._target_weight:
                self.get_logger().info(f"✅ Target weight reached (Scale {scale_name}): {current:.1f}g >= {self._target_weight}g")
                
                msg = Bool()
                msg.data = True
                self._weight_ok_pub.publish(msg)
                
                self.get_logger().info("📢 Published weight_ok signal!")
                break
            
            time.sleep(0.5)
        
        self.get_logger().info("⏹️  Weight monitoring stopped")

    def _wait_for_stable_weight(self, min_weight: float, timeout_s: float) -> bool:
        """컵이 저울에 제대로 올려졌는지 확인"""
        scale_name = self._active_scale.upper() if self._active_scale else 'UNKNOWN'
        self.get_logger().info(f"⏳ Waiting for stable weight on Scale {scale_name} (>= {min_weight}g)...")
        
        stable_duration = 0.0
        required_stable_time = 0.5
        
        start_time = time.time()
        prev_weight = 0.0
        
        while (time.time() - start_time) < timeout_s:
            if self._active_scale == 'a':
                current = self._current_weight_a
            elif self._active_scale == 'b':
                current = self._current_weight_b
            else:
                self.get_logger().error("❌ Invalid active scale!")
                return False
            
            if current >= min_weight:
                weight_change = abs(current - prev_weight)
                
                if weight_change < 2.0:
                    stable_duration += 0.1
                    if stable_duration >= required_stable_time:
                        self.get_logger().info(f"✅ Stable weight detected on Scale {scale_name}: {current:.1f}g")
                        return True
                else:
                    stable_duration = 0.0
                
                if int(stable_duration * 10) % 2 == 0:
                    self.get_logger().debug(
                        f"   Weight: {current:.1f}g (stable: {stable_duration:.1f}s)"
                    )
            else:
                stable_duration = 0.0
            
            prev_weight = current
            time.sleep(0.1)
        
        self.get_logger().error(
            f"❌ Cup placement timeout on Scale {scale_name}: current={current:.1f}g < {min_weight}g"
        )
        return False

    def _fail(self, step, detail):
        """실패 처리"""
        self.get_logger().error("=" * 60)
        self.get_logger().error(f"❌ FAILED at step: {step}")
        self.get_logger().error(f"   Reason: {detail}")
        self.get_logger().error("=" * 60)
        return False


def main():
    rclpy.init()
    try:
        node = NetworkManager()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        node.get_logger().info("🤖 NetworkManager is ready! (Ctrl+C to quit)")
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()