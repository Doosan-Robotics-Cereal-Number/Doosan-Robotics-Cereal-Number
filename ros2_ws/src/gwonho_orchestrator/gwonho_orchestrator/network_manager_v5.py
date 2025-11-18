# gwonho_orchestrator/network_manager.py (v8-optimized-fixed)
import json
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String, Float32, Bool, Float32MultiArray
from sensor_msgs.msg import JointState
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
    v8-optimized-fixed: 상대 무게 측정 추가
    - 컵 무게를 기준으로 영점 조정 (상대 무게 측정)
    - order_done: go_home 완료 후 1회만 발행
    - robot_status: task + control_command만 발행 (경량화)
    - robot_joint_states: 조인트 상태 실시간 발행
    """

    def __init__(self):
        super().__init__('network_manager')

        # 파라미터
        self.declare_parameter('robot_id', 'dsr01')
        self.robot_id = self.get_parameter('robot_id').value

        # 상태
        self._busy_lock = threading.Lock()
        self._current_weight_a = 0.0
        self._current_weight_b = 0.0
        self._target_weight = 0.0
        self._monitoring = False
        self._active_scale = None

        # 🔥 컵 무게 (영점 기준)
        self._cup_weight_a = 0.0
        self._cup_weight_b = 0.0
        
        # 컵 감지 상태
        self._personal_cup_detected = False
        self._pickup_st1_detected = False
        self._pickup_st2_detected = False
        
        # 개인컵 좌표 수신 상태
        self._personal_cup_coords = None
        self._personal_cup_coords_event = threading.Event()
        
        # 🔥 실시간 상태 관리 (최소화)
        self._current_task = "idle"
        self._current_control_cmd = "none"
        self._current_joint_states = None

        # ActionClient
        action_name = f'/{self.robot_id}/motion_control/run'
        self.get_logger().info(f"[NetworkManager] Creating ActionClient for: '{action_name}'")
        self._ac = ActionClient(self, RunMotion, action_name)

        # 주문 구독 (절대 경로)
        order_topic = f'/{self.robot_id}/kiosk/order'
        self.create_subscription(String, order_topic, self._on_order, 10)
        
        # 저울 구독
        weight_topic_a = f'/{self.robot_id}/weight/scale_a/current'
        weight_topic_b = f'/{self.robot_id}/weight/scale_b/current'
        self.create_subscription(Float32, weight_topic_a, self._on_weight_value_a, 10)
        self.create_subscription(Float32, weight_topic_b, self._on_weight_value_b, 10)
        
        # 컵 감지 토픽 구독
        self.create_subscription(Bool, '/cup_detections/personal_cup_st', self._on_personal_cup, 10)
        self.create_subscription(Bool, '/cup_detections/pickup_st1', self._on_pickup_st1, 10)
        self.create_subscription(Bool, '/cup_detections/pickup_st2', self._on_pickup_st2, 10)
        
        # 개인컵 좌표 구독
        self.create_subscription(Float32MultiArray, '/cup_stable_coordinates', self._on_cup_coords, 10)
        
        # 🔥 조인트 상태 구독
        joint_state_topic = f'/{self.robot_id}/joint_states'
        self.create_subscription(JointState, joint_state_topic, self._on_joint_states, 10)
        
        # 토픽 발행자들
        weight_ok_topic = f'/{self.robot_id}/weight_ok'
        self._weight_ok_pub = self.create_publisher(Bool, weight_ok_topic, 10)
        
        self._check_cup_pub = self.create_publisher(Bool, '/check_cup', 10)
        self._check_cup_done_pub = self.create_publisher(Bool, '/check_cup_done', 10)
        
        # 개인컵 감지 제어 토픽
        self._call_cup_coords_pub = self.create_publisher(Bool, '/call_cup_stable_coordinates', 10)
        self._arrive_cup_coords_pub = self.create_publisher(Bool, '/arrive_cup_stable_coordinates', 10)
        
        # Motion Control에게 좌표 전달용 Publisher
        self._cup_coords_to_motion_pub = self.create_publisher(Float32MultiArray, '/cup_stable_coordinates', 10)
        
        # 🔥 주문 완료 발행
        self._order_done_pub = self.create_publisher(String, '/kiosk/order_done', 10)
        
        # 🔥 실시간 상태 발행 (경량화)
        self._joint_states_pub = self.create_publisher(JointState, '/robot_joint_states', 10)
        self._robot_status_pub = self.create_publisher(String, '/robot_status', 10)

        # 🔥 상태 발행 스레드 시작
        self._status_publishing = True
        self._status_thread = threading.Thread(target=self._publish_status_loop, daemon=True)
        self._status_thread.start()

        self.get_logger().info("=" * 60)
        self.get_logger().info(f"[NetworkManager] v8-optimized-fixed initialized")
        self.get_logger().info(f"[NetworkManager] ✅ Relative weight measurement enabled")
        self.get_logger().info(f"[NetworkManager] Subscribed to:")
        self.get_logger().info(f"  - {order_topic} (주문)")
        self.get_logger().info(f"[NetworkManager] Publishing:")
        self.get_logger().info(f"  - /kiosk/order_done (on completion)")
        self.get_logger().info(f"  - /robot_joint_states (10Hz)")
        self.get_logger().info(f"  - /robot_status (10Hz, lightweight)")
        self.get_logger().info("=" * 60)

    # -------------------------
    # 🔥 실시간 상태 발행 루프 (최적화)
    # -------------------------
    def _publish_status_loop(self):
        """
        실시간 상태를 주기적으로 발행 (10Hz)
        - 조인트 상태
        - task + control_command만 (경량화)
        """
        rate = 10.0  # Hz
        sleep_time = 1.0 / rate
        
        while self._status_publishing and rclpy.ok():
            try:
                # 1. 조인트 상태 발행
                if self._current_joint_states is not None:
                    self._joint_states_pub.publish(self._current_joint_states)
                
                # 2. 로봇 상태 발행 (경량화: task + control_command만)
                status = {
                    "task": self._current_task,
                    "control_command": self._current_control_cmd
                }
                
                status_msg = String()
                status_msg.data = json.dumps(status)
                self._robot_status_pub.publish(status_msg)
                
            except Exception as e:
                self.get_logger().error(f"❌ Status publishing error: {e}")
            
            time.sleep(sleep_time)
        
        self.get_logger().info("⏹️  Status publishing stopped")
    
    def _on_joint_states(self, msg: JointState):
        """조인트 상태 수신 및 저장"""
        self._current_joint_states = msg
    
    def _set_task(self, task_name: str):
        """현재 태스크 설정"""
        self._current_task = task_name
        self.get_logger().info(f"📋 Task: {task_name}")
    
    def _set_control_cmd(self, cmd: str):
        """현재 제어 커맨드 설정"""
        self._current_control_cmd = cmd

    # -------------------------
    # 컵 감지 콜백 함수들
    # -------------------------
    def _on_personal_cup(self, msg: Bool):
        """개인컵 감지"""
        self._personal_cup_detected = msg.data
        if msg.data:
            self.get_logger().info("🔍 개인컵 탐지")

    def _on_pickup_st1(self, msg: Bool):
        """Pickup 위치 1 감지"""
        self._pickup_st1_detected = msg.data

    def _on_pickup_st2(self, msg: Bool):
        """Pickup 위치 2 감지"""
        self._pickup_st2_detected = msg.data
    
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
        """주문 메시지 파싱"""
        raw = (msg.data or "").strip()
        self.get_logger().info(f"🧾 order msg: {raw!r}")
        if not raw:
            self.get_logger().error("❌ 빈 주문 메시지 수신")
            return

        seat_cmd, target_g, cup_type = None, None, "store"
        
        # JSON 파싱 시도
        try:
            payload = json.loads(raw)
            seat = payload.get("seat")
            if seat:
                seat = str(seat).strip().upper()
                if seat in ['A', 'B']:
                    seat_cmd = f"start_sequence_{seat.lower()}"
                else:
                    self.get_logger().error(f"❌ 잘못된 자리: '{seat}'")
                    return
            else:
                seat_cmd = payload.get("cmd") or payload.get("command")

            portion = str(payload.get("portion", "")).strip()
            target_g = PORTION_MAP.get(portion)
            
            cup_type_raw = str(payload.get("cup_type", "매장컵")).strip()
            cup_type = CUP_TYPE_MAP.get(cup_type_raw, "store")
            
        except Exception:
            # 쉼표 구분 파싱
            try:
                parts = [x.strip() for x in raw.split(',')]
                if len(parts) < 2:
                    raise ValueError("Invalid format")
                
                seat_cmd = parts[0]
                target_g = PORTION_MAP.get(parts[1])
                
                if len(parts) >= 3:
                    cup_type = CUP_TYPE_MAP.get(parts[2], "store")
                else:
                    cup_type = "store"
                
            except Exception:
                self.get_logger().error(f"❌ 잘못된 주문 형식: '{raw}'")
                return

        if not seat_cmd:
            self.get_logger().error(f"❌ 자리/명령을 해석할 수 없음: '{raw}'")
            return
        
        if target_g is None:
            self.get_logger().error(f"❌ 용량을 해석할 수 없음: '{raw}'")
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
    # 🔥 주문 완료 발행 (최적화)
    # -------------------------
    def _publish_order_done(self, success: bool):
        """
        주문 완료 메시지 발행
        - go_home 완료 후 1회만 호출됨
        """
        order_done = {
            "success": "true" if success else "false"
        }
        
        msg = String()
        msg.data = json.dumps(order_done)
        self._order_done_pub.publish(msg)
        
        if success:
            self.get_logger().info(f"✅ Order done: success")
        else:
            self.get_logger().error(f"❌ Order failed")

    # -------------------------
    # 공통: 모션 실행
    # -------------------------
    def run_motion(self, name: str, params: dict, timeout_s: float):
        """ActionClient를 이용해 RunMotion 실행"""
        self._set_task(name)
        self._set_control_cmd("preparing")
        
        self.get_logger().info(f"🎯 Running motion: '{name}'")

        goal = RunMotion.Goal()
        goal.name = name
        goal.params_json = json.dumps(params or {})

        # 목표 전송
        self._set_control_cmd("sending_goal")
        send_future = self._ac.send_goal_async(goal, feedback_callback=self._on_feedback)
        start = time.time()
        while not send_future.done():
            if timeout_s and (time.time() - start) > timeout_s:
                self.get_logger().error(f"❌ send_goal timeout")
                self._set_control_cmd("timeout")
                return False, "send_timeout"
            time.sleep(0.01)

        try:
            gh = send_future.result()
        except Exception as e:
            self.get_logger().error(f"❌ Failed to send goal: {e}")
            self._set_control_cmd("error")
            return False, f"send_failed: {e}"

        if not gh or not gh.accepted:
            self.get_logger().error("❌ Goal rejected")
            self._set_control_cmd("rejected")
            return False, "rejected"

        self._set_control_cmd("executing")

        # 결과 대기
        res_future = gh.get_result_async()
        start = time.time()
        while not res_future.done():
            if timeout_s and (time.time() - start) > timeout_s:
                self.get_logger().error("❌ get_result timeout")
                try:
                    gh.cancel_goal_async()
                except Exception:
                    pass
                self._set_control_cmd("timeout")
                return False, "result_timeout"
            time.sleep(0.01)

        res = res_future.result().result
        ok = bool(res.success)
        self._set_control_cmd("completed" if ok else "failed")
        return ok, res.detail

    def _on_feedback(self, fb_msg):
        """피드백 수신"""
        fb = fb_msg.feedback
        if hasattr(fb, 'stage') and fb.stage:
            self._set_control_cmd(fb.stage)

    # -------------------------
    # 컵 감지 및 pickup 위치 결정
    # -------------------------
    def _wait_for_personal_cup(self, timeout_s: float = 30.0) -> bool:
        """개인컵 좌표 대기"""
        self._set_task("waiting_personal_cup")
        
        self._personal_cup_coords = None
        self._personal_cup_coords_event.clear()
        
        self._call_cup_coords_pub.publish(Bool(data=True))
        self.get_logger().info("📸 개인컵 감지 요청")
        
        self.get_logger().info("⏳ 개인컵을 올려주세요...")
        start_time = time.time()
        
        while (time.time() - start_time) < timeout_s:
            elapsed = time.time() - start_time
            if int(elapsed) % 5 == 0 and int(elapsed) > 0:
                remaining = timeout_s - elapsed
                self.get_logger().info(f"⏳ 남은 시간: {remaining:.0f}초")
            
            if self._personal_cup_coords_event.wait(timeout=0.5):
                self._arrive_cup_coords_pub.publish(Bool(data=True))
                self.get_logger().info("✅ 개인컵 좌표 수신 완료")
                return True
            
            time.sleep(0.5)
        
        self._arrive_cup_coords_pub.publish(Bool(data=True))
        self.get_logger().warn("⚠️  타임아웃: 매장컵으로 진행")
        return False
    
    def _handle_cup_detection_and_pickup(self):
        """컵 감지 및 pickup 위치 결정"""
        self._set_task("cup_detection")
        
        self._check_cup_pub.publish(Bool(data=True))
        self.get_logger().info("📸 Check cup requested")
        
        time.sleep(2)

        while self._personal_cup_detected:
            self.get_logger().info("⏳ 개인컵 탐지됨 - 대기 중...")
            time.sleep(0.5)
        
        # pickup 위치 결정
        if self._pickup_st1_detected and self._pickup_st2_detected:
            self.get_logger().error("🚨 시리얼을 수거해 주세요")
            
            while self._pickup_st1_detected and self._pickup_st2_detected:
                self.get_logger().warn("⏳ 대기 중...")
                time.sleep(1.0)
            
            if self._pickup_st1_detected:
                ok, detail = self.run_motion("move_pickup_2", {"speed_scale": 0.7}, timeout_s=60.0)
                if not ok:
                    return self._fail("move_pickup_2", detail)
            elif self._pickup_st2_detected:
                ok, detail = self.run_motion("move_pickup_1", {"speed_scale": 0.7}, timeout_s=60.0)
                if not ok:
                    return self._fail("move_pickup_1", detail)
            else:
                ok, detail = self.run_motion("move_pickup_1", {"speed_scale": 0.7}, timeout_s=60.0)
                if not ok:
                    return self._fail("move_pickup_1", detail)
        
        elif self._pickup_st1_detected:
            ok, detail = self.run_motion("move_pickup_2", {"speed_scale": 0.7}, timeout_s=60.0)
            if not ok:
                return self._fail("move_pickup_2", detail)
        
        elif self._pickup_st2_detected:
            ok, detail = self.run_motion("move_pickup_1", {"speed_scale": 0.7}, timeout_s=60.0)
            if not ok:
                return self._fail("move_pickup_1", detail)
        
        else:
            ok, detail = self.run_motion("move_pickup_1", {"speed_scale": 0.7}, timeout_s=60.0)
            if not ok:
                return self._fail("move_pickup_1", detail)
        
        self._check_cup_done_pub.publish(Bool(data=True))
        self.get_logger().info("✅ Check cup done")
        
        return True

    # -------------------------
    # 시퀀스 A/B
    # -------------------------
    def _sequence_a(self, target_g: float, cup_type: str = "store"):
        """시퀀스 A"""
        seat = "A"
        try:
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"🚀 SEQUENCE A START (target={target_g}g, cup={cup_type})")
            self.get_logger().info("=" * 60)

            self._active_scale = 'a'

            # 개인컵인 경우
            if cup_type == "personal":
                self.get_logger().info("📍 [0/9] Detecting personal cup...")
                
                if self._wait_for_personal_cup(timeout_s=30.0):
                    self.get_logger().info("📍 [1/9] Personal cup detected")
                    
                    # 개인컵 위치로 준비
                    ok, detail = self.run_motion("ready_personal_cup", {"speed_scale": 0.7}, timeout_s=60.0)
                    if not ok:
                        return self._fail_with_done(f"ready_personal_cup: {detail}")
                    
                    # 좌표 전달
                    if self._personal_cup_coords:
                        coords_msg = Float32MultiArray()
                        coords_msg.data = self._personal_cup_coords
                        self._cup_coords_to_motion_pub.publish(coords_msg)
                        time.sleep(0.5)
                    
                    # 개인컵 집기
                    self.get_logger().info("📍 [2/9] Grabbing personal cup...")
                    ok, detail = self.run_motion("grab_personal_cup", {"speed_scale": 0.7}, timeout_s=60.0)
                    if not ok:
                        return self._fail_with_done(f"grab_personal_cup: {detail}")
                    
                    # 🔥 매장컵 집는 동작 (자세 전환)
                    self.get_logger().info("📍 [3/9] Adjusting grip...")
                    ok, detail = self.run_motion("grab_cup", {"speed_scale": 0.7}, timeout_s=60.0)
                    if not ok:
                        return self._fail_with_done(f"grab_cup: {detail}")
                    
                    step_offset = 3
                    total_steps = 9
                else:
                    self.get_logger().info("📍 [1/7] Timeout - using store cup")
                    ok, detail = self.run_motion("grab_cup", {"speed_scale": 0.7}, timeout_s=60.0)
                    if not ok:
                        return self._fail_with_done(f"grab_cup: {detail}")
                    
                    step_offset = 1
                    total_steps = 7
            else:
                # 매장컵
                self.get_logger().info("📍 [1/7] Grabbing cup...")
                ok, detail = self.run_motion("grab_cup", {"speed_scale": 0.7}, timeout_s=60.0)
                if not ok:
                    return self._fail_with_done(f"grab_cup: {detail}")
                
                step_offset = 1
                total_steps = 7

            # 2. A 위치로 이동
            self.get_logger().info(f"📍 [{step_offset+1}/{total_steps}] Moving to A...")
            ok, detail = self.run_motion("move_to_a", {"speed_scale": 0.7}, timeout_s=60.0)
            if not ok:
                return self._fail_with_done(f"move_to_a: {detail}")

            # 3. 저울 안정화 대기
            self._set_task("waiting_cup_placement")
            self.get_logger().info(f"📍 [{step_offset+2}/{total_steps}] Waiting for cup placement...")
            time.sleep(5)
            if not self._wait_for_stable_weight(min_weight=10.0, timeout_s=15.0):
                return self._fail_with_done("Cup not detected on scale A")

            # 4. 시리얼 주기
            self.get_logger().info(f"📍 [{step_offset+3}/{total_steps}] Dispensing cereal...")
            self._target_weight = target_g
            self._monitoring = True
            
            monitor_thread = threading.Thread(target=self._monitor_weight, daemon=True)
            monitor_thread.start()
            
            params = {"speed_scale": 0.7, "target_g": float(target_g)}
            ok, detail = self.run_motion("give_cereal_a", params, timeout_s=180.0)
            
            self._monitoring = False
            
            if not ok:
                return self._fail_with_done(f"give_cereal_a: {detail}")

            # 5. 컵 회수
            self.get_logger().info(f"📍 [{step_offset+4}/{total_steps}] Retrieving cup...")
            ok, detail = self.run_motion("retrieve_cup_from_a", {"speed_scale": 0.7}, timeout_s=60.0)
            if not ok:
                return self._fail_with_done(f"retrieve_cup_from_a: {detail}")

            # 6. 컵 감지 및 pickup
            self.get_logger().info(f"📍 [{step_offset+5}/{total_steps}] Cup detection and pickup...")
            if not self._handle_cup_detection_and_pickup():
                return self._fail_with_done("Cup detection failed")
            
            # 7. 🔥 홈 위치로 복귀
            self.get_logger().info(f"📍 [{step_offset+6}/{total_steps}] Going to home...")
            ok, detail = self.run_motion("go_home", {"speed_scale": 0.7}, timeout_s=60.0)
            if not ok:
                return self._fail_with_done(f"go_home: {detail}")

            self.get_logger().info("=" * 60)
            self.get_logger().info("✅ SEQUENCE A COMPLETED!")
            self.get_logger().info("=" * 60)
            
            # 🔥 성공 시 여기서만 order_done 발행
            self._publish_order_done(True)
            
        finally:
            self._monitoring = False
            self._active_scale = None
            self._target_weight = 0.0
            self._current_task = "idle"
            self._current_control_cmd = "none"
            
            # 🔥 컵 무게 리셋
            self._cup_weight_a = 0.0
            self._cup_weight_b = 0.0
            
            self._personal_cup_detected = False
            self._pickup_st1_detected = False
            self._pickup_st2_detected = False
            
            if self._busy_lock.locked():
                self._busy_lock.release()

    def _sequence_b(self, target_g: float, cup_type: str = "store"):
        """시퀀스 B"""
        seat = "B"
        try:
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"🚀 SEQUENCE B START (target={target_g}g, cup={cup_type})")
            self.get_logger().info("=" * 60)

            self._active_scale = 'b'

            # 개인컵인 경우
            if cup_type == "personal":
                self.get_logger().info("📍 [0/9] Detecting personal cup...")
                
                if self._wait_for_personal_cup(timeout_s=30.0):
                    self.get_logger().info("📍 [1/9] Personal cup detected")
                    
                    # 개인컵 위치로 준비
                    ok, detail = self.run_motion("ready_personal_cup", {"speed_scale": 0.7}, timeout_s=60.0)
                    if not ok:
                        return self._fail_with_done(f"ready_personal_cup: {detail}")
                    
                    # 좌표 전달
                    if self._personal_cup_coords:
                        coords_msg = Float32MultiArray()
                        coords_msg.data = self._personal_cup_coords
                        self._cup_coords_to_motion_pub.publish(coords_msg)
                        time.sleep(0.5)
                    
                    # 개인컵 집기
                    self.get_logger().info("📍 [2/9] Grabbing personal cup...")
                    ok, detail = self.run_motion("grab_personal_cup", {"speed_scale": 0.7}, timeout_s=60.0)
                    if not ok:
                        return self._fail_with_done(f"grab_personal_cup: {detail}")
                    
                    # 🔥 매장컵 집는 동작 (자세 전환)
                    self.get_logger().info("📍 [3/9] Adjusting grip...")
                    ok, detail = self.run_motion("grab_cup", {"speed_scale": 0.7}, timeout_s=60.0)
                    if not ok:
                        return self._fail_with_done(f"grab_cup: {detail}")
                    
                    step_offset = 3
                    total_steps = 9
                else:
                    self.get_logger().info("📍 [1/7] Timeout - using store cup")
                    ok, detail = self.run_motion("grab_cup", {"speed_scale": 0.7}, timeout_s=60.0)
                    if not ok:
                        return self._fail_with_done(f"grab_cup: {detail}")
                    
                    step_offset = 1
                    total_steps = 7
            else:
                # 매장컵
                self.get_logger().info("📍 [1/7] Grabbing cup...")
                ok, detail = self.run_motion("grab_cup", {"speed_scale": 0.7}, timeout_s=60.0)
                if not ok:
                    return self._fail_with_done(f"grab_cup: {detail}")
                
                step_offset = 1
                total_steps = 7

            # 2. B 위치로 이동
            self.get_logger().info(f"📍 [{step_offset+1}/{total_steps}] Moving to B...")
            ok, detail = self.run_motion("move_to_b", {"speed_scale": 0.7}, timeout_s=60.0)
            if not ok:
                return self._fail_with_done(f"move_to_b: {detail}")

            # 3. 저울 안정화 대기
            self._set_task("waiting_cup_placement")
            self.get_logger().info(f"📍 [{step_offset+2}/{total_steps}] Waiting for cup placement...")
            time.sleep(5)
            if not self._wait_for_stable_weight(min_weight=10.0, timeout_s=15.0):
                return self._fail_with_done("Cup not detected on scale B")

            # 4. 시리얼 주기
            self.get_logger().info(f"📍 [{step_offset+3}/{total_steps}] Dispensing cereal...")
            self._target_weight = target_g
            self._monitoring = True
            
            monitor_thread = threading.Thread(target=self._monitor_weight, daemon=True)
            monitor_thread.start()
            
            params = {"speed_scale": 0.7, "target_g": float(target_g)}
            ok, detail = self.run_motion("give_cereal_b", params, timeout_s=180.0)
            
            self._monitoring = False
            
            if not ok:
                return self._fail_with_done(f"give_cereal_b: {detail}")

            # 5. 컵 회수
            self.get_logger().info(f"📍 [{step_offset+4}/{total_steps}] Retrieving cup...")
            ok, detail = self.run_motion("retrieve_cup_from_b", {"speed_scale": 0.7}, timeout_s=60.0)
            if not ok:
                return self._fail_with_done(f"retrieve_cup_from_b: {detail}")

            # 6. 컵 감지 및 pickup
            self.get_logger().info(f"📍 [{step_offset+5}/{total_steps}] Cup detection and pickup...")
            if not self._handle_cup_detection_and_pickup():
                return self._fail_with_done("Cup detection failed")
            
            # 7. 🔥 홈 위치로 복귀
            self.get_logger().info(f"📍 [{step_offset+6}/{total_steps}] Going to home...")
            ok, detail = self.run_motion("go_home", {"speed_scale": 0.7}, timeout_s=60.0)
            if not ok:
                return self._fail_with_done(f"go_home: {detail}")

            self.get_logger().info("=" * 60)
            self.get_logger().info("✅ SEQUENCE B COMPLETED!")
            self.get_logger().info("=" * 60)
            
            # 🔥 성공 시 여기서만 order_done 발행
            self._publish_order_done(True)
            
        finally:
            self._monitoring = False
            self._active_scale = None
            self._target_weight = 0.0
            self._current_task = "idle"
            self._current_control_cmd = "none"
            
            # 🔥 컵 무게 리셋
            self._cup_weight_a = 0.0
            self._cup_weight_b = 0.0
            
            self._personal_cup_detected = False
            self._pickup_st1_detected = False
            self._pickup_st2_detected = False
            
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
        무게 모니터링 스레드
        🔥 컵 무게를 기준으로 상대적인 증가량 측정
        """
        self._set_task("dispensing_cereal")
        scale_name = self._active_scale.upper() if self._active_scale else 'UNKNOWN'
        
        # 🔥 절대 목표 무게 = 컵 무게 + 시리얼 목표량
        if self._active_scale == 'a':
            cup_weight = self._cup_weight_a
        elif self._active_scale == 'b':
            cup_weight = self._cup_weight_b
        else:
            self.get_logger().error("❌ Invalid active scale!")
            return
        
        absolute_target = cup_weight + self._target_weight
        
        self.get_logger().info(f"⏳ Weight monitoring (Scale {scale_name})")
        self.get_logger().info(f"   Cup weight: {cup_weight:.1f}g")
        self.get_logger().info(f"   Cereal target: {self._target_weight:.1f}g")
        self.get_logger().info(f"   Absolute target: {absolute_target:.1f}g")
        
        while self._monitoring:
            if self._active_scale == 'a':
                current = self._current_weight_a
            elif self._active_scale == 'b':
                current = self._current_weight_b
            else:
                break
            
            # 🔥 현재 시리얼 무게 계산
            cereal_weight = current - cup_weight
            
            # 🔥 절대 무게로 비교
            if current >= absolute_target:
                self.get_logger().info(f"✅ Target reached!")
                self.get_logger().info(f"   Total weight: {current:.1f}g")
                self.get_logger().info(f"   Cereal weight: {cereal_weight:.1f}g")
                self._weight_ok_pub.publish(Bool(data=True))
                break
            
            time.sleep(0.5)

    def _wait_for_stable_weight(self, min_weight: float, timeout_s: float) -> bool:
        """
        컵이 저울에 제대로 올려졌는지 확인
        🔥 안정화된 무게를 컵 무게로 저장 (영점 기준)
        """
        scale_name = self._active_scale.upper() if self._active_scale else 'UNKNOWN'
        
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
                return False
            
            if current >= min_weight:
                weight_change = abs(current - prev_weight)
                
                if weight_change < 2.0:
                    stable_duration += 0.1
                    if stable_duration >= required_stable_time:
                        # 🔥 안정화된 무게를 컵 무게로 저장
                        if self._active_scale == 'a':
                            self._cup_weight_a = current
                            self.get_logger().info(f"✅ Cup weight on Scale A: {current:.1f}g (baseline)")
                        elif self._active_scale == 'b':
                            self._cup_weight_b = current
                            self.get_logger().info(f"✅ Cup weight on Scale B: {current:.1f}g (baseline)")
                        
                        self.get_logger().info(f"✅ Stable weight: {current:.1f}g")
                        return True
                else:
                    stable_duration = 0.0
            else:
                stable_duration = 0.0
            
            prev_weight = current
            time.sleep(0.1)
        
        self.get_logger().error(f"❌ Cup placement timeout (Scale {scale_name})")
        return False

    def _fail(self, step, detail):
        """실패 처리"""
        self._set_task("failed")
        self._set_control_cmd("error")
        self.get_logger().error("=" * 60)
        self.get_logger().error(f"❌ FAILED: {step} - {detail}")
        self.get_logger().error("=" * 60)
        return False
    
    def _fail_with_done(self, detail: str):
        """실패 처리 + order_done 발행"""
        self._publish_order_done(False)
        return self._fail("sequence", detail)
    
    def destroy_node(self):
        """노드 종료"""
        self._status_publishing = False
        if self._status_thread.is_alive():
            self._status_thread.join(timeout=1.0)
        super().destroy_node()


def main():
    rclpy.init()
    try:
        node = NetworkManager()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        node.get_logger().info("🤖 NetworkManager v8-optimized-fixed ready!")
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()