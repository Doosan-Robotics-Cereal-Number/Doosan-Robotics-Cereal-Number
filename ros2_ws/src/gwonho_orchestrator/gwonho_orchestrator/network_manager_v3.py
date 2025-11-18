# gwonho_orchestrator/network_manager.py (v4)
import json
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String, Float32, Bool
from gwonho_interfaces.action import RunMotion


# 한글/영문 모두 수용
PORTION_MAP = {
    "적게": 100.0, "보통": 200.0, "많이": 300.0,
    "small": 100.0, "medium": 200.0, "large": 300.0,
}


class NetworkManager(Node):
    """
    v4: weight_ok 토픽 발행 추가
    - 무게 모니터링: weight/current 구독
    - 목표 달성 시: weight_ok 발행 (Bool)
    - motion_control이 이 신호를 받아 동작 중단
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
        
        # weight_ok 토픽 발행자 생성 (공통)
        weight_ok_topic = f'/{self.robot_id}/weight_ok'
        self._weight_ok_pub = self.create_publisher(Bool, weight_ok_topic, 10)

        self.get_logger().info("=" * 60)
        self.get_logger().info(f"[NetworkManager] namespace: {self.get_namespace()}")
        self.get_logger().info(f"[NetworkManager] Subscribed to:")
        self.get_logger().info(f"  - kiosk/order (String)")
        self.get_logger().info(f"  - {weight_topic_a} (Float32)")
        self.get_logger().info(f"  - {weight_topic_b} (Float32)")
        self.get_logger().info(f"[NetworkManager] Publishing:")
        self.get_logger().info(f"  - {weight_ok_topic} (Bool)")
        self.get_logger().info("=" * 60)

    # -------------------------
    # 주문 수신 및 파싱
    # -------------------------
    def _on_order(self, msg: String):
        """
        주문 메시지 파싱
        - JSON: {"seat":"A", "portion":"많이"}
        - 쉼표: "start_sequence_a,많이"
        """
        raw = (msg.data or "").strip()
        if not raw:
            self.get_logger().error("❌ 빈 주문 메시지 수신")
            return

        seat_cmd, target_g = None, None
        
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
            
        except Exception:
            # 2) 쉼표 구분 파싱
            try:
                parts = [x.strip() for x in raw.split(',', 1)]
                if len(parts) != 2:
                    raise ValueError("Invalid format")
                
                seat_cmd_str, portion_str = parts
                seat_cmd = seat_cmd_str
                target_g = PORTION_MAP.get(portion_str)
                
            except Exception:
                self.get_logger().error(
                    f"❌ 잘못된 주문 형식: '{raw}'\n"
                    f"   예시 1: 'start_sequence_a,많이'\n"
                    f"   예시 2: '{{\"seat\":\"A\",\"portion\":\"많이\"}}'"
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
                t = threading.Thread(target=self._sequence_a, args=(target_g,), daemon=True)
                t.start()
            elif seat_cmd == "start_sequence_b":
                t = threading.Thread(target=self._sequence_b, args=(target_g,), daemon=True)
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
    # 시퀀스 A/B
    # -------------------------
    def _sequence_a(self, target_g: float):
        """
        시퀀스 A: grab_cup → move_to_a → [저울 A 확인] → give_cereal_a(무게 모니터링)
        """
        try:
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"🚀 SEQUENCE A START (target={target_g}g)")
            self.get_logger().info("=" * 60)

            # 🔥 저울 A 활성화
            self._active_scale = 'a'

            # 1. 컵 잡기
            self.get_logger().info("📍 [1/4] Grabbing cup...")
            ok, detail = self.run_motion("grab_cup", {"speed_scale": 0.7}, timeout_s=60.0)
            if not ok:
                return self._fail("grab_cup", detail)

            # 2. A 위치로 이동
            self.get_logger().info("📍 [2/4] Moving to position A...")
            ok, detail = self.run_motion("move_to_a", {"speed_scale": 0.7}, timeout_s=60.0)
            if not ok:
                return self._fail("move_to_a", detail)

            # 3. 저울 A 안정화 대기 (컵이 제대로 올려졌는지)
            self.get_logger().info("📍 [3/4] Waiting for cup placement on scale A...")
            if not self._wait_for_stable_weight(min_weight=10.0, timeout_s=15.0):
                return self._fail("cup_placement", "Cup not detected on scale A")

            # 4. 🔥 무게 모니터링 시작 + 시리얼 주기
            self.get_logger().info("📍 [4/4] Dispensing cereal with weight monitoring (Scale A)...")
            self._target_weight = target_g
            self._monitoring = True
            
            # 무게 모니터링 스레드 시작
            monitor_thread = threading.Thread(target=self._monitor_weight, daemon=True)
            monitor_thread.start()
            
            # motion_control에서 give_cereal_a 실행 (반복 붓기)
            params = {"speed_scale": 0.7, "target_g": float(target_g)}
            ok, detail = self.run_motion("give_cereal_a", params, timeout_s=180.0)
            
            # 모니터링 종료
            self._monitoring = False
            
            if not ok:
                return self._fail("give_cereal_a", detail)

            self.get_logger().info("=" * 60)
            self.get_logger().info("✅ SEQUENCE A COMPLETED!")
            self.get_logger().info("=" * 60)
        finally:
            self._monitoring = False
            self._active_scale = None
            if self._busy_lock.locked():
                self._busy_lock.release()

    def _sequence_b(self, target_g: float):
        """
        시퀀스 B: grab_cup → move_to_b → [저울 B 확인] → give_cereal_b(무게 모니터링)
        """
        try:
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"🚀 SEQUENCE B START (target={target_g}g)")
            self.get_logger().info("=" * 60)

            # 🔥 저울 B 활성화
            self._active_scale = 'b'

            # 1. 컵 잡기
            self.get_logger().info("📍 [1/4] Grabbing cup...")
            ok, detail = self.run_motion("grab_cup", {"speed_scale": 0.7}, timeout_s=60.0)
            if not ok:
                return self._fail("grab_cup", detail)

            # 2. B 위치로 이동
            self.get_logger().info("📍 [2/4] Moving to position B...")
            ok, detail = self.run_motion("move_to_b", {"speed_scale": 0.7}, timeout_s=60.0)
            if not ok:
                return self._fail("move_to_b", detail)

            # 3. 저울 B 안정화 대기
            self.get_logger().info("📍 [3/4] Waiting for cup placement on scale B...")
            if not self._wait_for_stable_weight(min_weight=10.0, timeout_s=15.0):
                return self._fail("cup_placement", "Cup not detected on scale B")

            # 4. 🔥 무게 모니터링 시작 + 시리얼 주기
            self.get_logger().info("📍 [4/4] Dispensing cereal with weight monitoring (Scale B)...")
            self._target_weight = target_g
            self._monitoring = True
            
            # 무게 모니터링 스레드 시작
            monitor_thread = threading.Thread(target=self._monitor_weight, daemon=True)
            monitor_thread.start()
            
            # motion_control에서 give_cereal_b 실행
            params = {"speed_scale": 0.7, "target_g": float(target_g)}
            ok, detail = self.run_motion("give_cereal_b", params, timeout_s=180.0)
            
            # 모니터링 종료
            self._monitoring = False
            
            if not ok:
                return self._fail("give_cereal_b", detail)

            self.get_logger().info("=" * 60)
            self.get_logger().info("✅ SEQUENCE B COMPLETED!")
            self.get_logger().info("=" * 60)
        finally:
            self._monitoring = False
            self._active_scale = None
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
        - _active_scale에 따라 A 또는 B 저울 모니터링
        """
        scale_name = self._active_scale.upper() if self._active_scale else 'UNKNOWN'
        self.get_logger().info(f"⏳ Weight monitoring started (Scale {scale_name}, target={self._target_weight}g)")
        
        while self._monitoring:
            # 활성 저울에 따라 현재 무게 선택
            if self._active_scale == 'a':
                current = self._current_weight_a
            elif self._active_scale == 'b':
                current = self._current_weight_b
            else:
                self.get_logger().error("❌ Invalid active scale!")
                break
            
            if current >= self._target_weight:
                self.get_logger().info(f"✅ Target weight reached (Scale {scale_name}): {current:.1f}g >= {self._target_weight}g")
                
                # 🔥 weight_ok 신호 발행
                msg = Bool()
                msg.data = True
                self._weight_ok_pub.publish(msg)
                
                self.get_logger().info("📢 Published weight_ok signal!")
                break
            
            # 주기적 로그 (2초마다)
            time.sleep(0.5)
        
        self.get_logger().info("⏹️  Weight monitoring stopped")

    def _wait_for_stable_weight(self, min_weight: float, timeout_s: float) -> bool:
        """
        컵이 저울에 제대로 올려졌는지 확인
        - min_weight 이상의 무게가 감지되면 성공
        - 0.5초 동안 안정적으로 유지되어야 함
        - _active_scale에 따라 A 또는 B 저울 확인
        """
        scale_name = self._active_scale.upper() if self._active_scale else 'UNKNOWN'
        self.get_logger().info(f"⏳ Waiting for stable weight on Scale {scale_name} (>= {min_weight}g)...")
        
        stable_duration = 0.0
        required_stable_time = 0.5  # 0.5초 안정화
        
        start_time = time.time()
        prev_weight = 0.0
        
        while (time.time() - start_time) < timeout_s:
            # 활성 저울에 따라 현재 무게 선택
            if self._active_scale == 'a':
                current = self._current_weight_a
            elif self._active_scale == 'b':
                current = self._current_weight_b
            else:
                self.get_logger().error("❌ Invalid active scale!")
                return False
            
            # 최소 무게 이상이고, 변화가 적으면 안정화로 판단
            if current >= min_weight:
                weight_change = abs(current - prev_weight)
                
                if weight_change < 2.0:  # 2g 이하 변화
                    stable_duration += 0.1
                    if stable_duration >= required_stable_time:
                        self.get_logger().info(f"✅ Stable weight detected on Scale {scale_name}: {current:.1f}g")
                        return True
                else:
                    stable_duration = 0.0  # 변화가 크면 리셋
                
                if int(stable_duration * 10) % 2 == 0:  # 0.2초마다
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
        # Lock은 finally에서 해제됨


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