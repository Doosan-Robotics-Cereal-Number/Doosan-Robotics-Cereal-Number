import json
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Empty, Bool, String
from gwonho_interfaces.action import RunMotion


class NetworkManager(Node):
    def __init__(self):
        super().__init__('network_manager')
        
        # 파라미터
        self.declare_parameter('robot_id', 'dsr01')
        self.robot_id = self.get_parameter('robot_id').value

        # 상태 변수
        self._busy = False
        self._weight_ok_flag = False

        

        # ActionClient 생성
        action_name = f'/{self.robot_id}/motion_control/run'
        self.get_logger().info(f"[NetworkManager] Creating ActionClient for: '{action_name}'")
        self.ac = ActionClient(self, RunMotion, action_name)

        # 여러 시퀀스를 위한 토픽 구독
        self.create_subscription(Empty, 'start_sequence_a', self._on_start_sequence_a, 10)
        self.create_subscription(Empty, 'start_sequence_b', self._on_start_sequence_b, 10)
        self.create_subscription(Empty, 'retrieve_a', self._on_retrieve_a, 10)
        self.create_subscription(Empty, 'retrieve_b', self._on_retrieve_b, 10)
        self.create_subscription(Empty, 'pickup_1', self._on_pickup_1, 10)
        self.create_subscription(Empty, 'pickup_2', self._on_pickup_2, 10)
        
        # 무게 확인 신호
        self.create_subscription(Bool, 'weight_ok', self._on_weight_ok, 10)

        self.get_logger().info(f"[NetworkManager] namespace: {self.get_namespace()}")
        self.get_logger().info(f"[NetworkManager] Subscribed to:")
        self.get_logger().info(f"  - start_sequence_a")
        self.get_logger().info(f"  - start_sequence_b")
        self.get_logger().info(f"  - retrieve_a")
        self.get_logger().info(f"  - retrieve_b")
        self.get_logger().info(f"  - pickup_1")
        self.get_logger().info(f"  - pickup_2")
        self.get_logger().info(f"  - weight_ok")

    def run_motion(self, name: str, params: dict, timeout_s: float):
        """모션 실행 - 개선된 버전"""
        self.get_logger().info(f"🎯 Running motion: '{name}'")
        
        goal = RunMotion.Goal()
        goal.name = name
        goal.params_json = json.dumps(params or {})

        send_future = self.ac.send_goal_async(goal, feedback_callback=self._on_feedback)
        
        start_time = time.time()
        while not send_future.done():
            if timeout_s and (time.time() - start_time) > timeout_s:
                self.get_logger().error(f"❌ send_goal timeout after {time.time() - start_time:.1f}s")
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

        self.get_logger().info(f"✅ Goal '{name}' accepted, waiting for result...")
        res_future = gh.get_result_async()
        
        start_time = time.time()
        while not res_future.done():
            if timeout_s and (time.time() - start_time) > timeout_s:
                self.get_logger().error("❌ get_result timeout, canceling...")
                gh.cancel_goal_async()
                return False, "result_timeout"
            time.sleep(0.01)

        res = res_future.result().result
        return bool(res.success), res.detail

    def _on_feedback(self, fb_msg):
        """피드백 수신"""
        fb = fb_msg.feedback
        self.get_logger().info(f"  [{fb.progress*100:.0f}%] {fb.stage}")

    # ================================================================
    # 시퀀스 시작 콜백들
    # ================================================================
    
    def _on_start_sequence_a(self, _):
        """시퀀스 A 시작: 컵 잡기 → A로 이동 → 시리얼 주기"""
        if self._busy:
            self.get_logger().warn("⚠️  Busy, ignoring request")
            return
        thread = threading.Thread(target=self._sequence_a, daemon=True)
        thread.start()

    def _on_start_sequence_b(self, _):
        """시퀀스 B 시작: 컵 잡기 → B로 이동 → 시리얼 주기"""
        if self._busy:
            self.get_logger().warn("⚠️  Busy, ignoring request")
            return
        thread = threading.Thread(target=self._sequence_b, daemon=True)
        thread.start()

    def _on_retrieve_a(self, _):
        """A에서 컵 회수"""
        if self._busy:
            self.get_logger().warn("⚠️  Busy, ignoring request")
            return
        thread = threading.Thread(target=self._retrieve_from_a, daemon=True)
        thread.start()

    def _on_retrieve_b(self, _):
        """B에서 컵 회수"""
        if self._busy:
            self.get_logger().warn("⚠️  Busy, ignoring request")
            return
        thread = threading.Thread(target=self._retrieve_from_b, daemon=True)
        thread.start()

    def _on_pickup_1(self, _):
        """픽업 위치 1로 이동"""
        if self._busy:
            self.get_logger().warn("⚠️  Busy, ignoring request")
            return
        thread = threading.Thread(target=self._pickup_1, daemon=True)
        thread.start()

    def _on_pickup_2(self, _):
        """픽업 위치 2로 이동"""
        if self._busy:
            self.get_logger().warn("⚠️  Busy, ignoring request")
            return
        thread = threading.Thread(target=self._pickup_2, daemon=True)
        thread.start()

    # ================================================================
    # 실제 시퀀스 구현
    # ================================================================

    def _sequence_a(self):
        """시퀀스 A: grab → move_to_a → wait weight → give_cereal_a"""
        self._busy = True
        self._weight_ok_flag = False
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("🚀 SEQUENCE A: Serving at position A")
        self.get_logger().info("=" * 60)

        # 1. 컵 잡기
        self.get_logger().info("📍 [1/4] Grabbing cup...")
        ok, detail = self.run_motion("grab_cup", {"speed_scale": 0.7}, timeout_s=40.0)
        if not ok:
            return self._fail("grab_cup", detail)

        # 2. A 위치로 이동
        self.get_logger().info("📍 [2/4] Moving to position A...")
        ok, detail = self.run_motion("move_to_a", {"speed_scale": 0.7}, timeout_s=40.0)
        if not ok:
            return self._fail("move_to_a", detail)

        # 3. 무게 확인 대기
        self.get_logger().info("📍 [3/4] Waiting for weight confirmation...")
        if not self._wait_weight_ok(timeout_s=15.0):
            return self._fail("weight_ok", "timeout")

        # 4. 시리얼 주기
        self.get_logger().info("📍 [4/4] Dispensing cereal at A...")
        ok, detail = self.run_motion("give_cereal_a", {"speed_scale": 0.7}, timeout_s=60.0)
        if not ok:
            return self._fail("give_cereal_a", detail)

        self.get_logger().info("=" * 60)
        self.get_logger().info("✅ SEQUENCE A COMPLETED!")
        self.get_logger().info("=" * 60)
        self._busy = False

    def _sequence_b(self):
        """시퀀스 B: grab → move_to_b → wait weight → give_cereal_b"""
        self._busy = True
        self._weight_ok_flag = False
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("🚀 SEQUENCE B: Serving at position B")
        self.get_logger().info("=" * 60)

        # 1. 컵 잡기
        self.get_logger().info("📍 [1/4] Grabbing cup...")
        ok, detail = self.run_motion("grab_cup", {"speed_scale": 0.7}, timeout_s=40.0)
        if not ok:
            return self._fail("grab_cup", detail)

        # 2. B 위치로 이동
        self.get_logger().info("📍 [2/4] Moving to position B...")
        ok, detail = self.run_motion("move_to_b", {"speed_scale": 0.7}, timeout_s=40.0)
        if not ok:
            return self._fail("move_to_b", detail)

        # 3. 무게 확인 대기
        self.get_logger().info("📍 [3/4] Waiting for weight confirmation...")
        if not self._wait_weight_ok(timeout_s=15.0):
            return self._fail("weight_ok", "timeout")

        # 4. 시리얼 주기
        self.get_logger().info("📍 [4/4] Dispensing cereal at B...")
        ok, detail = self.run_motion("give_cereal_b", {"speed_scale": 0.7}, timeout_s=60.0)
        if not ok:
            return self._fail("give_cereal_b", detail)

        self.get_logger().info("=" * 60)
        self.get_logger().info("✅ SEQUENCE B COMPLETED!")
        self.get_logger().info("=" * 60)
        self._busy = False

    def _retrieve_from_a(self):
        """A 위치에서 컵 회수"""
        self._busy = True
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("🔄 RETRIEVE: Getting cup from position A")
        self.get_logger().info("=" * 60)

        ok, detail = self.run_motion("retrieve_cup_from_a", {"speed_scale": 0.7}, timeout_s=60.0)
        if not ok:
            return self._fail("retrieve_cup_from_a", detail)

        self.get_logger().info("✅ Retrieved cup from A!")
        self._busy = False

    def _retrieve_from_b(self):
        """B 위치에서 컵 회수"""
        self._busy = True
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("🔄 RETRIEVE: Getting cup from position B")
        self.get_logger().info("=" * 60)

        ok, detail = self.run_motion("retrieve_cup_from_b", {"speed_scale": 0.7}, timeout_s=60.0)
        if not ok:
            return self._fail("retrieve_cup_from_b", detail)

        self.get_logger().info("✅ Retrieved cup from B!")
        self._busy = False

    def _pickup_1(self):
        """픽업 위치 1로 이동"""
        self._busy = True
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("📦 PICKUP: Moving to pickup location 1")
        self.get_logger().info("=" * 60)

        ok, detail = self.run_motion("move_pickup_1", {"speed_scale": 0.7}, timeout_s=40.0)
        if not ok:
            return self._fail("move_pickup_1", detail)

        self.get_logger().info("✅ Reached pickup location 1!")
        self._busy = False

    def _pickup_2(self):
        """픽업 위치 2로 이동"""
        self._busy = True
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("📦 PICKUP: Moving to pickup location 2")
        self.get_logger().info("=" * 60)

        ok, detail = self.run_motion("move_pickup_2", {"speed_scale": 0.7}, timeout_s=40.0)
        if not ok:
            return self._fail("move_pickup_2", detail)

        self.get_logger().info("✅ Reached pickup location 2!")
        self._busy = False

    # ================================================================
    # 유틸리티 함수들
    # ================================================================

    def _on_weight_ok(self, msg: Bool):
        """무게 확인 신호 수신"""
        if msg.data:
            self.get_logger().info("⚖️  Weight OK signal received!")
            self._weight_ok_flag = True

    def _wait_weight_ok(self, timeout_s: float) -> bool:
        """무게 확인 신호 대기"""
        self.get_logger().info(f"⏳ Waiting for weight_ok (timeout: {timeout_s}s)...")
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if self._weight_ok_flag:
                self.get_logger().info("✅ Weight confirmed!")
                return True
            time.sleep(0.1)
        self.get_logger().error(f"❌ Weight confirmation timeout after {timeout_s}s")
        return False

    def _fail(self, step, detail):
        """실패 처리"""
        self.get_logger().error("=" * 60)
        self.get_logger().error(f"❌ FAILED at step: {step}")
        self.get_logger().error(f"   Reason: {detail}")
        self.get_logger().error("=" * 60)
        self._busy = False
        self._weight_ok_flag = False
        return False


def main():
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=4)
    node = NetworkManager()
    executor.add_node(node)
    
    try:
        node.get_logger().info("=" * 60)
        node.get_logger().info("🤖 NetworkManager is ready!")
        node.get_logger().info("=" * 60)
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down NetworkManager...")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()