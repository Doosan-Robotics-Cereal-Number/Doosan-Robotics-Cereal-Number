import json
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Empty, Bool
from gwonho_interfaces.action import RunMotion


class NetworkManager(Node):
    def __init__(self):
        super().__init__('network_manager')
        
        self.declare_parameter('robot_id', 'dsr01')
        self.declare_parameter('start_topic', 'order_done_start_store_cup')
        self.declare_parameter('weight_ok_topic', 'weight_ok')

        self.robot_id = self.get_parameter('robot_id').value
        self.start_topic = self.get_parameter('start_topic').value
        self.weight_ok_topic = self.get_parameter('weight_ok_topic').value

        self._busy = False
        self._weight_ok_flag = False

        action_name = f'/{self.robot_id}/motion_control/run'
        self.get_logger().info(f"[NetworkManager] Creating ActionClient for: '{action_name}'")
        self.ac = ActionClient(self, RunMotion, action_name)

        self.create_subscription(Empty, self.start_topic, self._on_start, 10)
        self.create_subscription(Bool, self.weight_ok_topic, self._on_weight_ok, 10)

        self.get_logger().info(f"[NetworkManager] namespace: {self.get_namespace()}")
        self.get_logger().info(f"[NetworkManager] listening: '{self.start_topic}', '{self.weight_ok_topic}'")
        
        

    def _check_server_availability(self):
        if self.ac.server_is_ready():
            self.get_logger().info("[NetworkManager] ✅ Action server is AVAILABLE")
        else:
            self.get_logger().warn("[NetworkManager] ⚠️  Action server NOT available yet...")

    def run_motion(self, name: str, params: dict, timeout_s: float):
        self.get_logger().info(f"🎯 ATTEMPTING TO RUN MOTION '{name}' (timeout: {timeout_s}s)...")
        
        goal = RunMotion.Goal()
        goal.name = name
        goal.params_json = json.dumps(params or {})

        self.get_logger().info(f"📤 SENDING GOAL FOR '{name}'...")
        send_future = self.ac.send_goal_async(goal, feedback_callback=self._on_feedback)
        
        start_time = time.time()
        while not send_future.done():
            if timeout_s and (time.time() - start_time) > timeout_s:
                self.get_logger().error(f"send_goal timeout after {time.time() - start_time:.1f}s")
                return False, "send_timeout"
            time.sleep(0.01)

        try:
            gh = send_future.result()
        except Exception as e:
            self.get_logger().error(f"Failed to send goal: {e}")
            return False, f"send_failed: {e}"
        
        if not gh or not gh.accepted:
            self.get_logger().error("Goal rejected by server")
            return False, "rejected"

        self.get_logger().info(f"✅ Goal '{name}' accepted! Waiting for result...")
        res_future = gh.get_result_async()
        
        start_time = time.time()
        while not res_future.done():
            if timeout_s and (time.time() - start_time) > timeout_s:
                self.get_logger().error("get_result timeout, canceling...")
                gh.cancel_goal_async()
                return False, "result_timeout"
            time.sleep(0.01)

        res = res_future.result().result
        return bool(res.success), res.detail

    def _on_feedback(self, fb_msg):
        fb = fb_msg.feedback
        self.get_logger().info(f"[feedback] {fb.progress:.2f} {fb.stage}")

    def _on_start(self, _):
        if self._busy:
            self.get_logger().warn("Start ignored: busy")
            return
        
        thread = threading.Thread(target=self._run_sequence, daemon=True)
        thread.start()
    
    def _run_sequence(self):
        self._busy = True
        self._weight_ok_flag = False
        self.get_logger().info("=" * 50)
        self.get_logger().info("🚀 Start received → Beginning sequence")
        self.get_logger().info("=" * 50)

        self.get_logger().info("📍 Step 1/3: grab_cup")
        ok, detail = self.run_motion("grab_cup", {"bin": "cup_tray_1", "speed_scale": 0.7}, timeout_s=30.0)
        if not ok:
            self._fail("grab_cup", detail)
            return

        self.get_logger().info("📍 Step 2/3: move_to_a")
        ok, detail = self.run_motion("move_to_a", {"speed_scale": 0.8}, timeout_s=30.0)
        if not ok:
            self._fail("move_to_a", detail)
            return

        self.get_logger().info("📍 Step 3/3: waiting for weight_ok signal...")
        if not self._wait_weight_ok(timeout_s=10.0):
            self._fail("weight_ok", "timeout waiting for weight signal")
            return

        self.get_logger().info("✅ weight_ok received → give_cereal_a")
        ok, detail = self.run_motion("give_cereal_a", {"ticks": 120}, timeout_s=30.0)
        if not ok:
            self._fail("give_cereal_a", detail)
            return

        self.get_logger().info("=" * 50)
        self.get_logger().info("✅ [SUCCESS] All steps completed!")
        self.get_logger().info("=" * 50)
        self._busy = False

    def _on_weight_ok(self, msg: Bool):
        if msg.data:
            self.get_logger().info("⚖️  weight_ok signal received!")
            self._weight_ok_flag = True

    def _wait_weight_ok(self, timeout_s: float) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if self._weight_ok_flag:
                return True
            time.sleep(0.05)
        return False

    def _fail(self, step, detail):
        self.get_logger().error("=" * 50)
        self.get_logger().error(f"❌ [FAIL] {step}: {detail}")
        self.get_logger().error("=" * 50)
        self._busy = False
        self._weight_ok_flag = False
        return False


def main():
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=4)
    node = NetworkManager()
    executor.add_node(node)
    
    try:
        node.get_logger().info("NetworkManager spinning with MultiThreadedExecutor... Press Ctrl+C to stop")
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down NetworkManager...")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()