# gwonho_orchestrator/motion_control.py

import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.qos import QoSProfile
from gwonho_interfaces.action import RunMotion
import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"
ACTION_REL = "motion_control/run"

def main():
    rclpy.init()
    node: Node = rclpy.create_node("motion_control", namespace=ROBOT_ID)
    node.get_logger().info(f"[MotionControl] ns={node.get_namespace()} model={ROBOT_MODEL}")
    node.get_logger().info(f"[MotionControl] action(rel)={ACTION_REL}")

    ready = {"robot": False, "init_attempted": False}
    handlers = {}

    def execute_cb(goal_handle):
        node.get_logger().info("=" * 60)
        node.get_logger().info(f"🎯 GOAL RECEIVED: {goal_handle.request.name}")
        node.get_logger().info("=" * 60)
        
        node.get_logger().error("=" * 60)
        node.get_logger().error(f"🎯 GOAL RECEIVED: {goal_handle.request.name}")
        node.get_logger().error(f"🎯 PARAMS: {goal_handle.request.params_json}")
        node.get_logger().error("=" * 60)
        if not ready["robot"]:
            node.get_logger().warn("⚠️  execute called but robot not ready yet!")
            goal_handle.abort()
            return RunMotion.Result(success=False, detail="robot_mode_not_ready")
        
        name = goal_handle.request.name
        try:
            params = json.loads(goal_handle.request.params_json or "{}")
        except json.JSONDecodeError as e:
            node.get_logger().error(f"Invalid JSON params: {e}")
            params = {}
        
        fn = handlers.get(name)
        if not fn:
            node.get_logger().error(f"Unknown motion step: {name}")
            goal_handle.abort()
            return RunMotion.Result(success=False, detail=f"unknown step: {name}")

        def check_cancel():
            if goal_handle.is_cancel_requested:
                raise RuntimeError("canceled")
        
        try:
            ok, detail = fn(params, goal_handle, check_cancel)
            (goal_handle.succeed() if ok else goal_handle.abort())
            return RunMotion.Result(success=ok, detail=detail)
        except Exception as e:
            node.get_logger().error(f"{name} exception: {e}", exc_info=True)
            goal_handle.abort()
            return RunMotion.Result(success=False, detail=f"{name} exception: {e}")

    def on_cancel(_):
        return CancelResponse.ACCEPT

    # ActionServer 생성
    server = ActionServer(
        node, RunMotion, ACTION_REL,
        execute_callback=execute_cb, cancel_callback=on_cancel,
        goal_service_qos_profile=QoSProfile(depth=10),
        result_service_qos_profile=QoSProfile(depth=10),
        cancel_service_qos_profile=QoSProfile(depth=10),
        feedback_pub_qos_profile=QoSProfile(depth=10),
        status_pub_qos_profile=QoSProfile(depth=10),
    )

    # 디버그: 엔드포인트 확인
    def _debug_endpoints():
        topics = {n for n, _ in node.get_topic_names_and_types()}
        svcs = {n for n, _ in node.get_service_names_and_types()}
        tt = sorted([n for n in topics if "motion_control/run/_action" in n])
        ss = sorted([n for n in svcs if "motion_control/run/_action" in n])
        node.get_logger().info("[DEBUG] action topics:  " + (", ".join(tt) if tt else "<none>"))
        node.get_logger().info("[DEBUG] action services:" + (", ".join(ss) if ss else "<none>"))
    
    node.create_timer(1.0, _debug_endpoints)

    # DSR 초기화 (지연 실행)
    def _late_init():
        if ready["init_attempted"]:
            return
        
        late_timer.cancel()
        ready["init_attempted"] = True
        
        node.get_logger().info("🔧 Initializing DSR robot interface...")
        
        DR_init.__dsr__id = ROBOT_ID
        DR_init.__dsr__model = ROBOT_MODEL
        DR_init.__dsr__node = node

        try:
            from DSR_ROBOT2 import (
                movej, movel, posj, posx, wait,
                set_robot_mode, ROBOT_MODE_AUTONOMOUS
            )
            node.get_logger().info("✅ DSR_ROBOT2 imported successfully")
        except ImportError as e:
            node.get_logger().error(f"❌ Failed to import DSR_ROBOT2: {e}")
            return

        def fb(gh, p, s):
            m = RunMotion.Feedback()
            m.progress = float(p)
            m.stage = s
            gh.publish_feedback(m)

        def step_grab_cup(params, gh, check_cancel):
            sp = float(params.get("speed_scale", 0.7))
            q_pre = posj(0, -40, 90, 0, 90, 0)
            x_dn = posx(373, 100, 404, 12, -180, 12)
            x_up = posx(373, 100, 504, 12, -180, 12)
            fb(gh, 0.1, "approach"); movej(q_pre, 50*sp, 50*sp); check_cancel()
            fb(gh, 0.4, "down"); movel(x_dn, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.6, "grasp"); wait(0.2); check_cancel()
            fb(gh, 0.9, "retreat"); movel(x_up, 200*sp, 200*sp); check_cancel()
            fb(gh, 1.0, "done")
            return True, "grab_cup done"

        def step_move_to_a(params, gh, check_cancel):
            sp = float(params.get("speed_scale", 0.8))
            qA = posj(0, -35, 95, 0, 85, 0)
            xpl = posx(473, 100, 404, 12, -180, 12)
            fb(gh, 0.2, "approach_A"); movej(qA, 50*sp, 50*sp); check_cancel()
            fb(gh, 0.8, "linear_in"); movel(xpl, 200*sp, 200*sp); check_cancel()
            fb(gh, 1.0, "done")
            return True, "move_to_a done"

        def step_give_cereal_a(params, gh, check_cancel):
            ticks = int(params.get("ticks", 100))
            fb(gh, 0.1, "dial_grasp"); wait(0.2); check_cancel()
            fb(gh, 0.8, f"rotate ~ ticks={ticks}"); wait(1.0); check_cancel()
            fb(gh, 1.0, "done")
            return True, "give_cereal_a done"

        handlers.update({
            "grab_cup": step_grab_cup,
            "move_to_a": step_move_to_a,
            "give_cereal_a": step_give_cereal_a,
        })
        
        node.get_logger().info(f"✅ Registered {len(handlers)} motion handlers")

        # 로봇 모드 설정 - 비동기 서비스 호출
        from dsr_msgs2.srv import SetRobotMode, GetRobotMode
        
        set_mode_client = node.create_client(SetRobotMode, f'/{ROBOT_ID}/system/set_robot_mode')
        get_mode_client = node.create_client(GetRobotMode, f'/{ROBOT_ID}/system/get_robot_mode')
        
        attempt_count = [0]
        pending_request = [None]
        
        def try_set_mode():
            if ready["robot"]:
                return
            
            attempt_count[0] += 1
            
            # 이전 요청이 진행 중이면 스킵
            if pending_request[0] is not None and not pending_request[0].done():
                if attempt_count[0] % 3 == 1:
                    node.get_logger().warn(f"⏳ Waiting for previous request... (attempt #{attempt_count[0]})")
                return
            
            # 서비스 사용 가능 여부 확인
            if not get_mode_client.service_is_ready():
                if attempt_count[0] % 3 == 1:
                    node.get_logger().warn(f"⏳ Waiting for robot services... (attempt #{attempt_count[0]})")
                return
            
            node.get_logger().info(f"🔄 Checking robot mode... (attempt #{attempt_count[0]})")
            
            # 먼저 현재 모드 확인
            req = GetRobotMode.Request()
            future = get_mode_client.call_async(req)
            pending_request[0] = future
            
            def on_get_mode_response(fut):
                try:
                    response = fut.result()
                    current_mode = response.robot_mode
                    node.get_logger().info(f"📊 Current robot mode: {current_mode}")
                    
                    if current_mode == ROBOT_MODE_AUTONOMOUS:
                        # 이미 AUTONOMOUS 모드
                        ready["robot"] = True
                        node.get_logger().info("🎉 Robot is already in AUTONOMOUS mode!")
                        node.get_logger().info("=" * 60)
                        node.get_logger().info("✅ MOTION CONTROL IS NOW READY TO ACCEPT GOALS")
                        node.get_logger().info("=" * 60)
                    else:
                        # 모드 변경 필요
                        node.get_logger().info(f"🔄 Setting robot mode to AUTONOMOUS (current: {current_mode})...")
                        set_req = SetRobotMode.Request()
                        set_req.robot_mode = ROBOT_MODE_AUTONOMOUS
                        set_future = set_mode_client.call_async(set_req)
                        pending_request[0] = set_future
                        
                        def on_set_mode_response(set_fut):
                            try:
                                set_response = set_fut.result()
                                if set_response.success:
                                    ready["robot"] = True
                                    node.get_logger().info("🎉 Robot mode set to AUTONOMOUS (READY!)")
                                    node.get_logger().info("=" * 60)
                                    node.get_logger().info("✅ MOTION CONTROL IS NOW READY TO ACCEPT GOALS")
                                    node.get_logger().info("=" * 60)
                                else:
                                    node.get_logger().error("❌ Failed to set robot mode")
                            except Exception as e:
                                node.get_logger().error(f"❌ set_robot_mode error: {e}")
                            finally:
                                pending_request[0] = None
                        
                        set_future.add_done_callback(on_set_mode_response)
                        
                except Exception as e:
                    node.get_logger().error(f"❌ get_robot_mode error: {e}")
                    pending_request[0] = None
            
            future.add_done_callback(on_get_mode_response)
        
        node.create_timer(2.0, try_set_mode)

    late_timer = node.create_timer(0.1, _late_init)

    try:
        node.get_logger().info("Spinning motion_control node... Press Ctrl+C to stop")
        rclpy.spin(node)
    finally:
        server.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

    