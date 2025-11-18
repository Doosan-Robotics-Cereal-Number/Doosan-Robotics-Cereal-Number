# gwonho_orchestrator/motion_control.py (v7 - fixed gripper initialization order)

import json
import time
import traceback
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.qos import QoSProfile
from gwonho_interfaces.action import RunMotion
from std_msgs.msg import Bool,Float32MultiArray
import DR_init
from dsr_example.simple.adaptive_gripper_drl import GripperController

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

    # =========================
    # Action execute callback
    # =========================
    def execute_cb(goal_handle):
        node.get_logger().info("=" * 60)
        node.get_logger().info(f"🎯 GOAL RECEIVED: {goal_handle.request.name}")
        node.get_logger().info("=" * 60)

        node.get_logger().debug("=" * 60)
        node.get_logger().debug(f"🎯 GOAL RECEIVED: {goal_handle.request.name}")
        node.get_logger().debug(f"🎯 PARAMS: {goal_handle.request.params_json}")
        node.get_logger().debug("=" * 60)

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
                goal_handle.canceled()
                raise RuntimeError("canceled")

        try:
            ok, detail = fn(params, goal_handle, check_cancel)
            (goal_handle.succeed() if ok else goal_handle.abort())
            return RunMotion.Result(success=ok, detail=detail)
        except Exception as e:
            node.get_logger().error(f"{name} exception: {e}\n{traceback.format_exc()}")
            goal_handle.abort()
            return RunMotion.Result(success=False, detail=f"{name} exception: {e}")

    def on_cancel(_):
        return CancelResponse.ACCEPT

    server = ActionServer(
        node, RunMotion, ACTION_REL,
        execute_callback=execute_cb, cancel_callback=on_cancel,
        goal_service_qos_profile=QoSProfile(depth=10),
        result_service_qos_profile=QoSProfile(depth=10),
        cancel_service_qos_profile=QoSProfile(depth=10),
        feedback_pub_qos_profile=QoSProfile(depth=10),
        status_pub_qos_profile=QoSProfile(depth=10),
    )

    _dbg_count = {'n': 0}

    def _debug_endpoints():
        if _dbg_count['n'] >= 5:
            return
        topics = {n for n, _ in node.get_topic_names_and_types()}
        svcs = {n for n, _ in node.get_service_names_and_types()}
        tt = sorted([n for n in topics if "motion_control/run/_action" in n])
        ss = sorted([n for n in svcs if "motion_control/run/_action" in n])
        node.get_logger().info("[DEBUG] action topics:  " + (", ".join(tt) if tt else "<none>"))
        node.get_logger().info("[DEBUG] action services:" + (", ".join(ss) if ss else "<none>"))
        _dbg_count['n'] += 1

    node.create_timer(1.0, _debug_endpoints)

    # =========================
    # DSR init (late)
    # =========================
    def _late_init():
        if ready["init_attempted"]:
            return

        late_timer.cancel()
        ready["init_attempted"] = True

        node.get_logger().info("🔧 Initializing DSR robot interface...")

        DR_init.__dsr__id = ROBOT_ID
        DR_init.__dsr__model = ROBOT_MODEL
        DR_init.__dsr__node = node

        # 🔥 gripper는 나중에 AUTONOMOUS 모드 설정 후 초기화
        gripper = None
        
        try:
            from DSR_ROBOT2 import (
                movej, movel, movesx, amovel, posj, posx, wait,
                set_robot_mode, ROBOT_MODE_AUTONOMOUS,
                task_compliance_ctrl, release_compliance_ctrl, DR_TOOL,
                set_desired_force, release_force ,get_current_posx,
                get_current_posj
            )
            
            node.get_logger().info("✅ DSR_ROBOT2 imported successfully")
            
            # 🔥 그리퍼는 로봇 모드가 AUTONOMOUS로 설정된 후에 초기화됩니다
            node.get_logger().info("⏳ Gripper will be initialized after robot mode is set to AUTONOMOUS")

        except ImportError as e:
            node.get_logger().error(f"❌ Failed to import DSR_ROBOT2: {e}")
            return

        # -------- move_wait client (for blocking sync) --------
        from dsr_msgs2.srv import MoveWait
        move_wait_cli = node.create_client(MoveWait, f'/{ROBOT_ID}/motion/move_wait')

        def wait_motion_end(timeout_s: float = 60.0) -> bool:
            if not move_wait_cli.wait_for_service(timeout_sec=5.0):
                node.get_logger().error("move_wait service not available")
                return False
            try:
                fut = move_wait_cli.call_async(MoveWait.Request())
                rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout_s)
                return fut.result() is not None
            except Exception as e:
                node.get_logger().error(f"move_wait call failed: {e}")
                return False

        # -------- get_tool_force client (for contact detection) --------
        from dsr_msgs2.srv import GetToolForce, MoveStop
        get_force_cli = node.create_client(GetToolForce, f'/{ROBOT_ID}/aux_control/get_tool_force')
        stop_cli = node.create_client(MoveStop, f'/{ROBOT_ID}/stop')


        # 개인컵 좌표 저장소
        personal_cup_coords = {'x': None, 'y': None, 'z': None, 'received': False}

        def on_personal_cup_coords(msg):
            """좌표 수신"""
            if len(msg.data) >= 3:
                personal_cup_coords['x'] = float(msg.data[0])
                personal_cup_coords['y'] = float(msg.data[1])
                personal_cup_coords['z'] = float(msg.data[2])
                personal_cup_coords['received'] = True

        # 구독 생성
        personal_cup_sub = node.create_subscription(
            Float32MultiArray,
            '/cup_stable_coordinates',
            on_personal_cup_coords,
            10
        )




        def quick_stop():
            """로봇 움직임 즉시 정지"""
            if not stop_cli.wait_for_service(timeout_sec=1.0):
                node.get_logger().error("stop service not available")
                return False
            
            req = Stop.Request()
            req.stop_mode = 1  # DR_QSTOP (Quick stop - Stop Category 2)
            
            try:
                fut = stop_cli.call_async(req)
                rclpy.spin_until_future_complete(node, fut, timeout_sec=1.0)
                if fut.result() and fut.result().success:
                    node.get_logger().info("✅ Robot stopped (QUICK)")
                    return True
                else:
                    node.get_logger().error("❌ Stop failed")
                    return False
            except Exception as e:
                node.get_logger().error(f"Stop call failed: {e}")
                return False
            

        def wait_for_contact(axis: int, min_force: float, max_force: float, ref: int = 0, timeout_s: float = 10.0) -> bool:
            """
            🔥 툴에 가해지는 외력을 측정하여 접촉 감지
            axis: 0(X), 1(Y), 2(Z)
            min_force: 최소 힘 (N) - 음수 가능
            max_force: 최대 힘 (N) - 음수 가능
            ref: 기준 좌표계 (0=Base, 1=Tool)
            
            Returns: True if contact detected, False if timeout
            """
            if not get_force_cli.wait_for_service(timeout_sec=5.0):
                node.get_logger().error("get_tool_force service not available")
                return False

            axis_names = {0: 'X', 1: 'Y', 2: 'Z'}
            node.get_logger().info(f"⏳ Waiting for contact on {axis_names.get(axis, '?')} axis (force: {min_force}~{max_force}N)...")

            start_time = time.time()
            check_interval = 0.05  # 50ms마다 체크
            
            while (time.time() - start_time) < timeout_s:
                try:
                    # 🔥 현재 툴에 가해지는 외력 측정
                    req = GetToolForce.Request()
                    req.ref = ref
                    
                    fut = get_force_cli.call_async(req)
                    rclpy.spin_until_future_complete(node, fut, timeout_sec=0.5)
                    
                    if fut.result() and fut.result().success:
                        tool_force = fut.result().tool_force
                        current_force = tool_force[axis]
                        
                        # 🔥 힘이 목표 범위에 들어왔는지 확인
                        if min_force <= current_force <= max_force:
                            elapsed = time.time() - start_time
                            node.get_logger().info(f"✅ Contact detected after {elapsed:.2f}s!")
                            node.get_logger().info(f"   Force on {axis_names.get(axis)} axis: {current_force:.2f}N")
                            quick_stop()
                            return True
                        
                        # 🔥 1초마다 현재 힘 로깅
                        elapsed = time.time() - start_time
                        if int(elapsed * 10) % 10 == 0:  # 1초마다
                            node.get_logger().info(
                                f"   Still waiting... ({int(elapsed)}s) - "
                                f"Current force: {current_force:.2f}N (target: {min_force}~{max_force}N)"
                            )
                    else:
                        node.get_logger().warn("get_tool_force call failed")

                except Exception as e:
                    node.get_logger().warn(f"Force measurement failed: {e}")

                time.sleep(check_interval)

            node.get_logger().warn(f"⚠️ Contact detection timeout after {timeout_s}s")
            return False

        # ========================================
        # 🔥 weight_ok 신호 구독
        # ========================================
        weight_ok_flag = {'reached': False}

        def on_weight_ok(msg: Bool):
            """network_manager가 목표 무게 달성 시 발행"""
            if msg.data:
                weight_ok_flag['reached'] = True
                node.get_logger().info("✅ weight_ok signal received!")

        weight_ok_topic = f'/{ROBOT_ID}/weight_ok'
        weight_ok_sub = node.create_subscription(
            Bool,
            weight_ok_topic,
            on_weight_ok,
            10
        )
        node.get_logger().info(f"✅ Subscribed to: {weight_ok_topic}")

        # -------- feedback helper --------
        def fb(gh, p, s):
            m = RunMotion.Feedback()
            m.progress, m.stage = float(p), str(s)
            gh.publish_feedback(m)

        # -------- 🔥 gripper helper (safe call) --------
        def gripper_safe_move(stroke: int):
            """그리퍼가 있으면 이동, 없으면 스킵"""
            if gripper is not None:
                try:
                    gripper.move(stroke)
                except Exception as e:
                    node.get_logger().warn(f"⚠️  Gripper move failed: {e}")
            else:
                node.get_logger().debug(f"⚠️  Gripper disabled - skipping move({stroke})")

        # -------- Motion 래퍼 --------
        def movej_blocking(q, v, a):
            movej(q, v, a)
            if not wait_motion_end(timeout_s=60.0):
                raise RuntimeError("movej_blocking: motion timeout")

        def movel_blocking(x, v, a, *, mod=None, ref=None):
            if ref is not None:
                movel(x, v, a, ref=ref)
            else:
                movel(x, v, a)
            if not wait_motion_end(timeout_s=60.0):
                raise RuntimeError("movel_blocking: motion timeout")

        def amovel_blocking(x, v, a):
            amovel(x, v, a)
            if not wait_motion_end(timeout_s=60.0):
                raise RuntimeError("amovel_blocking: motion timeout")

        def movesx_blocking(path, vel, acc, mod=0):
            movesx(path, vel, acc, mod)
            if not wait_motion_end(120):
                raise RuntimeError("movesx timeout")

        # ========================================
        # Step 정의
        # ========================================
        def step_movesx(params, gh, check_cancel):
            sp = float(params.get("speed_scale", 1.0))
            path = [
                posx(620, -176.29, 122.680, 90, -90, 90),
                posx(620, -176.29, 122.680, 90, -90, 90),
                posx(592.47, 13.960, 122.68, 180, -90, 90),
            ]
            fb(gh, 0.2, "starting_movesx"); check_cancel()
            fb(gh, 0.5, "executing_path"); movesx_blocking(path, 200*sp, 200*sp, 0); check_cancel()
            fb(gh, 1.0, "movesx_done")
            return True, "movesx done"

        def step_go_to_home(params, gh, check_cancel):
            sp = float(params.get("speed_scale", 0.7))
            q_go_to_home_1 = posj(180, -0, -90, -0, -90, 90)
            q_go_to_home_2 = posj(180, -0, -90, -0, -90, -90)

            fb(gh, 0.0, "release"); gripper.fast_move(0); wait(0.5); check_cancel()
            fb(gh, 0.25, "go_to_home_1"); movej_blocking(q_go_to_home_1, 50*sp, 50*sp); check_cancel()
            fb(gh, 0.50, "go_to_home_2"); movej_blocking(q_go_to_home_2, 50*sp, 50*sp); check_cancel()
            fb(gh, 1.00, "go_to_home_done")
            return True, "go_to_home_done"

        def step_grab_store_cup(params, gh, check_cancel):
            sp = float(params.get("speed_scale", 0.7))
            x_dn = posx(200, -130, 40, 90, 90, -90)
            x_st = posx(200, -35, 40, 90, 90, -90)
            x_up = posx(200, -35, 50, 90, 90, -90)
            x_xoffset = posx(250, -70, 75, 90, 90, -90)
            x_ready_to_carry = posx(340, -180, 75, 180, -90, 90)

            step_go_to_home(params, gh, check_cancel)
            fb(gh, 0.1, "approach"); movel_blocking(x_dn, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.3, "go_straight"); movel_blocking(x_st, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.5, "grab"); gripper.move(740); wait(0.5); check_cancel()
            fb(gh, 0.7, "grab_up"); movel_blocking(x_up, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.8, "move_xoffset"); movel_blocking(x_xoffset, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.9, "ready_to_carry"); movel_blocking(x_ready_to_carry, 200*sp, 200*sp); check_cancel()
            fb(gh, 1.0, "done")
            return True, "grab_cup done"

        def step_move_to_a(params, gh, check_cancel):
            sp = float(params.get("speed_scale", 0.7))
            x_move_to_a1 = posx(360, -240, 90, 180, -90, 90)
            x_move_to_a2 = posx(395, -295, 90, 180, -90, 90)
            x_move_to_a3 = posx(395, -295, 60, 160, -90, 90)

            fb(gh, 0.2, "x_move_to_a1"); movel_blocking(x_move_to_a1, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.5, "x_move_to_a2"); movel_blocking(x_move_to_a2, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.7, "x_move_to_a3"); movel_blocking(x_move_to_a3, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.9, "release"); gripper.fast_move(0); wait(0.5); check_cancel()
            fb(gh, 1.0, "move_to_a_done")
            return True, "move_to_a done"

        def step_move_to_b(params, gh, check_cancel):
            sp = float(params.get("speed_scale", 0.7))
            x_move_to_b1 = posx(600, -150, 75, 180, -90, 90)
            x_move_to_b2 = posx(730, -150, 90, 90, -90, 90)
            x_move_to_b3 = posx(730, -220, 90, 90, -90, 90)
            x_move_to_b4 = posx(730, -220, 60, 90, -90, 90)

            fb(gh, 0.1, "x_move_to_b1"); movel_blocking(x_move_to_b1, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.3, "x_move_to_b2"); movel_blocking(x_move_to_b2, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.5, "x_move_to_b3"); movel_blocking(x_move_to_b3, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.7, "x_move_to_b4"); movel_blocking(x_move_to_b4, 50*sp, 50*sp); check_cancel()
            fb(gh, 0.9, "release"); gripper.fast_move(0); wait(0.5); check_cancel()
            fb(gh, 1.0, "move_to_b_done")
            return True, "move_to_b done"

        def step_give_cereal_a(params, gh, check_cancel):
            """
            시리얼 주기 A
            🔥 Y축 접촉 감지 + weight_ok 신호를 받을 때까지 붓기 동작 반복
            """
            sp = float(params.get("speed_scale", 0.7))
            target_g = float(params.get("target_g", 200.0))

            node.get_logger().info(f"[give_cereal_a] Target: {target_g}g")

            # 원본 좌표들
            x_give_cereal_a1 = posx(300, -260, 70, 160, -90, 90)
            x_give_cereal_a2 = posx(335, -210, 70, 0, 90, -90)
            x_give_cereal_a3 = posx(335, -210, 150, 180, -90, 90)
            q_give_cereal_a4 = posj(180, -0, -120, -0, -60, -90)
            x_give_cereal_a5 = posx(450, 25, 220, 180, 180, -90)
            x_give_cereal_a6 = posx(505, -60, 260, 90, -90, -90)
            q_give_cereal_a7 = posj(192.56, -38.11, -91.54, -81.91, -99.64, 220)
            x_give_cereal_a8 = posx(500, -112, 260, 90, -90, 90)
            x_give_cereal_a9 = posx(0, 0, -50, 0, 0, 0)    # tool 기준
            x_give_cereal_a10 = posx(0, 0, 68, 0, 0, 0)    # tool 기준
            x_give_cereal_a11 = posx(0, 0, 0, 0, 0, 100)  # tool 기준
            x_give_cereal_a12 = posx(0, 0, 0, 0, 0, -150) # tool 기준
            q_give_cereal_a13 = posj(186, -37.41, -94.85, -86, -94.43, 222.4)

            # 초기 위치 이동
            fb(gh, 0.10, "give_cereal_a1"); movel_blocking(x_give_cereal_a1, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.15, "give_cereal_a2"); movel_blocking(x_give_cereal_a2, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.20, "give_cereal_a3"); movel_blocking(x_give_cereal_a3, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.25, "give_cereal_a4"); movej_blocking(q_give_cereal_a4, 50*sp, 50*sp); check_cancel()
            fb(gh, 0.30, "give_cereal_a5"); movel_blocking(x_give_cereal_a5, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.35, "give_cereal_a6"); movel_blocking(x_give_cereal_a6, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.40, "give_cereal_a7"); movej_blocking(q_give_cereal_a7, 50*sp, 50*sp); check_cancel()

            # 컴플라이언스 + 힘제어 시작
            fb(gh, 0.43, "grab"); gripper.fast_move(740); wait(0.5); check_cancel()
            fb(gh, 0.46, "compliance_start"); task_compliance_ctrl([700, 700, 700, 100, 100, 100]); check_cancel()
            
            
            # 🔥 천천히 이동 시작 (비동기)
            fb(gh, 0.48, "give_cereal_a8"); 
            amovel(x_give_cereal_a8, 30*sp, 30*sp)  # 속도 낮춤: 50 → 30
            node.get_logger().info("🔄 Moving forward slowly, waiting for contact...")

            # 🔥 Y축 접촉 감지 대기 ()
            # Y축 음의 방향으로 힘이 가해지면 접촉으로 판단
            fb(gh, 0.49, "waiting_contact")
            if not wait_for_contact(axis=1, min_force=7.3, max_force=10.0, ref=0, timeout_s=15.0):
                node.get_logger().error("❌ Contact not detected after 15s!")
                node.get_logger().warn("⚠️  Continuing anyway, but position may be incorrect...")
            else:
                node.get_logger().info("✅ Contact confirmed! Stopping motion...")

            check_cancel()

            # 컴플라이언스/힘제어 해제
            fb(gh, 0.5, "compliance_stop"); 
            release_compliance_ctrl()
            check_cancel()
            # 살짝 뒤로 
            fb(gh, 0.60, "give_cereal_a9"); movel_blocking(x_give_cereal_a9, 200*sp, 200*sp, ref=DR_TOOL); check_cancel()

            # 그리퍼 열고 살짝 앞으로 가서 그리퍼 닫기 
            fb(gh, 0.55, "release"); gripper.fast_move(0); wait(0.5); check_cancel()
            fb(gh, 0.60, "give_cereal_a10"); movel_blocking(x_give_cereal_a10, 200*sp, 200*sp, ref=DR_TOOL); check_cancel()
            fb(gh, 0.65, "grab"); gripper.move(740); wait(0.5); check_cancel()

            # 🔥 weight_ok 신호 초기화
            weight_ok_flag['reached'] = False

            # 🔥 weight_ok를 받을 때까지 붓기 동작 반복
            fb(gh, 0.7, "pouring_start")
            loop_count = 0
            max_loops = 100
            start_time = time.time()
            timeout_s = 120.0

            node.get_logger().info(f"⏳ Waiting for weight_ok signal (target={target_g}g)...")

            while (time.time() - start_time) < timeout_s and loop_count < max_loops:
                if weight_ok_flag['reached']:
                    node.get_logger().info(f"✅ Weight target reached! (after {loop_count} loops)")
                    break

                fb(gh, 0.75, f"pouring_loop_{loop_count}")
                movel_blocking(x_give_cereal_a11, 200*sp, 200*sp, ref=DR_TOOL); check_cancel()
                movel_blocking(x_give_cereal_a12, 200*sp, 200*sp, ref=DR_TOOL); check_cancel()

                loop_count += 1

                if loop_count % 5 == 0:
                    node.get_logger().info(f"   Pouring... (loop {loop_count})")

            else:
                node.get_logger().warn(f"⚠️  Pouring stopped: timeout or max loops (loops={loop_count})")

            fb(gh, 0.8, "release"); gripper.fast_move(0); wait(0.5); check_cancel()
            fb(gh, 0.90, "give_cereal_a13"); movej_blocking(q_give_cereal_a13, 50*sp, 50*sp); check_cancel()
            fb(gh, 1.00, "give_cereal_a_done")
            
            return True, f"give_cereal_a_done (loops={loop_count})"

        def step_give_cereal_b(params, gh, check_cancel):
            """
            시리얼 주기 B
            🔥 Y축 접촉 감지 + weight_ok 신호를 받을 때까지 붓기 동작 반복
            """
            sp = float(params.get("speed_scale", 0.7))
            target_g = float(params.get("target_g", 200.0))
 
            node.get_logger().info(f"[give_cereal_b] Target: {target_g}g")

            # 원본 좌표들
            x_give_cereal_b1 = posx(730, -150, 70, 90, -90, 90)
            x_give_cereal_b2 = posx(737, -77.34, 269.52, 90.98, -90.48, 89.44)
            x_give_cereal_b3 = posx(735, -110, 250, 90, -90, 90)
            x_give_cereal_b4 = posx(733.280, -55.62, -50, 91.16, -90.37, 88.96)    
            x_give_cereal_b5 = posx(735.670, -126.270, 266.410, 89.26, -90.45, 89.09)    
            x_give_cereal_b6 = posx(0, 0, 0, 0, 0, 100)   # tool 기준
            x_give_cereal_b7 = posx(0, 0, 0, 0, 0, -120)  # tool 기준
            q_give_cereal_b8 = posj(187.35, -79.49, -10.94, 270.53, -96.36, -180.08)

            # 초기 위치 이동
            fb(gh, 0.10, "give_cereal_b1"); movel_blocking(x_give_cereal_b1, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.20, "give_cereal_b2"); movel_blocking(x_give_cereal_b2, 200*sp, 200*sp); check_cancel()

            # 컴플라이언스 + 힘제어 시작
            fb(gh, 0.33, "grab"); gripper.fast_move(740); wait(0.5); check_cancel()
            fb(gh, 0.36, "compliance_start"); task_compliance_ctrl([700, 700, 700, 100, 100, 100]); check_cancel()
        
            # 🔥 천천히 이동 시작 (비동기)
            fb(gh, 0.40, "give_cereal_b3"); 
            amovel(x_give_cereal_b3, 30*sp, 30*sp)  # 속도 낮춤: 50 → 30
            node.get_logger().info("🔄 Moving forward slowly, waiting for contact...")

            # 🔥 Y축 접촉 감지 대기 (음수 힘: -15N ~ -3N)
            # Y축 음의 방향으로 힘이 가해지면 접촉으로 판단
            fb(gh, 0.49, "waiting_contact")
            if not wait_for_contact(axis=1, min_force=7.3, max_force=10.0, ref=0, timeout_s=15.0):
                node.get_logger().error("❌ Contact not detected after 15s!")
                node.get_logger().warn("⚠️  Continuing anyway, but position may be incorrect...")
            else:
                node.get_logger().info("✅ Contact confirmed! Stopping motion...")
            
            check_cancel()

            # 컴플라이언스/힘제어 해제
            fb(gh, 0.50, "compliance_stop"); 
            release_compliance_ctrl()

            fb(gh, 0.55, "give_cereal_b4"); movel_blocking(x_give_cereal_b4, 200*sp, 200*sp); check_cancel()

            check_cancel()
            fb(gh, 0.60, "release"); gripper.fast_move(0); wait(0.5); check_cancel()
            fb(gh, 0.65, "give_cereal_b5"); movel_blocking(x_give_cereal_b5, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.70, "grab"); gripper.move(500); wait(0.5); check_cancel()

            # 🔥 weight_ok 신호 초기화
            weight_ok_flag['reached'] = False

            # 🔥 weight_ok를 받을 때까지 붓기 동작 반복
            fb(gh, 0.75, "pouring_start")
            loop_count = 0
            max_loops = 100
            start_time = time.time()
            timeout_s = 120.0

            node.get_logger().info(f"⏳ Waiting for weight_ok signal (target={target_g}g)...")

            while (time.time() - start_time) < timeout_s and loop_count < max_loops:
                if weight_ok_flag['reached']:
                    node.get_logger().info(f"✅ Weight target reached! (after {loop_count} loops)")
                    break

                fb(gh, 0.8, f"pouring_loop_{loop_count}")
                movel_blocking(x_give_cereal_b6, 200*sp, 200*sp, ref=DR_TOOL); check_cancel()
                movel_blocking(x_give_cereal_b7, 200*sp, 200*sp, ref=DR_TOOL); check_cancel()

                loop_count += 1

                if loop_count % 5 == 0:
                    node.get_logger().info(f"   Pouring... (loop {loop_count})")

            else:
                node.get_logger().warn(f"⚠️  Pouring stopped: timeout or max loops (loops={loop_count})")

            # 마무리
            fb(gh, 0.85, "release"); gripper.fast_move(0); wait(0.5); check_cancel()
            fb(gh, 0.90, "give_cereal_b8"); movej_blocking(q_give_cereal_b8, 50*sp, 50*sp); check_cancel()
            fb(gh, 1.00, "give_cereal_b_done")

            return True, f"give_cereal_b_done (loops={loop_count})"

        def step_retrieve_cup_from_a(params, gh, check_cancel):
            sp = float(params.get("speed_scale", 0.7))
            x_retrieve_cup_from_a1 = posx(500, -80, 250, 90, -90, 90)
            x_retrieve_cup_from_a2 = posx(500, -80, 90, 90, -90, 90)
            x_retrieve_cup_from_a3 = posx(500, -230, 90, 90, -90, 90)
            x_retrieve_cup_from_a4 = posx(500, -230, 70, 90, -90, 90)
            x_retrieve_cup_from_a5 = posx(500, -140, 90, 90, -90, 90)
            x_retrieve_cup_from_a6 = posx(635, -64, 90, 180, -90, 90)
            x_retrieve_cup_from_a7 = posx(635, -64, 300, 180, -90, 90)

            fb(gh, 0.1, "retrieve_cup_from_a1"); movel_blocking(x_retrieve_cup_from_a1, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.2, "retrieve_cup_from_a2"); movel_blocking(x_retrieve_cup_from_a2, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.3, "retrieve_cup_from_a3"); movel_blocking(x_retrieve_cup_from_a3, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.4, "retrieve_cup_from_a4"); movel_blocking(x_retrieve_cup_from_a4, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.5, "grab"); gripper.move(740); wait(0.5); check_cancel()
            fb(gh, 0.6, "retrieve_cup_from_a5"); movel_blocking(x_retrieve_cup_from_a5, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.7, "retrieve_cup_from_a6"); movel_blocking(x_retrieve_cup_from_a6, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.8, "retrieve_cup_from_a7"); movel_blocking(x_retrieve_cup_from_a7, 200*sp, 200*sp); check_cancel()
            fb(gh, 1.0, "retrieve_cup_from_a_done")
            return True, "retrieve_cup_from_a_done"

        def step_retrieve_cup_from_b(params, gh, check_cancel):
            sp = float(params.get("speed_scale", 0.7))
            x_retrieve_cup_from_b1 = posx(730, -150, 70, 90, -90, 90)
            x_retrieve_cup_from_b2 = posx(730, -220, 70, 90, -90, 90)
            x_retrieve_cup_from_b3 = posx(726, -190, 70, 90, -90, 90)
            x_retrieve_cup_from_b4 = posx(726, -190, 90, 90, -90, 90)
            x_retrieve_cup_from_b5 = posx(730, -150, 130, 90, -90, 90)
            q_retrieve_cup_from_b6 = posj(179.64, -64, -66, 271.8, -89.14, -140)
            x_retrieve_cup_from_b7 = posx(635, -64, 300, 180, -90, 90)

            fb(gh, 0.1, "retrieve_cup_from_b1"); movel_blocking(x_retrieve_cup_from_b1, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.2, "retrieve_cup_from_b2"); movel_blocking(x_retrieve_cup_from_b2, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.3, "grab"); gripper.move(740); wait(0.5); check_cancel()
            fb(gh, 0.4, "retrieve_cup_from_b3"); movel_blocking(x_retrieve_cup_from_b3, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.5, "retrieve_cup_from_b4"); movel_blocking(x_retrieve_cup_from_b4, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.6, "retrieve_cup_from_b5"); movel_blocking(x_retrieve_cup_from_b5, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.7, "retrieve_cup_from_b6"); movej_blocking(q_retrieve_cup_from_b6, 50*sp, 50*sp); check_cancel()
            step_movesx(params, gh, check_cancel)
            fb(gh, 0.9, "retrieve_cup_from_b7"); movel_blocking(x_retrieve_cup_from_b7, 200*sp, 200*sp); check_cancel()
            fb(gh, 1.0, "retrieve_cup_from_b_done")
            return True, "retrieve_cup_from_b_done"

        def step_move_pickup_1(params, gh, check_cancel):
            sp = float(params.get("speed_scale", 0.7))
            x_move_pickup_1_1 = posx(680, 255, 300, 180, -90, 90)
            x_move_pickup_1_2 = posx(680, 255, 90, 180, -90, 90)
            x_move_pickup_1_3 = posx(680, 255, 300, 180, -90, 90)
            x_move_pickup_1_4 = posx(500, 255, 300, 180, -90, 90)

            fb(gh, 0.1, "move_pickup_1_1"); movel_blocking(x_move_pickup_1_1, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.3, "move_pickup_1_2"); movel_blocking(x_move_pickup_1_2, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.5, "release"); gripper.fast_move(0); wait(0.5); check_cancel()
            fb(gh, 0.7, "move_pickup_1_3"); movel_blocking(x_move_pickup_1_3, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.9, "move_pickup_1_4"); movel_blocking(x_move_pickup_1_4, 200*sp, 200*sp); check_cancel()
            fb(gh, 1.0, "move_pickup_1_done")
            return True, "move_pickup_1_done"

        def step_move_pickup_2(params, gh, check_cancel):
            sp = float(params.get("speed_scale", 0.7))
            x_move_pickup_2_1 = posx(700, 360, 300, 180, -90, 90)
            x_move_pickup_2_2 = posx(700, 360, 90, 180, -90, 90)
            x_move_pickup_2_3 = posx(700, 360, 300, 180, -90, 90)
            x_move_pickup_2_4 = posx(500, 360, 300, 180, -90, 90)

            fb(gh, 0.1, "move_pickup_2_1"); movel_blocking(x_move_pickup_2_1, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.3, "move_pickup_2_2"); movel_blocking(x_move_pickup_2_2, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.5, "release"); gripper.fast_move(0); wait(0.5); check_cancel()
            fb(gh, 0.7, "move_pickup_2_3"); movel_blocking(x_move_pickup_2_3, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.9, "move_pickup_2_4"); movel_blocking(x_move_pickup_2_4, 200*sp, 200*sp); check_cancel()
            fb(gh, 1.0, "move_pickup_2_done")
            return True, "move_pickup_2_done"
        

        def step_ready_personal_cup (params, gh, check_cancel): 
            sp = float(params.get("speed_scale", 0.7))
            q_ready_personal_cup = posj(164.71, -58.85, -92, -103.43, 97.38, 118.29)
            fb(gh, 1.0, "q_ready_personal_cup "); movej_blocking(q_ready_personal_cup , 50*sp, 50*sp); check_cancel()
            fb(gh, 1.0, "ready_personal_cup_done")
            return True, "ready_personal_cup_done"
        

        def step_grab_personal_cup(params, gh, check_cancel): 
            """🔥 좌표 기반 개인컵 잡기"""
            sp = float(params.get("speed_scale", 0.7))
            
            # 좌표 수신 확인
            if not personal_cup_coords['received']:
                node.get_logger().error("❌ Personal cup coordinates not received!")
                return False, "coordinates_not_received"
            
            x = personal_cup_coords['x']
            y = personal_cup_coords['y']
            z = personal_cup_coords['z']
            
            node.get_logger().info(f"🎯 Using coordinates: X={x:.1f}, Y={y:.1f}, Z={z:.1f}")
            
            try:
                # 현재 회전 유지
                current_pos = get_current_posx()[0]
                rx, ry, rz = current_pos[3], current_pos[4], current_pos[5]
                
                # [1/9] X 좌표만 이동
                fb(gh, 0.05, "moving_to_x")
                node.get_logger().info(f"📍 [1/9] Moving to X={x:.1f}...")
                waypoint_x = [x, current_pos[1], current_pos[2], rx, ry, rz]
                movel_blocking(posx(waypoint_x), 200*sp, 200*sp)  # ✅ blocking
                check_cancel()
                
                # [2/9] 그리퍼 열기
                fb(gh, 0.15, "opening_gripper")
                node.get_logger().info("📍 [2/9] Opening gripper...")
                gripper.fast_move(0)
                wait(1.0)  # ✅ wait 추가
                check_cancel()
                
                # [3/9] 목표 위치로 이동
                fb(gh, 0.25, "moving_to_target")
                node.get_logger().info(f"📍 [3/9] Moving to target...")
                target = [x, y, z, rx, ry, rz]
                movel_blocking(posx(target), 200*sp, 200*sp)  # ✅ blocking
                check_cancel()
                
                # [4/9] 그리퍼 닫기
                fb(gh, 0.35, "closing_gripper")
                node.get_logger().info("📍 [4/9] Closing gripper...")
                gripper.move(740)
                wait(1.0)  # ✅ wait 추가
                check_cancel()
                
                # [5/9] 원래 위치로 복귀
                fb(gh, 0.45, "returning")
                node.get_logger().info("📍 [5/9] Returning...")
                movel_blocking(current_pos, 200*sp, 200*sp)  # ✅ blocking
                check_cancel()
                
                # [6/9] 개인컵 이동 자세
                fb(gh, 0.55, "move_personal_cup")
                node.get_logger().info("📍 [6/9] Moving to carry position...")
                q_move_personal_cup = posj(135.45, -50.21, -129.72, -134.33, 87.7, 90)
                movej_blocking(q_move_personal_cup, 50*sp, 50*sp)  # ✅ blocking
                check_cancel()
                
                gripper.fast_move(0)
                wait(0.5)  # ✅ wait 추가
                check_cancel()
                
                # [7/9] 홈 경유점들
                fb(gh, 0.65, "waypoints")
                node.get_logger().info("📍 [7/9] Moving through waypoints...")
                x_personal_go_to_home1 = posx(205, -29, 150, 90, 90, -90)
                movel_blocking(x_personal_go_to_home1, 200*sp, 200*sp)
                check_cancel()
                
                fb(gh, 0.70, "waypoint2")
                x_personal_go_to_home2 = posx(205, -200, 150, 90, 90, -90)
                movel_blocking(x_personal_go_to_home2, 200*sp, 200*sp)
                check_cancel()
                
                fb(gh, 0.75, "waypoint3")
                q_personal_go_to_home3 = posj(118.83, 0, -109.85, -148.12, 114.09, 102.24)
                movej_blocking(q_personal_go_to_home3, 50*sp, 50*sp)
                check_cancel()
                
                fb(gh, 0.80, "waypoint4")
                q_personal_go_to_home4 = posj(90, 0, -109.85, 0, 114.09, 102.24)
                movej_blocking(q_personal_go_to_home4, 50*sp, 50*sp)
                check_cancel()
                
                fb(gh, 0.85, "waypoint5")
                q_personal_go_to_home5 = posj(180, 0, -109.85, 0, 114.09, 102.24)
                movej_blocking(q_personal_go_to_home5, 50*sp, 50*sp)
                check_cancel()
                
                # [8/9] 홈 위치로 최종 이동
                fb(gh, 0.90, "going_home")
                node.get_logger().info("📍 [8/9] Going to home position...")
                step_go_to_home(params, gh, check_cancel)  # ✅ 파라미터 전달
                
                fb(gh, 1.0, "grab_personal_cup_done")
                node.get_logger().info("✅ [9/9] Personal cup grabbed successfully!")
                
                # 좌표 리셋
                personal_cup_coords['received'] = False
                
                return True, "grab_personal_cup_done"
            
            except Exception as e:
                node.get_logger().error(f"❌ grab_personal_cup error: {e}")
                personal_cup_coords['received'] = False  # ✅ 에러 시에도 리셋
                return False, f"error: {e}"

                
        # 등록
        handlers.update({
            "ready_personal_cup" :step_ready_personal_cup,
            "grab_personal_cup" : step_grab_personal_cup,
            "grab_cup": step_grab_store_cup,
            "move_to_a": step_move_to_a,
            "move_to_b": step_move_to_b,
            "give_cereal_a": step_give_cereal_a,
            "give_cereal_b": step_give_cereal_b,
            "retrieve_cup_from_a": step_retrieve_cup_from_a,
            "retrieve_cup_from_b": step_retrieve_cup_from_b,
            "move_pickup_1": step_move_pickup_1,
            "move_pickup_2": step_move_pickup_2,
            "go_home": step_go_to_home,
        })
        node.get_logger().info(f"✅ Registered {len(handlers)} motion handlers")

        # -------- Robot mode to AUTONOMOUS --------
        from dsr_msgs2.srv import SetRobotMode, GetRobotMode
        set_mode_client = node.create_client(SetRobotMode, f'/{ROBOT_ID}/system/set_robot_mode')
        get_mode_client = node.create_client(GetRobotMode, f'/{ROBOT_ID}/system/get_robot_mode')

        attempt_count = [0]
        pending_request = [None]

        def try_set_mode():
            if ready["robot"]:
                return

            attempt_count[0] += 1

            if pending_request[0] is not None and not pending_request[0].done():
                if attempt_count[0] % 3 == 1:
                    node.get_logger().warn(f"⏳ Waiting for previous request... (attempt #{attempt_count[0]})")
                return

            if not get_mode_client.service_is_ready():
                if attempt_count[0] % 3 == 1:
                    node.get_logger().warn(f"⏳ Waiting for robot services... (attempt #{attempt_count[0]})")
                return

            node.get_logger().info(f"🔄 Checking robot mode... (attempt #{attempt_count[0]})")
            req = GetRobotMode.Request()
            future = get_mode_client.call_async(req)
            pending_request[0] = future

            def on_get_mode_response(fut):
                try:
                    response = fut.result()
                    current_mode = response.robot_mode
                    node.get_logger().info(f"📊 Current robot mode: {current_mode}")

                    if current_mode == ROBOT_MODE_AUTONOMOUS:
                        ready["robot"] = True
                        node.get_logger().info("🎉 Robot is already in AUTONOMOUS mode!")
                        
                        # 🔥 그리퍼 초기화 (AUTONOMOUS 모드 확인 후)
                        node.get_logger().info("🔧 Initializing gripper...")
                        try:
                            nonlocal gripper
                            gripper = GripperController(node=node, namespace=ROBOT_ID)
                            if gripper.initialize():
                                node.get_logger().info("✅ Gripper initialized successfully")
                                wait(2)
                            else:
                                node.get_logger().warn("⚠️  Gripper initialization failed - continuing without gripper")
                                gripper = None
                        except Exception as e:
                            node.get_logger().warn(f"⚠️  Gripper setup failed: {e} - continuing without gripper")
                            gripper = None
                        
                        node.get_logger().info("=" * 60)
                        node.get_logger().info("✅ MOTION CONTROL IS NOW READY TO ACCEPT GOALS")
                        node.get_logger().info("=" * 60)
                    else:
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
                                    
                                    # 🔥 그리퍼 초기화 (모드 변경 성공 후)
                                    node.get_logger().info("🔧 Initializing gripper...")
                                    try:
                                        nonlocal gripper
                                        gripper = GripperController(node=node, namespace=ROBOT_ID)
                                        if gripper.initialize():
                                            node.get_logger().info("✅ Gripper initialized successfully")
                                            wait(2)
                                        else:
                                            node.get_logger().warn("⚠️  Gripper initialization failed - continuing without gripper")
                                            gripper = None
                                    except Exception as e:
                                        node.get_logger().warn(f"⚠️  Gripper setup failed: {e} - continuing without gripper")
                                        gripper = None
                                    
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