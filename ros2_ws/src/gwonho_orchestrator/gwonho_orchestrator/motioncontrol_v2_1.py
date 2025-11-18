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

    _dbg_count = {'n': 0}
    def _debug_endpoints():
        if _dbg_count['n'] >= 5:
            return
        topics = {n for n, _ in node.get_topic_names_and_types()}
        svcs   = {n for n, _ in node.get_service_names_and_types()}
        tt = sorted([n for n in topics if "motion_control/run/_action" in n])
        ss = sorted([n for n in svcs   if "motion_control/run/_action" in n])
        node.get_logger().info("[DEBUG] action topics:  " + (", ".join(tt) if tt else "<none>"))
        node.get_logger().info("[DEBUG] action services:" + (", ".join(ss) if ss else "<none>"))
        _dbg_count['n'] += 1

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
                movej, movel, movesx, amovel, posj, posx, wait,
                set_robot_mode, ROBOT_MODE_AUTONOMOUS,
                task_compliance_ctrl, release_compliance_ctrl, DR_TOOL,
                set_desired_force, release_force
            )
            node.get_logger().info("✅ DSR_ROBOT2 imported successfully")
        except ImportError as e:
            node.get_logger().error(f"❌ Failed to import DSR_ROBOT2: {e}")

            return
        

        from dsr_msgs2.srv import MoveWait

        move_wait_cli = node.create_client(MoveWait, f'/{ROBOT_ID}/motion/move_wait')

        def wait_motion_end(timeout_s: float = 30.0) -> bool:
            # 서비스 준비 대기(최대 5초)
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


        def fb(gh, p, s):
            m = RunMotion.Feedback()
            m.progress = float(p)
            m.stage = s
            gh.publish_feedback(m)

        def step_movesx(params, gh, check_cancel):
            sp = float(params.get("speed_scale", 1.0))
            
            # 여러 경유점 정의
            path = [
                posx(620, -176.29, 122.680, 90, -90, 90),   # 점1
                posx(620, -176.29, 122.680, 90, -90, 90),   # 점2
                posx(592.47, 13.960, 122.68, 180, -90, 90),   # 점3
            ]
            
            fb(gh, 0.2, "starting_movesx")
            check_cancel()
            
            fb(gh, 0.5, "executing_path")
            movesx(path, vel=200*sp, acc=200*sp, mod=0)  # mod=0: 각 점에서 멈춤
            check_cancel()
            
            fb(gh, 1.0, "movesx_done")
            return True, "movesx done"
        

        def movej_blocking(q, v, a):
            ret = movej(q, v, a)      # 기존 호출
            ok  = wait_motion_end(60) # 완료까지 대기
            if not ok:
                raise RuntimeError("movej timeout")
            return ret

        def movel_blocking(x, v, a, *, mod=None):
            if mod is None:
                ret = movel(x, v, a)
            else:
                ret = movel(x, v, a, mod=mod)  # 필요시 mod=0 등 그대로 전달
            ok  = wait_motion_end(60)
            if not ok:
                raise RuntimeError("movel timeout")
            return ret

        def movesx_blocking(path, *, vel, acc, mod=0):
            ret = movesx(path, vel=vel, acc=acc, mod=mod)
            ok  = wait_motion_end(120)
            if not ok:
                raise RuntimeError("movesx timeout")
            return ret

        def step_go_to_home(params, gh, check_cancel ): 
            sp = float(params.get("speed_scale", 0.7))
            q_go_to_home_1 = posj(180, -0, -90, -0, -90, 90)
            q_go_to_home_2 = posj(180, -0, -90, -0, -90, -90)

            fb(gh, 0.25, "go_to_home_1"); movej(q_go_to_home_1, 50*sp, 50*sp); check_cancel()
            fb(gh, 0.25, "go_to_home_2"); movej(q_go_to_home_2, 50*sp, 50*sp); check_cancel()

            fb(gh, 1.0, "go_to_home_done")
            return True, "go_to_home_done"

        def step_grab_store_cup(params, gh, check_cancel):
            sp = float(params.get("speed_scale", 0.7))
            x_dn = posx(200, -130, 40, 90, 90, -90)
            x_st = posx(200, -35, 40, 90, 90, -90)
            x_up = posx(200, -35, 50, 90, 90, -90)
            x_xoffset = posx(250, -70, 75, 90, 90, -90)
            x_ready_to_carry = posx(340, -180, 75, 180, -90, 90)
            step_go_to_home(params, gh, check_cancel)
            fb(gh, 0.1, "approach"); movel(x_dn, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.3, "go_straight"); movel(x_st, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.5, "grab"); wait(0.2); check_cancel()
            fb(gh, 0.7, "grab_up"); movel(x_up, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.7, "move_xoffset"); movel(x_xoffset, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.7, "ready_to_carry "); movel(x_ready_to_carry, 200*sp, 200*sp); check_cancel()
            fb(gh, 1.0, "done")
            return True, "grab_cup done"

        def step_move_to_a(params, gh, check_cancel):
            sp = float(params.get("speed_scale", 0.7))
            x_move_to_a1 = posx(360, -240, 90, 180, -90, 90)
            x_move_to_a2 = posx(395, -295, 90, 180, -90, 90)
            x_move_to_a3 = posx(395, -295, 60, 160, -90, 90)

            fb(gh, 0.2, "x_move_to_a1"); movel(x_move_to_a1, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.5, "x_move_to_a2"); movel(x_move_to_a2, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.5, "x_move_to_a3"); movel(x_move_to_a3, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.8, "relese"); wait(0.2); check_cancel()
            fb(gh, 1.0, "move_to_a done")
            return True, "move_to_a done"
        
        def step_move_to_b(params, gh, check_cancel):
            sp = float(params.get("speed_scale", 0.7))
            x_move_to_b1 = posx(600, -150, 75, 180, -90, 90)
            x_move_to_b2 = posx(730, -150, 90, 90, -90, 90)
            x_move_to_b3 = posx(730, -220, 90, 90, -90, 90)
            x_move_to_b4 = posx(730, -220, 60, 90, -90, 90)

            fb(gh, 0.1, "x_move_to_b1"); movel(x_move_to_b1, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.3, "x_move_to_b2"); movel(x_move_to_b2, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.5, "x_move_to_b3"); movel(x_move_to_b3, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.7, "x_move_to_b4"); movel(x_move_to_b4, 50*sp, 50*sp); check_cancel()
            fb(gh, 0.9, "relese"); wait(0.2); check_cancel()
            fb(gh, 1.0, "move_to_b done")
            return True, "move_to_b done"
        
        
        

        def step_give_cereal_a (params, gh, check_cancel):
            sp = float(params.get("speed_scale", 0.7))
           
            x_give_cereal_a1 = posx(300, -260, 70, 160, -90, 90)
            x_give_cereal_a2 = posx(335, -210, 70, 0, 90, -90)
            x_give_cereal_a3 = posx(335, -210, 150, 180, -90, 90)
            q_give_cereal_a4 = posj(180, -0, -120, -0, -60, -90)
            x_give_cereal_a5 = posx(450, 25, 220, 180, 180, -90)
            x_give_cereal_a6 = posx(505, -60, 260, 90, -90, -90)
            q_give_cereal_a7 = posj(192.56, -38.11, -91.54, -81.91, -99.64, 220)
            x_give_cereal_a8 = posx(500, -112, 260, 90, -90, 90)
            x_give_cereal_a9 = posx(0, 0, 15, 0, 0, 0)   #tool 기준 
            x_give_cereal_a10 = posx(0, 0, 0, 0, 0, 100)   #tool 기준 
            x_give_cereal_a11 = posx(0, 0, 0, 0, 0, -150)   #tool 기준 
            q_give_cereal_a12 = posj(186, -37.41, -94.85, -86, -94.43, 222.4)

            fb(gh, 0.1, "give_cereal_a1"); movel(x_give_cereal_a1, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.15, "give_cereal_a2"); movel(x_give_cereal_a2, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.2, "give_cereal_a3"); movel(x_give_cereal_a3, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.25, "give_cereal_a4"); movej(q_give_cereal_a4, 50*sp, 50*sp); check_cancel()
            fb(gh, 0.3, "give_cereal_a5"); movel(x_give_cereal_a5, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.35, "give_cereal_a6"); movel(x_give_cereal_a6, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.4, "give_cereal_a7"); movej(q_give_cereal_a7, 50*sp, 50*sp); check_cancel()
            #on compliance
            fb(gh, 0.43, "grab"); wait(0.2); check_cancel()
            fb(gh, 0.46, "compliance_control_start"); task_compliance_ctrl([500, 500, 500, 100, 100, 100])
            fb(gh, 0.48, "give_cereal_a8"); amovel(x_give_cereal_a8, 50*sp, 50*sp); check_cancel() #비동기 
            #get_tool_force_Y
            fd = [0, 8, 0, 0, 0, 0];fctrl_dir= [0, 1,0, 0, 0, 0];set_desired_force(fd, dir=fctrl_dir)
            #on compliance off
            fb(gh, 0.46, "compliance_control_stop"); release_compliance_ctrl() ;release_force()
            wait(1.0)

            fb(gh, 0.5, "release"); wait(0.2); check_cancel()
            fb(gh, 0.55, "give_cereal_a9"); movel(x_give_cereal_a9, 200*sp, 200*sp,ref=DR_TOOL); check_cancel() #tool 기준으로 살짝 앞으로
            fb(gh, 0.6, "grab"); wait(0.2); check_cancel()

            #저울값이 일정 값 이상이 될때 까지 반복 수행 
            fb(gh, 0.65, "give_cereal_a10"); movel(x_give_cereal_a10, 200*sp, 200*sp,ref=DR_TOOL); check_cancel() 
            fb(gh, 0.75, "give_cereal_a11"); movel(x_give_cereal_a11, 200*sp, 200*sp,ref=DR_TOOL); check_cancel() 

            fb(gh, 0.8, "relese"); wait(0.2); check_cancel()
            fb(gh, 0.2, "give_cereal_a12"); movej(q_give_cereal_a12, 50*sp, 50*sp); check_cancel()
            fb(gh, 1.0, "give_cereal_a_done")
            return True, "give_cereal_a_done"
        
        def step_give_cereal_b (params, gh, check_cancel):
            sp = float(params.get("speed_scale", 0.7))
           
            x_give_cereal_b1 = posx(730, -150, 70, 90, -90, 90)
            x_give_cereal_b2 = posx(730, -60, 250, 90, -90, 90)
            x_give_cereal_b3 = posx(735, -110, 250, 90, -90, 90)
            x_give_cereal_b4 = posx(0, 0, 15, 0, 0, 0)   #tool 기준 
            x_give_cereal_b5 = posx(0, 0, 0, 0, 0, 100)   #tool 기준 
            x_give_cereal_b6 = posx(0, 0, 0, 0, 0, -120)   #tool 기준 
            q_give_cereal_b7 = posj(184.86, -82.91, -7.12, 270, -94.91, -178)

            fb(gh, 0.1, "give_cereal_b1"); movel(x_give_cereal_b1, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.2, "give_cereal_b2"); movel(x_give_cereal_b2, 200*sp, 200*sp); check_cancel()
            #on compliance
            fb(gh, 0.43, "grab"); wait(0.2); check_cancel()
            fb(gh, 0.46, "compliance_control_start"); task_compliance_ctrl([500, 500, 500, 100, 100, 100])
            fb(gh, 0.3, "give_cereal_b3"); amovel(x_give_cereal_b3, 50*sp, 50*sp); check_cancel() #비동기 
            #get_tool_force_Y
            fd = [0, 8, 0, 0, 0, 0];fctrl_dir= [0, 1,0, 0, 0, 0];set_desired_force(fd, dir=fctrl_dir)
            #on compliance off
            fb(gh, 0.46, "compliance_control_stop"); release_compliance_ctrl() ;release_force()
            wait(1.0)

            fb(gh, 0.4, "relese"); wait(0.2); check_cancel()
            fb(gh, 0.55, "give_cereal_b4"); movel(x_give_cereal_b4, 200*sp, 200*sp,ref=DR_TOOL); check_cancel() #tool 기준으로 살짝 앞으로
            fb(gh, 0.5, "grab"); wait(0.2); check_cancel()

            #저울값이 일정 값 이상이 될때 까지 반복 수행 
            fb(gh, 0.6, "give_cereal_b5"); movel(x_give_cereal_b5, 200*sp, 200*sp,ref=DR_TOOL); check_cancel() 
            fb(gh, 0.7, "give_cereal_b6"); movel(x_give_cereal_b6,200*sp, 200*sp,ref=DR_TOOL); check_cancel() 
            fb(gh, 0.8, "release"); wait(0.2); check_cancel()
            fb(gh, 0.9, "give_cereal_b7"); movej(q_give_cereal_b7, 50*sp, 50*sp); check_cancel()
            fb(gh, 1.0, "give_cereal_b_done")
            return True, "give_cereal_b_done"
        

        def step_retrieve_cup_from_a(params, gh, check_cancel):
            sp = float(params.get("speed_scale", 0.7))
            x_retrieve_cup_from_a1 = posx(500, -80, 250, 90,-90, 90)
            x_retrieve_cup_from_a2 = posx(500, -80, 90, 90,-90, 90)
            x_retrieve_cup_from_a3 = posx(500, -230, 90, 90, -90, 90)
            x_retrieve_cup_from_a4 = posx(500, -230, 70, 90, -90, 90)
            x_retrieve_cup_from_a5 = posx(500, -140, 90, 90, -90, 90)
            x_retrieve_cup_from_a6 = posx(635, -64, 90, 180, -90, 90)
            x_retrieve_cup_from_a7 = posx(635, -64, 300, 180, -90, 90)


            fb(gh, 0.1, "retrieve_cup_from_a1"); movel(x_retrieve_cup_from_a1, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.2, "retrieve_cup_from_a2"); movel(x_retrieve_cup_from_a2, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.3, "retrieve_cup_from_a3"); movel(x_retrieve_cup_from_a3, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.4, "retrieve_cup_from_a4"); movel(x_retrieve_cup_from_a4, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.5, "grab"); wait(0.2); check_cancel()
            fb(gh, 0.6, "retrieve_cup_from_a5"); movel(x_retrieve_cup_from_a5, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.7, "retrieve_cup_from_a6"); movel(x_retrieve_cup_from_a6, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.8, "retrieve_cup_from_a7"); movel(x_retrieve_cup_from_a7, 200*sp, 200*sp); check_cancel()
            fb(gh, 1.0, "retrieve_cup_from_a_done")
            return True, "retrieve_cup_from_a_done"
        

        def step_retrieve_cup_from_b(params, gh, check_cancel):
            sp = float(params.get("speed_scale", 0.7))
            x_retrieve_cup_from_b1 = posx(730, -150, 70, 90,-90, 90)
            x_retrieve_cup_from_b2 = posx(730, -220, 70, 90,-90, 90)
            x_retrieve_cup_from_b3 = posx(726, -190, 70, 90, -90, 90)
            x_retrieve_cup_from_b4 = posx(726, -190, 90, 90, -90, 90)
            x_retrieve_cup_from_b5 = posx(730, -150, 130, 90, -90, 90)
            q_retrieve_cup_from_b6 = posj(179.64, -64, -66, 271.8, -89.14, -140)
            x_retrieve_cup_from_b7 = posx(635, -64, 300, 180, -90, 90)


            fb(gh, 0.1, "retrieve_cup_from_b1"); movel(x_retrieve_cup_from_b1, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.2, "retrieve_cup_from_b2"); movel(x_retrieve_cup_from_b2, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.3, "grab"); wait(0.2); check_cancel()
            fb(gh, 0.4, "retrieve_cup_from_b3"); movel(x_retrieve_cup_from_b3, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.5 , "retrieve_cup_from_b4"); movel(x_retrieve_cup_from_b4, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.6, "retrieve_cup_from_b5"); movel(x_retrieve_cup_from_b5, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.7, "retrieve_cup_from_b6"); movej(q_retrieve_cup_from_b6, 50*sp, 50*sp); check_cancel()
            step_movesx(params, gh, check_cancel)
            fb(gh, 0.9, "retrieve_cup_from_b7"); movel(x_retrieve_cup_from_b7, 200*sp, 200*sp); check_cancel()
            fb(gh, 1.0, "retrieve_cup_from_b_done")
            return True, "retrieve_cup_from_b_done"
        

        def step_move_pickup_1 (params, gh, check_cancel):
            sp = float(params.get("speed_scale", 0.7))
            x_move_pickup_1_1 = posx(700, 200, 300, 180,-90, 90)
            x_move_pickup_1_2 = posx(700, 200, 90, 180,-90, 90)
            x_move_pickup_1_3 = posx(700, 200, 300, 180, -90, 90)
            x_move_pickup_1_4 = posx(500, 200, 30, 180, -90, 90)


            fb(gh, 0.1, "move_pickup_1_1"); movel(x_move_pickup_1_1, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.3, "move_pickup_1_2"); movel(x_move_pickup_1_2, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.5, "release"); wait(0.2); check_cancel()
            fb(gh, 0.7, "move_pickup_1_3"); movel(x_move_pickup_1_3, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.9, "move_pickup_1_4"); movel(x_move_pickup_1_4, 200*sp, 200*sp); check_cancel()
            
         
            fb(gh, 1.0, "move_pickup_1_done")
            return True, "move_pickup_1_done"
        
        def step_move_pickup_2 (params, gh, check_cancel):
            sp = float(params.get("speed_scale", 0.7))
            x_move_pickup_2_1 = posx(700, 360, 300, 180,-90, 90)
            x_move_pickup_2_2 = posx(700, 360, 90, 180,-90, 90)
            x_move_pickup_2_3 = posx(700, 360, 300, 180, -90, 90)
            x_move_pickup_2_4 = posx(500, 360, 30, 180, -90, 90)


            fb(gh, 0.1, "move_pickup_2_1"); movel(x_move_pickup_2_1, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.3, "move_pickup_2_2"); movel(x_move_pickup_2_2, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.5, "release"); wait(0.2); check_cancel()
            fb(gh, 0.7, "move_pickup_2_3"); movel(x_move_pickup_2_3, 200*sp, 200*sp); check_cancel()
            fb(gh, 0.9, "move_pickup_2_4"); movel(x_move_pickup_2_4, 200*sp, 200*sp); check_cancel()
            
         
            fb(gh, 1.0, "move_pickup_2_done")
            return True, "move_pickup_2_done"

        

        handlers.update({
            "grab_cup": step_grab_store_cup,
            "move_to_a": step_move_to_a,
            "move_to_b": step_move_to_b,
            "give_cereal_a": step_give_cereal_a,
            "give_cereal_b": step_give_cereal_b,
            "retrieve_cup_from_a": step_retrieve_cup_from_a,  
            "retrieve_cup_from_b": step_retrieve_cup_from_b,  
            "move_pickup_1": step_move_pickup_1,              
            "move_pickup_2": step_move_pickup_2,              
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

    