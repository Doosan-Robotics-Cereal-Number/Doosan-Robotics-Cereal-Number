import cv2
import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
import time

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import message_filters

import DR_init
from dsr_example.simple.gripper_drl_controller import GripperController

ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"
VELOCITY, ACC = 50, 50

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__("robot_controller_node")

        self.bridge = CvBridge()

        self.get_logger().info("ROS 2 구독자 설정을 시작합니다...")

        self.intrinsics = None
        self.latest_cv_color = None
        self.latest_cv_depth_mm = None

        # 카메라 토픽 구독
        self.color_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/color/image_raw'
        )
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/aligned_depth_to_color/image_raw'
        )
        self.info_sub = message_filters.Subscriber(
            self, CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info'
        )

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.info_sub], 
            queue_size=10, 
            slop=0.1
        )
        self.ts.registerCallback(self.synced_callback)

        # 컵 감지 토픽 구독
        self.cup_sub = self.create_subscription(
            Float32MultiArray,
            '/cup_detections',
            self.cup_detection_callback,
            10
        )

        self.get_logger().info("컬러/뎁스/카메라정보 및 컵 감지 토픽 구독 대기 중...")
        self.get_logger().info("화면이 나오지 않으면 Launch 명령어를 확인하세요.")

        self.gripper = None
        self.is_moving = False  # 로봇이 동작 중인지 확인
        
        try:
            from DSR_ROBOT2 import wait
            self.gripper = GripperController(node=self, namespace=ROBOT_ID)
            wait(2)
            if not self.gripper.initialize():
                self.get_logger().error("Gripper initialization failed. Exiting.")
                raise Exception("Gripper initialization failed")
            self.get_logger().info("그리퍼를 활성화합니다...")
            self.gripper_is_open = True
            self.gripper.move(700)
            
        except Exception as e:
            self.get_logger().error(f"An error occurred during gripper setup: {e}")
            rclpy.shutdown()

        self.get_logger().info("RealSense ROS 2 구독자와 로봇 컨트롤러가 초기화되었습니다.")

    def synced_callback(self, color_msg, depth_msg, info_msg):
        try:
            self.latest_cv_color = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            self.latest_cv_depth_mm = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
        
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge 변환 오류: {e}")
            return

        if self.intrinsics is None:
            self.intrinsics = rs.intrinsics()
            self.intrinsics.width = info_msg.width
            self.intrinsics.height = info_msg.height
            self.intrinsics.ppx = info_msg.k[2]
            self.intrinsics.ppy = info_msg.k[5]
            self.intrinsics.fx = info_msg.k[0]
            self.intrinsics.fy = info_msg.k[4]
            
            if info_msg.distortion_model == 'plumb_bob' or info_msg.distortion_model == 'rational_polynomial':
                self.intrinsics.model = rs.distortion.brown_conrady
            else:
                self.intrinsics.model = rs.distortion.none
            
            self.intrinsics.coeffs = list(info_msg.d)
            self.get_logger().info("카메라 내장 파라미터(Intrinsics) 수신 완료.")

    def cup_detection_callback(self, msg):
        """컵 감지 토픽 콜백 - 컵이 감지되면 자동으로 로봇 이동"""
        if self.is_moving:
            return  # 이미 로봇이 동작 중이면 무시
        
        if len(msg.data) < 4:
            return  # 데이터가 충분하지 않으면 무시
        
        # 첫 번째 컵의 좌표만 사용 (여러 개 감지되어도 첫 번째만)
        X = msg.data[0]  # 카메라 좌표계 X (미터)
        Y = msg.data[1]  # 카메라 좌표계 Y (미터)
        Z = msg.data[2]  # 카메라 좌표계 Z (미터)
        depth = msg.data[3]  # 깊이 (미터)

        self.get_logger().info(f"컵 감지됨! 카메라 좌표: X={X:.3f}, Y={Y:.3f}, Z={Z:.3f}, Depth={depth:.3f}m")

        # 카메라 좌표를 로봇 좌표로 변환 (mm 단위로)
        # 카메라 위치: (711, -143, 978) 기준
        x_mm = Y * 1000  # 카메라의 Y축 → 로봇의 X축
        y_mm = X * 1000  # 카메라의 X축 → 로봇의 Y축
        z_mm = Z * 1000  # 카메라의 Z축 → 로봇의 Z축

        # 로봇 베이스 좌표계로 최종 변환 (카메라 위치 711, -143, 978 반영)
        final_x = 711 + x_mm
        final_y = -143 + y_mm
        final_z = 978 - z_mm
        
        # 안전 높이 제한
        if final_z <= 150:
            final_z = 150

        self.get_logger().info(f"로봇 목표 좌표: X={final_x:.1f}, Y={final_y:.1f}, Z={final_z:.1f}")

        # 로봇 이동 및 그리퍼 제어
        self.move_robot_and_control_gripper(final_x, final_y, final_z)

    def stop_camera(self):
        pass

    def terminate_gripper(self):
        if self.gripper:
            self.gripper.terminate()

    def mouse_callback(self, event, u, v, flags, param):
        """수동 모드: 마우스 클릭으로도 동작 가능"""
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.is_moving:
                self.get_logger().warn("로봇이 이미 동작 중입니다.")
                return
                
            if self.latest_cv_depth_mm is None or self.intrinsics is None:
                self.get_logger().warn("아직 뎁스 프레임 또는 카메라 정보가 수신되지 않았습니다.")
                return

            try:
                depth_mm = self.latest_cv_depth_mm[v, u]
            except IndexError:
                self.get_logger().warn(f"클릭 좌표(u={u}, v={v})가 이미지 범위를 벗어났습니다.")
                return
            
            if depth_mm == 0:
                print(f"({u}, {v}) 지점의 깊이를 측정할 수 없습니다 (값: 0).")
                return

            depth_m = float(depth_mm) / 1000.0

            point_3d = rs.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], depth_m)

            x_mm = point_3d[1] * 1000
            y_mm = point_3d[0] * 1000
            z_mm = point_3d[2] * 1000

            # 카메라 위치 기준 변환
            final_x = 711 + x_mm
            final_y = -143 + y_mm
            final_z = 978 - z_mm
            
            if final_z <= 150:
                final_z = 150

            print("--- 변환된 최종 3D 좌표 ---")
            print(f"픽셀 좌표: (u={u}, v={v}), Depth: {depth_m*1000:.1f} mm")
            print(f"로봇 목표 좌표: X={final_x:.1f}, Y={final_y:.1f}, Z={final_z:.1f}\n")

            self.move_robot_and_control_gripper(final_x, final_y, final_z)
            print("=" * 50)

    def move_robot_and_control_gripper(self, x, y, z):
        from DSR_ROBOT2 import get_current_posx, movel, wait, movej
        from DR_common2 import posx, posj
        
        self.is_moving = True  # 동작 시작
        
        try:
            # 초기 자세 (조인트 각도)
            p_init_joint = posj(165, -60, -90, -105, 100, 15)
            # 초기 자세 (태스크 좌표)
            p_init_task = posx(483.24, 40.390, 55, 92, 91, 88.67)
            
            self.get_logger().info("초기 자세(조인트)로 이동합니다.")
            movej(p_init_joint, VELOCITY, ACC)
            wait(1.0)
            
            self.get_logger().info("초기 자세(태스크)로 이동합니다.")
            movel(p_init_task, vel=VELOCITY, acc=ACC)
            wait(1.0)
            
            # 현재 자세에서 회전값 유지
            current_pos = get_current_posx()[0]
            
            # 목표 지점 위쪽 (안전 높이)
            target_pos_list_up = [x, y, z + 100, current_pos[3], current_pos[4], current_pos[5]]
            # 최종 목표 지점
            target_pos_list = [x, y, z, current_pos[3], current_pos[4], current_pos[5]]

            # 목표 위쪽으로 이동
            self.get_logger().info(f"목표 위쪽으로 이동합니다: {target_pos_list_up}")
            movel(posx(target_pos_list_up), vel=VELOCITY, acc=ACC)
            wait(0.5)

            # 목표 지점으로 하강
            self.get_logger().info(f"목표 지점으로 하강합니다: {target_pos_list}")
            movel(posx(target_pos_list), vel=VELOCITY, acc=ACC)
            wait(0.5)

            # 그리퍼 열기
            self.get_logger().info("그리퍼를 엽니다.")
            self.gripper.move(700)
            wait(2)

            # 위로 상승
            self.get_logger().info("컵을 들어올립니다.")
            movel(posx(target_pos_list_up), vel=VELOCITY, acc=ACC)
            wait(0.5)

            # 그리퍼 닫기
            self.get_logger().info("그리퍼를 닫습니다.")
            self.gripper.move(0)
            wait(2)

            # 초기 자세로 복귀
            self.get_logger().info("초기 자세로 복귀합니다.")
            movej(p_init_joint, VELOCITY, ACC)
            wait(1.0)

        except Exception as e:
            self.get_logger().error(f"로봇 이동 및 그리퍼 제어 중 오류 발생: {e}")
        
        finally:
            self.is_moving = False  # 동작 완료

def main(args=None):
    rclpy.init(args=args)

    dsr_node = rclpy.create_node("dsr_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node

    try:
        from DSR_ROBOT2 import get_current_posx, movel, wait, movej
        from DR_common2 import posx, posj
    except ImportError as e:
        print(f"DSR_ROBOT2 라이브러리를 임포트할 수 없습니다: {e}")
        rclpy.shutdown()
        exit(1)

    robot_controller = RobotControllerNode()

    cv2.namedWindow("RealSense Camera")
    cv2.setMouseCallback("RealSense Camera", robot_controller.mouse_callback)

    print("컵이 감지되면 자동으로 로봇이 이동합니다.")
    print("수동 모드: 카메라 영상에서 원하는 지점을 클릭할 수도 있습니다.")
    print("'ESC' 키를 누르면 종료됩니다.")

    try:
        while rclpy.ok():
            rclpy.spin_once(robot_controller, timeout_sec=0.001)
            rclpy.spin_once(dsr_node, timeout_sec=0.001)

            if robot_controller.latest_cv_color is not None:
                display_image = robot_controller.latest_cv_color.copy()
                
                h, w, _ = display_image.shape
                cv2.circle(display_image, (w // 2, h // 2), 3, (0, 0, 255), -1)
                
                # 로봇 상태 표시
                status_text = "MOVING..." if robot_controller.is_moving else "READY"
                cv2.putText(display_image, status_text, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0) if not robot_controller.is_moving else (0, 165, 255), 2)
                
                cv2.imshow("RealSense Camera", display_image)

            if cv2.waitKey(1) & 0xFF == 27:
                break
    
    except KeyboardInterrupt:
        print("Ctrl+C로 종료합니다...")

    finally:
        print("프로그램을 종료합니다...")
        robot_controller.terminate_gripper()
        cv2.destroyAllWindows()
        robot_controller.destroy_node()
        dsr_node.destroy_node()
        rclpy.shutdown()
        print("종료 완료.")

if __name__ == '__main__':
    main()