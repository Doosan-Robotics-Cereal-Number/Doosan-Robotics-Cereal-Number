#!/usr/bin/env python3
"""
Voice Order Listener Node

키오스크에서 /voice_order/start 토픽을 받으면
음성 주문 시스템(main_gpt.py)을 실행하는 노드
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
import threading
from ament_index_python.packages import get_package_share_directory


class VoiceOrderListener(Node):
    def __init__(self):
        super().__init__('voice_order_listener')

        # Subscriber 생성
        self.subscription = self.create_subscription(
            String,
            '/dsr01/kiosk/start_voice_order',
            self.voice_order_callback,
            10
        )

        # Publisher 생성 (주문 결과 발행용)
        self.order_publisher = self.create_publisher(
            String,
            '/dsr01/kiosk/order',
            10
        )

        # main_gpt.py 경로 (share 폴더에서 찾기)
        package_share_dir = get_package_share_directory('vocie_order_bridge')
        self.voice_order_path = os.path.join(package_share_dir, 'voice_order')
        self.main_gpt_script = os.path.join(self.voice_order_path, 'main_gpt.py')

        # 실행 중인 프로세스
        self.voice_process = None

        self.get_logger().info('🎙️ Voice Order Listener 노드 시작')
        self.get_logger().info(f'📁 Script path: {self.main_gpt_script}')

        # 파일 존재 확인
        if not os.path.exists(self.main_gpt_script):
            self.get_logger().error(f'❌ main_gpt.py를 찾을 수 없음: {self.main_gpt_script}')
        else:
            self.get_logger().info('✓ main_gpt.py 확인됨')


    def voice_order_callback(self, msg):
        """토픽을 받으면 실행되는 콜백 함수"""

        command = msg.data.strip().lower()

        if command == 'start_voice_order':
            self.get_logger().info('')
            self.get_logger().info('═══════════════════════════════════════')
            self.get_logger().info('🎙️ 음성 주문 시작 신호 수신!')
            self.get_logger().info('═══════════════════════════════════════')

            # 이미 실행 중이면 종료
            if self.voice_process and self.voice_process.poll() is None:
                self.get_logger().warn('⚠️  이미 실행 중. 기존 프로세스 종료')
                self.voice_process.terminate()
                try:
                    self.voice_process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.voice_process.kill()

            # main_gpt.py 실행
            self.start_voice_order()

        elif command == 'stop':
            self.get_logger().info('🛑 음성 주문 중지 신호 수신')
            self.stop_voice_order()


    def start_voice_order(self):
        """main_gpt.py 실행"""

        if not os.path.exists(self.main_gpt_script):
            self.get_logger().error(f'❌ main_gpt.py 없음: {self.main_gpt_script}')
            return

        try:
            self.get_logger().info(f'▶️  실행: python3 {self.main_gpt_script}')

            # subprocess로 실행 (stdout 캡처)
            # -u 옵션: unbuffered (즉시 출력)
            self.voice_process = subprocess.Popen(
                ['python3', '-u', self.main_gpt_script],
                cwd=self.voice_order_path,
                env=os.environ.copy(),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1  # 라인 버퍼링
            )

            self.get_logger().info(f'✓ 프로세스 시작 (PID: {self.voice_process.pid})')

            # 별도 스레드에서 stdout 모니터링
            monitor_thread = threading.Thread(
                target=self.monitor_voice_process_output,
                daemon=True
            )
            monitor_thread.start()

        except Exception as e:
            self.get_logger().error(f'❌ 실행 실패: {e}')

    def monitor_voice_process_output(self):
        """subprocess의 stdout을 모니터링하고 주문 결과 감지"""

        if not self.voice_process or not self.voice_process.stdout:
            return

        try:
            for line in iter(self.voice_process.stdout.readline, ''):
                if not line:
                    break

                # 라인 출력 (로깅)
                line = line.rstrip()
                if line:
                    self.get_logger().info(f'[main_gpt] {line}')

                # 주문 결과 감지
                if '[VOICE_ORDER_RESULT]' in line:
                    # 결과 추출
                    order_csv = line.split('[VOICE_ORDER_RESULT]')[1].strip()
                    self.get_logger().info('')
                    self.get_logger().info('═══════════════════════════════════════')
                    self.get_logger().info(f'📦 주문 결과 감지: {order_csv}')
                    self.get_logger().info('═══════════════════════════════════════')

                    # ROS2 토픽 발행
                    self.publish_order(order_csv)

        except Exception as e:
            self.get_logger().error(f'❌ stdout 모니터링 에러: {e}')

    def publish_order(self, order_csv):
        """주문 정보를 ROS2 토픽으로 발행"""

        try:
            msg = String()
            msg.data = order_csv
            self.order_publisher.publish(msg)

            self.get_logger().info('')
            self.get_logger().info('✅ ROS2 토픽 발행 성공!')
            self.get_logger().info(f'📤 토픽: /dsr01/kiosk/order')
            self.get_logger().info(f'📦 데이터: {order_csv}')
            self.get_logger().info('═══════════════════════════════════════')
            self.get_logger().info('')

        except Exception as e:
            self.get_logger().error(f'❌ 토픽 발행 실패: {e}')


    def stop_voice_order(self):
        """프로세스 중지"""

        if self.voice_process and self.voice_process.poll() is None:
            self.get_logger().info('🛑 프로세스 종료 중...')
            self.voice_process.terminate()
            self.voice_process = None


def main(args=None):
    rclpy.init(args=args)
    node = VoiceOrderListener()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('🛑 Ctrl+C 감지')
        
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
