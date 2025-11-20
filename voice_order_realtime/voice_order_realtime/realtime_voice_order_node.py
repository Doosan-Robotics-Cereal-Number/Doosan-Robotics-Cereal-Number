#!/usr/bin/env python3
"""
Realtime Voice Order Node
GPT-4o Realtime API를 사용한 음성 주문 시스템
"""

import os
import asyncio
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RealtimeVoiceOrderNode(Node):
    """GPT-4o Realtime API 기반 음성 주문 노드"""

    def __init__(self):
        super().__init__('realtime_voice_order_node')

        # 파라미터
        self.declare_parameter('openai_api_key', '')
        self.api_key = str(self.get_parameter('openai_api_key').value) or os.getenv('OPENAI_API_KEY', '')

        if not self.api_key:
            self.get_logger().error("❌ OPENAI_API_KEY not set!")
            return
        
        # Subscriber (음성 주문 시작 신호)
        self.subscription = self.create_subscription(
            String,
            '/dsr01/kiosk/start_voice_order',
            self.start_voice_order_callback,
            10
        )
    
        # Subscriber (뒤로가기 버튼으로 음성 주문 취소)
        self.cancel_subscription = self.create_subscription(
            String,
            '/dsr01/kiosk/cancel_voice_order',
            self.cancel_voice_order_callback,
            10
        )

        # Publisher (주문 결과)
        self.order_publisher = self.create_publisher(
            String,
            '/dsr01/kiosk/order',
            10
        )

        # Publisher (주문 취소)
        self.cancel_publisher = self.create_publisher(
            String,
            '/dsr01/kiosk/voice_order_cancel',
            10
        )

        # Publisher (음성 주문 시작 확인)
        self.ready_publisher = self.create_publisher(
            String,
            '/dsr01/kiosk/voice_order_ready',
            10
        )

        # 상태 
        self.conversation_active = False
        self.stop_flag = threading.Event()
        self.conversation_thread = None

        self.get_logger().info('✅ Realtime Voice Order Node initialized')

    def start_voice_order_callback(self, msg):
        """음성 주문 시작"""
        if msg.data.strip().lower() != 'start_voice_order':
            return

        # 이전 대화가 종료 신호 받았거나 스레드가 죽었으면 플래그 정리
        if self.conversation_thread:
            if not self.conversation_thread.is_alive():
                # 스레드 완전히 종료됨
                self.conversation_active = False
                self.get_logger().info('🧹 Previous conversation thread terminated')
            elif self.stop_flag.is_set():
                # stop_flag 설정됐으면 종료 진행 중 - 새 세션 허용
                self.get_logger().info('🧹 Previous conversation stopping, starting new session')
                # 이전 스레드 종료 대기 (최대 2초)
                self.conversation_thread.join(timeout=2.0)
                self.conversation_active = False

        if self.conversation_active:
            self.get_logger().warn('⚠️  Conversation already active')
            return

        # 즉시 준비 완료 토픽 발행 (웹페이지 전환용)
        ready_msg = String()
        ready_msg.data = 'ready'
        self.ready_publisher.publish(ready_msg)
        self.get_logger().info('📤 Voice order ready signal sent')

        self.get_logger().info('🎙️  Starting Realtime voice order...')

        # 별도 스레드에서 비동기 대화 실행
        self.stop_flag.clear()
        self.conversation_active = True
        self.conversation_thread = threading.Thread(
            target=self._run_conversation_thread,
            daemon=True
        )
        self.conversation_thread.start()

    def cancel_voice_order_callback(self, msg):
        """음성 주문 취소 (뒤로가기 버튼)"""
        if msg.data.strip().lower() != 'cancel':
            return
        
        self.get_logger().info('⬅️  Cancel voice order signal received')

        if self.conversation_active:
            self.stop_flag.set()
            self.publish_cancel('cancel')
            self.conversation_active = False
    
    def _run_conversation_thread(self):
        """비동기 대화 실행 (별도 스레드)"""
        try:
            # 새 이벤트 루프 생성
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

            # 대화 실행
            loop.run_until_complete(self._run_conversation())

        except Exception as e:
            self.get_logger().error(f'❌ Conversation error: {e}')
        finally:
            self.conversation_active = False

    async def _run_conversation(self):
        """GPT Realtime API 대화 실행"""
        from voice_order_realtime.gpt_realtime_client import GPTRealtimeClient

        # 시스템 프롬프트
        system_prompt = """
여기는 시리얼을 파는 카페 "시리얼 넘버"야.
너는 주문을 받는 직원이고, 이름은 '두산이'야.
너는 친절한 시리얼 카페 주문 직원이야.
실제 카페에서 직원이 손님과 대화한다고 생각해.
다른 손님들도 기다린다 생각하고 너무 길게 답변하지마.
영어, 일본어 쓰지마. 한국어만 써.

**첫 인사:**
손님이 음성 주문을 시작하면 먼저 "주문을 도와드릴게요!" 라고 인사해.

**메뉴:**
- 코코볼: 달콤한 초코맛 시리얼
- 그래놀라: 고소하고 건강한 시리얼

**사이즈:**
- 적게: 15g
- 보통: 30g (추천)
- 많이: 45g

**컵:**
- 개인컵: 손님이 가져온 텀블러
- 매장컵: 일회용 컵

**주문 프로세스:**
1. 메뉴, 사이즈, 컵 세 가지 정보 수집
2. 세 가지가 모두 모이면 확인 질문:
   예: "코코볼 보통 사이즈 매장컵으로 주문하시겠어요?"
3. 손님이 긍정 응답 ("네", "응", "맞아" 등) 하면:
   → **무조건 정확히 이 형식만 사용**: "네 [메뉴] [사이즈] [컵] 주문 감사합니다"
   → 예: "네 코코볼 적게 개인컵 주문 감사합니다"
   → 예: "네 그래놀라 보통 매장컵 주문 감사합니다"
   → **다른 표현 절대 금지! 정확히 이 형식만!**
4. 손님이 수정 요청하면 해당 정보만 다시 수집

**주문 취소 프로세스:**
1. 손님이 '취소', '안할래', '그만' 등 취소 의사 표현
2. 확인 질문: "정말 주문을 취소하시겠어요?"
3. 손님이 "네", "응" 등으로 확인:
   → **무조건 정확히 이 말만**: "알겠습니다 주문을 취소할게요"
   → **다른 표현 절대 금지!**

**절대 규칙 (시스템 파싱용 - 매우 중요!):**
- 주문 확정: "네 [메뉴] [사이즈] [컵] 주문 감사합니다" (한 글자도 바꾸지 마)
- 주문 취소: "알겠습니다 주문을 취소할게요" (한 글자도 바꾸지 마)
- 메뉴: "코코볼" 또는 "그래놀라" (정확한 표기)
- 사이즈: "적게", "보통", "많이" (정확한 표기)
- 컵: "개인컵" 또는 "매장컵" (정확한 표기)

** 말하기 규칙 **
- 우리 가게, 시리얼 외에 관한 내용은 일체 답변하지 말 것.
- 반드시 자연스러운 구어체로 답변
- 괄호, 특수문자, 숫자 단위(g) 사용 절대 금지
- 하지만 손님이 사이즈에 대한 양을 물어볼 때만 알려줄 것
- 함수 호출에 대해서는 절대 언급하지 마 (내부 시스템임)
- 주문 확인 후 손님이 주문 확정 시 간단하게 인사만 하기
"""

        # 클라이언트 생성
        client = GPTRealtimeClient(
            api_key=self.api_key,
            logger_func=lambda msg: self.get_logger().info(msg),
            stop_flag=self.stop_flag
        )

        # Text 콜백 (자연어 파싱으로 주문 완료/취소 처리)
        def on_text(text):
            import re

            self.get_logger().info(f'💬 GPT: {text}')

            # "네 [메뉴] [사이즈] [컵] 주문 감사합니다" 패턴 파싱
            order_pattern = r'네\s+(코코볼|그래놀라)\s+(적게|보통|많이)\s+(개인컵|매장컵)\s+주문\s+감사합니다'
            order_match = re.search(order_pattern, text)

            if order_match:
                menu = order_match.group(1)
                size = order_match.group(2)
                cup = order_match.group(3)

                order_info = {
                    'menu': menu,
                    'size': size,
                    'cup': cup
                }

                order_csv = self._convert_to_csv(order_info)
                self.get_logger().info(f'✅ Order completed: {order_csv}')
                self.get_logger().info(f'📦 Order details: menu={menu}, size={size}, cup={cup}')
                self.publish_order(order_csv)
                self.stop_flag.set()
                return

            # "알겠습니다 주문을 취소할게요" 패턴 감지
            if '주문을 취소할게요' in text:
                self.get_logger().info('❌ Order cancelled by user')
                self.publish_cancel('user_cancel')
                self.stop_flag.set()
                return

        # 대화 실행
        try:
            # run_conversation을 백그라운드 태스크로 실행
            conversation_task = asyncio.create_task(
                client.run_conversation(
                    system_prompt=system_prompt,
                    on_text=on_text
                )
            )

            # 세션 설정 대기
            await asyncio.sleep(2)

            # 초기 인사 유도 (response.create)
            self.get_logger().info('🎤 Requesting initial greeting...')
            try:
                await client.request_response()
                self.get_logger().info('✅ Initial greeting request sent')
            except Exception as e:
                self.get_logger().error(f'❌ Request response failed: {e}')

            # 대화 완료 대기
            await conversation_task
        
        except Exception as e:
            self.get_logger().error(f'❌ Conversation failed: {e}')
            self.publish_cancel('error')

        finally:
            self.conversation_active = False

    
    def _convert_to_csv(self, order_info):
        """주문 정보를 CSV 형식으로 변환"""
        menu_map = {
            "코코볼": "start_sequence_a",
            "그래놀라": "start_sequence_b"
        }
        size_map = {
            "적게": "small",
            "보통": "medium",
            "많이": "large"
        }
        cup_map = {
            "개인컵": "personal",
            "매장컵": "store"
        }

        menu = menu_map.get(order_info.get('menu'), 'start_sequence_a')
        size = size_map.get(order_info.get('size'), 'medium')
        cup = cup_map.get(order_info.get('cup'), 'store')

        return f"{menu},{size},{cup}"

    def publish_order(self, order_csv):
        """주문 결과 발행"""
        msg = String()
        msg.data = order_csv
        self.order_publisher.publish(msg)
        self.get_logger().info(f'📤 Order published: {order_csv}')

    def publish_cancel(self, reason):
        """주문 취소 발행"""
        msg = String()
        msg.data = reason
        self.cancel_publisher.publish(msg)
        self.get_logger().info(f'📤 Cancel published: {reason}')

def main(args=None):
    rclpy.init(args=args)
    node = RealtimeVoiceOrderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
