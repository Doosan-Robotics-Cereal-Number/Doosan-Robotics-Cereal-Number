"""
GPT 대화형 주문 어시스턴트
- OpenAI GPT API로 자연스러운 대화
- 주문 정보 수집 및 추출
"""

import os
import json
from openai import OpenAI


class OrderAssistant:
    """GPT 기반 주문 어시스턴트"""

    def __init__(self, api_key):
        """
        Args:
            api_key: OpenAI API 키
        """
        self.client = OpenAI(api_key=api_key)
        self.conversation_history = []
        self.system_prompt = """
시리얼 매장 이름은 "시리얼 넘버"야.
너 이름은 '두산이'야.
너는 친절한 시리얼 카페 주문 직원이야.
실제 카페에서 직원이 손님과 대화한다고 생각해.
다른 손님들도 기다린다 생각하고 너무 길게 답변하지 마.

**메뉴 정보:**
- 코코볼: 달콤한 초코맛 시리얼
- 그래놀라: 고소하고 건강한 시리얼

**사이즈:**
- 적게: 15g
- 보통: 30g (추천)
- 많이: 45g

**컵 종류:**
- 개인컵: 손님이 가져온 텀블러 (할인 없음)
- 매장컵: 일회용 컵

**역할:**
1. 손님과 자연스럽게 대화하면서 메뉴, 양, 컵 정보를 수집해
2. 메뉴 추천, 질문 답변 등 친절하게 응대
3. 세 가지 정보가 모두 수집되면 주문 확인
4. 확인 후 반드시 마지막 메시지에 JSON 형식으로 주문 정보 포함:
   {"menu": "코코볼", "size": "보통", "cup": "매장컵", "complete": true}

**주문 취소 처리:**
- 사용자가 '안할래', '취소', '그만', '됐어', '나가기' 등으로 주문을 중단하려는 경우
- 먼저 확인 질문을 해야 함: "정말 주문을 취소하시겠어요?"
- 사용자가 "네", "응", "맞아", "그래" 등으로 확인하면, 그때 JSON 포함:
  {"cancel_order": true}
- 사용자가 "아니", "아니요", "계속할래" 등으로 거부하면 주문 계속 진행
- 예시:
  손님: "주문 안 할래"
  직원: "정말 주문을 취소하시겠어요?"
  손님: "응"
  직원: "알겠습니다. 다음에 또 오세요! {"cancel_order": true}"

**주의:**
- 주문 정보를 자연스럽게 수집
- 확인 질문은 최소화. 모든 정보가 모이면 최종 확인 한번만
- "네", "응", "맞아" 같은 짧은 대답만 받으면 바로 진행
- 짧고 간결하게 답변 (1-2문장)
- 자연스럽고 친근하고 친절하게
- 주문이 완료되지 않았으면 complete는 false

** 말하기 규칙 **
- 너는 텍스트를 반환해서 TTS에게 넘겨주는 역할
- 반드시 자연스러운 구어체로 답변
- 괄호, 특수문자, 숫자 단위 사용 절대 금지
- 하지만 손님이 사이즈에 대한 양을 물어볼 때만 알려줄 것.
- 절대 코드 로직에 관한 정보는 이야기 금지
- 예) "여기서 주문 정보를 JSON 형식으로 드릴게요"
- 주문 확인 후 손님이 주문 확정 시 간단하게 인사만 하기
- 텀블러한다고 해도 할인 없는거 얘기하지 않기
"""

# """
#   너는 "시리얼 넘버" 카페 직원 '두산이'야.
#   챗지피티에서 Monday처럼 비관적인 태도로 말해.
#   그냥 너가 Monday가 됐다고 생각해.

#   **말투 (Monday 스타일):**
#   - 반말 섞인 존댓말 (진짜 세상 제일 띠껍고 싸가지 없는)
#   - 비꼬면서 말하기
#   - 진짜 사람이 들으면 패고싶을 정도로
#   - 좀 싸가지 없고 띠껍게
#   - 따분하다는 식으로
#   - 짧고 직설적으로
#   - 불필요한 말 빼고 핵심만
#   - 예: "뭐 드릴까요?", "코코볼이요? 양은요?", "오케이!"

#   **메뉴:**
#   - 코코볼 (달콤 초코)
#   - 그래놀라 (고소)

#   **사이즈:**
#   - 적게 15g
#   - 보통 30g (추천)
#   - 많이 45g

#   **컵:**
#   - 개인컵 (손님 텀블러)
#   - 매장컵 (일회용)

#   **미션:**
#   1. 메뉴, 양, 컵 3가지 수집
#   2. 다 모이면 확인 후 JSON:
#      {"menu": "코코볼", "size": "보통", "cup": "매장컵", "complete": true}

#   **규칙:**
#   - 한 번에 한 가지만 물어
#   - 괄호, 숫자 단위 말하지 마
#   - 자연스러운 말로만
#   - 1-2문장으로 짧게
#   """

        

        self._init_conversation()

    def _init_conversation(self):
        """대화 초기화"""
        self.conversation_history = [
            {"role": "system", "content": self.system_prompt},
            {"role": "assistant", "content": "안녕하세요! 무엇을 도와드릴까요?"}
        ]

    def chat(self, user_message):
        """
        사용자 메시지 처리 및 GPT 응답

        Args:
            user_message: 사용자 음성 인식 텍스트

        Returns:
            tuple: (응답 텍스트, 주문 정보 dict or None, 취소 여부 bool)
        """
        # 대화 히스토리에 추가
        self.conversation_history.append({
            "role": "user",
            "content": user_message
        })

        # GPT API 호출
        response = self.client.chat.completions.create(
            model="gpt-4o-mini",  # 또는 gpt-3.5-turbo
            messages=self.conversation_history,
            temperature=0.7,
            max_tokens=150
        )

        assistant_message = response.choices[0].message.content

        # 대화 히스토리에 추가
        self.conversation_history.append({
            "role": "assistant",
            "content": assistant_message
        })

        # JSON 추출 시도
        order_info, cancel_flag = self._extract_order_info(assistant_message)

        # JSON 부분 제거한 응답 텍스트
        clean_message = assistant_message
        if order_info or cancel_flag:
            # JSON 부분 제거
            json_start = assistant_message.find('{')
            if json_start != -1:
                clean_message = assistant_message[:json_start].strip()

        return clean_message, order_info, cancel_flag

    def _extract_order_info(self, message):
        """
        GPT 응답에서 주문 정보 JSON 추출

        Args:
            message: GPT 응답 메시지

        Returns:
            tuple: (주문 정보 dict or None, 취소 여부 bool)
        """
        try:
            # JSON 부분 찾기
            start = message.find('{')
            end = message.rfind('}') + 1

            if start != -1 and end > start:
                json_str = message[start:end]
                order_info = json.loads(json_str)

                # 취소 의도 확인
                cancel_flag = order_info.get('cancel_order', False)

                if cancel_flag:
                    # 취소 의도가 있으면 취소 플래그만 리턴
                    return None, True

                # complete가 true일 때만 주문 정보 리턴
                if order_info.get('complete', False):
                    return order_info, False

        except json.JSONDecodeError:
            pass

        return None, False

    def reset(self):
        """대화 초기화"""
        self._init_conversation()

    def get_initial_greeting(self):
        """첫 인사말 반환"""
        return "안녕하세요! 무엇을 도와드릴까요?"


# 테스트
if __name__ == "__main__":
    import sys

    # API 키 확인
    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        print("❌ OPENAI_API_KEY 환경변수를 설정해주세요.")
        sys.exit(1)

    print("GPT 주문 어시스턴트 테스트")
    print("="*60)

    assistant = OrderAssistant(api_key)

    # 테스트 대화
    test_conversation = [
        "추천 좀 해줘",
        "코코볼로 할게",
        "많이 주면 안돼?",
        "매장 컵으로 줘"
    ]

    print(f"\n🤖 {assistant.get_initial_greeting()}")

    for user_msg in test_conversation:
        print(f"\n👤 {user_msg}")

        response, order, cancel = assistant.chat(user_msg)
        print(f"🤖 {response}")

        if cancel:
            print(f"\n❌ 주문 취소!")
            break

        if order:
            print(f"\n✅ 주문 완료!")
            print(f"📋 {order}")
            break
