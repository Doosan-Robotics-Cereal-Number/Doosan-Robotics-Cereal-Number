"""
GPT 대화형 음성 주문 시스템
- 음성으로 자연스럽게 대화하면서 주문
"""

import os
from audio import record_audio_vad, speak
from speech import SpeechRecognizer
from gpt_assistant import OrderAssistant


def convert_order_to_csv(order_info):
    """
    주문 정보를 CSV 형식으로 변환

    Args:
        order_info: {"menu": "코코볼", "size": "보통", "cup": "매장컵", "complete": true}

    Returns:
        str: "start_sequence_a,medium,store" 형식
    """
    # 메뉴 매핑
    menu_map = {
        "코코볼": "start_sequence_a",
        "그래놀라": "start_sequence_b"
    }

    # 사이즈 매핑
    size_map = {
        "적게": "small",
        "보통": "medium",
        "많이": "large"
    }

    # 컵 매핑
    cup_map = {
        "개인컵": "personal",
        "매장컵": "store"
    }

    menu = menu_map.get(order_info.get('menu'), 'start_sequence_a')
    size = size_map.get(order_info.get('size'), 'medium')
    cup = cup_map.get(order_info.get('cup'), 'store')

    return f"{menu},{size},{cup}"


def main():
    """메인 GPT 대화형 주문 프로세스"""

    print("="*60)
    print("🎙️  GPT 대화형 음성 주문 시스템")
    print("="*60)

    # API 키 확인
    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        print("\n❌ OPENAI_API_KEY 환경변수를 설정해주세요.")
        print("export OPENAI_API_KEY='your-api-key'")
        return

    # 초기화
    recognizer = SpeechRecognizer(model_name="base")
    assistant = OrderAssistant(api_key)

    print("\n✅ 시스템 준비 완료!")

    while True:
        print("\n" + "="*60)
        print("새로운 주문을 시작합니다")
        print("="*60)

        # 초기 인사
        greeting = assistant.get_initial_greeting()
        print(f"\n🤖 {greeting}")
        speak(greeting)

        # 대화 루프
        order_complete = False
        no_response_count = 0  # 무응답 카운터

        while not order_complete:
            try:
                # 1. 녹음 (VAD - 자동 시작/종료, 10초 타임아웃)
                audio_file = record_audio_vad(timeout=10)

                # 타임아웃 체크
                if audio_file is None:
                    print("\n⏱️  주문 시간이 초과되었습니다.")
                    speak("주문 시간이 초과되었습니다. 처음부터 다시 시작해주세요.")

                    # 취소 신호 출력 (voice_order_listener가 감지)
                    print("[VOICE_ORDER_CANCEL]timeout")
                    break  # 대화 루프 종료 → 다음 주문으로

            except KeyboardInterrupt:
                print("\n\n👋 시스템 종료")
                return

            try:
                # 2. 음성 인식
                user_text = recognizer.transcribe(audio_file)
                print(f"\n👤 손님: {user_text}")

                # 빈 응답 체크 (무응답 카운터)
                if not user_text or len(user_text.strip()) < 2:
                    no_response_count += 1
                    print(f"⚠️  무응답 ({no_response_count}/3)")

                    if no_response_count >= 3:
                        print("\n🔇 반응이 없어 주문을 종료합니다.")
                        speak("반응이 없어 주문을 종료합니다.")
                        print("[VOICE_ORDER_CANCEL]no_response")
                        break

                    speak("잘 안 들렸어요. 다시 말씀해주세요.")
                    continue
                else:
                    no_response_count = 0  # 정상 응답 시 카운터 리셋

                # 3. GPT 대화
                response_text, order_info, cancel_flag = assistant.chat(user_text)
                print(f"🤖 직원: {response_text}")

                # 4. TTS 응답
                speak(response_text)

                # 5. 취소 의도 체크
                if cancel_flag:
                    print("\n❌ 손님이 주문을 취소했습니다.")
                    # speak("알겠습니다. 주문을 취소하겠습니다.")
                    print("[VOICE_ORDER_CANCEL]user_cancel")
                    break

                # 6. 주문 완료 체크
                if order_info:
                    print("\n" + "="*60)
                    print("✅ 주문 접수 완료!")
                    print("="*60)
                    print(f"📋 주문 내역:")
                    print(f"   메뉴: {order_info.get('menu')}")
                    print(f"   양: {order_info.get('size')}")
                    print(f"   컵: {order_info.get('cup')}")
                    print("="*60)

                    # CSV 변환
                    order_csv = convert_order_to_csv(order_info)
                    print(f"📦 변환된 주문: {order_csv}")

                    # ROS2 토픽 발행을 위해 특수 포맷으로 출력
                    print(f"[VOICE_ORDER_RESULT]{order_csv}")

                    speak("주문이 접수되었습니다. 감사합니다!")

                    order_complete = True  # 플래그 설정
                    break  # 즉시 루프 탈출!

            except Exception as e:
                print(f"❌ 에러 발생: {e}")
                speak("죄송합니다. 다시 말씀해주세요.")

            finally:
                # 임시 파일 정리
                if os.path.exists(audio_file):
                    os.remove(audio_file)

        # 주문 완료/취소 후 종료 (더 이상 재시작하지 않음)
        print("\n👋 시스템 종료")
        break


if __name__ == "__main__":
    main()
