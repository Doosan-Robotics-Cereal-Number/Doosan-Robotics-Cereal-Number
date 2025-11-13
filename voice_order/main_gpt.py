"""
GPT 대화형 음성 주문 시스템
- 음성으로 자연스럽게 대화하면서 주문
"""

import os
from audio import record_audio_vad, speak
from speech import SpeechRecognizer
from gpt_assistant import OrderAssistant


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

        while not order_complete:
            try:
                # 1. 녹음 (VAD - 자동 시작/종료, 10초 타임아웃)
                audio_file = record_audio_vad(timeout=10)

                # 타임아웃 체크
                if audio_file is None:
                    print("\n⏱️  주문 시간이 초과되었습니다.")
                    speak("주문 시간이 초과되었습니다. 처음부터 다시 시작해주세요.")
                    break  # 대화 루프 종료 → 다음 주문으로

            except KeyboardInterrupt:
                print("\n\n👋 시스템 종료")
                return

            try:
                # 2. 음성 인식
                user_text = recognizer.transcribe(audio_file)
                print(f"\n👤 손님: {user_text}")

                # 3. GPT 대화
                response_text, order_info = assistant.chat(user_text)
                print(f"🤖 직원: {response_text}")

                # 4. TTS 응답
                speak(response_text)

                # 5. 주문 완료 체크
                if order_info:
                    print("\n" + "="*60)
                    print("✅ 주문 접수 완료!")
                    print("="*60)
                    print(f"📋 주문 내역:")
                    print(f"   메뉴: {order_info.get('menu')}")
                    print(f"   양: {order_info.get('size')}")
                    print(f"   컵: {order_info.get('cup')}")
                    print("="*60)

                    # TODO: WebSocket으로 ROS에 전송
                    print(f"📤 주문 전송: {order_info}")

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

        # 다음 주문 대기
        assistant.reset()

        try:
            cont = input("\n다음 주문을 받으시겠습니까? (y/n): ")
            if cont.lower() != 'y':
                print("\n👋 시스템 종료")
                break
        except KeyboardInterrupt:
            print("\n\n👋 시스템 종료")
            break


if __name__ == "__main__":
    main()
