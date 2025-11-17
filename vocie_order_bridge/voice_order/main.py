"""
음성 주문 시스템 메인
- 녹음 → 인식 → 파싱 → 확인 → 완료
"""

import os
from audio import record_audio, speak
from speech import SpeechRecognizer
from order_parser import parse_order, validate_order, format_order_text


def main():
    """메인 음성 주문 프로세스"""

    print("="*60)
    print("🎙️  음성 주문 시스템")
    print("="*60)

    # 음성 인식기 초기화 (한번만)
    recognizer = SpeechRecognizer(model_name="base")

    while True:
        print("\n" + "="*60)
        try:
            input("Enter를 누르면 주문 시작 (종료: Ctrl+C): ")
        except KeyboardInterrupt:
            print("\n\n👋 시스템 종료")
            break

        # 1. 녹음
        audio_file = record_audio(duration=5)

        try:
            # 2. 음성 인식
            text = recognizer.transcribe(audio_file)

            # 3. 주문 파싱
            order = parse_order(text)
            print(f"\n📋 파싱 결과: {order}")

            # 4. 주문 검증 및 확인
            if validate_order(order):
                # 완전한 주문
                response = format_order_text(order)
                speak(response)

                # 최종 확인
                confirm = input("\n주문 확정하시겠습니까? (y/n): ")
                if confirm.lower() == 'y':
                    speak("주문이 접수되었습니다. 감사합니다!")
                    print("✅ 주문 완료!")

                    # TODO: 여기서 WebSocket으로 ROS에 전송
                    print(f"📤 주문 전송: {order}")
                else:
                    speak("주문이 취소되었습니다.")
                    print("❌ 주문 취소")

            else:
                # 불완전한 주문
                response = format_order_text(order)
                speak(response)
                print("⚠️ 주문 정보가 부족합니다. 다시 말씀해주세요.")

        except Exception as e:
            print(f"❌ 에러 발생: {e}")
            speak("죄송합니다. 다시 말씀해주세요.")

        finally:
            # 임시 파일 정리
            if os.path.exists(audio_file):
                os.remove(audio_file)


if __name__ == "__main__":
    main()
