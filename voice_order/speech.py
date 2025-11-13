"""
음성 인식 모듈
- Whisper 모델 로드
- 음성 → 텍스트 변환
"""

import whisper


class SpeechRecognizer:
    """음성 인식기"""

    def __init__(self, model_name="base"):
        """
        Args:
            model_name: Whisper 모델 (tiny, base, small, medium, large)
        """
        print(f"📥 Whisper '{model_name}' 모델 로딩 중...")
        self.model = whisper.load_model(model_name)
        print("✅ 모델 로드 완료!")

    def transcribe(self, audio_file):
        """
        음성 파일을 텍스트로 변환

        Args:
            audio_file: 오디오 파일 경로

        Returns:
            str: 인식된 텍스트
        """
        print("🔄 음성 인식 중...")
        result = self.model.transcribe(audio_file, language="ko")
        text = result['text'].strip()
        print(f"✅ 인식 완료: '{text}'")
        return text


# 테스트
if __name__ == "__main__":
    import os

    print("음성 인식 모듈 테스트")

    # 테스트 오디오 파일이 있다면
    test_files = ["test.wav", "temp_record.wav"]

    recognizer = SpeechRecognizer(model_name="base")

    for file in test_files:
        if os.path.exists(file):
            print(f"\n테스트 파일: {file}")
            text = recognizer.transcribe(file)
            print(f"결과: {text}")
            break
    else:
        print("\n테스트할 오디오 파일이 없습니다.")
        print("audio.py로 먼저 녹음하세요.")
