import whisper
import pyaudio
import wave
import os
from gtts import gTTS
from playsound import playsound

def record_audio(duration=5, filename="test.wav"):
    """마이크로 음성 녹음"""
    CHUNK = 1024    # 한번에 읽을 오디오 데이터 크기
    FORMAT = pyaudio.paInt16    # 오디오 포맷. 16인트 정수 (CD 품질)
    CHANNELS = 1    # 채널 수. 1 = 모노 (한쪽만), 2=스테레오 (양쪽)
    RATE = 16000    # 샘플링 레이트 (초당 16000번) 16kHz

    p = pyaudio.PyAudio()   # 오디오 다루는 라이브러리 객체 생성

    print(f"\n🎤 {duration}초간 녹음 시작...")
    print("   말해보세요: '아메리카노 한잔이요'")

    stream = p.open(
        format=FORMAT,
        channels=CHANNELS,
        rate=RATE,
        input=True,
        frames_per_buffer=CHUNK
    )   # 마이크 스트림 열기.

    frames = []     # 녹음된 데이터 저장할 빈 리스트

    # 5초 녹음 -> 약 78번 반복
    for i in range(0, int(RATE / CHUNK * duration)):    # 몇 번 읽어야 duration 초가 되는지 계산
        data = stream.read(CHUNK)   # 마이크에서 1024 바이트 읽기
        frames.append(data)         # 읽은 데이터 추가

    print("✅ 녹음 완료!")

    stream.stop_stream()
    stream.close()
    p.terminate()       # 마이크 스트림 정리 (닫기)

    # WAV 파일로 저장
    wf = wave.open(filename, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()

    return filename

def transcribe_audio(audio_file, model):
    """Whisper로 음성 인식"""
    print("\n🔄 음성 인식 중...")
    result = model.transcribe(audio_file, language="ko")
    return result['text']       # 결과는 딕셔너리

def speak_text(text):
    """TTS로 텍스트를 음성으로 출력"""
    print(f"\n🔊 음성 출력: '{text}'")
    tts = gTTS(text=text, lang='ko')
    tts.save("response.mp3")
    playsound("response.mp3")
    os.remove("response.mp3")

if __name__ == "__main__":
    print("="*50)
    print("Whisper 음성인식 테스트")
    print("="*50)

    # Whisper 모델 로드 (처음 실행 시 다운로드됨)
    print("\n📥 Whisper 모델 로딩 중... (처음엔 다운로드 시간 좀 걸려요)")
    model = whisper.load_model("base")  # tiny, base, small 중 선택
    print("✅ 모델 로드 완료!")

    while True:
        print("\n" + "="*50)
        input("Enter를 누르면 녹음 시작 (종료: Ctrl+C): ")

        # 녹음
        audio_file = record_audio(duration=5)

        # 인식
        text = transcribe_audio(audio_file, model)

        print(f"\n✅ 인식 결과: '{text}'")

        # TTS로 따라 말하기
        speak_text(text)

        # 파일 삭제
        os.remove(audio_file)
