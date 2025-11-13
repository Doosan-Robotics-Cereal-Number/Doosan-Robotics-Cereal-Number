"""
오디오 입출력 모듈
- 마이크 녹음
- TTS 음성 출력
"""

import pyaudio
import wave
import os
import webrtcvad
import collections
from gtts import gTTS
from playsound import playsound


def record_audio(duration=5, filename="temp_record.wav"):
    """
    마이크로 음성 녹음

    Args:
        duration: 녹음 시간 (초)
        filename: 저장할 파일명

    Returns:
        str: 저장된 파일 경로
    """
    CHUNK = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 16000

    p = pyaudio.PyAudio()

    print(f"\n🎤 {duration}초간 녹음 시작...")

    stream = p.open(
        format=FORMAT,
        channels=CHANNELS,
        rate=RATE,
        input=True,
        frames_per_buffer=CHUNK
    )

    frames = []

    for i in range(0, int(RATE / CHUNK * duration)):
        data = stream.read(CHUNK)
        frames.append(data)

    print("✅ 녹음 완료!")

    stream.stop_stream()
    stream.close()
    p.terminate()

    # WAV 파일로 저장
    wf = wave.open(filename, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()

    return filename


"""
VAD (Voice Activity Detection)
- 20ms (0.02초)마다 마이크 데이터 체크
- 사람이 말하고 있나 판단
- 침묵 1.5초 지속 -> 녹음 종료 
0"""
def record_audio_vad(filename="temp_record.wav", silence_duration=1.5, timeout=10):
    """
    VAD를 사용한 동적 음성 녹음
    사람이 말을 끝내면 (침묵 감지) 자동으로 녹음 종료

    Args:
        filename: 저장할 파일명
        silence_duration: 침묵으로 판단할 시간 (초)
        timeout: 음성 시작 대기 시간 (초), 이 시간 안에 말 안 하면 None 리턴

    Returns:
        str: 저장된 파일 경로, 타임아웃 시 None
    """
    # === 설정 ===
    RATE = 16000  # 샘플링 레이트 (VAD는 8000, 16000, 32000만 지원)
    CHUNK = 320   # 20ms 분량 (16000 * 0.02 = 320)
    FORMAT = pyaudio.paInt16
    CHANNELS = 1

    # === VAD 초기화 ===
    # mode: 0(가장 관대)~3(가장 엄격)
    # 3 = 확실한 음성만 인식 (배경 소음 무시)
    vad = webrtcvad.Vad(3)

    # === PyAudio 초기화 ===
    p = pyaudio.PyAudio()
    stream = p.open(
        format=FORMAT,
        channels=CHANNELS,
        rate=RATE,
        input=True,
        frames_per_buffer=CHUNK
    )

    print(f"\n🎤 음성을 감지하면 녹음이 시작됩니다... (타임아웃: {timeout}초)")

    frames = []  # 녹음 데이터 저장
    speech_detected = False  # 음성이 시작됐는지
    silence_chunks = 0  # 연속 침묵 프레임 카운트
    silence_threshold = int(silence_duration * RATE / CHUNK)  # 침묵으로 판단할 프레임 수

    # 타임아웃 관련
    timeout_chunks = 0  # 음성 시작 전 대기 시간 카운터
    timeout_threshold = int(timeout * RATE / CHUNK)  # 타임아웃 프레임 수 (10초 = 500 프레임)

    try:
        while True:
            # === 마이크에서 20ms 데이터 읽기 ===
            data = stream.read(CHUNK, exception_on_overflow=False)

            # === VAD로 음성 감지 ===
            # True = 음성 있음, False = 침묵
            is_speech = vad.is_speech(data, RATE)

            if is_speech:
                # 음성 감지됨!
                if not speech_detected:
                    print("🗣️  녹음 시작! (말씀하세요)")
                    speech_detected = True

                frames.append(data)  # 데이터 저장
                silence_chunks = 0   # 침묵 카운터 리셋

            else:
                # 침묵 감지됨
                if speech_detected:
                    # 음성이 시작된 이후의 침묵
                    frames.append(data)  # 침묵도 일단 저장 (자연스러운 끊김 위해)
                    silence_chunks += 1  # 침묵 카운터 증가

                    # 일정 시간 이상 침묵이면 종료
                    if silence_chunks > silence_threshold:
                        print("✅ 녹음 완료! (침묵 감지)")
                        break

                else:
                    # 음성이 아직 시작 안 됨 (대기 중)
                    timeout_chunks += 1  # 타임아웃 카운터 증가

                    # 타임아웃 체크
                    if timeout_chunks > timeout_threshold:
                        print(f"⏱️  타임아웃! ({timeout}초 동안 음성 없음)")
                        stream.stop_stream()
                        stream.close()
                        p.terminate()
                        return None  # None 리턴 = 타임아웃

    except KeyboardInterrupt:
        print("\n⚠️  녹음 중단")

    finally:
        # === 정리 ===
        stream.stop_stream()
        stream.close()
        p.terminate()

    # === WAV 파일로 저장 ===
    if frames:
        wf = wave.open(filename, 'wb')
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames))
        wf.close()
        print(f"💾 파일 저장: {filename}")
        return filename
    else:
        print("❌ 녹음된 음성이 없습니다")
        return None


def speak(text, filename="temp_response.mp3"):
    """
    TTS로 텍스트를 음성으로 출력

    Args:
        text: 말할 텍스트
        filename: 임시 음성 파일명
    """
    print(f"\n🔊 음성 출력: '{text}'")

    try:
        tts = gTTS(text=text, lang='ko')
        tts.save(filename)
        playsound(filename)
        os.remove(filename)
    except Exception as e:
        print(f"❌ TTS 에러: {e}")


# 테스트
if __name__ == "__main__":
    print("="*60)
    print("오디오 모듈 VAD 테스트")
    print("="*60)

    # VAD 녹음 테스트
    print("\n말씀하세요! (말 끝나면 1.5초 후 자동 종료)")
    audio_file = record_audio_vad()

    if audio_file:
        print(f"녹음 파일: {audio_file}")

        # TTS 테스트
        speak("녹음이 완료되었습니다.")

        # 정리
        if os.path.exists(audio_file):
            os.remove(audio_file)
    else:
        print("녹음 실패")
