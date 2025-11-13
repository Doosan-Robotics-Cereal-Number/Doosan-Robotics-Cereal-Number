# 🎙️ Voice Order System

GPT 기반 음성 주문 시스템 - 시리얼 카페용

## 📋 기능

- **음성 인식**: OpenAI Whisper 기반 STT
- **대화형 AI**: GPT-4o-mini를 활용한 자연스러운 주문 대화
- **음성 출력**: gTTS 기반 TTS
- **VAD**: 음성 감지 및 자동 녹음 시작/종료
- **타임아웃**: 10초 무응답 시 자동 종료

## 🏗️ 구조

```
voice_order/
├── audio.py          # 오디오 입출력 (녹음, TTS)
├── speech.py         # 음성 인식 (Whisper)
├── gpt_assistant.py  # GPT 대화 관리
├── order_parser.py   # 주문 파싱 (미사용)
└── main_gpt.py       # 메인 실행 파일
```

## 🚀 설치

### 1. 저장소 클론

```bash
git clone https://github.com/Doosan-Robotics-Cereal-Number/Doosan-Robotics-Cereal-Number.git
cd Doosan-Robotics-Cereal-Number/voice_order
```

### 2. 시스템 패키지 설치

```bash
# Ubuntu/Debian (라즈베리파이 포함)
sudo apt-get update
sudo apt-get install portaudio19-dev python3-pyaudio ffmpeg

# macOS
brew install portaudio ffmpeg
```

### 3. Python 패키지 설치

```bash
pip install -r requirements.txt
```

**설치되는 패키지:**
- `openai-whisper` - OpenAI Whisper 음성 인식
- `openai` - OpenAI GPT API 클라이언트
- `gtts` - Google Text-to-Speech
- `playsound` - 오디오 재생
- `pyaudio` - 마이크 입력
- `webrtcvad` - 음성 감지 (VAD)
- `numpy` - 수치 연산

## ⚙️ 설정

### OpenAI API 키 설정

```bash
export OPENAI_API_KEY='your-api-key-here'
```

## 🎯 사용법

### 메인 시스템 실행

```bash
cd voice_order
python3 main_gpt.py
```

### 모듈별 테스트

```bash
# 오디오 (녹음/TTS) 테스트
python3 audio.py

# 음성 인식 테스트
python3 speech.py

# GPT 대화 테스트
python3 gpt_assistant.py
```

## 📝 주문 흐름

1. 시스템 시작 → AI 인사
2. 음성 감지 시 자동 녹음 시작
3. 말 끝나면 (1.5초 침묵) 자동 종료
4. Whisper로 음성 인식
5. GPT가 대화로 주문 정보 수집
   - 메뉴: 코코볼 / 그래놀라
   - 양: 적게 / 보통 / 많이
   - 컵: 개인컵 / 매장컵
6. 주문 완료 → JSON 출력

## 🛠️ 설정 변경

### GPT 모델 변경

`gpt_assistant.py` 77줄:
```python
model="gpt-4o-mini"  # 또는 gpt-4o, gpt-3.5-turbo
```

### VAD 민감도 조절

`audio.py` 88줄:
```python
vad = webrtcvad.Vad(3)  # 0(관대) ~ 3(엄격)
```

### 타임아웃 시간 조절

`main_gpt.py` 47줄:
```python
audio_file = record_audio_vad(timeout=10)  # 초 단위
```

### 침묵 감지 시간 조절

`audio.py` 73줄:
```python
record_audio_vad(silence_duration=1.5)  # 초 단위
```

## 🐛 문제 해결

### pyaudio 설치 오류
```bash
sudo apt-get install portaudio19-dev python3-pyaudio
```

### ffmpeg 오류
```bash
sudo apt-get install ffmpeg
```

### 마이크 인식 안됨
```bash
# 마이크 목록 확인
arecord -l
```

## 📦 의존성

- Python 3.8+
- OpenAI API 키
- 마이크 (USB 또는 내장)
- 인터넷 연결 (GPT, gTTS)

## 📄 라이선스

MIT License

## 👥 기여

이슈 및 PR 환영합니다!
