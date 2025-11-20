#!/usr/bin/env python3
"""
GPT-4 Realtime API WebSocket Client
음성 입출력을 처리하는 클라이언트
"""

import asyncio
import base64
import json
import logging
import threading
import queue
from typing import Optional, Callable, Dict, Any
from dataclasses import dataclass

import websockets
import pyaudio

logger = logging.getLogger(__name__)


@dataclass
class AudioConfig:
    """오디오 설정"""
    sample_rate: int = 24000  # GPT Realtime은 24kHz 고정
    channels: int = 1
    chunk_size: int = 4096  # 약 170ms @ 24kHz


class GPTRealtimeClient:
    """GPT-4 Realtime WebSocket 클라이언트"""

    def __init__(
        self,
        api_key: str,
        logger_func: Optional[Callable[[str], None]] = None,
        stop_flag: Optional[threading.Event] = None,
    ):
        """
        초기화

        Args:
            api_key: OpenAI API 키
            logger_func: 로깅 함수 (ROS2 get_logger() 등)
            stop_flag: 외부에서 대화를 종료할 수 있는 이벤트 플래그
        """
        self.api_key = api_key
        self.logger = logger_func or logger.info
        self.websocket = None
        self.connected = False

        # 오디오 설정
        self.audio_config = AudioConfig()

        # 큐 (메인 스레드와 비동기 스레드 간 통신)
        self.input_queue = queue.Queue()   # 마이크 입력 → WebSocket
        self.output_queue = queue.Queue()  # WebSocket → 스피커 출력
        self.event_queue = queue.Queue()   # 이벤트 콜백

        # 스레드
        self.receive_thread = None
        self.audio_input_thread = None
        self.audio_output_thread = None

        # 오디오 스트림
        self.audio_input_stream = None
        self.audio_output_stream = None
        self.pyaudio_instance = None

        # 상태
        self.stop_event = threading.Event()
        self.external_stop_flag = stop_flag  # 외부에서 대화 중단하는 플래그
        self.session_id = None
        self.accept_audio_input = False  # 마이크 입력 수락 여부 (기본값: 비활성화, 인사말 완료 후 활성화)
        self.total_audio_size = 0        # 전체 오디오 크기 합계 (재생 시간 계산용)
        self.audio_start_time = None     # 오디오 재생 시작 시간 (정확한 대기 시간 계산용)

        # 텍스트 누적 버퍼
        self._accumulated_text = ""

    async def connect(self, model: str = "gpt-realtime-mini") -> bool:
        """
        GPT Realtime WebSocket에 연결

        Args:
            model: 모델명

        Returns:
            연결 성공 여부
        """
        try:
            url = f"wss://api.openai.com/v1/realtime?model={model}"
            self.logger("🔗 Connecting to GPT Realtime API...")

            headers = {
                "Authorization": f"Bearer {self.api_key}",
                "OpenAI-Beta": "realtime=v1",
            }

            # websockets 버전에 따라 다른 인자 사용
            try:
                self.websocket = await websockets.connect(
                    url,
                    subprotocols=["realtime"],
                    additional_headers=headers,
                )
            except TypeError:
                try:
                    self.websocket = await websockets.connect(
                        url,
                        subprotocols=["realtime"],
                        extra_headers=headers,
                    )
                except TypeError:
                    self.logger("⚠️ Using basic connection (headers may not be included)...")
                    self.websocket = await websockets.connect(
                        url,
                        subprotocols=["realtime"],
                    )

            self.connected = True
            self.logger("✅ Connected to GPT Realtime API")
            return True

        except Exception as e:
            self.logger(f"❌ Connection error: {e}")
            import traceback
            traceback.print_exc()
            return False

    async def setup_session(self, system_prompt: str = "") -> bool:
        """
        세션 설정

        Args:
            system_prompt: 시스템 프롬프트

        Returns:
            설정 성공 여부
        """
        try:
            if not self.websocket:
                self.logger("❌ WebSocket not connected")
                return False

            session_config = {
                "type": "session.update",
                "session": {
                    "modalities": ["text", "audio"],
                    "instructions": system_prompt
                    or """당신은 친절한 카페 어시스턴트입니다.
고객과 자연스럽고 따뜻한 대화를 나눕니다.
고객의 질문에 답하고, 메뉴를 추천합니다.
한국어로 대화합니다.""",
                    "voice": "alloy",
                    "input_audio_format": "pcm16",
                    "output_audio_format": "pcm16",
                    "temperature": 0.8,
                    "max_response_output_tokens": 2000,
                },
            }

            await self.websocket.send(json.dumps(session_config))
            self.logger("✅ Session configured")
            return True

        except Exception as e:
            self.logger(f"❌ Session setup error: {e}")
            return False

    async def send_audio_chunk(self, audio_data: bytes):
        """오디오 청크 전송"""
        try:
            if not self.websocket:
                return

            audio_base64 = base64.b64encode(audio_data).decode("utf-8")
            event = {
                "type": "input_audio_buffer.append",
                "audio": audio_base64,
            }
            await self.websocket.send(json.dumps(event))

        except Exception as e:
            self.logger(f"❌ Send audio error: {e}")

    async def commit_audio_buffer(self):
        """오디오 버퍼 커밋 (음성 입력 완료)"""
        try:
            if not self.websocket:
                return

            event = {"type": "input_audio_buffer.commit"}
            await self.websocket.send(json.dumps(event))

        except Exception as e:
            self.logger(f"❌ Commit audio error: {e}")

    async def request_response(self):
        """응답 생성 요청 (초기 인사용)"""
        try:
            if not self.websocket:
                return

            event = {"type": "response.create"}
            await self.websocket.send(json.dumps(event))
            self.logger("🎤 Response generation requested")

        except Exception as e:
            self.logger(f"❌ Request response error: {e}")

    async def send_image(self, image_data: bytes, image_format: str = "jpeg") -> bool:
        """
        이미지 전송 (비전 모달리티) + 응답 요청

        Args:
            image_data: 이미지 바이너리 데이터
            image_format: 이미지 포맷 (jpeg, png)

        Returns:
            전송 성공 여부
        """
        try:
            if not self.websocket:
                return False

            image_base64 = base64.b64encode(image_data).decode("utf-8")
            data_url = f"data:image/{image_format};base64,{image_base64}"

            # 사용자 메시지로 이미지 + 프롬프트 전달
            event = {
                "type": "conversation.item.create",
                "item": {
                    "type": "message",
                    "role": "user",
                    "content": [
                        {
                            "type": "input_text",
                            "text": (
                                "새로운 손님이 키오스크 앞에 서 있습니다. "
                                "당신은 CocoSkool 무인 시리얼 카페 직원 '두산이'입니다. "
                                "이미지나 카메라, 사진 같은 말은 절대 하지 말고, "
                                "바로 앞에 있는 손님에게 말하듯 자연스럽게 인사해 주세요. "
                                "시간대에 맞춰 코코볼 또는 그라놀라를 1~2문장 안에서 추천하고, "
                                "궁금한 점이 있으면 두산이를 불러달라고 안내해 주세요."
                            ),
                        },
                        {
                            "type": "input_image",
                            "image_url": data_url,
                        },
                    ],
                },
            }

            await self.websocket.send(json.dumps(event))
            self.logger("📷 Image sent to Realtime API")

            # 응답 생성 요청
            response_create_event = {
                "type": "response.create"
            }
            await self.websocket.send(json.dumps(response_create_event))
            self.logger("🎙️ Requesting response generation...")

            return True

        except Exception as e:
            self.logger(f"❌ Send image error: {e}")
            return False

    async def receive_events(self):
        """WebSocket에서 이벤트 수신 (비동기)"""
        try:
            async for message in self.websocket:
                try:
                    event = json.loads(message)
                    await self._handle_event(event)
                except json.JSONDecodeError:
                    # 바이너리 데이터 (오디오)일 가능성 → 출력 큐로 보냄
                    self.output_queue.put(message)

        except Exception as e:
            self.logger(f"❌ Receive error: {e}")
            self.connected = False

    async def _handle_event(self, event: Dict[str, Any]):
        """이벤트 처리"""
        event_type = event.get("type", "")

        # Function call 관련 이벤트 디버깅
        if "function" in event_type or "tool" in event_type:
            self.logger(f"🔧 Function event: {event_type}, data: {event}")
        # self.logger(f"📌 Event received: {event_type}")

        # 세션 관련
        if event_type == "session.created":
            self.session_id = event.get("session", {}).get("id")
            self.logger(f"📌 Session created: {self.session_id}")

        elif event_type == "session.updated":
            self.logger("📌 Session updated")

        # 오디오 스트림 응답
        elif event_type in ("response.audio.delta", "response.output_audio.delta"):
            delta = event.get("delta", {})
            audio_b64 = ""

            if isinstance(delta, dict):
                audio_b64 = delta.get("audio", "")
            elif isinstance(delta, str):
                audio_b64 = delta  # 혹시 옛 포맷 대응

            if audio_b64:
                try:
                    audio_bytes = base64.b64decode(audio_b64)
                    chunk_size = len(audio_bytes)

                    # 첫 오디오 수신 시 시작 시간 기록
                    if self.audio_start_time is None:
                        import time
                        self.audio_start_time = time.time()

                    self.total_audio_size += chunk_size
                    # self.logger(
                    #     f"🔊 Audio delta: {chunk_size} bytes (total: {self.total_audio_size} bytes)"
                    # )
                    self.output_queue.put(audio_bytes)
                except Exception as e:
                    self.logger(f"⚠️ Audio decode error: {e}")

        # 텍스트 스트림 응답 (오디오 트랜스크립트 포함)
        elif event_type in ("response.output_text.delta", "response.text.delta", "response.audio_transcript.delta"):
            # audio_transcript.delta는 delta 필드 사용
            if event_type == "response.audio_transcript.delta":
                text_delta = event.get("delta", "")
            else:
                delta = event.get("delta", {})
                text_delta = ""

                # 최신 포맷: delta = { "content": [ { "type": "output_text", "text": "..." }, ... ] }
                if isinstance(delta, dict) and "content" in delta:
                    for item in delta["content"]:
                        if item.get("type") in ("output_text", "output_text.delta"):
                            text_delta += item.get("text", "")
                elif isinstance(delta, str):
                    # 옛 포맷 혹은 단순 문자열
                    text_delta = delta

            if text_delta:
                self._accumulated_text += text_delta
                # event_queue에 넣어서 on_text 콜백에서 처리 (로그는 거기서 한 번만)
                self.event_queue.put({"type": "text", "content": text_delta})

        # 오디오 스트림 완료
        elif event_type == "response.audio.done":
            if self.total_audio_size > 0:
                import time
                playback_time = self.total_audio_size / 2 / self.audio_config.sample_rate

                # 경과 시간 계산
                if self.audio_start_time:
                    elapsed = time.time() - self.audio_start_time
                    remaining = max(playback_time - elapsed, 0.5)  # 최소 0.5초
                    self.logger(
                        f"✅ Audio done (total: {playback_time:.2f}s, elapsed: {elapsed:.2f}s, remaining: {remaining:.2f}s)"
                    )
                else:
                    remaining = playback_time
                    self.logger(f"✅ Audio stream done (total: {playback_time:.2f}s)")

                playback_time = remaining
            else:
                playback_time = 0.5
                self.logger("✅ Audio stream done (no audio data)")

            self.event_queue.put({"type": "audio_done", "playback_time": playback_time})
            self.total_audio_size = 0
            self.audio_start_time = None  # 리셋

        # 응답 전체 완료
        elif event_type == "response.done":
            # 빈 응답 체크 (텍스트도 없고 오디오도 없음)
            if not self._accumulated_text and self.total_audio_size == 0:
                self.logger("⚠️ Empty response detected - requesting retry")
                try:
                    # 다시 응답 요청
                    retry_event = {"type": "response.create"}
                    await self.websocket.send(json.dumps(retry_event))
                    self.logger("🔄 Retry request sent")
                except Exception as e:
                    self.logger(f"❌ Retry failed: {e}")
                return

            # 정상 응답 처리
            if self._accumulated_text:
                self.logger(f"✅ Response complete: '{self._accumulated_text}'")
                # 전체 텍스트를 event_queue에 넣어서 JSON 파싱 가능하도록
                self.event_queue.put({"type": "text_complete", "content": self._accumulated_text})
                self._accumulated_text = ""
            else:
                self.logger("⚠️ Response complete (audio only, no text)")

            self.event_queue.put({"type": "response_done"})

        # 음성 입력 시작/종료
        elif event_type == "input_audio_buffer.speech_started":
            self.event_queue.put({"type": "speech_started"})
            self.logger("🎤 Speech started (Customer speaking)")

        elif event_type == "input_audio_buffer.speech_stopped":
            self.accept_audio_input = False
            self.event_queue.put({"type": "speech_stopped"})
            self.logger("🛑 Speech stopped (Microphone paused for GPT response)")

        # Function Call 처리
        elif event_type == "response.function_call_arguments.done":
            call_id = event.get("call_id", "")
            item_id = event.get("item_id", "")
            name = event.get("name", "")
            arguments_str = event.get("arguments", "{}")

            try:
                arguments = json.loads(arguments_str)
                self.logger(f"📞 Function call: {name}({arguments})")

                if name == "complete_order":
                    self.event_queue.put({
                        "type": "function_call",
                        "function": "complete_order",
                        "arguments": arguments
                    })
                elif name == "cancel_order":
                    self.event_queue.put({
                        "type": "function_call",
                        "function": "cancel_order",
                        "arguments": {}
                    })
            except json.JSONDecodeError as e:
                self.logger(f"⚠️ Function arguments parse error: {e}")

        # 에러
        elif event_type == "error":
            error_msg = event.get("error", {}).get("message", "Unknown error")
            self.logger(f"❌ API error: {error_msg}")

    def start_audio_io(self):
        """오디오 입출력 시작"""
        try:
            self.pyaudio_instance = pyaudio.PyAudio()

            # 입력 스트림 (마이크)
            self.audio_input_stream = self.pyaudio_instance.open(
                format=pyaudio.paInt16,
                channels=self.audio_config.channels,
                rate=self.audio_config.sample_rate,
                input=True,
                frames_per_buffer=self.audio_config.chunk_size,
            )

            # 출력 스트림 (스피커)
            self.audio_output_stream = self.pyaudio_instance.open(
                format=pyaudio.paInt16,
                channels=self.audio_config.channels,
                rate=self.audio_config.sample_rate,
                output=True,
                frames_per_buffer=self.audio_config.chunk_size,
            )

            self.logger("✅ Audio I/O initialized")

            # 스레드 시작
            self.audio_input_thread = threading.Thread(
                target=self._audio_input_loop,
                daemon=True,
            )
            self.audio_input_thread.start()

            self.audio_output_thread = threading.Thread(
                target=self._audio_output_loop,
                daemon=True,
            )
            self.audio_output_thread.start()

        except Exception as e:
            self.logger(f"❌ Audio I/O error: {e}")

    def _audio_input_loop(self):
        """마이크 입력 루프 (accept_audio_input이 True일 때만 전송)"""
        try:
            while not self.stop_event.is_set():
                try:
                    frame = self.audio_input_stream.read(
                        self.audio_config.chunk_size,
                        exception_on_overflow=False,
                    )
                    if self.accept_audio_input:
                        self.input_queue.put(frame)
                except Exception as e:
                    self.logger(f"⚠️ Audio input error: {e}")
                    break

        except Exception as e:
            self.logger(f"❌ Audio input loop error: {e}")

    def _audio_output_loop(self):
        """스피커 출력 루프"""
        error_count = 0
        last_error_msg = None

        try:
            while not self.stop_event.is_set():
                # 외부 stop_flag 체크 - 즉시 오디오 중단
                if self.external_stop_flag and self.external_stop_flag.is_set():
                    self.logger("🛑 External stop - aborting audio playback")
                    try:
                        # 출력 큐 비우기
                        while not self.output_queue.empty():
                            try:
                                self.output_queue.get_nowait()
                            except queue.Empty:
                                break
                        # 오디오 스트림 즉시 중단
                        if self.audio_output_stream and self.audio_output_stream.is_active():
                            self.audio_output_stream.stop_stream()
                    except Exception as e:
                        self.logger(f"⚠️ Audio abort error: {e}")
                    break

                try:
                    audio_data = self.output_queue.get(timeout=0.1)
                    if isinstance(audio_data, bytes) and len(audio_data) > 0:
                        try:
                            self.audio_output_stream.write(audio_data, exception_on_underflow=False)
                        except Exception as write_error:
                            # PyAudio write 에러는 무시하고 계속 진행
                            error_msg = str(write_error)
                            if error_msg != last_error_msg:
                                self.logger(f"⚠️ Audio output write error (suppressing further): {write_error}")
                                last_error_msg = error_msg
                            error_count += 1

                except queue.Empty:
                    continue
                except Exception as e:
                    self.logger(f"⚠️ Audio output error: {e}")
                    continue
        except Exception as e:
            self.logger(f"❌ Audio output loop error: {e}")

    def stop_audio_io(self):
        """오디오 입출력 중지"""
        try:
            self.stop_event.set()

            if self.audio_input_stream:
                self.audio_input_stream.stop_stream()
                self.audio_input_stream.close()

            if self.audio_output_stream:
                self.audio_output_stream.stop_stream()
                self.audio_output_stream.close()

            if self.pyaudio_instance:
                self.pyaudio_instance.terminate()

            self.logger("✅ Audio I/O stopped")

        except Exception as e:
            self.logger(f"❌ Stop audio I/O error: {e}")

    async def close(self):
        """WebSocket 연결 종료"""
        try:
            self.stop_audio_io()

            if self.websocket:
                await self.websocket.close()
                self.connected = False
                self.logger("✅ WebSocket closed")

        except Exception as e:
            self.logger(f"❌ Close error: {e}")

    async def run_conversation(
        self,
        system_prompt: str = "",
        on_text: Optional[Callable[[str], None]] = None,
        on_audio: Optional[Callable[[bytes], None]] = None,
        on_function_call: Optional[Callable[[str, dict], None]] = None,
        image_data: Optional[bytes] = None,
        image_format: str = "jpeg",
    ):
        """
        대화 실행 (음성 입출력 + 선택적 이미지)

        순서:
        1. 연결 및 세션 설정
        2. 이미지 전송 (선택)
        3. 오디오 I/O 시작
        4. GPT의 응답 스트림 처리
        5. 고객 음성 입력 처리
        """
        try:
            # 연결
            if not await self.connect():
                return

            # 세션 설정
            if not await self.setup_session(system_prompt):
                return

            # 이미지 전송
            if image_data:
                await self.send_image(image_data, image_format)
                self.logger("⏳ Waiting for initial greeting response...")

            # 오디오 I/O 시작
            self.start_audio_io()

            # 수신 태스크
            receive_task = asyncio.create_task(self.receive_events())

            # 오디오 전송 태스크
            async def send_audio():
                while not self.stop_event.is_set():
                    if self.external_stop_flag and self.external_stop_flag.is_set():
                        self.logger("🛑 External stop flag detected - stopping audio send")
                        break
                    try:
                        audio_data = self.input_queue.get_nowait()
                        await self.send_audio_chunk(audio_data)
                    except queue.Empty:
                        await asyncio.sleep(0.01)
                    except Exception as e:
                        self.logger(f"⚠️ Send audio task error: {e}")

            send_task = asyncio.create_task(send_audio())

            # 이벤트 처리 태스크
            response_count = 0

            async def handle_events():
                nonlocal response_count
                while not self.stop_event.is_set():
                    if self.external_stop_flag and self.external_stop_flag.is_set():
                        self.logger("🛑 External stop flag detected - stopping event handling")
                        break
                    try:
                        event = self.event_queue.get_nowait()

                        if event["type"] == "text" and on_text:
                            on_text(event["content"])

                        elif event["type"] == "text_complete" and on_text:
                            # 전체 누적 텍스트를 on_text에 전달 (JSON 파싱용)
                            on_text(event["content"])

                        elif event["type"] == "audio" and on_audio:
                            on_audio(event["content"])

                        elif event["type"] == "function_call" and on_function_call:
                            function_name = event["function"]
                            arguments = event["arguments"]
                            on_function_call(function_name, arguments)

                        elif event["type"] == "audio_done":
                            playback_time = event.get("playback_time", 0.5)
                            self.logger(f"⏳ Waiting {playback_time:.2f}s for audio playback...")
                            await asyncio.sleep(playback_time)
                            self.accept_audio_input = True
                            self.logger("🎤 Microphone enabled (audio playback complete)")

                        elif event["type"] == "response_done":
                            response_count += 1
                            if response_count == 1:
                                self.logger("✅ Initial greeting completed")

                    except queue.Empty:
                        await asyncio.sleep(0.01)

            event_task = asyncio.create_task(handle_events())

            # 모든 태스크 대기
            await asyncio.gather(receive_task, send_task, event_task)

        except Exception as e:
            self.logger(f"❌ Conversation error: {e}")

        finally:
            await self.close()
