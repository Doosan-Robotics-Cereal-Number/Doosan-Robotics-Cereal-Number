#!/usr/bin/env python3
"""
cleanup_orchestrator_node.py

RealSense 안정 이벤트를 받아 최신 디버그 이미지를 GPT Realtime Mini에 전달하고
음성 상호작용을 트리거하기 위한 스캐폴딩 노드.
"""

from __future__ import annotations

import asyncio
import base64
import binascii
import json
import os
import subprocess
import threading
import time
from collections import Counter
from typing import Optional, Dict, Any, List

import requests

import cv2
import numpy as np
import rclpy
import websockets
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from websockets.exceptions import InvalidStatusCode, InvalidMessage


class CleanupOrchestratorNode(Node):
    """Stable object 이벤트를 GPT Realtime 워크플로우로 연결."""

    def __init__(self):
        super().__init__("cleanup_orchestrator")

        # Parameters
        self.declare_parameter("stable_event_topic", "/workspace_stable_object")
        self.declare_parameter("debug_image_topic", "/workspace_debug")
        self.declare_parameter("gpt_model", "gpt-5")
        self.declare_parameter("tts_model", "gpt-realtime-mini")
        self.declare_parameter("openai_api_key_env", "OPENAI_API_KEY")
        self.declare_parameter("audio_voice", "echo")
        self.declare_parameter("audio_speed", 1.0)
        self.declare_parameter("request_cooldown_sec", 30.0)
        self.declare_parameter("ask_prompt", "물건을 치워 드릴까요?")
        self.declare_parameter("enable_requests", True)
        self.declare_parameter("auto_play_audio", True)
        self.declare_parameter("event_batch_delay_sec", 0.8)

        self.stable_event_topic = str(self.get_parameter("stable_event_topic").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.gpt_model = str(self.get_parameter("gpt_model").value)
        self.tts_model = str(self.get_parameter("tts_model").value)
        self.audio_voice = str(self.get_parameter("audio_voice").value)
        self.audio_speed = max(0.25, float(self.get_parameter("audio_speed").value))
        self.request_cooldown_sec = max(0.0, float(self.get_parameter("request_cooldown_sec").value))
        self.ask_prompt = str(self.get_parameter("ask_prompt").value)
        self.enable_requests = bool(self.get_parameter("enable_requests").value)
        self.auto_play_audio = bool(self.get_parameter("auto_play_audio").value)
        self.event_batch_delay_sec = max(0.0, float(self.get_parameter("event_batch_delay_sec").value))

        api_env = str(self.get_parameter("openai_api_key_env").value or "OPENAI_API_KEY")
        self.api_key = os.getenv(api_env)
        if not self.api_key:
            self.get_logger().warn(f"{api_env} is not set. GPT 요청은 건너뜁니다.")

        self.http_session = requests.Session()
        self.bridge = CvBridge()
        self.latest_image: Optional[np.ndarray] = None
        self.latest_stamp: Optional[float] = None
        self.last_request_time = 0.0
        self.inflight_lock = threading.Lock()
        self.batch_lock = threading.Lock()
        self.pending_events: List[Dict[str, Any]] = []
        self.batch_timer: Optional[threading.Timer] = None

        self.debug_sub = self.create_subscription(Image, self.debug_image_topic, self._on_debug_image, 10)
        self.event_sub = self.create_subscription(String, self.stable_event_topic, self._on_stable_event, 10)

        self.get_logger().info("🧹 Cleanup orchestrator ready.")

    # ------------------------------------------------------------------
    def _on_debug_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as err:
            self.get_logger().warn(f"Failed to decode debug image: {err}")
            return
        self.latest_image = frame
        self.latest_stamp = self._stamp_to_seconds(msg.header.stamp) if msg.header else time.time()

    def _on_stable_event(self, msg: String):
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError as err:
            self.get_logger().warn(f"Stable event JSON parse error: {err}")
            return

        if self.latest_image is None:
            self.get_logger().warn("Stable event received but no debug image cached yet.")
            return

        if not self.enable_requests or not self.api_key:
            self.get_logger().info("Requests disabled or API key missing; logging only.")
            self._log_event(event)
            return

        with self.batch_lock:
            self.pending_events.append(event)
            if self.batch_timer:
                self.batch_timer.cancel()
            if self.event_batch_delay_sec <= 0.0:
                self.batch_timer = None
            else:
                self.batch_timer = threading.Timer(self.event_batch_delay_sec, self._process_pending_events)
                self.batch_timer.daemon = True
                self.batch_timer.start()

        if self.event_batch_delay_sec <= 0.0:
            self._process_pending_events()

    def _process_pending_events(self):
        with self.batch_lock:
            events = self.pending_events
            self.pending_events = []
            self.batch_timer = None

        if not events:
            return

        if self.latest_image is None:
            self.get_logger().warn("Batch ready but no debug image cached; skipping request.")
            return

        now = time.time()
        if now - self.last_request_time < self.request_cooldown_sec:
            self.get_logger().info("Skipping GPT request due to cooldown.")
            return

        frame = self.latest_image.copy()
        snapshot_bytes = self._encode_snapshot(frame)
        if snapshot_bytes is None:
            self.get_logger().warn("Failed to encode snapshot for GPT.")
            return

        self.last_request_time = now
        threading.Thread(
            target=self._handle_event_thread,
            args=(events, snapshot_bytes),
            daemon=True,
        ).start()

    # ------------------------------------------------------------------
    def _handle_event_thread(self, events: List[Dict[str, Any]], snapshot_bytes: bytes):
        with self.inflight_lock:
            self.get_logger().debug("Snapshot captured in memory (disk save skipped).")
            text = self._request_text_summary(events, snapshot_bytes)
            if not text:
                return
            tts_text = self._build_audio_prompt_from_text(text) or text
            audio = self._request_tts_audio(tts_text)
            self._handle_gpt_response({"text": text, "audio": audio, "tts_text": tts_text})

    def _request_text_summary(self, events: List[Dict[str, Any]], snapshot_bytes: bytes) -> Optional[str]:
        if not self.enable_requests:
            return None
        self.get_logger().info("🧠 Requesting GPT text summary...")
        try:
            return self._request_text_summary_rest(events, snapshot_bytes)
        except Exception as err:
            self.get_logger().error(f"Text summary request failed: {err}")
            return None

    def _request_text_summary_rest(self, events: List[Dict[str, Any]], snapshot_bytes: bytes) -> Optional[str]:
        if not self.api_key:
            return None
        data_url = self._to_data_url(snapshot_bytes)
        if data_url is None:
            return None

        instructions = (
            "너는 작업 공간 정리 도우미다. 속도보다 정확도가 100배 더 중요하니 신중히 추론하라.\n"
            "이미지 속 각 ID를 다음 단일 형식으로만 보고하되, 실제 물체명을 확실히 확인한 다음 기입하라:\n"
            "ID N: <정확한 한국어 물체명>. 물건을 치워 드릴까요?\n"
            "<한국어 물체명>은 컵, 텀블러, 우유곽, 휴지 같은 짧은 명사가 바람직하지만 이는 단순 예시에 불과하며, "
            "실제 관측된 물체에 가장 잘 맞는 명사를 자유롭게 골라야 한다. 애매하면 가장 가능성 높은 명사를 1개만 선택하라. "
            "여러 ID가 있으면 줄바꿈으로 구분하고, 이 형식 외의 문장/설명/접속사는 절대 추가하지 마라."
        )

        user_prompt = self._build_user_prompt(events)
        payload = {
            "model": self.gpt_model,
            "input": [
                {
                    "role": "user",
                    "content": [
                        {"type": "input_text", "text": f"{instructions}\n{user_prompt}"},
                        {"type": "input_image", "image_url": data_url, "detail": "high"},
                    ],
                }
            ],
        }
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json",
        }

        try:
            response = self.http_session.post(
                "https://api.openai.com/v1/responses",
                headers=headers,
                json=payload,
                timeout=60,
            )
        except requests.RequestException as err:
            self.get_logger().error(f"Responses request failed: {err}")
            return None

        if response.status_code != 200:
            self.get_logger().error(f"Responses API error {response.status_code}: {response.text}")
            return None

        try:
            data = response.json()
        except ValueError as err:
            self.get_logger().error(f"Failed to parse Responses JSON: {err}")
            return None

        result = self._extract_text_from_response(data)
        if result:
            self.get_logger().info("📝 GPT text summary received.")
        else:
            self.get_logger().warn("Responses API returned no text output.")
        return result

    def _request_tts_audio(self, text: str) -> Optional[bytes]:
        if not text or not self.enable_requests:
            return None
        self.get_logger().info("🎙️ Requesting GPT TTS audio...")
        try:
            return asyncio.run(self._tts_async(text))
        except RuntimeError as err:
            if "asyncio.run()" in str(err):
                loop = asyncio.new_event_loop()
                try:
                    asyncio.set_event_loop(loop)
                    return loop.run_until_complete(self._tts_async(text))
                finally:
                    asyncio.set_event_loop(None)
                    loop.close()
            self.get_logger().error(f"TTS request failed: {err}")
            return None
        except Exception as err:
            self.get_logger().error(f"TTS exception: {err}")
            return None

    async def _tts_async(self, text: str) -> Optional[bytes]:
        self.get_logger().info(f"🎙️ Starting TTS with voice={self.audio_voice}, speed={self.audio_speed}")

        url = f"wss://api.openai.com/v1/realtime?model={self.tts_model or self.gpt_model}"
        headers = [
            ("Authorization", f"Bearer {self.api_key}"),
            ("OpenAI-Beta", "realtime=v1"),
        ]
        try:
            ws = await websockets.connect(url, extra_headers=headers, ping_interval=20)
        except TypeError as err:
            if "extra_headers" in str(err):
                ws = await websockets.connect(url, additional_headers=headers, ping_interval=20)
            else:
                raise
        except Exception as err:
            self.get_logger().error(f"TTS websocket open failed: {err}")
            return None

        self.get_logger().debug("Realtime TTS session opened.")

        audio_chunks: List[str] = []
        seen_events: Counter[str] = Counter()
        misc_log_budget = 3
        try:
            async with ws:
                await ws.send(
                    json.dumps(
                        {
                            "type": "session.update",
                            "session": {
                                "modalities": ["audio", "text"],
                                "voice": self.audio_voice,
                                "instructions": "사용자 텍스트를 글자 하나도 빠뜨리지 말고 그대로 읽어라. "
                                "감탄사나 어미를 새로 만들지 말고 전달된 문장만 정확하게 낭독하라.",
                                "temperature": 0.6,
                            },
                        }
                    )
                )
                safe_text = text.replace('"', "'")
                instructions = (
                    "따옴표 안 문장을 한 글자도 바꾸지 말고 그대로 읽어라. "
                    "추가 멘트나 감탄사는 절대 하지 마라. "
                    f'문장: "{safe_text}"'
                )
                await ws.send(
                    json.dumps(
                        {
                            "type": "response.create",
                            "response": {
                                "modalities": ["audio", "text"],
                                "instructions": instructions,
                                "temperature": 0.6,
                            },
                        }
                    )
                )
                async for raw in ws:
                    evt = json.loads(raw)
                    evt_type = evt.get("type")
                    seen_events[evt_type] += 1
                    if evt_type in ("response.output_audio.delta", "response.audio.delta"):
                        delta = evt.get("delta")
                        if isinstance(delta, dict):
                            chunk = delta.get("audio")
                        else:
                            chunk = delta
                        if chunk:
                            audio_chunks.append(chunk)
                    elif evt_type in ("response.output_text.delta", "response.text.delta", "response.audio_transcript.delta"):
                        if misc_log_budget > 0:
                            self.get_logger().debug(f"TTS text delta (ignored): {evt.get('delta')}")
                            misc_log_budget -= 1
                    elif evt_type in ("response.output_audio.done", "response.audio.done", "response.completed", "response.done"):
                        break
                    elif evt_type == "response.error":
                        self.get_logger().error(f"TTS realtime error: {evt}")
                        break
                    elif misc_log_budget > 0:
                        self.get_logger().debug(f"TTS event ({evt_type}): {evt}")
                        misc_log_budget -= 1
        except asyncio.TimeoutError:
            self.get_logger().error("Realtime TTS session timed out.")
            return None
        except (InvalidStatusCode, InvalidMessage) as err:
            self.get_logger().error(f"TTS websocket error: {err}")
            return None
        except Exception as err:
            self.get_logger().error(f"TTS session failed: {err}")
            return None

        if not audio_chunks:
            counts = ", ".join(f"{k}={v}" for k, v in seen_events.items())
            self.get_logger().warn(
                "Realtime TTS produced no audio chunks. "
                + (f"Events seen: {counts}" if counts else "No events received after response request.")
            )
            return None

        audio = self._merge_audio_chunks(audio_chunks)
        if audio:
            self.get_logger().info("🔉 GPT audio received.")
        return audio

    def _handle_gpt_response(self, data: Dict[str, Any]):
        text_response = data.get("text", "") if data else ""
        audio_bytes = data.get("audio") if data else None
        tts_text = data.get("tts_text") or text_response
        if text_response:
            self.get_logger().info("GPT text summary:\n" + text_response)
        if audio_bytes:
            self.get_logger().info("Received audio response.")
            if self.auto_play_audio:
                spoken = (tts_text or "").replace("\n", " ").strip()
                if spoken:
                    self.get_logger().info(f"🔊 Speaking text: {spoken}")
                self._play_audio_stream(audio_bytes)

    # ------------------------------------------------------------------
    def _build_user_prompt(self, events: List[Dict[str, Any]]) -> str:
        lines = [
            "다음 안정 객체들이 동시에 감지되었습니다. 각 ID에 대해 실제 물체명을 보고하세요.",
            "이미지의 ID 표기를 확인하고 아래 형식으로만 답하세요: 'ID N: 실제 물체명. 물건을 치워 드릴까요?'",
            "단, '물체명'이라는 단어 그대로 출력하지 말고 컵/병/휴지/박스 같은 실제 명사를 넣으세요.",
            "예시는 참고용일 뿐이며, 실제로 보이는 물체에 가장 알맞은 한국어 명사를 자유롭게 선택하세요.",
        ]
        for event in events:
            obj_id = event.get("id", "?")
            pos = event.get("position_mm")
            px = event.get("pixel")
            desc = f"- ID {obj_id}"
            if pos is not None:
                desc += f": 위치 {pos} mm"
            if px:
                desc += f", 픽셀 {px}"
            lines.append(desc)
        lines.append(self.ask_prompt)
        return "\n".join(lines)

    def _encode_snapshot(self, frame: np.ndarray) -> Optional[bytes]:
        success, buffer = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
        if not success:
            return None
        return buffer.tobytes()

    def _to_data_url(self, snapshot_bytes: bytes) -> Optional[str]:
        if not snapshot_bytes:
            return None
        b64 = base64.b64encode(snapshot_bytes).decode("ascii")
        return "data:image/jpeg;base64," + b64

    @staticmethod
    def _extract_text_from_response(payload: Dict[str, Any]) -> Optional[str]:
        outputs = payload.get("output") or []
        chunks: List[str] = []
        for item in outputs:
            if not isinstance(item, dict):
                continue
            if item.get("type") != "message":
                continue
            for block in item.get("content", []):
                if not isinstance(block, dict):
                    continue
                if block.get("type") == "output_text":
                    chunks.append(str(block.get("text", "")))
        text = "".join(chunks).strip()
        if text:
            return text
        raw_text = payload.get("output_text")
        if isinstance(raw_text, list):
            merged = "".join(str(part) for part in raw_text).strip()
            return merged or None
        if isinstance(raw_text, str):
            stripped = raw_text.strip()
            return stripped or None
        return None

    def _build_audio_prompt_from_text(self, text: str) -> Optional[str]:
        names = self._extract_item_names(text)
        if not names:
            return None
        if len(names) == 1:
            target = names[0] + self._object_particle(names[0])
        elif len(names) == 2:
            connector = self._conjunction_particle(names[0])
            target = f"{names[0]}{connector} {names[1]}{self._object_particle(names[1])}"
        else:
            prefix = ", ".join(names[:-1])
            target = f"{prefix}, 그리고 {names[-1]}{self._object_particle(names[-1])}"
        return f"{target} 치워 드릴까요?"

    @staticmethod
    def _extract_item_names(text: str) -> List[str]:
        names: List[str] = []
        for line in text.splitlines():
            line = line.strip()
            if not line or not line.lower().startswith("id"):
                continue
            parts = line.split(":", 1)
            if len(parts) != 2:
                continue
            rest = parts[1].strip()
            if not rest:
                continue
            name = rest.split(".", 1)[0].strip()
            if name and not name.lower().startswith("물체명"):
                names.append(name)
        return names

    @staticmethod
    def _object_particle(word: str) -> str:
        return "을" if CleanupOrchestratorNode._has_final_consonant(word) else "를"

    @staticmethod
    def _conjunction_particle(word: str) -> str:
        return "과" if CleanupOrchestratorNode._has_final_consonant(word) else "와"

    @staticmethod
    def _has_final_consonant(word: str) -> bool:
        if not word:
            return False
        ch = word[-1]
        code = ord(ch)
        if 0xAC00 <= code <= 0xD7A3:
            return (code - 0xAC00) % 28 != 0
        return False

    def _merge_audio_chunks(self, chunks: List[str]) -> Optional[bytes]:
        if not chunks:
            return None
        decoded: List[bytes] = []
        for idx, chunk in enumerate(chunks):
            if not chunk:
                continue
            if not isinstance(chunk, str):
                self.get_logger().warn(
                    f"Skipping audio chunk #{idx} because it is type {type(chunk).__name__}, expected str."
                )
                continue
            try:
                decoded.append(base64.b64decode(chunk))
            except (ValueError, binascii.Error, TypeError) as err:
                self.get_logger().warn(f"Failed to decode audio chunk #{idx}: {err}")
        if not decoded:
            return None
        return b"".join(decoded)

    def _play_audio_stream(self, audio_bytes: bytes):
        try:
            proc = subprocess.Popen(
                ["ffplay", "-autoexit", "-nodisp", "-loglevel", "error", "-f", "s16le", "-ar", "16000", "-ac", "1", "-i", "pipe:0"],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )
        except FileNotFoundError:
            self.get_logger().warn("aplay not found; skipping audio playback.")
            return
        except Exception as err:
            self.get_logger().warn(f"Failed to start audio playback: {err}")
            return

        try:
            stderr_output = proc.communicate(input=audio_bytes, timeout=30)[1] or b""
        except subprocess.TimeoutExpired:
            proc.kill()
            self.get_logger().warn("Audio playback timed out; ffplay was terminated.")
            return
        except Exception as err:
            proc.kill()
            self.get_logger().warn(f"Failed to stream audio: {err}")
            return

        if proc.returncode != 0:
            message = stderr_output.decode("utf-8", errors="ignore").strip()
            detail = f": {message}" if message else ""
            self.get_logger().warn(f"ffplay exited with code {proc.returncode}{detail}")
        elif stderr_output:
            self.get_logger().debug(stderr_output.decode("utf-8", errors="ignore").strip())

    @staticmethod
    def _stamp_to_seconds(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9

    def _log_event(self, event: Dict[str, Any]):
        self.get_logger().info(f"[Dry-run] Stable event: {json.dumps(event, ensure_ascii=False)}")

    def destroy_node(self):
        try:
            self.http_session.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CleanupOrchestratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
