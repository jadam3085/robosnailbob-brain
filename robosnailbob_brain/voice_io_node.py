#!/usr/bin/env python3
"""
voice_io_node — wake word + VAD + STT + Piper TTS + audio cues

Publishes:  /voice/input  (std_msgs/String) — transcribed speech
Subscribes: /voice/output (std_msgs/String) — sentence chunks to speak
"""

import os
import queue
import socket
import subprocess
import tempfile
import threading
import time
import wave

import numpy as np
import rclpy
import webrtcvad

from faster_whisper import WhisperModel

# ── Block network during openwakeword import (hangs on update check) ──────────
_orig_getaddrinfo = socket.getaddrinfo
socket.getaddrinfo = lambda *a, **kw: (_ for _ in ()).throw(OSError('offline'))
from openwakeword.model import Model as WakeWordModel
socket.getaddrinfo = _orig_getaddrinfo
# ─────────────────────────────────────────────────────────────────────────────

# ── Piper in-process API (eliminates per-sentence model reload overhead) ──────
try:
    from piper.voice import PiperVoice as _PiperVoice
    _PIPER_NATIVE = True
except ImportError:
    _PiperVoice = None
    _PIPER_NATIVE = False
# ─────────────────────────────────────────────────────────────────────────────

from rclpy.node import Node
from std_msgs.msg import String


# ── Paths ─────────────────────────────────────────────────────────────────────

WHISPER_MODEL_PATH = '/home/jadam/robot_ws/src/robosnailbob_brain/models/whisper-tiny-en'
PIPER_MODEL_PATH   = '/home/jadam/robot_ws/src/robosnailbob_brain/voices/en_US-ryan-high.onnx'
SOUNDS_DIR         = '/home/jadam/robot_ws/src/robosnailbob_brain/sounds'
OWW_MODELS_DIR     = '/home/jadam/.local/lib/python3.12/site-packages/openwakeword/resources/models'

# ── Audio config ──────────────────────────────────────────────────────────────

SAMPLE_RATE               = 16000
FRAME_DURATION_MS         = 30
FRAME_SIZE                = int(SAMPLE_RATE * FRAME_DURATION_MS / 1000) * 2
SILENCE_MS                = 1000   # was 800 — extra margin for natural speech pauses
SILENCE_LIMIT             = int(SILENCE_MS / FRAME_DURATION_MS)
WAKEWORD_CHUNK            = 1280
MIN_SPEECH_FRAMES         = 3
SESSION_TIMEOUT_SEC       = 8      # was 6 — more time for back-and-forth
POST_SESSION_COOLDOWN_SEC = 2.0
WAKEWORD_FALLBACK         = 'hey_jarvis_v0.1'  # used if custom model not found
AUDIO_CHECK_INTERVAL_SEC  = 30.0   # throttle PipeWire health checks


# ── Node ──────────────────────────────────────────────────────────────────────

class VoiceIONode(Node):

    def __init__(self):
        super().__init__('voice_io_node')

        self.declare_parameter('vad_aggressiveness',  1)
        self.declare_parameter('wakeword_model',      'hey_snailbob')
        self.declare_parameter('wakeword_threshold',  0.5)
        self.declare_parameter('beam_size',           1)

        vad_level         = self.get_parameter('vad_aggressiveness').value
        ww_model_name     = self.get_parameter('wakeword_model').value
        self.ww_threshold = self.get_parameter('wakeword_threshold').value
        self.beam_size    = self.get_parameter('beam_size').value

        # Publishers / subscribers
        self.pub = self.create_publisher(String, '/voice/input', 10)
        self.create_subscription(
            String, '/voice/output', self._on_voice_output, 10)

        # VAD
        self.vad = webrtcvad.Vad(vad_level)

        # Whisper STT
        self.get_logger().info('Loading Whisper...')
        self.stt = WhisperModel(
            WHISPER_MODEL_PATH, device='cpu', compute_type='int8')
        self.get_logger().info('Whisper ready.')

        # Wake word — prefer custom model, fall back to hey_jarvis
        ww_path = os.path.join(OWW_MODELS_DIR, f'{ww_model_name}.onnx')
        if not os.path.exists(ww_path):
            self.get_logger().warn(
                f'Wake word model not found: {ww_path}\n'
                f'  → Falling back to {WAKEWORD_FALLBACK}. '
                f'Train a custom model and place it at {ww_path} to activate.')
            ww_path = os.path.join(OWW_MODELS_DIR, f'{WAKEWORD_FALLBACK}.onnx')
        self.get_logger().info(f'Loading wake word: {ww_path}')
        self.wakeword = WakeWordModel(wakeword_model_paths=[ww_path])
        self.get_logger().info('Wake word ready.')

        # Piper TTS — in-process preferred (eliminates per-sentence startup cost)
        self._last_audio_check = 0.0
        if _PIPER_NATIVE:
            try:
                self.tts_voice = _PiperVoice.load(PIPER_MODEL_PATH)
                self.get_logger().info('Piper TTS loaded in-process (fast mode).')
            except Exception as e:
                self.tts_voice = None
                self.get_logger().warn(
                    f'Piper in-process load failed: {e} — falling back to subprocess.')
        else:
            self.tts_voice = None
            self.get_logger().warn(
                'piper.voice not importable — using subprocess fallback (slower).')

        # State
        self.tts_queue        = queue.Queue()
        self.is_speaking      = threading.Event()
        self.session_active   = False
        self.session_deadline = 0.0

        # Child process tracking for clean shutdown
        self._child_procs = []
        self._child_lock  = threading.Lock()

        # Threads
        threading.Thread(target=self._record_loop, daemon=True).start()
        threading.Thread(target=self._tts_loop,    daemon=True).start()

        self.get_logger().info('voice_io_node ready — waiting for wake word.')

    # ── PipeWire watchdog ─────────────────────────────────────────────────────

    def _ensure_audio(self):
        """Restart PipeWire if it's not running. Throttled to once per 30 s."""
        if time.time() - self._last_audio_check < AUDIO_CHECK_INTERVAL_SEC:
            return
        self._last_audio_check = time.time()
        result = subprocess.run(
            ['systemctl', '--user', 'is-active', 'pipewire'],
            capture_output=True, text=True
        )
        if result.stdout.strip() != 'active':
            self.get_logger().warn('PipeWire not active — restarting...')
            subprocess.run(
                ['systemctl', '--user', 'restart', 'pipewire', 'pipewire-pulse'],
                stderr=subprocess.DEVNULL
            )
            time.sleep(1.5)
            self.get_logger().info('PipeWire restarted.')

    # ── Process tracking ──────────────────────────────────────────────────────

    def _track(self, proc):
        with self._child_lock:
            self._child_procs.append(proc)
        return proc

    def _untrack(self, *procs):
        with self._child_lock:
            self._child_procs[:] = [
                p for p in self._child_procs if p not in procs]

    def kill_all_children(self):
        with self._child_lock:
            for proc in self._child_procs:
                try:
                    proc.terminate()
                except Exception:
                    pass
            self._child_procs.clear()

    # ── Audio feedback ────────────────────────────────────────────────────────

    def _play_sound(self, name: str):
        path = os.path.join(SOUNDS_DIR, name)
        if os.path.exists(path):
            subprocess.run(
                ['aplay', '-q', path],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )

    # ── TTS output ────────────────────────────────────────────────────────────

    def _on_voice_output(self, msg: String):
        text = msg.data.strip()
        if text:
            self.tts_queue.put(text)

    def _speak_sentence(self, text: str):
        """Speak one sentence — uses in-process PiperVoice if loaded, else subprocess."""
        text = ' '.join(text.split())   # collapse newlines and extra whitespace
        if not text:
            return

        self._ensure_audio()

        if self.tts_voice is not None:
            # In-process synthesis — no Python startup or model reload per sentence
            audio = b''.join(
                self.tts_voice.synthesize_stream_raw(
                    text, length_scale=0.9, sentence_silence=0.0))
            if not audio:
                return
            proc = self._track(subprocess.Popen(
                ['aplay', '-r', '22050', '-f', 'S16_LE', '-t', 'raw', '-'],
                stdin=subprocess.PIPE,
                stderr=subprocess.DEVNULL
            ))
            proc.communicate(audio)
            self._untrack(proc)
        else:
            # Subprocess fallback
            piper = self._track(subprocess.Popen(
                ['python3', '-m', 'piper',
                 '--model', PIPER_MODEL_PATH,
                 '--length_scale', '0.9',
                 '--output_raw'],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL
            ))
            aplay = self._track(subprocess.Popen(
                ['aplay', '-r', '22050', '-f', 'S16_LE', '-t', 'raw', '-'],
                stdin=piper.stdout,
                stderr=subprocess.DEVNULL
            ))
            piper.stdin.write(text.encode())
            piper.stdin.close()
            piper.stdout.close()
            piper.wait()
            aplay.wait()
            self._untrack(piper, aplay)

    def _tts_loop(self):
        """Drain TTS queue — no inter-sentence sleep for natural conversational flow."""
        while rclpy.ok():
            try:
                sentence = self.tts_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            self.is_speaking.set()
            self._speak_sentence(sentence)

            if self.tts_queue.empty():
                self._extend_session()
                self._play_sound('response_done.wav')
                self.is_speaking.clear()

    # ── Main recording loop ───────────────────────────────────────────────────

    def _record_loop(self):
        self.get_logger().info('Waiting for wake word...')

        while rclpy.ok():

            if self.is_speaking.is_set():
                time.sleep(0.05)
                continue

            now = time.time()

            # ── Session mode ───────────────────────────────────────────────
            if self.session_active and now < self.session_deadline:

                wav_path, speech_frames = self._record_vad()

                if wav_path is None:
                    continue

                text = self._transcribe(wav_path, speech_frames)
                os.unlink(wav_path)

                if text:
                    self.get_logger().info(f'Heard: {text}')
                    self._extend_session()
                    msg = String()
                    msg.data = text
                    self.pub.publish(msg)
                else:
                    self.get_logger().debug('Nothing transcribed.')

                continue

            # ── Session close ──────────────────────────────────────────────
            if self.session_active:
                self.get_logger().info('Session ended.')
                self.session_active = False
                self._play_sound('listen_stop.wav')
                time.sleep(POST_SESSION_COOLDOWN_SEC)
                self._reset_wakeword_buffer()

            # ── Wake word phase ────────────────────────────────────────────
            detected = self._wait_for_wakeword()

            if detected:
                self.session_active   = True
                self.session_deadline = time.time() + SESSION_TIMEOUT_SEC
                self.get_logger().info('Session started.')
                self._reset_wakeword_buffer()
                self._play_sound('listen_start.wav')

    # ── Wake word helpers ─────────────────────────────────────────────────────

    def _reset_wakeword_buffer(self):
        """Clear openWakeWord's rolling prediction buffer to prevent immediate re-triggers."""
        try:
            for key in self.wakeword.prediction_buffer:
                self.wakeword.prediction_buffer[key].clear()
        except Exception:
            pass

    def _wait_for_wakeword(self) -> bool:
        proc = self._track(subprocess.Popen(
            ['arecord', '-f', 'S16_LE', '-r', '16000', '-c', '1',
             '-t', 'raw', '--buffer-size=4096'],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL
        ))
        detected = False
        try:
            while rclpy.ok() and not self.is_speaking.is_set():
                raw = proc.stdout.read(WAKEWORD_CHUNK * 2)
                if len(raw) < WAKEWORD_CHUNK * 2:
                    break
                audio  = np.frombuffer(raw, dtype=np.int16)
                scores = self.wakeword.predict(audio)
                score  = max(scores.values()) if scores else 0.0
                if score >= self.ww_threshold:
                    self.get_logger().info(f'Wake word detected ({score:.2f})')
                    detected = True
                    break
        finally:
            proc.terminate()
            proc.wait()
            self._untrack(proc)
        return detected

    # ── VAD recording ─────────────────────────────────────────────────────────

    def _record_vad(self):
        """Record until sustained silence.
        Returns (wav_path, speech_frame_count) or (None, 0)."""
        tmp = tempfile.NamedTemporaryFile(suffix='.wav', delete=False)
        tmp.close()

        proc = self._track(subprocess.Popen(
            ['arecord', '-f', 'S16_LE', '-r', str(SAMPLE_RATE),
             '-c', '1', '-t', 'raw'],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL
        ))

        frames             = []
        silence_frames     = 0
        speech_frame_count = 0
        started            = False

        try:
            while rclpy.ok():

                if self.is_speaking.is_set():
                    proc.terminate()
                    proc.wait()
                    os.unlink(tmp.name)
                    return None, 0

                frame = proc.stdout.read(FRAME_SIZE)
                if len(frame) < FRAME_SIZE:
                    break

                is_speech = self.vad.is_speech(frame, SAMPLE_RATE)

                if is_speech:
                    started             = True
                    silence_frames      = 0
                    speech_frame_count += 1
                    frames.append(frame)
                elif started:
                    silence_frames += 1
                    frames.append(frame)
                    if silence_frames >= SILENCE_LIMIT:
                        break
        finally:
            proc.terminate()
            proc.wait()
            self._untrack(proc)

        if not started or not frames:
            os.unlink(tmp.name)
            return None, 0

        with wave.open(tmp.name, 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(SAMPLE_RATE)
            wf.writeframes(b''.join(frames))

        return tmp.name, speech_frame_count

    # ── STT ───────────────────────────────────────────────────────────────────

    def _transcribe(self, wav_path: str, speech_frame_count: int) -> str:
        if speech_frame_count < MIN_SPEECH_FRAMES:
            self.get_logger().debug(
                f'Skipping — only {speech_frame_count} speech frames')
            return ''
        segments, _ = self.stt.transcribe(
            wav_path,
            beam_size=self.beam_size,
            no_speech_threshold=0.6,
            condition_on_previous_text=False
        )
        parts = []
        for s in segments:
            if s.no_speech_prob > 0.6:
                continue
            parts.append(s.text.strip())
        return ' '.join(parts).strip()

    # ── Session helpers ───────────────────────────────────────────────────────

    def _extend_session(self):
        self.session_deadline = time.time() + SESSION_TIMEOUT_SEC


def main(args=None):
    rclpy.init(args=args)
    node = VoiceIONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down — killing audio processes...')
        node.kill_all_children()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
