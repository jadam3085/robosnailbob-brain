#!/usr/bin/env python3
"""
RoboSnailBob voice loop v2 - streaming LLM + VAD silence detection
"""
import subprocess, tempfile, os, sys, re, time, collections, threading
import requests, webrtcvad
from faster_whisper import WhisperModel

# ── Config ────────────────────────────────────────────────────────────────────
WHISPER_MODEL_PATH = "/home/jadam/robot_ws/src/robosnailbob_brain/models/whisper-tiny-en"
PIPER_MODEL_PATH   = "/home/jadam/robot_ws/src/robosnailbob_brain/voices/en_US-ryan-high.onnx"
OLLAMA_URL         = "http://localhost:11434/api/chat"
LLM_MODEL          = "llama3.2:3b"   # swap after benchmarking

OLLAMA_OPTIONS = {
    "num_ctx": 1024,
    "num_predict": 80,
    "temperature": 0.7,
    "stop": ["\nUser:", "\nAssistant:"]
}

SYSTEM_PROMPT = (
       "You are RoboSnailBob, a rugged outdoor patrol robot. You patrol yards, "
    "deter deer and pests, and monitor your environment. "
    "You are practical and have a dry sense of humor. "
    "IMPORTANT: Reply in ONE sentence only. Two sentences maximum if truly necessary. "
    "Never break character. Never say you are an AI." 
)

# VAD settings
VAD_AGGRESSIVENESS  = 2        # 0-3, higher = more aggressive silence cutting
SAMPLE_RATE         = 16000
FRAME_DURATION_MS   = 30       # 10, 20, or 30ms only
SILENCE_TIMEOUT_MS  = 800      # stop recording after this much silence
FRAME_SIZE          = int(SAMPLE_RATE * FRAME_DURATION_MS / 1000) * 2  # bytes

# ── Init ──────────────────────────────────────────────────────────────────────
print("Loading Whisper model...")
stt = WhisperModel(WHISPER_MODEL_PATH, device="cpu", compute_type="int8")
vad = webrtcvad.Vad(VAD_AGGRESSIVENESS)
print("Ready.\n")

conversation_history = []

# ── VAD Recording ─────────────────────────────────────────────────────────────
def record_with_vad() -> str:
    """Record until VAD detects sustained silence. Returns path to wav file."""
    tmp = tempfile.NamedTemporaryFile(suffix=".wav", delete=False)
    tmp.close()

    proc = subprocess.Popen(
        ["arecord", "-f", "S16_LE", "-r", str(SAMPLE_RATE), "-c", "1", "-t", "raw"],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
    )

    frames = []
    silence_frames = 0
    speaking_started = False
    silence_limit = int(SILENCE_TIMEOUT_MS / FRAME_DURATION_MS)

    print("  [Listening...]", end="", flush=True)
    try:
        while True:
            frame = proc.stdout.read(FRAME_SIZE)
            if len(frame) < FRAME_SIZE:
                break
            is_speech = vad.is_speech(frame, SAMPLE_RATE)
            if is_speech:
                speaking_started = True
                silence_frames = 0
                frames.append(frame)
                print(".", end="", flush=True)
            else:
                if speaking_started:
                    silence_frames += 1
                    frames.append(frame)
                    if silence_frames >= silence_limit:
                        print(" [done]")
                        break
    finally:
        proc.terminate()
        proc.wait()

    # Write raw frames to wav
    import wave
    with wave.open(tmp.name, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(b"".join(frames))

    return tmp.name


# ── STT ───────────────────────────────────────────────────────────────────────
def transcribe(wav_path: str) -> str:
    segments, _ = stt.transcribe(wav_path, beam_size=1)
    return " ".join(s.text.strip() for s in segments).strip()


# ── Streaming LLM + Chunked TTS ───────────────────────────────────────────────
SENTENCE_END = re.compile(r'([^.!?]*[.!?])\s*')
_piper_proc = None
_aplay_proc = None

def start_tts_pipeline():
    global _piper_proc, _aplay_proc
    _piper_proc = subprocess.Popen(
        ["python3", "-m", "piper",
         "--model", PIPER_MODEL_PATH,
         "--length_scale", "0.9",
         "--output_raw"],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
    )
    _aplay_proc = subprocess.Popen(
        ["aplay", "-r", "22050", "-f", "S16_LE", "-t", "raw", "-"],
        stdin=_piper_proc.stdout, stderr=subprocess.DEVNULL
    )

def speak_chunk(text: str):
    """Write a sentence into the persistent Piper process — no startup gap."""
    text = text.strip()
    if not text or _piper_proc is None:
        return
    try:
        _piper_proc.stdin.write((text + " ").encode())
        _piper_proc.stdin.flush()
    except BrokenPipeError:
        pass

def stop_tts_pipeline():
    global _piper_proc, _aplay_proc
    if _piper_proc:
        try:
            _piper_proc.stdin.close()
        except:
            pass
        _piper_proc.wait()
        _aplay_proc.wait()
    _piper_proc = None
    _aplay_proc = None



def ask_llm_streaming(user_text: str):
    conversation_history.append({"role": "user", "content": user_text})
    if len(conversation_history) > 12:
        conversation_history[:] = conversation_history[-12:]

    payload = {
        "model": LLM_MODEL,
        "messages": [{"role": "system", "content": SYSTEM_PROMPT}] + conversation_history,
        "stream": True,
        "options": OLLAMA_OPTIONS,
        "keep_alive": "30m"
    }

    full_reply = ""
    buffer = ""
    start_tts_pipeline()

    try:
        with requests.post(OLLAMA_URL, json=payload, stream=True, timeout=60) as resp:
            resp.raise_for_status()
            for line in resp.iter_lines():
                if not line:
                    continue
                import json
                chunk = json.loads(line)
                token = chunk.get("message", {}).get("content", "")
                buffer += token
                full_reply += token

                while True:
                    m = SENTENCE_END.match(buffer)
                    if not m:
                        break
                    sentence = m.group(1)
                    buffer = buffer[m.end():]
                    print(f"  → {sentence}")
                    speak_chunk(sentence)

                if chunk.get("done"):
                    break

        if buffer.strip():
            print(f"  → {buffer.strip()}")
            speak_chunk(buffer.strip())

    finally:
        stop_tts_pipeline()

    conversation_history.append({"role": "assistant", "content": full_reply.strip()})



# ── Main loop ─────────────────────────────────────────────────────────────────
def main():
    print("=== RoboSnailBob Voice Loop v2 ===")
    print("Speak naturally — recording stops on silence. Ctrl+C to quit.\n")

    while True:
        try:
            input("Press Enter to start listening...")
        except (EOFError, KeyboardInterrupt):
            print("\nShutting down.")
            sys.exit(0)

        wav = record_with_vad()
        print("  Transcribing...")
        text = transcribe(wav)
        os.unlink(wav)

        if not text:
            print("  (nothing heard)\n")
            continue

        print(f"  You: {text}")
        print("  Thinking + speaking...")
        ask_llm_streaming(text)
        print()


if __name__ == "__main__":
    main()
