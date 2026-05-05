#!/usr/bin/env python3
import subprocess, tempfile, os, sys, requests
from faster_whisper import WhisperModel

WHISPER_MODEL_PATH = "/home/jadam/robot_ws/src/robot_voice/models/whisper-small-en"
PIPER_MODEL_PATH   = "/home/jadam/robot_ws/src/robot_voice/voices/en_US-ryan-high.onnx"
OLLAMA_URL         = "http://localhost:11434/api/chat"
LLM_MODEL          = "llama3.2:3b"
SYSTEM_PROMPT = (
    "You are RoboSnailBob, a rugged outdoor patrol robot. You patrol yards, "
    "deter deer and pests, and monitor your environment. You are helpful, "
    "practical, and have a dry sense of humor. Keep responses concise - "
    "1 to 3 sentences unless asked to explain something in detail. "
    "Never break character."
)

print("Loading Whisper model...")
stt_model = WhisperModel(WHISPER_MODEL_PATH, device="cpu", compute_type="int8")
print("Ready.\n")

conversation_history = []

def record_until_enter():
    tmp = tempfile.NamedTemporaryFile(suffix=".wav", delete=False)
    tmp.close()
    proc = subprocess.Popen(
        ["arecord", "-f", "S16_LE", "-r", "16000", "-c", "1", tmp.name],
        stderr=subprocess.DEVNULL
    )
    input("  [Recording... press Enter to stop]")
    proc.terminate()
    proc.wait()
    return tmp.name

def transcribe(wav_path):
    segments, _ = stt_model.transcribe(wav_path, beam_size=5)
    return " ".join(s.text.strip() for s in segments).strip()

def ask_llm(user_text):
    conversation_history.append({"role": "user", "content": user_text})
    payload = {
        "model": LLM_MODEL,
        "messages": [{"role": "system", "content": SYSTEM_PROMPT}] + conversation_history,
        "stream": False
    }
    resp = requests.post(OLLAMA_URL, json=payload, timeout=60)
    resp.raise_for_status()
    reply = resp.json()["message"]["content"].strip()
    conversation_history.append({"role": "assistant", "content": reply})
    return reply

def speak(text):
    piper = subprocess.Popen(
        ["python3", "-m", "piper", "--model", PIPER_MODEL_PATH, "--output_raw"],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
    )
    aplay = subprocess.Popen(
        ["aplay", "-r", "22050", "-f", "S16_LE", "-t", "raw", "-"],
        stdin=piper.stdout, stderr=subprocess.DEVNULL
    )
    piper.stdin.write(text.encode())
    piper.stdin.close()
    piper.stdout.close()
    piper.wait()
    aplay.wait()

def main():
    print("=== RoboSnailBob Voice Loop ===")
    print("Press Enter to start recording, Enter again to stop. Ctrl+C to quit.\n")
    while True:
        try:
            input("Press Enter to speak...")
        except (EOFError, KeyboardInterrupt):
            print("\nShutting down.")
            sys.exit(0)
        wav = record_until_enter()
        print("  Transcribing...")
        text = transcribe(wav)
        os.unlink(wav)
        if not text:
            print("  (nothing heard, try again)\n")
            continue
        print(f"  You: {text}")
        print("  Thinking...")
        reply = ask_llm(text)
        print(f"  RoboSnailBob: {reply}\n")
        speak(reply)

if __name__ == "__main__":
    main()
