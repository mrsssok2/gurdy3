"""Voice front end for the agent.

  # live, mock world, CPU (voice image):   scripts/voice_chat.sh        (see below)
  python3 -m brain.voice.cli --mic [--vad] [--mock | --remote HOST:PORT] [--ollama MODEL | keyword planner]
  python3 -m brain.voice.cli --wav utterance.wav ...                    (one recorded utterance through the same pipeline, mock world)

--mic      : persistent mic stream (WSLg parecord, 1.5 s warm-up discarded), Piper replies through paplay.
             Push-to-talk by default (Enter = start, Enter = send); --vad = always listening. A typed line is a command as text.
--mock     : the agent runs in this process on the mock world (default). --remote HOST:PORT: the agent runs elsewhere
             (`python3 -m brain.ros.run_text --serve PORT` in the sim image; see scripts/start_agent_chat.sh VOICE=true).
Models: --stt-model (CTranslate2 Whisper dir, default /models/whisper-turbo-ct2), --tts-model (Piper .onnx, default /models/tts/piper-lessac.onnx).
The Whisper vocabulary bias is fed with the agent's place names. Speaking (barge-in): talk or press Enter while it speaks and it stops at once."""
from __future__ import annotations

import argparse
import os
import queue
import sys
import threading

import numpy as np

from .audio import FRAME_N, AplaySink, ArecordSource, frames, read_wav
from .pipeline import VoiceConfig, VoicePipeline


def build_local_host(ollama: str | None, lab: bool = False):
    """`lab=True`: the mock world generated from the Gazebo flat_lab files (web dashboard); default: the evaluation mock world."""
    from ..agent.graph import Agent
    from ..mock import MockBackend, gazebo_flat_lab
    from ..tools import ToolLayer
    from .host import AgentHost
    b = MockBackend(gazebo_flat_lab() if lab else None)
    from ..incidents.backend import virtual_wall, with_incidents
    b = with_incidents(b, None, b.world.map_id, wall=virtual_wall(b))          # temporal memory (RAM only: a mock world has no restarts)
    if ollama:
        from ..agent.ollama_llm import OllamaLLM
        llm = OllamaLLM(model=ollama, constrain=not ollama.startswith("gurdy-sft"))
    else:
        from ..agent.llm import KeywordLLM
        llm = KeywordLLM(b.known_places, smart=True)
    return AgentHost(Agent(ToolLayer(b), llm), sim_speed=3.0), b      # mock world: virtual clock runs 3x wall time


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mic", action="store_true")
    ap.add_argument("--wav")
    ap.add_argument("--vad", action="store_true", help="always listening (default: push-to-talk with Enter)")
    ap.add_argument("--mock", action="store_true", help="agent in this process on the mock world (default)")
    ap.add_argument("--remote", help="HOST:PORT of an agent served by brain.ros.run_text --serve")
    ap.add_argument("--ollama", default=os.environ.get("MODEL") or None, help="LLM model name (needs OLLAMA_URL); default: keyword planner")
    ap.add_argument("--stt-model", default="/models/whisper-turbo-ct2")
    ap.add_argument("--tts-model", default="/models/tts/piper-lessac.onnx")
    ap.add_argument("--device", default=os.environ.get("VOICE_DEVICE", "auto"), help="Whisper device: auto (GPU if available, else CPU) | cuda | cpu. Env VOICE_DEVICE")
    ap.add_argument("--no-debug-wav", action="store_true", help="do not save each utterance to brain/runs/voice_debug/")
    ap.add_argument("--bias", choices=["none", "prompt", "full"], default=os.environ.get("VOICE_BIAS", "prompt"),
                    help="Whisper vocabulary bias: none | prompt (default) | full (prompt + hotwords). Env VOICE_BIAS")
    ap.add_argument("--no-bias", action="store_true", help="same as --bias none")
    ap.add_argument("--serve-mock", type=int, metavar="PORT", help="no audio: serve the mock-world agent on PORT for a --remote voice client (brain image, has langgraph)")
    a = ap.parse_args()
    if a.serve_mock:
        from .host import serve
        host, _ = build_local_host(a.ollama)
        host.start()
        serve(host, a.serve_mock)
        print(f"voice server on :{a.serve_mock} (mock world, {'LLM ' + a.ollama if a.ollama else 'keyword planner'})", flush=True)
        threading.Event().wait()
        return
    if not (a.mic or a.wav):
        ap.error("give --mic or --wav")

    if a.remote:
        from .host import RemoteHost
        host, mock = RemoteHost(a.remote), None
    else:
        host, mock = build_local_host(a.ollama if a.mic or a.ollama else None)
    from .stt import load_whisper
    from .tts import PiperTTS, Speaker
    print("loading Whisper + Piper ...", flush=True)
    stt = load_whisper(a.stt_model, a.device, places=host.places() or None, bias="none" if a.no_bias else a.bias)
    print(f"Whisper on the {stt.device.upper()}", flush=True)
    tts = PiperTTS(a.tts_model)

    if a.wav:                                       # one utterance, synchronous, mock world
        agent = host.agent
        pipe = VoicePipeline(stt, Speaker(tts), lambda t, c: agent.say(t, c).reply, lambda: agent.layer.call("stop"),
                             lambda: agent.executor.state == "running", VoiceConfig())
        for f in frames(np.concatenate([read_wav(a.wav), np.zeros(FRAME_N * 40, np.float32)])):
            pipe.feed(f)
        for e in pipe.log:
            print(e)
        return

    from .live import LivePipeline, LockedSpeaker, run_live, stdin_controls
    mode = "vad" if a.vad else "ptt"
    host.start()
    pipe = LivePipeline(stt, LockedSpeaker(Speaker(tts, AplaySink())), host, VoiceConfig(mode=mode),
                        debug_dir=None if a.no_debug_wav else os.environ.get("VOICE_DEBUG_DIR", "brain/runs/voice_debug"))
    ctl: queue.Queue = queue.Queue()
    src = ArecordSource()
    print("opening the microphone (1.5 s warm-up) ...", flush=True)
    src.start()
    if src.warmup_rms < 1e-4:
        print("  warning: the mic delivered silence while warming up (check `scripts/mic_test.sh list`)")
    stdin_controls(ctl, mode)
    print(("Always listening: just speak." if a.vad else "Push-to-talk: press Enter, speak, press Enter.")
          + " Type a command to send it as text. 'stop' works any time. 'q' quits.\nReady.", flush=True)
    try:
        run_live(pipe, src, ctl)
    except KeyboardInterrupt:
        pass
    finally:
        pipe.close(); host.close(); src.close(); pipe.speaker.stop()


if __name__ == "__main__":
    main()
    sys.stdout.flush()
    os._exit(0)
