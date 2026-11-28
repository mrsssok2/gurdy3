#!/usr/bin/env python3
"""Re-transcribe the saved utterances (brain/runs/voice_debug/*.wav) with different Whisper settings, to see what hurts short phrases.
  scripts/gpu_lock.sh run gurdy3 -- docker compose run --rm voice python3 scripts/voice_ab.py [DIR]
Configs: no_bias | prompt_only | full (initial_prompt + hotwords, what the live loop uses) | lead0.3 (full, 0.3 s of silence prepended)."""
import json, sys
from pathlib import Path

import numpy as np

from brain.voice.audio import SAMPLE_RATE, read_wav
from brain.voice.stt import FasterWhisperSTT

d = Path(sys.argv[1] if len(sys.argv) > 1 else "brain/runs/voice_debug")
stt = FasterWhisperSTT("/models/whisper-turbo-ct2", "cpu", bias="full")   # "full" so that stt.hotwords is really set (the default is prompt-only)
full = (stt.prompt, stt.hotwords)
HINT = "Alpha. Bravo. Room A. Room B. A. B. Yes. No. Stop."        # what the live loop uses while "Which room: A or B?" is pending
# name: (initial_prompt, hotwords, silence prepended s, silence appended s, vad_filter)
cfgs = {"no_bias": (None, None, 0, 0, True), "prompt_only": (full[0], None, 0, 0, True), "full": (*full, 0, 0, True),
        "lead0.3": (*full, 0.3, 0, True), "tail0.5": (full[0], None, 0, 0.5, True), "no_vad": (full[0], None, 0, 0, False),
        "question_prompt": (HINT, None, 0, 0, True), "norm": (full[0], None, 0, 0, True), "norm+tail": (full[0], None, 0, 0.5, True)}
import os
if os.environ.get("VOICE_AB_CONFIGS"):                    # e.g. VOICE_AB_CONFIGS=prompt_only,norm,norm+tail (CPU Whisper is slow)
    cfgs = {k: v for k, v in cfgs.items() if k in os.environ["VOICE_AB_CONFIGS"].split(",")}
rows = [json.loads(l) for l in (d / "index.jsonl").read_text().splitlines()] if (d / "index.jsonl").exists() else [{"wav": p.name} for p in sorted(d.glob("*.wav"))]
for r in rows:
    audio = read_wav(d / r["wav"])
    print(f"\n{r['wav']}  live: {r.get('text')!r} conf {r.get('confidence')}  ({len(audio) / SAMPLE_RATE:.1f} s, rms {r.get('rms', 0):.3f})")
    n = int(0.1 * SAMPLE_RATE)
    lv = [20 * np.log10(np.sqrt(np.mean(audio[i:i + n] ** 2)) + 1e-9) for i in range(0, len(audio) - n + 1, n)]
    print("  level per 0.1 s (dB), last 8:", [round(x) for x in lv[-8:]], "| floor", round(min(lv)))
    for name, (pr, hw, lead, tail, vad) in cfgs.items():
        stt.prompt, stt.hotwords, stt.vad_filter = pr, hw, vad
        a = np.concatenate([np.zeros(int(lead * SAMPLE_RATE), np.float32), audio, np.zeros(int(tail * SAMPLE_RATE), np.float32)])
        if name.startswith("norm"):                       # peak-normalise quiet clips (the mic here peaks around -26 dB)
            a = a * min(10 ** (20 / 20), 0.7 / max(float(np.abs(a).max()), 1e-4))
        t = stt.transcribe(a)
        print(f"  {name:12s} {t.text!r:40s} conf {t.confidence if t.confidence is None else round(t.confidence, 2)}")
