"""One-off: synthesise the short WAVs in brain/tests/data/voice with Piper (CPU). Run in the vision image:
scripts/gpu_lock.sh run gurdy3 -- docker run --rm --entrypoint python3 -v $PWD:/w -v ~/p5_models:/m -v ~/p5_pkgs:/pk -v ~/p5_tts:/t -e PYTHONPATH=/pk:/t -w /w gurdy2/vision:cuda scripts/make_voice_test_wavs.py"""
import sys
import numpy as np
sys.path.insert(0, "/w")
from brain.voice.audio import resample, write_wav
from piper import PiperVoice

PHRASES = {"stop": "Stop.", "halt": "Halt!", "walk_forward": "Walk forward two meters.",
           "long_then_stop": "Please walk forward slowly and then turn to the left, and stop.",
           "question": "Where are you?"}
v = PiperVoice.load("/m/tts/piper-lessac.onnx", use_cuda=False)
for name, text in PHRASES.items():
    a = np.concatenate([c.audio_float_array for c in v.synthesize(text)]).astype(np.float32)
    a = resample(a, v.config.sample_rate)
    a *= 0.5 / np.abs(a).max()
    write_wav(f"/w/brain/tests/data/voice/{name}.wav", a)
    print(name, len(a) / 16000)
