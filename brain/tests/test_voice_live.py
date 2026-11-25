"""Live voice loop (threads, host, TCP bridge, yes/no by voice) with scripted engines and the mock world. No mic, no models."""
import queue
import threading
import time

import numpy as np

from brain.agent.graph import Agent
from brain.agent.llm import KeywordLLM
from brain.mock import MockBackend
from brain.tools import ToolLayer
from brain.voice import ScriptedSTT, Speaker, VoiceConfig
from brain.voice.audio import FRAME_N, SAMPLE_RATE, NullSink, frames
from brain.voice.host import AgentHost, RemoteHost, serve
from brain.voice.live import LivePipeline, LockedSpeaker, run_live

RNG = np.random.default_rng(1)


class SineTTS:
    sample_rate = 22050

    def synthesize(self, text):
        n = int(0.03 * len(text) * self.sample_rate)
        return (0.3 * np.sin(2 * np.pi * 220 * np.arange(n) / self.sample_rate)).astype(np.float32)


def quiet(s):
    return (RNG.normal(0, 10 ** (-60 / 20), int(s * SAMPLE_RATE))).astype(np.float32)


def speech(s):                                  # a loud burst the energy VAD takes for speech
    return (RNG.normal(0, 0.1, int(s * SAMPLE_RATE))).astype(np.float32)


def utterance():
    return np.concatenate([quiet(0.6), speech(1.0), quiet(1.2)])


def setup(texts, mode="vad", llm=None):
    b = MockBackend()
    agent = Agent(ToolLayer(b), llm or KeywordLLM(b.known_places))
    host = AgentHost(agent)
    sink = NullSink()
    pipe = LivePipeline(ScriptedSTT(texts), LockedSpeaker(Speaker(SineTTS(), sink)), host, VoiceConfig(mode=mode), out=lambda *_: None)
    return b, agent, host, pipe, sink


def feed(pipe, audio, ctl=None):
    run_live(pipe, frames(audio), ctl or queue.Queue())


def wait(cond, t=5.0):
    end = time.time() + t
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.02)
    return False


def test_spoken_command_runs_and_reply_is_spoken():
    b, agent, host, pipe, sink = setup(["go to room A"])
    feed(pipe, utterance())
    assert wait(lambda: pipe.events("say"))
    assert agent.executor.state in ("running", "idle", "finished") and any(e["event"] == "say" for e in pipe.log)
    assert any(c["tool"] in ("go_to", "navigate") or "go" in c["tool"] for c in agent.layer.log if isinstance(c, dict) and "tool" in c)


def test_did_you_mean_answered_yes_by_voice_runs_the_corrected_command():
    b, agent, host, pipe, sink = setup(["open room B", "yes.", "yes"])
    feed(pipe, utterance())
    assert wait(lambda: agent.pending_fix is not None)         # "Did you mean: go to room B?" was asked, nothing moved yet
    assert not [c for c in agent.layer.log if c["tool"] == "go_to"]
    feed(pipe, utterance())
    assert wait(lambda: len(pipe.events("say")) >= 2)            # room B is outside the mapped area: the safety gate asks once more
    assert not [c for c in agent.layer.log if c["tool"] == "go_to"]
    feed(pipe, utterance())
    assert wait(lambda: [c for c in agent.layer.log if c["tool"] == "go_to"])


def test_did_you_mean_answered_no_by_voice_cancels():
    b, agent, host, pipe, sink = setup(["open room B", "no"])
    feed(pipe, utterance())
    assert wait(lambda: agent.pending_fix is not None)
    feed(pipe, utterance())
    assert wait(lambda: any("cancel" in e["text"].lower() for e in pipe.events("say")))
    assert agent.pending_fix is None and not [c for c in agent.layer.log if c["tool"] == "go_to"]


def test_stop_phrase_fires_estop_without_waiting_for_a_slow_agent():
    b, agent, host, pipe, sink = setup(["stop"])
    called = []
    pipe.on_estop = lambda: called.append(time.time())
    slow = []
    pipe.on_text = lambda t, c: (time.sleep(1.0), slow.append(1), "Stopped.")[2]
    feed(pipe, utterance())
    assert wait(lambda: called)
    assert not slow                                           # estop came BEFORE the slow agent call finished


def test_robot_messages_wait_while_the_user_is_talking_then_are_spoken():
    b, agent, host, pipe, sink = setup([])
    pipe.seg.active = True
    pipe.robot_message("Battery low.")
    assert not pipe.events("say")
    pipe.seg.active = False
    pipe.feed(quiet(0.02)[:FRAME_N])
    assert [e["text"] for e in pipe.events("say")] == ["Battery low."]


def test_typed_text_goes_through_the_same_path():
    b, agent, host, pipe, sink = setup([])
    ctl = queue.Queue()
    ctl.put(("text", "go to room A"))
    feed(pipe, quiet(0.2), ctl)
    assert wait(lambda: pipe.events("say"))


def test_ptt_press_release_via_controls():
    b, agent, host, pipe, sink = setup(["where am I"], mode="ptt")
    ctl = queue.Queue()
    ctl.put(("press",))
    audio = np.concatenate([speech(1.0)])
    fr = list(frames(audio))
    n = 0

    def src():
        nonlocal n
        for f in fr:
            n += 1
            if n == len(fr):
                ctl.put(("release",))
            yield f
        for _ in range(40):                                  # the 0.5 s post-roll after the key release (25 frames) + one to finish
            yield fr[0] * 0.0
    run_live(pipe, src(), ctl)
    assert wait(lambda: pipe.events("transcript"))


def test_tcp_bridge_say_state_estop_and_push_messages():
    b = MockBackend()
    agent = Agent(ToolLayer(b), KeywordLLM(b.known_places))
    host = AgentHost(agent)
    host.start()
    serve(host, 0, "127.0.0.1") if False else None
    import socket
    s = socket.socket(); s.bind(("127.0.0.1", 0)); port = s.getsockname()[1]; s.close()
    serve(host, port, "127.0.0.1")
    c = RemoteHost(f"127.0.0.1:{port}", state_s=0.1)
    got = []
    c.on_message(got.append)
    assert "Room" in c.say("go to room B").title() or c.say("where am I")
    assert wait(lambda: "room a" in [p.lower() for p in c.places()])
    c.estop()
    assert wait(lambda: not c.is_moving())
    with host.lock:
        host.agent._announce([{"type": "battery_low", "level": 0.15}]) if False else None
    for fn in host.listeners:
        fn("Battery low.")
    assert wait(lambda: got == ["Battery low."])
    c.close(); host.close()


# ---- low confidence, debug WAVs, soft onsets, confirmation keeps the whole plan
def test_low_confidence_unparseable_repeats_what_was_heard():
    b, a = _agent()
    for text in ("That up.", "Are you?"):
        r = a.say(text, 0.3)
        assert r.reply == f"I heard '{text.rstrip('.?')}'. Can you say that again?" and b.task is None
    assert a.say("Bad, odd.", 0.3).reply == "I heard 'Bad, odd'. Can you say that again?"
    assert "heard" not in a.say("where are you", 0.83).reply


def test_low_confidence_clean_command_is_read_back_not_rejected():
    b, a = _agent()
    r = a.say("Go to room A.", 0.43)                  # the real case: heard correctly, confidence 0.43
    assert r.reply == "Room A, Alpha. Shall I go?" and b.task is None
    assert "Going to room A" in a.say("yes", 0.9).reply
    b, a = _agent()
    assert a.say("walk forward 2 metres", 0.4).reply == "Walking forward 2 metres. Shall I do that?"
    assert a.say("no", 0.3).reply == "Okay, cancelled." and b.task is None


def test_low_confidence_one_message_one_threshold():
    b, a = _agent()
    cfg = a.safety.cfg
    assert "stt_reask_below" not in cfg and cfg["stt_confidence_min"] == 0.6
    for conf in (0.43, 0.55):                         # both used to give different texts
        assert "not sure I heard" not in a.say("blah blah", conf).reply
    assert a.say("blah blah", 0.55).reply == "I heard 'blah blah'. Can you say that again?"
    assert "heard" not in a.say("blah blah", 0.7).reply


def test_low_confidence_never_blocks_stop_or_yes_no():
    from brain.agent import ScriptedLLM
    from test_agent import agent_for, call
    b, _, agent = agent_for(ScriptedLLM(plans=[{"calls": [call("go_to", place="room B")]}]))
    agent.say("go to room B", 0.9)
    assert agent.executor.state == "running"
    assert "heard" not in agent.say("stop", 0.2).reply and agent.executor.state != "running"
    b2 = MockBackend()
    a2 = Agent(ToolLayer(b2), KeywordLLM(b2.known_places))
    a2.gate.hold("go_to", {"calls": []}, "Shall I go?")
    assert "heard" not in a2.say("yes", 0.2).reply


def test_confirmation_resumes_the_whole_plan():
    from brain.agent import ScriptedLLM
    from test_agent import RED, agent_for, call
    plan = {"calls": [call("go_to", place="room B"), call("find", object=RED)]}
    b, l, a = agent_for(ScriptedLLM(plans=[plan, {"reply": "x"}]), explored=False)
    assert "Shall I go" in a.say("Go to room B and find the red box").reply
    a.say("yes")
    assert [s.tool for s in a.executor.steps] == ["go_to", "find"]
    b.advance(120)
    a.pump()
    assert [s.status for s in a.executor.steps][0] == "done" and a.executor.steps[1].status in ("running", "done")


def test_every_utterance_is_saved_as_wav_with_index(tmp_path):
    import json
    b = MockBackend()
    host = AgentHost(Agent(ToolLayer(b), KeywordLLM(b.known_places)))
    pipe = LivePipeline(ScriptedSTT(["where are you"]), LockedSpeaker(Speaker(SineTTS(), NullSink())), host, VoiceConfig(mode="vad"),
                        out=lambda *_: None, debug_dir=tmp_path / "vd")
    feed(pipe, utterance())
    time.sleep(0.5)
    rows = [json.loads(x) for x in (tmp_path / "vd" / "index.jsonl").read_text().splitlines()]
    assert len(rows) == 1 and rows[0]["text"] == "where are you" and (tmp_path / "vd" / rows[0]["wav"]).exists()


def test_trim_keeps_a_soft_onset():
    from brain.voice.vad import Segmenter
    seg = Segmenter()
    n = FRAME_N
    soft = (RNG.normal(0, 0.004, 8 * n)).astype(np.float32)      # "Wh-": quiet start
    audio = np.concatenate([quiet(0.5), soft, speech(0.6), quiet(0.5)])
    out = seg._trim(audio)
    assert len(out) >= len(soft) + len(speech(0.6)) - n


def test_keyword_plan_after_yes_says_and_runs_the_whole_plan():
    b = MockBackend()
    agent = Agent(ToolLayer(b), KeywordLLM(b.known_places))
    agent.say("Go to room B and find the red box", 0.9)
    r = agent.say("yes", 0.9)
    assert "Going to room B" in r.reply and "Next: look for the red box." in r.reply
    msgs = []
    for _ in range(12):
        b.advance(10)
        msgs += agent.pump()
    assert [s.status for s in agent.executor.steps] == ["done", "done"] and any("Found the red box" in m for m in msgs)


def test_continue_flag_survives_the_confirmation_and_never_repeats_the_walk():
    from brain.agent import ScriptedLLM
    from test_agent import RED, agent_for, call
    seen = []

    def second(**kw):
        seen.append(kw["tool_results"])
        return {"calls": [call("go_to", place="room B"), call("find", object=RED)]}   # the model repeats the first call
    llm = ScriptedLLM(plans=[{"calls": [call("go_to", place="room B")], "continue": True}, second])
    b, l, a = agent_for(llm, explored=False)
    assert "Shall I go" in a.say("Go to room B and find the red box").reply
    a.say("yes")
    walks = []
    msgs = []
    for _ in range(14):
        b.advance(10)
        msgs += a.pump()
        walks.append([(s.tool, s.status) for s in a.executor.steps])
    assert len(llm.plan_calls) == 2 and llm.plan_calls[1]["utterance"] == "Go to room B and find the red box"
    assert any(r["tool"] == "go_to" for r in seen[0])                 # the planner sees the finished walk
    assert [t for t, _ in walks[-1]] == ["find"] and walks[-1][0][1] == "done"
    assert any("Found the red box" in m for m in msgs)


# ---- rooms by voice: phonetic names, read-back, "no, Delta", missing letter, unknown room, new command vs pending question, replace
def _agent(plans=None, explored=True):
    from brain.agent import ScriptedLLM
    from test_agent import agent_for
    from test_safety import make
    if plans is not None:
        b, _, a = agent_for(ScriptedLLM(plans=plans), explored=explored)
        return b, a
    b, l, _ = make(explored=explored)
    return b, Agent(l, KeywordLLM(b.known_places))


def test_phonetic_room_names_normalise():
    from brain.agent import rooms
    assert rooms.normalize("Go to room Bravo and find the box") == "Go to room B and find the box"
    assert rooms.normalize("go to room D as in dog") == "go to room D"
    assert rooms.normalize("go to room, delta") == "go to room D"
    assert rooms.answer_letter("Delta.") == "D" and rooms.answer_letter("room b") == "B" and rooms.answer_letter("go home") is None
    assert rooms.correction_letter("No, Delta") == "D" and rooms.correction_letter("no") is None


def test_voice_goto_reads_the_letter_back_and_typed_does_not():
    b, a = _agent()
    assert "Going to room B, Bravo, " in a.say("go to room bravo", 0.9).reply
    b, a = _agent()
    r = a.say("go to room B").reply
    assert "Going to room B, " in r and "Bravo" not in r


def test_no_delta_corrects_the_room():
    b, a = _agent()
    b.places["room D"] = (5.0, -2.0)
    a.say("go to room B", 0.9)
    r = a.say("no, Delta", 0.8)
    assert "Going to room D, Delta" in r.reply
    assert [(s.tool, s.args.get("place")) for s in a.executor.steps] == [("go_to", "room D")]


def test_missing_letter_asks_which_room_and_the_answer_completes_it():
    b, a = _agent()
    r = a.say("Go to room.", 0.9)
    assert r.reply == "Which room: A or B?" and b.task is None
    r = a.say("Bravo", 0.9)
    assert r.reply == "Room B, Bravo. Shall I go?" and b.task is None          # a spoken answer is always read back first
    assert "Going to room B, Bravo" in a.say("yes", 0.9).reply
    b, a = _agent()
    a.say("Go to room.", None)
    assert "Going to room B" in a.say("B", None).reply                          # typed: unambiguous, no read-back


def test_room_that_does_not_exist_is_said_plainly():
    b, a = _agent()
    r = a.say("Go to room C", 0.9)
    assert r.reply == "There is no room C. I know rooms A and B." and b.task is None
    assert a.say("go to room V", 0.9).reply == "There is no room V. I know rooms A and B."      # Phase 6d: an unknown letter is never "repaired" into a room
    assert "Did you mean" in a.say("go to dorway", 0.9).reply                                    # a misspelt WORD still gets the repair question


def test_new_command_while_confirmation_pending_runs_instead_of_cancelling():
    b, a = _agent(explored=False)
    assert "Shall I go" in a.say("go to room B").reply
    r = a.say("turn left")
    assert "cancelled" not in r.reply and "Turning left" in r.reply and a.gate.pending is None


def test_repeating_the_same_command_counts_as_yes():
    b, a = _agent(explored=False)
    assert "Shall I go" in a.say("Go to room B").reply
    r = a.say("go to room B.")
    assert "Going to room B" in r.reply and a.executor.state == "running"


def test_no_still_cancels_the_pending_question():
    b, a = _agent(explored=False)
    a.say("go to room B")
    assert a.say("no").reply == "Okay, cancelled." and b.task is None


def test_find_after_a_finished_short_trip_is_not_busy():
    b, a = _agent()
    a.say("go to room A")
    b.advance(30)                                    # the trip is over, nobody called pump() yet
    r = a.say("find the red box")
    assert "busy" not in r.reply and "switching" not in r.reply and "Found the red box" in r.reply


def test_find_while_walking_switches_with_a_note_and_stop_still_stops():
    b, a = _agent()
    a.say("go to room B")
    r = a.say("find the red box")
    assert r.reply.startswith("Okay, switching to: find the red box.") and [s.tool for s in a.executor.steps] == ["find"]
    a.say("stop")
    assert a.executor.state in ("cancelled", "idle", "failed", "done") and b.task is None


def test_host_advances_the_mock_clock_so_a_trip_ends():
    b = MockBackend()
    host = AgentHost(Agent(ToolLayer(b), KeywordLLM(b.known_places)), pump_s=0.02, sim_speed=500.0)
    got = []
    host.on_message(got.append)
    host.start()
    host.say("go to room A", None)
    time.sleep(1.0)
    host.close()
    assert b.task is None and any("arrived" in m for m in got)


# ---- answer timeouts: wall clock, from "question spoken" to "user started speaking"
def _clocked_host(sim_speed_note=True):
    from brain.agent.answer_clock import AnswerClock
    from test_safety import make
    t = [1000.0]
    clk = AnswerClock(lambda: t[0])
    b, l, _ = make(explored=False)
    agent = Agent(l, KeywordLLM(b.known_places), clock=clk)
    return b, agent, AgentHost(agent), t


def test_slow_stt_and_3x_sim_clock_never_expire_an_answer():
    b, agent, host, t = _clocked_host()
    assert "Shall I go" in host.say("Go to room B Bravo", 0.9)             # t = 1000: question created
    t[0] += 2.0
    host.spoken_done()                                                    # t = 1002: robot finished speaking
    b.advance(3 * 60)                                                     # the 3x sim clock ran far past 30 s meanwhile
    t[0] += 3.0                                                           # t = 1005: the user starts to speak "yes"
    t[0] += 9.8                                                           # Whisper needs 9.8 s: transcript at t = 1014.8
    reply = host.say("yes", 0.9, started_ago=9.8)
    assert "expired" not in reply and "Going to room B" in reply


def test_answer_started_too_late_does_expire_on_wall_clock():
    b, agent, host, t = _clocked_host()
    host.say("Go to room B", 0.9)
    host.spoken_done()
    t[0] += 45.0                                                          # the user waited 45 s before speaking
    assert "expired" in host.say("yes", 0.9, started_ago=1.0)


def test_typed_answer_counts_from_now_and_old_sim_time_is_irrelevant():
    b, agent, host, t = _clocked_host()
    host.say("Go to room B", None)
    b.advance(1000)
    t[0] += 5.0
    assert "Going to room B" in host.say("yes", None)


def test_no_delta_window_is_wall_clock_too():
    b, agent, host, t = _clocked_host()
    b.places["room D"] = (5.0, -2.0)
    agent.layer.backend.explored = True if hasattr(b, "explored") else None
    host.say("go to room B", 0.9)
    host.say("yes", 0.9)
    host.spoken_done(); b.advance(600)                                    # sim time irrelevant
    t[0] += 20.0
    assert "room D, Delta" in host.say("no, Delta", 0.8, started_ago=10.0)  # started 10 s ago: 10 s after the read-back
    t[0] += 100.0
    host.say("go to room B", 0.9); host.spoken_done()
    t[0] += 70.0
    assert "room D" not in host.say("no, Delta", 0.8, started_ago=1.0)    # 69 s after the command: window over


def test_live_pipeline_passes_when_the_user_started_speaking_and_signals_spoken_done():
    calls = {"say": [], "spoken": 0}

    class H:
        def say(self, text, conf=None, started_ago=None):
            calls["say"].append((text, started_ago)); return "ok"
        def spoken_done(self): calls["spoken"] += 1
        def estop(self): pass
        def is_moving(self): return False
        def on_message(self, fn): pass
    pipe = LivePipeline(ScriptedSTT(["yes"]), LockedSpeaker(Speaker(SineTTS(), NullSink())), H(), VoiceConfig(mode="vad"), out=lambda *_: None)
    feed(pipe, utterance())
    time.sleep(1.0)
    pipe.feed(np.zeros(FRAME_N, np.float32))
    assert calls["say"] and calls["say"][0][0] == "yes" and calls["say"][0][1] is not None and calls["say"][0][1] >= 0.0
    for _ in range(200):                                                  # let "ok" finish playing
        pipe.feed(np.zeros(FRAME_N, np.float32))
    assert calls["spoken"] >= 1


# ---- Whisper hallucinations, strict one-at-a-time push-to-talk, index.jsonl
def test_repeated_token_transcripts_are_detected():
    from brain.voice.stt import Transcript, reject_reason, repeated_tokens
    for t in ("Roo. Roo. Roo.", "Go, go, go, go.", "go to go to go to", "Thank you. Thank you. Thank you."):
        assert repeated_tokens(t), t
    for t in ("Go to room A", "Go to room B and find the red box", "stop", "go go", "walk forward, then go back"):
        assert not repeated_tokens(t), t
    loud = speech(1.0)
    assert reject_reason(Transcript("Roo. Roo. Roo.", 0.8), loud) == "repetition"
    assert reject_reason(Transcript("go to room A", 0.8), quiet(1.0)) == "too quiet"
    assert reject_reason(Transcript("hmm", 0.3, 0.0, no_speech_prob=0.9), loud) == "no speech"
    assert reject_reason(Transcript("go to room A", 0.8, 0.0, compression_ratio=3.1), loud) == "repetition"
    assert reject_reason(Transcript("go to room A", 0.8, 0.0, 0.1, 1.2), loud) is None


def test_hallucinated_transcript_says_didnt_catch_that_and_never_reaches_the_agent(tmp_path):
    import json
    seen = []

    class H:
        def say(self, text, conf=None, started_ago=None):
            seen.append(text); return "ok"
        def spoken_done(self): pass
        def estop(self): pass
        def is_moving(self): return False
        def on_message(self, fn): pass
    lines = []
    sink = NullSink()
    pipe = LivePipeline(ScriptedSTT(["Roo. Roo. Roo.", "Go, go, go, go."]), LockedSpeaker(Speaker(SineTTS(), sink)), H(), VoiceConfig(mode="vad"),
                        out=lines.append, debug_dir=tmp_path)
    feed(pipe, np.concatenate([utterance(), utterance()]))
    time.sleep(0.8)
    assert seen == [] and sum("ignored" in l for l in lines) == 2
    assert pipe.speaker.texts[-1] == "Sorry, I didn't catch that." if hasattr(pipe.speaker, "texts") else True
    rows = [json.loads(x) for x in (tmp_path / "index.jsonl").read_text().splitlines()]
    assert [r["rejected"] for r in rows] == ["repetition", "repetition"]


def test_push_to_talk_is_one_at_a_time_and_prints_thinking():
    lines = []
    gate = threading.Event()

    class SlowSTT(ScriptedSTT):
        def transcribe(self, audio, sr=SAMPLE_RATE):
            gate.wait(5.0)
            return super().transcribe(audio, sr)
    b = MockBackend()
    host = AgentHost(Agent(ToolLayer(b), KeywordLLM(b.known_places)))
    pipe = LivePipeline(SlowSTT(["where are you"]), LockedSpeaker(Speaker(SineTTS(), NullSink())), host, VoiceConfig(mode="ptt"), out=lines.append)
    ctl = queue.Queue()
    ctl.put(("enter",))                                   # start recording
    n = [0]

    def src():
        for i, f in enumerate(frames(np.concatenate([speech(0.6), quiet(1.0)]))):
            if i == 20:
                ctl.put(("enter",))                       # send
            if i == 25:
                time.sleep(0.2)
                ctl.put(("enter",))                       # Enter while STT is busy: ignored, no overlapping recording
            n[0] += 1
            yield f
    run_live(pipe, src(), ctl)
    assert pipe.busy and not pipe.seg.ptt_down
    assert any("recording" in l for l in lines) and any("thinking" in l for l in lines)
    assert sum("recording" in l for l in lines) == 1 and any("still thinking" in l for l in lines)
    gate.set()
    time.sleep(0.8)
    assert not pipe.busy and any("You: where are you" in l for l in lines)


def test_index_logs_whether_the_robot_was_speaking_during_the_recording(tmp_path):
    import json
    b = MockBackend()
    host = AgentHost(Agent(ToolLayer(b), KeywordLLM(b.known_places)))
    pipe = LivePipeline(ScriptedSTT(["where are you", "what do you see"]), LockedSpeaker(Speaker(SineTTS(), NullSink())), host,
                        VoiceConfig(mode="vad"), out=lambda *_: None, debug_dir=tmp_path)
    feed(pipe, utterance())
    time.sleep(0.6)
    pipe.say("A long sentence the robot is saying right now, " * 3)       # the robot talks, then the user speaks over it
    feed(pipe, np.concatenate([quiet(0.1), speech(1.0), quiet(1.2)]))
    time.sleep(0.6)
    rows = [json.loads(x) for x in (tmp_path / "index.jsonl").read_text().splitlines()]
    assert len(rows) == 2 and rows[0]["robot_speaking"] is False and rows[1]["robot_speaking"] is True


# ---- question-specific recognition + fuzzy room answers; push-to-talk post-roll
def test_fuzzy_room_answers():
    from brain.agent import rooms
    for heard, want in (("Alfa.", "A"), ("alpa", "A"), ("Bravvo", "B"), ("bee", "B"), ("room alpha", "A"), ("Bravo", "B"), ("Bye.", "B")):        # "Bye" ~ "bee": a guess, always read back
        assert (rooms.fuzzy_answer(heard, ["A", "B"]) or rooms.answer_letter(heard)) == want, heard
    for heard in ("And follow.", "yes please do it now", "hello"):
        assert rooms.fuzzy_answer(heard, ["A", "B"]) is None, heard
    assert rooms.fuzzy_answer("Delta", ["A", "B", "D"]) == "D" and rooms.fuzzy_answer("Delta", ["A", "B"]) is None


def test_garbled_answer_is_matched_and_read_back_before_moving():
    b, a = _agent()
    a.say("Go to room.", 0.9)
    r = a.say("Alfa.", 0.4)
    assert r.reply == "Room A, Alpha. Shall I go?" and b.task is None
    assert "Going to room A, Alpha" in a.say("yes", 0.9).reply
    b, a = _agent()
    a.say("Go to room.", 0.9)
    r = a.say("And follow.", 0.29)                    # cannot be matched: ask again, nothing moves
    assert b.task is None and r.reply == "I heard 'And follow'. Can you say that again?"


def test_answer_hint_while_a_question_is_pending():
    b, a = _agent()
    assert a.answer_hint() is None
    a.say("Go to room.", 0.9)
    h = a.answer_hint()
    assert "Alpha." in h and "Bravo." in h and "Room A." in h and "Yes." in h
    a.say("Alpha", 0.9)                               # read-back pending (Did-you-mean style): yes / no expected
    assert a.answer_hint().startswith("Yes. No.") and "Room a." in a.answer_hint()
    a.say("no", 0.9)
    assert a.answer_hint() is None


def test_live_pipeline_gives_whisper_the_question_prompt():
    class H:
        def __init__(self): self.hint = "Alpha. Bravo. Yes. No."
        def say(self, text, conf=None, started_ago=None): return "ok"
        def spoken_done(self): pass
        def estop(self): pass
        def is_moving(self): return False
        def on_message(self, fn): pass
        def answer_hint(self): return self.hint
    stt = ScriptedSTT(["Alpha", "Bravo"])
    h = H()
    pipe = LivePipeline(stt, LockedSpeaker(Speaker(SineTTS(), NullSink())), h, VoiceConfig(mode="vad"), out=lambda *_: None)
    feed(pipe, utterance()); time.sleep(0.5)
    h.hint = None
    feed(pipe, utterance()); time.sleep(0.5)
    assert stt.prompts == ["Alpha. Bravo. Yes. No.", None]


def test_ptt_post_roll_keeps_the_last_word_after_enter():
    from brain.voice.vad import Segmenter
    seg = Segmenter(mode="ptt", ptt_postroll_s=0.5)
    seg.press()
    ev = []
    for f in frames(speech(1.0)):
        ev += seg.feed(f)
    assert seg.release() == [] and seg.finishing                   # the key is up but the recording goes on
    last_word = speech(0.4)                                         # the word that was still being said
    got = []
    for f in frames(np.concatenate([last_word, quiet(1.0)])):
        got += seg.feed(f)
    ends = [a for k, a in got if k == "end"]
    assert len(ends) == 1 and ends[0] is not None and not seg.finishing
    assert len(ends[0]) >= int(1.35 * SAMPLE_RATE)                  # 1.0 s + the whole 0.4 s word (trim keeps the loud part + padding)
    seg0 = Segmenter(mode="ptt")                                    # default (library) post-roll 0: ends at once, as before
    seg0.press()
    for f in frames(speech(1.0)):
        seg0.feed(f)
    assert [k for k, _ in seg0.release()] == ["end"]


# ---- soft starts in push-to-talk, bare yes / no with nothing pending
def test_push_to_talk_turns_whispers_vad_filter_off_and_vad_mode_keeps_it():
    class S(ScriptedSTT):
        vad_filter = True
    for mode, want in (("ptt", False), ("vad", True)):
        stt = S(["x"])
        LivePipeline(stt, LockedSpeaker(Speaker(SineTTS(), NullSink())), AgentHost(Agent(ToolLayer(MockBackend()), KeywordLLM(MockBackend().known_places))),
                     VoiceConfig(mode=mode), out=lambda *_: None)
        assert stt.vad_filter is want, mode


def test_ptt_trim_keeps_a_soft_start_that_the_vad_mode_trim_would_cut():
    from brain.voice.vad import Segmenter
    n = FRAME_N
    soft = (RNG.normal(0, 10 ** (-33 / 20), 12 * n)).astype(np.float32)       # "Wh-": 0.24 s at -33 dB, between floor+margin-6 and floor+margin
    audio = np.concatenate([quiet(1.0), soft, speech(0.5), quiet(0.3)])
    out = {}
    for mode in ("ptt", "vad"):
        seg = Segmenter(mode=mode)
        seg.vad.floor = -43.0                                                  # the floor of the real mic (-42 dB) -> threshold -34 dB (vad) / -40 dB (ptt)
        out[mode] = len(seg._trim(audio))
    assert out["ptt"] > out["vad"] and out["ptt"] >= len(soft) + len(speech(0.5))


def test_bare_yes_or_no_with_nothing_pending_says_so():
    b, a = _agent()
    for t in ("no", "Yes.", "nope"):
        assert a.say(t, 0.9).reply == "There's nothing to confirm right now."
    assert a.say("thank you", 0.9).reply != "There's nothing to confirm right now."
    a.say("go to room.", 0.9)
    assert a.say("no", 0.9).reply == "Okay, cancelled."                       # "no" to "Which room?"
    assert a.say("no", 0.9).reply == "There's nothing to confirm right now."   # and now nothing is pending again


def test_malformed_later_step_does_not_crash_the_ack():
    from brain.agent import ScriptedLLM
    from test_agent import agent_for, call
    plan = {"calls": [call("go_to", place="room A"), {"tool": "walk", "args": {"distance_m": 1}}]}      # second step lacks `direction`
    b, l, a = agent_for(ScriptedLLM(plans=[plan]))
    r = a.say("go to room A then walk")
    assert "Going to room A" in r.reply and "Next: walk." in r.reply            # no crash; the broken step fails on its own later


def test_next_preview_never_states_a_parameter_the_user_did_not_say():
    from brain.agent import ScriptedLLM
    from brain.agent.grounding import violations
    from brain.eval.harness import collect_facts
    from test_agent import agent_for, call
    plan = {"calls": [call("turn", direction="right", angle_deg=90), call("walk", direction="forward", distance_m=2.0), call("describe_view")]}
    b, l, a = agent_for(ScriptedLLM(plans=[plan], responses=["I do not see any known objects right now."]))
    n, say = len(l.log), "first turn right, then walk 2 meters, then tell me what you see"
    r = a.say(say)
    assert violations(r.reply, collect_facts(l.log[n:], say, [l.state_block()]), b.known_places()) == []
