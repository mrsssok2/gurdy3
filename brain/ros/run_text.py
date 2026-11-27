"""Type commands against the real robot in Gazebo (RosBackend). Run inside the sim container after the sim is up
(see docs/results/phase5_ros_backend.md). Planner: keyword baseline by default, `--ollama MODEL` for the LLM.
  python3 -m brain.ros.run_text --places maps/flat_lab.places.yaml --map-id flat_lab
"""
from __future__ import annotations

import argparse
import os
import sys
import threading
import time


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--places", default=None)
    ap.add_argument("--map-id", default="unknown")
    ap.add_argument("--odom-topic", default="/odometry/filtered")
    ap.add_argument("--memory-db", default=None, help="SQLite file: objects / missions / places survive restarts (per --map-id)")
    ap.add_argument("--serve", type=int, default=None, metavar="PORT",
                    help="also serve the agent to a voice client (brain.voice.cli --remote HOST:PORT); the typed chat keeps working")
    ap.add_argument("--web", type=int, default=None, metavar="PORT",
                    help="also serve the mission-control dashboard (web/app) on this port, with telemetry from ROS; token = $GURDY_WEB_TOKEN or random (printed)")
    ap.add_argument("--web-bind", default="0.0.0.0", help="address the dashboard listens on inside the container (the compose / docker -p mapping decides who can reach it)")
    ap.add_argument("--speech-url", default=os.environ.get("GURDY_SPEECH_URL") or None, help="speech service for the dashboard's mic button / spoken replies")
    ap.add_argument("--tls-port", type=int, default=int(os.environ.get("GURDY_TLS_PORT") or 0) or None, help="dashboard https port (needs --tls-cert / --tls-key)")
    ap.add_argument("--tls-cert", default=os.environ.get("GURDY_TLS_CERT") or None)
    ap.add_argument("--tls-key", default=os.environ.get("GURDY_TLS_KEY") or None)
    ap.add_argument("--ca", default=os.environ.get("GURDY_TLS_CA") or None)
    ap.add_argument("--allowed-origin", action="append", default=None, metavar="URL",
                    help="a web page allowed to use this server from a browser (CORS + WebSocket Origin), e.g. https://mrsssok2.github.io; repeatable; 'none' = only the "
                         "server's own pages. Default: env GURDY_ALLOWED_ORIGINS or https://mrsssok2.github.io. Never a wildcard; the token is still required.")
    ap.add_argument("--ollama", default=None, help="model name; needs OLLAMA_URL")
    ap.add_argument("--fetch", action="store_true", help="Phase 9h: fetch commands on the real arm (needs the stack with arm:=true "
                    "object3d:=true; brain/ros/fetch_robot.py). Default off: fetch phrases go to the planner as before")
    a = ap.parse_args()
    from ..agent.graph import Agent
    from ..agent.llm import KeywordLLM
    from ..tools import ToolLayer
    from .backend import RosBackend
    from .rclpy_port import RclpyPort
    port = RclpyPort(map_id=a.map_id, odom_topic=a.odom_topic)
    while port.pose() is None:
        print("waiting for TF map->base_link ...")
        time.sleep(1.0)
    be = RosBackend(port, a.places)
    if a.memory_db:
        from ..memory import MemoryBackend, MemoryStore
        be = MemoryBackend(be, MemoryStore(a.memory_db, a.map_id))
        print(f"persistent memory: {a.memory_db} (map {a.map_id})")
    from pathlib import Path
    from ..incidents.backend import with_incidents
    be = with_incidents(be, str(Path(a.memory_db).with_name("incidents.db")) if a.memory_db else None, a.map_id)   # temporal memory; RAM-only without MEMORY
    if a.fetch:
        from ..fetch.session import FetchSession
        from .fetch_robot import RosFetchRobot
        from .rclpy_arm_port import RclpyArmPort
        arm = RclpyArmPort()
        if not arm.arm_ready():
            print("--fetch: arm_controller not answering (stack started with arm:=true?): fetch stays OFF")
        else:
            be.fetch_session = FetchSession(RosFetchRobot(be, arm))
            print(f"fetch: ON (MoveIt {'yes' if arm.moveit_available() else 'no: direct joint goals'})")
    if a.ollama:
        from ..agent.ollama_llm import OllamaLLM
        llm = OllamaLLM(model=a.ollama, constrain=not a.ollama.startswith("gurdy-sft"))   # fine-tuned model: unconstrained (phase5_eval_sft.md)
    else:
        llm = KeywordLLM(be.known_places, smart=True)
    agent = Agent(ToolLayer(be), llm)
    from .chat_ui import Chat
    chat = Chat(agent, be.known_places, prompt="> ")
    print("Gurdy (ROS). Type a command; 'status' line = '?'; '!estop' / '!release' = emergency stop latch; 'quit' to exit.")
    if a.serve or a.web:
        from ..voice.host import AgentHost, serve
        host = AgentHost(agent)
        host.lock = chat.lock                 # typed chat, voice and web share ONE agent lock
        host.start()                          # the chat's own pump thread is not started when voice / web serves (one pump, spoken + printed)
        host.on_message(lambda m: chat.say_async("Gurdy: " + m))
        if a.serve:
            serve(host, a.serve)
            print(f"voice server on :{a.serve}", flush=True)
        if a.web:
            from web.api.ros_extras import RosExtras
            from web.api.ros_telemetry import RosTelemetry
            from web.api.server import GurdyWebServer, parse_origins
            origins = parse_origins(",".join(a.allowed_origin)) if a.allowed_origin else None
            tel = RosTelemetry(host, port, RosExtras(port.node))
            from web.api.voice import SpeechClient
            speech = SpeechClient(a.speech_url) if a.speech_url else None
            srv = GurdyWebServer(host, telemetry=tel, bind=a.web_bind, port=a.web, speech=speech, tls_port=a.tls_port, tls_cert=a.tls_cert,
                                 tls_key=a.tls_key, ca_file=a.ca, allowed_origins=origins).start()
            print(f"mission control on port {srv.port} (real robot)" + (f", https on {srv.tls_port}" if srv.tls_port else "")
                  + (f", voice via {a.speech_url}" if speech else ""), flush=True)
            print(f"access token: {srv.token}", flush=True)
        if not sys.stdin.isatty():            # started without a terminal (voice / web mode of the start scripts): just serve
            threading.Event().wait()
    else:
        chat.start()                       # progress, alerts and new objects arrive on their own, without breaking what you type
    while True:
        try:
            text = input("> ").strip()
        except EOFError:
            break
        if text == "quit":
            break
        if text == "?":
            print(f"  t={port.now():.0f}s pose={port.pose()} mission={agent.executor.state}")
        elif text in ("!estop", "!release"):
            with chat.lock:
                be.estop(text == "!estop")
            print("Gurdy: " + ("Emergency stop engaged (latched)." if text == "!estop" else "Emergency stop released."))
        elif text:
            print("Gurdy:", chat.ask(text))
    chat.stop()


if __name__ == "__main__":
    main()
    import os
    import sys
    sys.stdout.flush()
    os._exit(0)          # rclpy's spin thread otherwise dies noisily at interpreter exit
