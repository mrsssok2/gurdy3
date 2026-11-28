"""Waits measured in SIM time (2026-11-28): with the Gazebo GUI the sim runs at RTF ~0.2, so a 3 s arm trajectory takes ~15 s
of wall time and a wall-clock timeout of `seconds + 10` gave up before the arm got there ("pre-grasp move failed" x3 in the
GUI demo take; headless at RTF ~0.7 it fit). Sim time comes from the latest /joint_states header stamp; a wall cap keeps a
paused or dead sim from hanging forever. Pure (clocks injected) -> tested on the host."""
import time


def wait_sim(done, sim_now, sim_timeout: float, wall_cap: float | None = None, poll: float = 0.02,
             wall_now=time.monotonic, sleep=time.sleep) -> bool:
    """Poll done() until it is true (-> True), sim_timeout seconds of SIM time passed, or wall_cap seconds of wall time
    passed (-> False). Without a sim clock (sim_now() None) sim_timeout is used as wall time (the old behaviour)."""
    wall_cap = wall_cap if wall_cap is not None else max(120.0, 20.0 * sim_timeout)
    t_w0, t_s0 = wall_now(), sim_now()
    while True:
        if done():
            return True
        s = sim_now()
        if t_s0 is None and s is not None:
            t_s0 = s
        elapsed_sim = (s - t_s0) if (s is not None and t_s0 is not None) else (wall_now() - t_w0)
        if elapsed_sim >= sim_timeout or wall_now() - t_w0 >= wall_cap:
            return done()
        sleep(poll)
