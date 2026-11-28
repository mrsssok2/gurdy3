"""FrameSaver (body RGB frames for VLM replay, detector_node `frame_dir`): sim-time rate limit, frame cap, disk guard, sidecar."""
import io
import json

import numpy as np
import pytest

from gurdy_core.vision.frame_saver import FrameSaver as _FrameSaver


def FrameSaver(*a, **kw):                  # pytest's tmp_path is on a 3.9 GB tmpfs here: below the 5 GB guard (tested separately)
    kw.setdefault("free_gb", lambda: 100.0)
    return _FrameSaver(*a, **kw)


def rgb(v=0):
    a = np.zeros((240, 320, 3), np.uint8)
    a[:, :160] = (200, 30, 30)            # left half red, right half blue: the channel order must survive
    a[:, 160:] = (30, 30, 200)
    return a


def test_rate_limit_in_sim_time_and_sidecar(tmp_path):
    s = FrameSaver(tmp_path / "frames", every_s=1.0)
    cam = [1.0, 2.0, 0.02, -0.5, 0.5, -0.5, 0.5]
    det = [{"cls": "barrel", "score": 0.9, "box": [1, 2, 3, 4], "color": "blue", "p": [4.9, 1.2, 0.0], "truncated": False}]
    saved = [s.maybe_save(t, rgb(), cam, det) for t in (10.0, 10.4, 10.99, 11.0, 12.5)]
    assert [x is not None for x in saved] == [True, False, False, True, True] and s.saved == 3
    side = json.loads((tmp_path / "frames" / "f_0000010.000.json").read_text())
    assert side == {"t": 10.0, "wh": [320, 240], "cam": cam, "detections": det}
    from PIL import Image
    im = np.asarray(Image.open(io.BytesIO((tmp_path / "frames" / "f_0000010.000.jpg").read_bytes())))
    assert im.shape == (240, 320, 3) and im[120, 40, 0] > 150 and im[120, 280, 2] > 150      # red stays red, blue stays blue


def test_sim_time_reset_still_saves(tmp_path):
    s = FrameSaver(tmp_path, every_s=1.0)
    assert s.maybe_save(50.0, rgb()) and s.maybe_save(0.2, rgb())        # a new sim (clock back to 0) is not "too soon"


def test_cap_and_disk_guard(tmp_path):
    s = FrameSaver(tmp_path / "a", every_s=0.0, max_frames=2)
    assert [bool(s.maybe_save(t, rgb())) for t in (1, 2, 3)] == [True, True, False] and s.stopped == "max_frames 2 reached"
    d = FrameSaver(tmp_path / "b", free_gb=lambda: 1.0)
    assert d.maybe_save(1.0, rgb()) is None and "GB free" in d.stopped
    assert not any((tmp_path / "b").iterdir())


def test_png_option(tmp_path):
    s = FrameSaver(tmp_path, png=True)
    p = s.maybe_save(3.0, rgb())
    assert p.endswith(".png") and open(p, "rb").read(8) == b"\x89PNG\r\n\x1a\n"


def test_detector_node_and_launch_are_wired_off_by_default():
    from pathlib import Path
    root = Path(__file__).resolve().parents[2]
    node = (root / "ros2_ws/src/gurdy_perception/gurdy_perception/detector_node.py").read_text()
    launch = (root / "ros2_ws/src/gurdy_bringup/launch/sim.launch.py").read_text()
    assert 'P("frame_dir", "")' in node and "self.frames.maybe_save(t, rgb, t_xyz + q, frame_dets)" in node
    assert 'DeclareLaunchArgument("detector_frames", default_value=""' in launch
    keys = {"cls", "score", "box", "color", "p", "truncated"}                   # the sidecar fields brain/vlm/frames_from_saved reads
    assert all(f'"{k}"' in node.split("frame_dets.append(")[1].split(")")[0] + node.split("frame_dets.append(")[1][:400] for k in keys)
