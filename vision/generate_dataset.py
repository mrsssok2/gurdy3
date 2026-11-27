#!/usr/bin/env python3
"""Synthetic detection dataset from Gazebo Harmonic (boundingbox_camera = automatic YOLO labels).

    # 1. plan: sample scenes, write worlds + textures + meshes (fast, no simulator needed)
    python3 vision/generate_dataset.py plan   --out datasets/synth_v1 --scenes 200 --views 12 --seed 0
    # 2. render: run each world headless in Gazebo, move the camera through its views, save frames (ROS image, `sim` service)
    python3 vision/generate_dataset.py render --out datasets/synth_v1 [--start 0 --count 50]   # resumable; one worker process +
    #    one fresh Gazebo server per scene (killed afterwards), retries, aborts if > 5 % of the scenes fail; logs in <out>/logs/
    # 3. dataset.yaml (+ per-split counts) for the trainer
    python3 vision/generate_dataset.py finalize --out datasets/synth_v1

Layout:  images/{train,val,test}/s0007_v03.png  labels/<split>/*.txt (YOLO)  depth/<split>/*.png (uint16 mm)
         meta/<split>/*.json (camera pose, intrinsics, ground-truth object centres in the camera frame)
Split is BY SCENE (scene_idx % 20: 14 train / 3 val / 3 test), so no object layout is shared between splits.
Randomised per scene: room size, floor/wall colour + texture, sun + lamps + ambient + shadows, 1-6 labelled objects
(7 classes, size/colour variants), 0-8 distractors, camera height 0.10-0.30 m, pitch, roll; per frame: exposure,
Gaussian noise, motion blur (vision/augment.py). Depth is the clean Gazebo depth (same as the robot's sensor).
Label ids in Gazebo = class id + 1 (0 = unlabelled background). The render step needs gz-transport python
(`gz.transport13`, in the ROS image); it cannot run on the host Python.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "gurdy_core"))
from gurdy_core.vision.classes import CLASSES, CLASSES_V3, GZ_LABEL_OFFSET  # noqa: E402
from vision import scene as S  # noqa: E402
from vision.augment import augment_rgb  # noqa: E402
from vision.textures import write_textures  # noqa: E402

SPLIT_NAMES = [n for n, _ in S.SPLITS]


def cmd_plan(a):
    out = Path(a.out)
    (out / "worlds").mkdir(parents=True, exist_ok=True)
    (out / "assets" / "meshes").mkdir(parents=True, exist_ok=True)
    write_textures(str(out / "assets" / "textures"), S.N_TEXTURES, seed=a.seed)
    cam = {}
    if a.robot_yaml:
        import yaml
        d = yaml.safe_load(open(a.robot_yaml))["sensors"]["depth_camera"]
        cam = {k: d[k] for k in ("width", "height", "hfov", "range", "pitch_down")}
    v3 = a.profile.startswith("v3")
    if v3:                                          # Phase 9b: 12 classes; the class list travels with the dataset
        from vision import scene_small as SS
        (out / "classes.json").write_text(json.dumps(list(SS.class_names(a.profile))))
    for i in range(a.scenes):
        sc = (SS.sample_scene_v3(a.seed * 100003 + i, a.views, a.profile, cam) if v3
              else S.sample_scene(a.seed * 100003 + i, a.views, cam, a.profile))
        sc["index"], sc["split"] = i, S.split_for_scene(i)
        (out / "worlds" / f"scene_{i:04d}.json").write_text(json.dumps(sc))
        mesh_dir = out / "assets" / "meshes" / f"s{i:04d}"
        mesh_dir.mkdir(exist_ok=True)
        writer = SS.world_sdf_v3 if v3 else S.world_sdf
        (out / "worlds" / f"scene_{i:04d}.sdf").write_text(
            writer(sc, str(mesh_dir.resolve()), str((out / "assets" / "textures").resolve())))
    print(f"planned {a.scenes} scenes -> {out}/worlds  (train/val/test scenes: "
          f"{[sum(S.split_for_scene(i) == s for i in range(a.scenes)) for s in SPLIT_NAMES]})")


def _n_classes(sc) -> int:
    """Number of label ids accepted from the boundingbox camera for this scene (v3 profiles: 12, else the 7 old ones)."""
    return len(CLASSES_V3) if str(sc.get("profile", "")).startswith("v3") else len(CLASSES)


def _gt_for_view(sc, view):
    if str(sc.get("profile", "")).startswith("v3"):
        from vision import scene_small as SS
        return SS.gt_for_view_v3(sc, view)
    return S.gt_for_view(sc, view)


def gt_3d(sc: dict, gt: list) -> list:
    """Add the object's yaw and size (w, d, h in m: box footprint of the generated parts / small-object catalogue) and its scene
    record kind (object / small) to each ground-truth entry, so 3D labels can be derived later."""
    rec = {o["name"]: ("object", o) for o in sc.get("objects", [])}
    rec.update({o["name"]: ("small", o) for o in sc.get("small", [])})
    out = []
    for g in gt:
        g = dict(g)
        kind, o = rec.get(g.get("name"), (None, None))
        if o is not None:
            g["yaw"] = float(o.get("yaw", 0.0))
            g["kind"] = kind
            try:
                if kind == "small":
                    from gurdy_core.vision.classes import SIZE_PRIOR_SMALL
                    g["size"] = [float(v) for v in SIZE_PRIOR_SMALL[o["cls"]]]
                else:
                    from vision import objects as OBJ
                    g["size"] = [float(v) for v in OBJ.footprint(o["cls"], o.get("scale", 1.0), o.get("variant"))]
            except Exception:                                   # noqa: BLE001 (size is optional metadata)
                pass
        out.append(g)
    return out


def cmd_relabel(a):
    """Rewrite labels/<split>/*.txt from meta boxes_raw with another size rule (no Gazebo). Frames rendered before boxes_raw
    existed are skipped and counted."""
    out = Path(a.out)
    names = _dataset_classes(out)
    thr = S.label_thresholds(a.labels, names)
    n = skipped = changed = 0
    for mf in sorted((out / "meta").glob("*/*.json")):
        m = json.loads(mf.read_text())
        if "boxes_raw" not in m:
            skipped += 1
            continue
        W, H = m["image_wh"]
        lines = S.yolo_lines([tuple(b) for b in m["boxes_raw"]], W, H, thresholds=thr)
        lf = out / "labels" / mf.parent.name / (mf.stem + ".txt")
        new = "\n".join(lines) + ("\n" if lines else "")
        if not lf.exists() or lf.read_text() != new:
            changed += 1
        lf.write_text(new)
        m["label_rule"] = a.labels
        mf.write_text(json.dumps(m))
        n += 1
    print(f"relabel {out} -> rule {a.labels}: {n} frames, {changed} label files changed, {skipped} skipped (no boxes_raw)")


def _dataset_classes(out: Path):
    f = out / "classes.json"
    return json.loads(f.read_text()) if f.exists() else list(CLASSES)


def rpy_to_quat(r, p, y):
    """Gazebo roll-pitch-yaw (extrinsic xyz) -> quaternion (x, y, z, w)."""
    cr, sr, cp, sp, cy, sy = np.cos(r / 2), np.sin(r / 2), np.cos(p / 2), np.sin(p / 2), np.cos(y / 2), np.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


class Capture:
    """Subscribes to the camera topics via gz-transport; keeps the latest frames keyed by sim stamp (ns)."""

    def __init__(self, world: str):
        from gz.msgs10.annotated_axis_aligned_2d_box_v_pb2 import AnnotatedAxisAligned2DBox_V
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.image_pb2 import Image as GzImage
        from gz.msgs10.pose_pb2 import Pose
        from gz.transport13 import Node
        self.Pose, self.Boolean, self.world = Pose, Boolean, world
        self.node = Node()
        self.lock = threading.Lock()
        self.rgb, self.depth, self.bbox = {}, {}, {}
        self.node.subscribe(GzImage, "/cam/image", lambda m: self._put(self.rgb, m))
        self.node.subscribe(GzImage, "/cam/depth_image", lambda m: self._put(self.depth, m))
        self.node.subscribe(AnnotatedAxisAligned2DBox_V, "/cam/bbox", lambda m: self._put(self.bbox, m))
        self.latest = 0

    def _put(self, store, m):
        ns = m.header.stamp.sec * 10**9 + m.header.stamp.nsec
        with self.lock:
            store[ns] = m
            self.latest = max(self.latest, ns)
            for k in sorted(store)[:-6]:
                del store[k]

    def set_pose(self, pos, rpy) -> bool:
        qx, qy, qz, qw = rpy_to_quat(*rpy)
        req = self.Pose()
        req.name = "cam"
        req.position.x, req.position.y, req.position.z = pos
        req.orientation.x, req.orientation.y, req.orientation.z, req.orientation.w = qx, qy, qz, qw
        ok, rep = self.node.request(f"/world/{self.world}/set_pose", req, self.Pose, self.Boolean, 3000)
        return bool(ok and rep.data)

    def frame_after(self, t_ns: int, timeout: float = 15.0):
        """First (rgb, depth, bbox) triple with the same stamp > t_ns."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self.lock:
                common = sorted(k for k in self.rgb if k > t_ns and k in self.depth and k in self.bbox)
                if common:
                    k = common[0]
                    return self.rgb[k], self.depth[k], self.bbox[k]
            time.sleep(0.02)
        return None


def _img(m, channels):
    return np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, channels) if channels == 3 else \
        np.frombuffer(m.data, np.float32).reshape(m.height, m.width)


def _log(*x):
    print(*x, file=sys.stderr, flush=True)


def render_scene(sc: dict, out: Path, a) -> tuple:
    """Render one scene with its own Gazebo server. Returns (frames saved, views lost to errors). Runs in a worker process
    (`render-scene`), so a hung server or transport node can always be killed from outside."""
    idx, split = sc["index"], sc["split"]
    world = out / "worlds" / f"scene_{idx:04d}.sdf"
    os.environ["GZ_PARTITION"] = f"synth{os.getpid()}"         # unique per worker = per server launch
    (out / "logs").mkdir(exist_ok=True)
    gz_log = open(out / "logs" / f"scene_{idx:04d}.gz.log", "w")
    srv = subprocess.Popen(["gz", "sim", "-s", "-r", "--headless-rendering", "-v", "3", str(world)], env=dict(os.environ),
                           stdout=gz_log, stderr=subprocess.STDOUT)
    n_ok, n_err, rng = 0, 0, np.random.default_rng(sc["seed"] + 7)
    try:
        cap = Capture(S.WORLD_NAME)
        t0 = time.time()
        while cap.latest == 0 and time.time() - t0 < 60 and srv.poll() is None:   # first frames (server start-up)
            time.sleep(0.2)
        _log(f"first frame after {time.time() - t0:.1f} s, stamp {cap.latest}, server exit code {srv.poll()}")
        if cap.latest == 0:
            raise RuntimeError(f"no camera frames from Gazebo (server exit code {srv.poll()})")
        for vi, view in enumerate(sc["views"]):
            t1 = time.time()
            if not cap.set_pose(view["pos"], view["rpy"]):      # service call: 3 s timeout
                n_err += 1
                _log(f"view {vi}: set_pose failed after {time.time() - t1:.1f} s, server exit code {srv.poll()}, latest stamp {cap.latest}")
                if n_err >= 2:                                  # the server is gone or hung: give up, the caller relaunches
                    break
                continue
            with cap.lock:
                t_set = cap.latest
            fr = cap.frame_after(t_set + int(0.35 * 1e9))       # skip frames rendered before/while the pose changed
            if fr is None:
                n_err += 1
                _log(f"view {vi}: no synchronised frame within 15 s, server exit code {srv.poll()}, latest stamp {cap.latest}")
                if n_err >= 2:
                    break
                continue
            rgb_m, depth_m, bb = fr
            rgb, depth = _img(rgb_m, 3), _img(depth_m, 0)
            H, W = rgb.shape[:2]
            boxes = [(b.label - GZ_LABEL_OFFSET, b.box.min_corner.x, b.box.min_corner.y, b.box.max_corner.x, b.box.max_corner.y)
                     for b in bb.annotated_box if 0 <= b.label - GZ_LABEL_OFFSET < _n_classes(sc)]
            thr = S.label_thresholds(getattr(a, "labels", "v3"), _dataset_classes(out))
            lines = S.yolo_lines(boxes, W, H, thresholds=thr)
            if not lines and rng.random() > a.negative_keep:
                continue
            name = f"s{idx:04d}_v{vi:02d}"
            for sub in ("images", "labels", "depth", "meta"):
                (out / sub / split).mkdir(parents=True, exist_ok=True)
            Image.fromarray(augment_rgb(np.array(rgb), rng)).save(out / "images" / split / f"{name}.png")
            (out / "labels" / split / f"{name}.txt").write_text("\n".join(lines) + ("\n" if lines else ""))
            d_mm = np.where(np.isfinite(depth), np.clip(depth * 1000, 0, 65535), 0).astype(np.uint16)
            Image.fromarray(d_mm).save(out / "depth" / split / f"{name}.png")
            (out / "meta" / split / f"{name}.json").write_text(json.dumps({
                "scene": idx, "view": vi, "cam_pos": view["pos"], "cam_rpy": view["rpy"], "camera": sc["camera"],
                "gt": gt_3d(sc, _gt_for_view(sc, view)), "stamp_ns": rgb_m.header.stamp.sec * 10**9 + rgb_m.header.stamp.nsec,
                # ALWAYS saved (2026-11-27): the unfiltered boxes, so the label size rule can change without a re-render
                # (`relabel`); the rule used for labels/*.txt is recorded next to them.
                "image_wh": [W, H], "boxes_raw": [[int(c), float(x1), float(y1), float(x2), float(y2)] for c, x1, y1, x2, y2 in boxes],
                "label_rule": getattr(a, "labels", "v3")}))
            n_ok += 1
    finally:
        srv.terminate()
        try:
            srv.wait(5)
        except subprocess.TimeoutExpired:
            srv.kill()
    return n_ok, n_err


def cmd_render_scene(a):
    """Worker: one scene, one server. Last stdout line = 'RESULT ok err'. Exit code 0 only if no view was lost."""
    out = Path(a.out)
    sc = json.loads((out / "worlds" / f"scene_{a.index:04d}.json").read_text())
    n_ok, n_err = render_scene(sc, out, a)
    print(f"RESULT {n_ok} {n_err}", flush=True)
    os._exit(0 if n_err == 0 else 3)                           # _exit: never wait for gz-transport threads


def _remove_scene_files(out: Path, idx: int):
    for f in out.glob(f"*/*/s{idx:04d}_v*"):
        f.unlink()


def _run_worker(out: Path, idx: int, a, attempt: int):
    """Run the worker in its own process group with a hard wall-clock limit; afterwards kill the whole group
    (python worker + gz server + anything they started), so nothing survives into the next scene."""
    import signal
    log = open(out / "logs" / f"scene_{idx:04d}.try{attempt}.log", "w")
    cmd = [sys.executable, os.path.abspath(__file__), "render-scene", "--out", str(out), "--index", str(idx),
           "--negative-keep", str(a.negative_keep), "--labels", getattr(a, "labels", "v3")]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=log, text=True, start_new_session=True)
    try:
        so, _ = p.communicate(timeout=a.scene_timeout)
        why = "" if p.returncode == 0 else f"worker exit code {p.returncode}"
    except subprocess.TimeoutExpired:
        so, why = "", f"worker timeout {a.scene_timeout:.0f} s"
    finally:
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        p.wait()
    res = [ln.split()[1:] for ln in (so or "").split("\n") if ln.startswith("RESULT")]
    n_ok, n_err = (int(res[-1][0]), int(res[-1][1])) if res else (0, -1)
    return (not why and n_err == 0), n_ok, why or f"{n_err} views lost"


def cmd_render(a):
    """Resumable. Every scene (and every retry) gets a fresh worker process + fresh Gazebo server, killed afterwards;
    a scene counts only if no view was lost. Stops with exit code 2 if too many scenes fail."""
    out = Path(a.out)
    (out / "logs").mkdir(exist_ok=True)
    scenes = sorted((out / "worlds").glob("scene_*.json"))[a.start:(a.start + a.count) if a.count else None]
    done_file = out / "rendered.txt"
    done = set(done_file.read_text().split()) if done_file.exists() else set()
    todo = [p for p in scenes if p.stem not in done]
    max_fail = max(2, int(a.max_fail_frac * len(scenes)))
    print(f"render: {len(scenes)} scenes selected, {len(scenes) - len(todo)} already done, {len(todo)} to do; "
          f"stop if more than {max_fail} fail or {a.max_consecutive} fail in a row", flush=True)
    n_fail = consec = n_retry = 0
    t_start = time.time()
    for k, p in enumerate(todo):
        idx = int(p.stem.split("_")[1])
        t0 = time.time()
        for attempt in range(1, a.attempts + 1):
            _remove_scene_files(out, idx)                       # never keep frames of a broken attempt
            ok, n, why = _run_worker(out, idx, a, attempt)
            if ok:
                break
            n_retry += attempt < a.attempts
            print(f"{p.stem}: attempt {attempt}/{a.attempts} failed ({why}); server killed, relaunching", flush=True)
            time.sleep(2.0)
        if ok:
            consec = 0
            with open(done_file, "a") as fh:
                fh.write(p.stem + "\n")
            split = json.loads(p.read_text())["split"]
            print(f"{p.stem} [{split}]: {n}/{a.views_hint or '?'} frames in {time.time() - t0:.0f} s"
                  f"{'' if attempt == 1 else f' (attempt {attempt})'} | done {k + 1 - n_fail}/{len(todo)}, failed {n_fail}, "
                  f"retries {n_retry}, elapsed {(time.time() - t_start) / 60:.1f} min", flush=True)
        else:
            _remove_scene_files(out, idx)
            n_fail += 1; consec += 1
            print(f"{p.stem}: FAILED after {a.attempts} attempts ({why}) | failed scenes {n_fail} (limit {max_fail})", flush=True)
            if n_fail > max_fail or consec >= a.max_consecutive:
                print(f"ABORT: {n_fail} failed scenes ({consec} in a row) - see {out}/logs/. Finished scenes are kept; "
                      f"run the same command again to resume.", flush=True)
                sys.exit(2)
    print(f"render finished: {len(todo) - n_fail} scenes rendered, {n_fail} failed, {n_retry} retries, "
          f"{(time.time() - t_start) / 60:.1f} min", flush=True)
    if n_fail:
        sys.exit(1)


def cmd_finalize(a):
    out = Path(a.out).resolve()
    counts = {s: len(list((out / "images" / s).glob("*.png"))) for s in SPLIT_NAMES if (out / "images" / s).exists()}
    names_list = _dataset_classes(out)
    inst = np.zeros(len(names_list), int)
    for f in (out / "labels").glob("*/*.txt"):
        for ln in f.read_text().split("\n"):
            if ln.strip():
                inst[int(ln.split()[0])] += 1
    names = "\n".join(f"  {i}: {n}" for i, n in enumerate(names_list))
    (out / "dataset.yaml").write_text(f"path: {out}\ntrain: images/train\nval: images/val\ntest: images/test\nnames:\n{names}\n")
    (out / "stats.json").write_text(json.dumps({"images": counts, "instances": dict(zip(names_list, map(int, inst)))}, indent=1))
    print("images", counts, "instances", dict(zip(names_list, map(int, inst))))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("plan"); p.add_argument("--out", required=True); p.add_argument("--scenes", type=int, default=200)
    p.add_argument("--views", type=int, default=12); p.add_argument("--seed", type=int, default=0)
    p.add_argument("--profile", default="v1", choices=["v1", "v2", "v3-body", "v3-wrist"],
                   help="scene sampler profile (v2: distractor families, more mannequins; v3-*: + small pickable classes, Phase 9b)")
    p.add_argument("--robot-yaml", default=str(Path(__file__).resolve().parents[1] / "ros2_ws/src/gurdy_description/config/robot.yaml"), help="camera intrinsics source")
    p.set_defaults(fn=cmd_plan)
    p = sub.add_parser("render"); p.add_argument("--out", required=True); p.add_argument("--start", type=int, default=0)
    p.add_argument("--count", type=int, default=0); p.add_argument("--negative-keep", type=float, default=0.1,
                                                                    help="probability to keep a frame with no labelled object")
    p.add_argument("--attempts", type=int, default=3, help="tries per scene, each with a fresh server")
    p.add_argument("--scene-timeout", type=float, default=150.0, help="hard wall-clock limit per attempt (s)")
    p.add_argument("--max-fail-frac", type=float, default=0.05, help="abort if more than this share of the scenes fail")
    p.add_argument("--max-consecutive", type=int, default=3, help="abort after this many failed scenes in a row")
    p.add_argument("--views-hint", type=int, default=12, help=argparse.SUPPRESS)
    p.add_argument("--labels", default="v3", choices=list(S.LABEL_RULES),
                   help="label size rule: v3 (default, 6 px / 80 px^2 every class) | v3.1 (small classes 3 px / 12 px^2)")
    p.set_defaults(fn=cmd_render)
    p = sub.add_parser("render-scene", help="internal worker: one scene"); p.add_argument("--out", required=True)
    p.add_argument("--index", type=int, required=True); p.add_argument("--negative-keep", type=float, default=0.1)
    p.add_argument("--labels", default="v3", choices=list(S.LABEL_RULES))
    p.set_defaults(fn=cmd_render_scene)
    p = sub.add_parser("relabel", help="rewrite labels from meta boxes_raw with another size rule (no Gazebo)")
    p.add_argument("--out", required=True); p.add_argument("--labels", required=True, choices=list(S.LABEL_RULES))
    p.set_defaults(fn=cmd_relabel)
    p = sub.add_parser("finalize"); p.add_argument("--out", required=True); p.set_defaults(fn=cmd_finalize)
    a = ap.parse_args()
    a.fn(a)


if __name__ == "__main__":
    main()
