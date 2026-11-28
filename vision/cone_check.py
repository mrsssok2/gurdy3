#!/usr/bin/env python3
"""Mini-cone colour check (2026-11-28). The lathe-cone OBJ rendered WHITE in Gazebo whatever the SDF <material> said, so
det_v3_s / det_v31_s were trained on white mini cones. This builds small check sets from EXISTING test scenes that contain a
mini cone (default: synth_v31_body test split), identical except for the mini-cone mesh files:

    Result of the 2-scene trial (docs/results/data/cone_check/variants.png): normals alone fix it, the .mtl alone does not.

    white    the meshes as rendered for training (plain OBJ)
    normals  + per-vertex normals
    mtl      + an .mtl with the variant colour (Kd)
    both     normals + .mtl

    python3 vision/cone_check.py make --variant both --out datasets/cone_check_both [--src datasets/synth_v31_body] [--n 30]
    then (sim image): python3 vision/generate_dataset.py render --out DIR --labels v3.1 ; ... finalize --out DIR
    then: vision/color_eval.py DIR (colour), vision/eval_detector.py / recall_by_distance.py (recall)
"""
import argparse
import json
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from vision import objects as O  # noqa: E402
from vision.small_objects import CATALOG, COL  # noqa: E402

VARIANTS = {"white": (False, False), "normals": (True, False), "mtl": (False, True), "both": (True, True)}


def cone_size():
    return next(size for kind, size, pos, rgb in CATALOG["mini_cone"]["visual"]((1, 1, 1)) if kind == "mesh_cone")


def scenes_with_cones(src: Path, split: str = "test") -> list:
    out = []
    for f in sorted((src / "worlds").glob("scene_*.json")):
        d = json.loads(f.read_text())
        if d.get("split") == split and any(o["cls"] == "mini_cone" for o in d.get("small", [])):
            out.append(d["index"])
    return out


def make(src: Path, dst: Path, variant: str, n: int, ids=None) -> list:
    normals, mtl = VARIANTS[variant]
    ids = ids or scenes_with_cones(src)[:n]
    (dst / "worlds").mkdir(parents=True, exist_ok=True)
    for name in ("classes.json",):
        if (src / name).exists():
            shutil.copy(src / name, dst / name)
    if (src / "assets" / "textures").exists() and not (dst / "assets" / "textures").exists():
        shutil.copytree(src / "assets" / "textures", dst / "assets" / "textures")
    r0, h0, ht = cone_size()
    src_c, dst_c = f"/datasets/{src.name}/", f"/datasets/{dst.name}/"      # absolute mesh URIs (container paths)
    for i in ids:
        sc = json.loads((src / "worlds" / f"scene_{i:04d}.json").read_text())
        sdf = (src / "worlds" / f"scene_{i:04d}.sdf").read_text().replace(src_c, dst_c)
        (dst / "worlds" / f"scene_{i:04d}.json").write_text(json.dumps(sc))
        (dst / "worlds" / f"scene_{i:04d}.sdf").write_text(sdf)
        md_src, md = src / "assets" / "meshes" / f"s{i:04d}", dst / "assets" / "meshes" / f"s{i:04d}"
        if md.exists():
            shutil.rmtree(md)
        shutil.copytree(md_src, md)
        for o in sc.get("small", []):
            if o["cls"] != "mini_cone":
                continue
            obj = md / f"{o['name']}_p1.obj"
            assert obj.exists(), obj
            if variant == "white":
                continue                                           # exactly the training mesh
            m = f"{o['name']}_p1.mtl" if mtl else None
            obj.write_text(O.cone_obj(r0, h0, ht, normals=normals, mtl=m))
            if m:
                (md / m).write_text(O.mtl_text(COL[o["colour"]]))
    return ids


def retrofit(src: Path, dst: Path) -> list:
    """dst = a copy of the dataset `src` in which every scene with a mini cone gets the FIXED (normals) cone mesh and is marked
    for re-rendering (its frames / labels / meta / depth removed, its line dropped from rendered.txt). Idempotent and
    resume-safe: the marker retrofit.json is written last; a second call after it exists changes nothing.
    Then: generate_dataset.py render --out dst --labels v3.1 (renders only the missing scenes) + finalize."""
    marker = dst / "retrofit.json"
    if marker.exists():
        return json.loads(marker.read_text())["scenes"]
    if not dst.exists():
        shutil.copytree(src, dst, ignore=shutil.ignore_patterns("logs", "*.cache"))
    src_c, dst_c = f"/datasets/{src.name}/", f"/datasets/{dst.name}/"      # absolute mesh URIs (container paths)
    r0, h0, ht = cone_size()
    ids = []
    for jf in sorted((dst / "worlds").glob("scene_*.json")):
        sc = json.loads(jf.read_text())
        sdf_f = jf.with_suffix(".sdf")
        sdf_f.write_text(sdf_f.read_text().replace(src_c, dst_c))
        cones = [o for o in sc.get("small", []) if o["cls"] == "mini_cone"]
        if not cones:
            continue
        i = sc["index"]
        ids.append(i)
        for o in cones:
            (dst / "assets" / "meshes" / f"s{i:04d}" / f"{o['name']}_p1.obj").write_text(O.cone_obj(r0, h0, ht, normals=True))
        for f in dst.glob(f"*/*/s{i:04d}_v*"):                    # images / labels / meta / depth of this scene
            f.unlink()
    done = dst / "rendered.txt"
    if done.exists():
        drop = {f"scene_{i:04d}" for i in ids}
        done.write_text("".join(f"{x}\n" for x in done.read_text().split() if x not in drop))
    marker.write_text(json.dumps({"src": str(src.name), "scenes": ids, "note": "mini-cone mesh with normals (coloured)"}))
    return ids


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    m = sub.add_parser("make")
    m.add_argument("--variant", choices=list(VARIANTS), required=True)
    m.add_argument("--out", required=True)
    m.add_argument("--src", default="datasets/synth_v31_body")
    m.add_argument("--n", type=int, default=30)
    m.add_argument("--ids", default="", help="comma-separated scene indices (default: the first --n test scenes with a cone)")
    r = sub.add_parser("retrofit", help="copy a dataset, fix the cone mesh, mark the cone scenes for re-rendering")
    r.add_argument("--src", required=True)
    r.add_argument("--dst", required=True)
    a = ap.parse_args()
    if a.cmd == "retrofit":
        got = retrofit((ROOT / a.src).resolve(), (ROOT / a.dst).resolve())
        print(f"retrofit {a.src} -> {a.dst}: {len(got)} scenes with a mini cone to re-render")
        return
    ids = [int(x) for x in a.ids.split(",") if x] or None
    got = make((ROOT / a.src).resolve(), (ROOT / a.out).resolve(), a.variant, a.n, ids)
    print(f"{a.variant}: {len(got)} scenes -> {a.out}: {got}")


if __name__ == "__main__":
    main()
