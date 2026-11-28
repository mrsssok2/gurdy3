"""vision/cone_check.py retrofit: v3.1 -> v3.2 copy where only the scenes with a mini cone get the fixed (normals) mesh and
are marked for re-rendering; idempotent (resume-safe). Synthetic mini dataset, no Gazebo."""
import json
from pathlib import Path

from vision import cone_check as CC


def fake_dataset(root: Path) -> Path:
    src = root / "datasets/synth_fake"
    for i, small in ((0, [{"cls": "mini_cone", "name": "sm0_mini_cone", "colour": "orange"}]), (1, [{"cls": "can", "name": "sm0_can", "colour": "blue"}])):
        (src / "worlds").mkdir(parents=True, exist_ok=True)
        (src / "worlds" / f"scene_{i:04d}.json").write_text(json.dumps({"index": i, "split": "train", "small": small}))
        (src / "worlds" / f"scene_{i:04d}.sdf").write_text(f"<uri>file:///home/ubuntu/gurdy2/datasets/synth_fake/assets/meshes/s{i:04d}/x.obj</uri>")
        md = src / "assets/meshes" / f"s{i:04d}"
        md.mkdir(parents=True)
        (md / f"{small[0]['name']}_p1.obj").write_text("# old\nv 0 0 0\n")
        for sub in ("images/train", "labels/train", "meta/train", "depth/train"):
            (src / sub).mkdir(parents=True, exist_ok=True)
            (src / sub / f"s{i:04d}_v00.png").write_text("x")
    (src / "rendered.txt").write_text("scene_0000\nscene_0001\n")
    return src


def test_retrofit_marks_only_cone_scenes_and_is_idempotent(tmp_path):
    src = fake_dataset(tmp_path)
    dst = tmp_path / "datasets/synth_fake32"
    assert CC.retrofit(src, dst) == [0]
    assert not list(dst.glob("*/*/s0000_v*")) and len(list(dst.glob("*/*/s0001_v*"))) == 4
    assert (dst / "rendered.txt").read_text().split() == ["scene_0001"]
    obj = (dst / "assets/meshes/s0000/sm0_mini_cone_p1.obj").read_text()
    assert "\nvn " in obj and "//" in obj                                    # normals: renders its colour
    assert (dst / "assets/meshes/s0001/sm0_can_p1.obj").read_text() == "# old\nv 0 0 0\n"
    assert "synth_fake32/assets" in (dst / "worlds/scene_0000.sdf").read_text()
    assert (src / "rendered.txt").read_text().split() == ["scene_0000", "scene_0001"]   # the source is untouched
    (dst / "images/train/s0000_v00.png").write_text("re-rendered")                # render stage ran ...
    (dst / "rendered.txt").write_text("scene_0001\nscene_0000\n")
    assert CC.retrofit(src, dst) == [0]                                         # ... a second call changes nothing
    assert (dst / "images/train/s0000_v00.png").exists() and "scene_0000" in (dst / "rendered.txt").read_text()
