"""Phase 9b: small pickable objects (models, mass properties, arm grasp data), lab4_fetch world, v3 dataset profiles."""
import json
import math
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "gurdy_core"))
sys.path.insert(0, str(ROOT / "scripts"))

from gurdy_core.vision.classes import CLASSES, CLASSES_V3, SIZE_PRIOR_SMALL, SMALL_CLASSES  # noqa: E402
from vision import small_objects as SO  # noqa: E402


def test_class_ids_old_unchanged_new_appended():
    assert CLASSES == ("box", "barrel", "fire_extinguisher", "traffic_cone", "toolbox", "mannequin", "chair")
    assert CLASSES_V3[:7] == CLASSES and CLASSES_V3[7:] == SMALL_CLASSES
    assert set(SO.CATALOG) == set(SMALL_CLASSES) == set(SIZE_PRIOR_SMALL)


@pytest.mark.parametrize("cls", SMALL_CLASSES)
def test_catalog_matches_arm_grasp_data(cls):
    np = pytest.importorskip("numpy")  # noqa: F841  (gurdy_core.arm needs numpy)
    from gurdy_core import arm as A
    c, p = SO.CATALOG[cls], A.PICKABLE[cls]
    assert c["mass"] == pytest.approx(p["mass"])
    assert tuple(c["size"]) == pytest.approx(SIZE_PRIOR_SMALL[cls])
    if p["shape"] == "box":
        assert tuple(c["size"]) == pytest.approx(tuple(p["size"]))
    else:
        assert c["size"][2] == pytest.approx(p["height"]) and c["size"][0] == pytest.approx(2 * p["radius"])
    # the width the jaws close on, measured on the collision solids at the grasp height
    zg = A.grasp_point(p)
    widths = [2 * s[0] if k == "cylinder" else s[1] for k, s, pos in c["collision"]
              if abs(zg - pos[2]) <= (s[1] if k == "cylinder" else s[2]) / 2 + 1e-9]
    assert widths, f"{cls}: nothing to grip at z = {zg}"
    assert min(widths) == pytest.approx(p["grip_w"], abs=0.004)


def test_toolbox_handle_leaves_room_for_the_fingers():
    from gurdy_core import arm as A
    p = A.PICKABLE["mini_toolbox"]
    a = A.load_arm()
    tips = A.grasp_point(p) - (a.L_tip - a.L3)             # finger tips this far below the grasp centre
    body_top = max(pos[2] + s[2] / 2 for k, s, pos in SO.CATALOG["mini_toolbox"]["collision"][:1])
    assert tips - body_top >= 0.009


@pytest.mark.parametrize("cls", SMALL_CLASSES)
def test_mass_properties_physical(cls):
    m, com, I = SO.mass_properties(cls)
    assert m == pytest.approx(SO.CATALOG[cls]["mass"])
    h = SO.CATALOG[cls]["size"][2]
    assert 0 < com[2] < h and abs(com[0]) < 0.01 and abs(com[1]) < 1e-12
    assert min(I) > 0 and I[0] + I[1] >= I[2] and I[1] + I[2] >= I[0] and I[0] + I[2] >= I[1]
    # sanity vs the bounding box (inertia of a solid filling the bounding box is an upper bound)
    sx, sy, sz = SO.CATALOG[cls]["size"]
    assert I[2] <= m * (sx * sx + sy * sy) / 12 * 1.01


def test_model_sdf_is_dynamic_with_friction_and_label(tmp_path):
    for cls, c in SO.CATALOG.items():
        for colour in c["variants"]:
            xml = SO.model_sdf("t", cls, colour, label=9, mesh_dir=str(tmp_path))
            m = ET.fromstring(xml)
            assert m.find("static").text == "false"
            assert float(m.find("link/inertial/mass").text) == pytest.approx(c["mass"])
            cols = m.findall("link/collision")
            assert len(cols) == len(c["collision"])
            assert all(float(cc.find("surface/friction/ode/mu").text) >= 1.0 for cc in cols)
            assert m.find("plugin/label").text == "9"
    assert len({v for c in SO.CATALOG.values() for v in c["variants"]}) >= 8          # colours shared across classes


def test_model_dirs(tmp_path):
    ds = SO.write_model_dirs(tmp_path)
    assert len(ds) == sum(len(c["variants"]) for c in SO.CATALOG.values())
    for d in ds:
        ET.parse(d / "model.sdf")
        ET.parse(d / "model.config")


# ------------------------------------------------------------------ lab4_fetch
def test_lab4_fetch_valid_and_up_to_date():
    import gen_world_lab4 as L4
    import gen_world_lab4_fetch as F
    assert F.validate() == [] and L4.validate() == []
    r = subprocess.run([sys.executable, str(ROOT / "scripts/gen_world_lab4_fetch.py"), "--check"], cwd=ROOT)
    assert r.returncode == 0, "lab4_fetch files out of date: run scripts/gen_world_lab4_fetch.py"
    r = subprocess.run([sys.executable, str(ROOT / "scripts/gen_world_lab4.py"), "--check"], cwd=ROOT)
    assert r.returncode == 0, "lab4 itself changed"


def test_lab4_fetch_world_and_gt():
    root = ET.parse(ROOT / "ros2_ws/src/gurdy_gazebo/worlds/lab4_fetch.sdf").getroot()
    assert root.find("world").get("name") == "lab4_fetch"
    names = {m.get("name") for m in root.iter("model")}
    gt = json.loads((ROOT / "docs/results/data/lab4_fetch/lab4_fetch_gt.json").read_text())
    small = [o for o in gt["objects"] if o["pickable"]]
    big = [o for o in gt["objects"] if not o["pickable"]]
    assert len(small) == 8 and len(big) == 10
    for o in small:
        assert o["model"] in names and o["room"] and o["class"] in SMALL_CLASSES
        assert o["z"] == pytest.approx(o["size"][2] / 2)
    assert {o["room"] for o in small} == {"room A", "room B", "room C", "room D"}
    lab4 = ET.parse(ROOT / "ros2_ws/src/gurdy_gazebo/worlds/lab4.sdf").getroot()
    assert {m.get("name") for m in lab4.iter("model")} <= names                       # every lab4 model is still there


# ------------------------------------------------------------------ v3 dataset profiles
@pytest.fixture(scope="module")
def ss():
    pytest.importorskip("numpy")
    pytest.importorskip("PIL")
    from vision import scene_small
    return scene_small


@pytest.mark.parametrize("profile", ["v3-body", "v3-wrist"])
def test_v3_scene_deterministic_and_labelled(ss, profile, tmp_path):
    a = ss.sample_scene_v3(11, 12, profile)
    b = ss.sample_scene_v3(11, 12, profile)
    assert json.dumps(a) == json.dumps(b)
    assert a["small"] and len(a["views"]) >= 10
    xml = ss.world_sdf_v3(a, str(tmp_path))
    root = ET.fromstring(xml)
    labels = {int(p.find("label").text) for p in root.iter("plugin") if p.get("name") == "gz::sim::systems::Label"}
    small_ids = {CLASSES_V3.index(o["cls"]) + 1 for o in a["small"]}
    assert small_ids <= labels and all(8 <= i <= 12 for i in small_ids)
    cams = [m for m in root.iter("model") if m.get("name") == "cam"]
    assert len(cams) == 1
    fov = float(cams[0].find("link/sensor/camera/horizontal_fov").text)
    if profile == "v3-wrist":
        assert fov == pytest.approx(1.2) and xml.count('name="finger') == 2
    else:
        assert fov == pytest.approx(1.047) and "finger" not in xml


def test_v3_views_see_small_objects(ss):
    n_small = n_views = 0
    for seed in range(5):
        for profile in ("v3-body", "v3-wrist"):
            sc = ss.sample_scene_v3(seed, 12, profile)
            for v in sc["views"]:
                n_views += 1
                n_small += sum(g["cls"] in SMALL_CLASSES for g in ss.gt_for_view_v3(sc, v))
    assert n_small / n_views > 0.8


def test_v1_v2_scenes_unchanged_by_the_v3_module(ss):
    """Importing / using scene_small must not touch the v1 / v2 random streams (old datasets stay reproducible)."""
    from vision import scene as S
    before = [json.dumps(S.sample_scene(s, 12, None, p)) for s in (0, 5) for p in ("v1", "v2")]
    ss.sample_scene_v3(3, 12, "v3-body")
    after = [json.dumps(S.sample_scene(s, 12, None, p)) for s in (0, 5) for p in ("v1", "v2")]
    assert before == after
