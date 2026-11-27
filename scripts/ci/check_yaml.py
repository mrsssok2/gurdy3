#!/usr/bin/env python3
"""CI: every git-tracked YAML file must parse (yaml.safe_load_all), be non-empty and have NO duplicate mapping keys.

PyYAML silently keeps the LAST of two equal keys, so a duplicated ROS parameter or test case id would be
lost without any error; this loader turns that into a failure. Only file names + line numbers are printed
(never file contents). Blind / held-out suites (any path containing "blind" or "heldout") are NEVER opened:
they are evaluation material that must stay unseen, so CI does not read them at all.

    python3 scripts/ci/check_yaml.py            # all tracked *.yaml / *.yml
    python3 scripts/ci/check_yaml.py FILE ...   # given files
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
NEVER_OPEN = ("blind", "heldout")        # path substrings; these files are not read, not even parsed


class UniqueKeyLoader(yaml.SafeLoader):
    pass


def _construct_mapping(loader, node, deep=False):
    seen: dict = {}
    for key_node, _ in node.value:            # explicit keys only: a key that overrides a `<<:` merge is legal
        if key_node.tag == "tag:yaml.org,2002:merge":
            continue
        key = loader.construct_object(key_node, deep=deep)
        try:
            hash(key)
        except TypeError:
            continue
        if key in seen:
            raise yaml.constructor.ConstructorError(
                None, None, f"duplicate key {key!r} (first at line {seen[key] + 1})", key_node.start_mark)
        seen[key] = key_node.start_mark.line
    return yaml.SafeLoader.construct_mapping(loader, node, deep=deep)


UniqueKeyLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping)


def tracked_yaml() -> list[Path]:
    out = subprocess.run(["git", "ls-files", "-z", "*.yaml", "*.yml"], cwd=ROOT, capture_output=True, check=True).stdout
    return [ROOT / p for p in out.decode().split("\0") if p and not any(w in p.lower() for w in NEVER_OPEN)]


def check(path: Path) -> str | None:
    try:
        docs = list(yaml.load_all(path.read_text(encoding="utf-8"), Loader=UniqueKeyLoader))
    except yaml.MarkedYAMLError as e:
        m = e.problem_mark or e.context_mark
        where = f"line {m.line + 1} col {m.column + 1}" if m else "?"
        return f"{where}: {e.problem or e.context}"
    except (yaml.YAMLError, UnicodeDecodeError) as e:
        return type(e).__name__
    if not docs or all(d is None for d in docs):
        return "empty document"
    return None


def main(argv: list[str]) -> int:
    files = [Path(a).resolve() for a in argv] if argv else tracked_yaml()
    bad = 0
    for f in files:
        err = check(f)
        if err:
            bad += 1
            print(f"YAML FAIL {f.relative_to(ROOT) if f.is_relative_to(ROOT) else f}: {err}")
    print(f"yaml: {len(files) - bad}/{len(files)} files OK")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
