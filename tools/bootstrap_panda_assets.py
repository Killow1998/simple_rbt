#!/usr/bin/env python3
"""Prepare pinned Panda assets; no Python is used by the simulator."""
import hashlib
import json
from pathlib import Path
import urllib.request

ROOT = Path(__file__).resolve().parents[1]


def prepare():
    manifest = json.loads((ROOT / "assets/manifests/panda_manifest.json").read_text())
    base = "https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/" + manifest["upstream"]["revision"] + "/"
    for entry in manifest["files"]:
        path = ROOT / entry["file"]
        if path.exists():
            data = path.read_bytes()
        else:
            with urllib.request.urlopen(base + entry["upstream"], timeout=60) as response:
                data = response.read()
        if len(data) != entry["bytes"] or hashlib.sha256(data).hexdigest() != entry["sha256"]:
            raise RuntimeError("Pinned Panda asset differs: " + str(path))
        if not path.exists():
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
    print("Panda assets verified:", len(manifest["files"]))


if __name__ == "__main__":
    prepare()
