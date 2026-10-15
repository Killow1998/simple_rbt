#!/usr/bin/env python3
"""Prepare pinned ROBOTIS sources; never used by the installed runtime."""
import argparse
import hashlib
import json
from pathlib import Path
import urllib.request

ROOT = Path(__file__).resolve().parent.parent
MANIFEST = ROOT / "assets/manifests/turtlebot3_waffle_pi_manifest.json"


def prepare():
    manifest = json.loads(MANIFEST.read_text())
    revision = manifest["upstream"]["revision"]
    base = "https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/" + revision + "/"
    for item in manifest["files"]:
        destination = ROOT / item["file"]
        if destination.exists():
            data = destination.read_bytes()
        else:
            request = urllib.request.Request(base + item["upstream"], headers={"User-Agent": "simple_rbt-asset-preparation"})
            with urllib.request.urlopen(request, timeout=60) as response:
                data = response.read()
        if len(data) != item["bytes"] or hashlib.sha256(data).hexdigest() != item["sha256"]:
            raise RuntimeError("Pinned TurtleBot3 asset differs: " + str(destination))
        if not destination.exists():
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(data)
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--export", action="store_true", help="Also update only this model and its catalog entry")
    options = parser.parse_args()
    manifest = prepare()
    if options.export:
        from turtlebot3_model import export_model
        export_model(ROOT)
    print(json.dumps({"robot": manifest["robot"], "revision": manifest["upstream"]["revision"], "files": len(manifest["files"])}))
