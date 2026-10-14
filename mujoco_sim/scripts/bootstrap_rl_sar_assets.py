#!/usr/bin/env python3
"""Install only hash-pinned model/controller assets listed by a local manifest.

Use --source-repo and --zoo-repo to copy from existing Git object stores. Without
these options files are fetched from raw.githubusercontent.com at frozen commits.
No existing file is replaced when its hash differs. No complete repository or
Torch runtime is installed. Model meshes remain ignored local vendor assets.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import urllib.request

ROOT = Path(__file__).resolve().parents[2]


def install(manifest_path, source_repo=None, zoo_repo=None):
    manifest = json.loads(Path(manifest_path).read_text())
    destination = ROOT / ".vendor" / manifest["vendor_directory"]
    local = {"rl_sar": source_repo, "rl_sar_zoo": zoo_repo}
    count = total = 0
    for entry in manifest["files"]:
        target = destination / entry["path"]
        if not target.resolve().is_relative_to(destination.resolve()):
            raise ValueError("Manifest destination escapes vendor directory")
        if target.exists():
            raw = target.read_bytes()
        else:
            source = manifest["sources"][entry["source"]]
            if local.get(entry["source"]):
                raw = subprocess.run([
                    "git", "-C", str(local[entry["source"]]), "show",
                    source["commit"] + ":" + entry["source_path"],
                ], check=True, capture_output=True).stdout
            else:
                repository = source["url"].removeprefix("https://github.com/").removesuffix(".git")
                url = f'https://raw.githubusercontent.com/{repository}/{source["commit"]}/{entry["source_path"]}'
                with urllib.request.urlopen(url, timeout=60) as response:
                    raw = response.read()
        if len(raw) != entry["bytes"] or hashlib.sha256(raw).hexdigest() != entry["sha256"]:
            raise ValueError(f"Asset differs from pinned source; left unchanged: {target}")
        if not target.exists():
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(raw)
        count += 1
        total += len(raw)
    return {"directory": str(destination), "files": count, "bytes": total}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=ROOT / "mujoco_sim/assets/go2w_manifest.json")
    parser.add_argument("--source-repo", type=Path)
    parser.add_argument("--zoo-repo", type=Path)
    args = parser.parse_args()
    print(json.dumps(install(args.manifest, args.source_repo, args.zoo_repo), indent=2))


if __name__ == "__main__":
    main()
