#!/usr/bin/env python3
"""Prepare locked build dependencies for the native MyGo console; no runtime Python."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile
import urllib.request

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def copy_notice(source, destination):
    if destination.is_file() and source.read_bytes() == destination.read_bytes():
        return
    if destination.exists():
        destination.chmod(destination.stat().st_mode | 0o200)
    shutil.copyfile(source, destination)
    destination.chmod(destination.stat().st_mode | 0o200)


def unpack(url, sha, target, top):
    if target.is_dir():
        return
    target.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="gui-prepare-", dir=ROOT / "build") as temporary:
        stage = Path(temporary)
        archive = stage / "source.tar.gz"
        with urllib.request.urlopen(url, timeout=120) as response, archive.open("wb") as output:
            shutil.copyfileobj(response, output)
        if digest(archive) != sha:
            raise RuntimeError("Dependency archive checksum mismatch: " + url)
        with tarfile.open(archive) as bundle:
            for member in bundle.getmembers():
                resolved = (stage / member.name).resolve()
                if not resolved.is_relative_to(stage.resolve()):
                    raise RuntimeError("Dependency archive path escapes its preparation directory")
                if member.issym() or member.islnk():
                    link = (resolved.parent if member.issym() else stage) / member.linkname
                    if not link.resolve().is_relative_to(stage.resolve()):
                        raise RuntimeError("Dependency archive link escapes its preparation directory")
            bundle.extractall(stage, filter="data")
        (stage / top).rename(target)


def apply_patches(target, patches):
    for entry in patches:
        patch = ROOT / "third_party/mygo" / entry["file"]
        if digest(patch) != entry["sha256"]:
            raise RuntimeError("GUI patch checksum mismatch: " + str(patch))
        sources = [(target / source["path"], source) for source in entry["sources"]]
        if all(path.is_file() and digest(path) == source["after_sha256"] for path, source in sources):
            continue
        if not all(path.is_file() and digest(path) == source["before_sha256"] for path, source in sources):
            raise RuntimeError("GUI patch source differs from its pinned version: " + entry["file"])
        subprocess.run(["patch", "--batch", "--forward", "-p1", "--input", str(patch)],
                       cwd=target, check=True)
        if not all(digest(path) == source["after_sha256"] for path, source in sources):
            raise RuntimeError("GUI patched source checksum mismatch: " + entry["file"])


def dependencies(env, go):
    subprocess.run([str(go), "mod", "download", "all"], cwd=ROOT / "console", env=env, check=True)
    result = subprocess.run([str(go), "list", "-m", "-json", "all"],
                            cwd=ROOT / "console", env=env, check=True, capture_output=True, text=True)
    decoder, cursor, modules = json.JSONDecoder(), 0, []
    while cursor < len(result.stdout):
        while cursor < len(result.stdout) and result.stdout[cursor].isspace():
            cursor += 1
        if cursor == len(result.stdout):
            break
        module, cursor = decoder.raw_decode(result.stdout, cursor)
        modules.append(module)
    notices = ROOT / "console/licenses"
    notices.mkdir(parents=True, exist_ok=True)
    for module in modules:
        if module.get("Main"):
            continue
        module_dir = module.get("Replace", module).get("Dir")
        if not module_dir or not Path(module_dir).is_dir():
            raise RuntimeError("Module source missing: " + module["Path"])
        directory = Path(module_dir)
        label = module["Path"].replace("/", "_")
        destination = notices / label
        destination.mkdir(exist_ok=True)
        for source in directory.rglob("*"):
            if source.is_file() and source.name.upper().startswith(("LICENSE", "COPYING", "NOTICE", "OFL")):
                relative = source.relative_to(directory)
                copied = destination / relative
                copied.parent.mkdir(parents=True, exist_ok=True)
                copy_notice(source, copied)
    standard = notices / "go"
    standard.mkdir(exist_ok=True)
    copy_notice(go.parent.parent / "LICENSE", standard / "LICENSE")
    (notices / "modules.json").write_text(json.dumps([
        {key: module[key] for key in ("Path", "Version", "Sum") if key in module}
        for module in modules if not module.get("Main")], indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", action="store_true")
    parser.add_argument("--output", type=Path, default=ROOT / "build/desktop/simple-rbt-console")
    args = parser.parse_args()
    lock = json.loads((ROOT / "third_party/mygo/manifest.json").read_text())
    version = lock["go"]["version"]
    toolchain = ROOT / "build/toolchains" / version
    (ROOT / "build").mkdir(exist_ok=True)
    unpack("https://go.dev/dl/" + lock["go"]["filename"], lock["go"]["sha256"],
           toolchain / "go", "go")
    unpack(lock["archive_url"], lock["archive_sha256"], ROOT / ".vendor/mygo", "mygo-" + lock["commit"])
    apply_patches(ROOT / ".vendor/mygo", lock.get("patches", []))
    go = toolchain / "go/bin/go"
    env = dict(os.environ, CGO_ENABLED="0", GOTOOLCHAIN="local",
               GOMODCACHE=str(ROOT / "build/go-modules"), GOCACHE=str(ROOT / "build/go-cache"))
    dependencies(env, go)
    if args.build:
        args.output.resolve().parent.mkdir(parents=True, exist_ok=True)
        version = (ROOT / "packaging/VERSION").read_text().strip()
        subprocess.run([str(go), "build", "-trimpath", f"-ldflags=-s -w -X simple-rbt/console/session.Version={version}", "-o", str(args.output.resolve()), "."],
                       cwd=ROOT / "console", env=env, check=True)
    print(go)


if __name__ == "__main__":
    main()
