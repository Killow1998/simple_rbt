#!/usr/bin/env python3
"""Prepare a relocatable Ubuntu 22.04 amd64 package; Python is not installed."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def run(*args, **kwargs):
    return subprocess.run(args, check=True, text=True, **kwargs)


def digest(path):
    with path.open("rb") as stream:
        if hasattr(hashlib, "file_digest"):
            return hashlib.file_digest(stream, "sha256").hexdigest()
        return hashlib.sha256(stream.read()).hexdigest()


def check_runtime(stage):
    binaries = stage / "usr/bin"
    for name in ("simple-rbt-sim", "sprobo"):
        binary = binaries / name
        if not binary.is_file():
            raise RuntimeError(f"Missing built runtime: {name}")
        dynamic = run("readelf", "-d", str(binary), capture_output=True).stdout
        paths = [line for line in dynamic.splitlines()
                 if "RPATH" in line or "RUNPATH" in line]
        forbidden = ("/home/", ".venv", "/usr/local/cuda")
        if (name == "simple-rbt-sim" and not paths) or any(
                (name == "simple-rbt-sim" and "$ORIGIN/../lib/simple-rbt" not in line)
                or any(path in line for path in forbidden) for line in paths):
            raise RuntimeError(f"Non-relocatable library search path in {name}: {paths}")
        if re.search(r"Shared library: \[(libroscpp|librclcpp|librcl|librmw|libddsc|libpython)",
                     dynamic):
            raise RuntimeError(f"ROS/Python dependency leaked into host runtime: {name}")
        if name == "sprobo" and "Shared library: [libQt" in dynamic:
            raise RuntimeError("The legacy Qt console cannot be shipped as the Go/mygo console")
    data = stage / "usr/share/simple-rbt"
    catalog = json.loads((data / "assets/catalog.json").read_text())
    for model in catalog["models"].values():
        xml_path = data / "assets" / model["file"]
        if digest(xml_path) != model["sha256"]:
            raise RuntimeError(f"Model digest mismatch: {xml_path.name}")
        xml = ET.parse(xml_path).getroot()
        compiler = xml.find("compiler")
        meshdir = compiler.get("meshdir", "") if compiler is not None else ""
        for element in xml.iter():
            reference = element.get("file")
            if not reference:
                continue
            if not reference.startswith("@PROJECT_ROOT@/"):
                reference = f"{meshdir}/{reference}"
            target = data / reference.removeprefix("@PROJECT_ROOT@/")
            if not target.is_file():
                raise RuntimeError(f"Missing referenced asset: {target}")
    for controller in catalog["controllers"].values():
        if "policy" in controller:
            policy = data / controller["policy"]
            if not policy.is_file() or digest(policy) != controller["policy_sha256"]:
                raise RuntimeError(f"Missing or changed policy: {policy}")
    for manifest in sorted((data / "assets/visual").glob("*/provenance.json")):
        provenance = json.loads(manifest.read_text())
        for part in provenance["parts"]:
            for asset in part["files"]:
                mesh = data / asset["file"]
                if not mesh.is_file() or digest(mesh) != asset["sha256"]:
                    raise RuntimeError(f"Missing or changed official visual mesh: {mesh}")
    for manifest in sorted((data / "assets/manifests").glob("*.json")):
        for asset in json.loads(manifest.read_text()).get("files", []):
            if "file" not in asset:
                continue
            resource = data / asset["file"]
            if not resource.is_file() or digest(resource) != asset["sha256"]:
                raise RuntimeError(f"Missing or changed manifest resource: {resource}")
    for link in stage.rglob("*"):
        if link.is_symlink() and (not link.exists() or
                                 stage.resolve() not in link.resolve().parents):
            raise RuntimeError(f"External/broken package symlink: {link}")
    for ros in ("ros1", "ros2"):
        entrypoint = data / "ros-adapters/deployment" / ros / "entrypoint.sh"
        if not entrypoint.is_file() or not os.access(entrypoint, os.X_OK):
            raise RuntimeError(f"Container entrypoint is not executable: {ros}")
    libraries = stage / "usr/lib/simple-rbt"
    for elf in list(binaries.iterdir()) + list(libraries.glob("*.so*")):
        versions = run("readelf", "--version-info", str(elf), capture_output=True).stdout
        # Jammy updates ship GCC 12's runtime; control.in requires it explicitly.
        for family, supported in (("GLIBC", (2, 35)), ("GLIBCXX", (3, 4, 30))):
            required = [tuple(map(int, value.split(".")))
                        for value in re.findall(rf"\b{family}_([0-9.]+)", versions)]
            if required and max(required) > supported:
                raise RuntimeError(f"{elf.name} requires {family} newer than Ubuntu 22.04")


def deduplicate_assets(data):
    """Package repeated CAD files as hard links, preserving all lookup paths."""
    originals = {}
    saved = 0
    for path in sorted(data.rglob("*")):
        if not path.is_file() or path.is_symlink() or path.stat().st_size < 65536:
            continue
        stat = path.stat()
        key = (stat.st_size, stat.st_mode, digest(path))
        original = originals.setdefault(key, path)
        if original != path:
            path.unlink()  # Only files in this invocation's temporary stage.
            os.link(original, path)
            saved += stat.st_size
    return saved


def source_epoch():
    if "SOURCE_DATE_EPOCH" in os.environ:
        return int(os.environ["SOURCE_DATE_EPOCH"])
    commit_time = run("git", "-C", str(ROOT), "log", "-1", "--format=%ct",
                      capture_output=True).stdout.strip()
    return int(commit_time)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", type=Path, default=ROOT / "build/desktop")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "output/packages")
    source_version = (ROOT / "packaging/VERSION").read_text().strip()
    parser.add_argument("--version", default=source_version)
    args = parser.parse_args()
    if not re.fullmatch(r"[0-9][A-Za-z0-9.+:~\-]*", args.version):
        parser.error("Version must be a Debian version starting with a digit")
    if args.version != source_version:
        parser.error("Version must match packaging/VERSION and the built framework/launcher")
    if run("dpkg", "--print-architecture", capture_output=True).stdout.strip() != "amd64":
        parser.error("This package currently targets Ubuntu 22.04 amd64")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    epoch = source_epoch()
    with tempfile.TemporaryDirectory(prefix="sprobo-deb-", dir=args.output_dir) as temporary:
        stage = Path(temporary) / "root"
        stage.mkdir()
        env = dict(os.environ, DESTDIR=str(stage.resolve()),
                   SOURCE_DATE_EPOCH=str(epoch), DPKG_DEB_THREADS_MAX="1")
        run("cmake", "--install", str(args.build_dir.resolve()), "--prefix", "/usr",
            "--component", "Runtime", "--strip", env=env)
        check_runtime(stage)
        executable_hashes = {name: digest(stage / "usr/bin" / name)
                             for name in ("simple-rbt-sim", "sprobo")}
        cuda_enabled = (stage / "usr/lib/simple-rbt/libcudart.so.12").is_file()
        saved = deduplicate_assets(stage / "usr/share/simple-rbt")
        inode_sizes = {(p.stat().st_dev, p.stat().st_ino): p.stat().st_size
                       for p in stage.rglob("*") if p.is_file() and not p.is_symlink()}
        installed_size = sum(math.ceil(size / 1024) for size in inode_sizes.values())
        control_dir = stage / "DEBIAN"
        control_dir.mkdir()
        control = (ROOT / "packaging/control.in").read_text().format(
            version=args.version, installed_size=installed_size)
        (control_dir / "control").write_text(control)
        for path in sorted(stage.rglob("*"), reverse=True) + [stage]:
            os.utime(path, (epoch, epoch), follow_symlinks=False)
        package = args.output_dir / f"sprobo_{args.version}_amd64.deb"
        run("dpkg-deb", "--build", "--root-owner-group", "--uniform-compression",
            "-Zxz", "-z6", str(stage), str(package), env=env)
    report = {
        "package": str(package.resolve()), "sha256": digest(package),
        "package_bytes": package.stat().st_size, "installed_kib": installed_size,
        "duplicate_asset_bytes_saved": saved, "source_date_epoch": epoch,
        "target": "Ubuntu 22.04 amd64", "runtime_python": False,
        "simulator_language": "C++17", "console_language": "Go",
        "framework": "simple_rbt", "launcher": "SpRobo", "cuda_enabled": cuda_enabled,
        "executable_sha256": executable_hashes,
    }
    package.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    package.with_suffix(".deb.sha256").write_text(report["sha256"] + "  " + package.name + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
