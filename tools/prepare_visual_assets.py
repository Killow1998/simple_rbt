#!/usr/bin/env python3
"""Prepare pinned official visual CAD; reuse verified local sources offline."""
import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import shutil
import subprocess
import sys
import urllib.parse
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
CACHE = ROOT / "build/visual-preparation"
DEPENDENCIES = {"numpy": "2.2.6", "trimesh": "5.1.1", "pycollada": "0.9.3",
                "fast-simplification": "0.2.0", "lxml": "6.1.3", "networkx": "3.4.2",
                "python-dateutil": "2.9.0.post0", "six": "1.17.0"}


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def dependencies():
    target = CACHE / "python"
    sys.path.insert(0, str(target))
    ready = all((target / (package.replace("-", "_") + "-" + version + ".dist-info")).exists()
                for package, version in DEPENDENCIES.items())
    if not ready:
        uv = shutil.which("uv")
        if not uv:
            raise RuntimeError("Run setup.sh first; visual preparation needs uv")
        subprocess.run([uv, "pip", "install", "--python", sys.executable,
                        "--target", str(target), "--no-deps"] +
                       [f"{name}=={version}" for name, version in DEPENDENCIES.items()], check=True)
    for package, version in DEPENDENCIES.items():
        if importlib.metadata.version(package) != version:
            raise RuntimeError(f"Visual preparation requires {package}=={version}")


def fetch(source):
    path = CACHE / "sources" / source["repo"] / source["path"]
    expected = source.get("lfs_sha256", source["sha256"])
    if path.is_file() and sha256(path) == expected:
        return path
    url = "https://raw.githubusercontent.com/" + source["repo"] + "/" + source["commit"] + "/" + urllib.parse.quote(source["path"])
    with urllib.request.urlopen(url, timeout=90) as response:
        data = response.read()
    blob = hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()
    if blob != source["git_blob"]:
        raise RuntimeError(f"Official source blob mismatch: {source['path']}")
    if "lfs_sha256" in source:
        with urllib.request.urlopen(source["download_lfs_url"], timeout=90) as response:
            data = response.read()
    if hashlib.sha256(data).hexdigest() != expected:
        raise RuntimeError(f"Official source SHA256 mismatch: {source['path']}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)
    return path


def export(recipe, source, output_dir):
    import collections
    import numpy as np
    import trimesh
    scene = trimesh.load(source, process=False, force="scene")
    transform = trimesh.transformations.euler_matrix(*recipe.get("rpy", [0, 0, 0]))
    transform[:3, 3] = recipe.get("position", [0, 0, 0])
    groups = collections.defaultdict(list)
    for node in scene.graph.nodes_geometry:
        placement, name = scene.graph.get(node)
        mesh = scene.geometry[name].copy()
        mesh.apply_transform(placement)
        mesh.apply_scale(recipe.get("scale", 1))
        mesh.apply_transform(transform)
        color = tuple(recipe["color"]) if "color" in recipe else (
            tuple(mesh.visual.material.main_color.tolist())
            if mesh.visual.kind == "texture" else (160, 160, 160, 255))
        groups[color].append(mesh)
    face_count = sum(len(mesh.faces) for group in groups.values() for mesh in group)
    files = []
    for index, (color, parts) in enumerate(sorted(groups.items())):
        mesh = trimesh.util.concatenate(parts)
        mesh.process(validate=True)
        original = mesh.copy()
        target = max(48, round(recipe.get("target_faces", face_count) * len(mesh.faces) / face_count))
        rejected = False
        if len(mesh.faces) > target:
            mesh = mesh.simplify_quadric_decimation(face_count=target)
            if np.max(np.abs(mesh.bounds-original.bounds)) > .005:
                mesh = original
                rejected = True
        path = output_dir / f"{recipe['part']}-{index}.obj"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(trimesh.exchange.obj.export_obj(
            mesh, include_color=False, include_texture=False, include_normals=False, digits=9))
        files.append({"file": str(path.relative_to(ROOT)), "rgba": [round(c/255, 8) for c in color],
                      "source_faces": len(original.faces), "faces": len(mesh.faces), "vertices": len(mesh.vertices),
                      "bounds_m": mesh.bounds.tolist(), "bbox_deviation_m": float(np.max(np.abs(mesh.bounds-original.bounds))),
                      "simplification_rejected_for_bounds": rejected, "sha256": sha256(path), "bytes": path.stat().st_size})
    return {"part": recipe["part"], "recipe": recipe, "files": files}


def prepare(robot, refresh):
    path = ROOT / "assets/visual" / robot / "provenance.json"
    data = json.loads(path.read_text())
    prepared = bool(data["parts"]) and all((ROOT/f["file"]).is_file() and sha256(ROOT/f["file"]) == f["sha256"]
                   for part in data["parts"] for f in part["files"])
    if prepared and not refresh:
        print(f"{robot}: verified prepared visual assets")
        return
    dependencies()
    sources = {entry["repo"]+"/"+entry["path"]: entry for entry in data["source_files"]}
    parts = [export(recipe, fetch(sources[recipe["source"]]), ROOT/data.get("output_directory", "assets/visual/"+robot))
             for recipe in data["recipes"]]
    if not refresh:
        actual = {f["file"]: f["sha256"] for part in parts for f in part["files"]}
        expected = {f["file"]: f["sha256"] for part in data["parts"] for f in part["files"]}
        if actual != expected:
            raise RuntimeError(f"Derived visual SHA256 mismatch: {robot}")
    else:
        data["parts"] = parts
        data["preparation"]["dependencies"] = DEPENDENCIES
        path.write_text(json.dumps(data, indent=2)+"\n")
    print(f"{robot}: prepared pinned official visual assets")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", action="append", choices=("ranger_mini", "scout_mini", "fr_mini", "ridgeback"))
    parser.add_argument("--refresh-manifests", action="store_true", help="Development: regenerate assets and record new derived hashes")
    args = parser.parse_args()
    for robot in args.robot or ["ranger_mini", "scout_mini", "fr_mini", "ridgeback"]:
        prepare(robot, args.refresh_manifests)


if __name__ == "__main__":
    main()
