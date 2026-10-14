"""Evidence files, source snapshots and independent Python payload digests."""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
import struct
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_ROOT = SCRIPT_DIR.parent
ROOT = PACKAGE_ROOT.parent
CPP_ROOT = ROOT / "mujoco_sim_cpp"
CPP_WORKER = CPP_ROOT / "build" / "simple-rbt-native"


def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def digest_json(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def source_fingerprints():
    paths = [SCRIPT_DIR / "validate_full_run.py", SCRIPT_DIR / "evaluate_terrains.py", SCRIPT_DIR / "check_ros2.py"]
    paths += list(SCRIPT_DIR.glob("full_run_*.py"))
    paths += list((PACKAGE_ROOT / "simple_rbt_mujoco").glob("*.py"))
    manifests = list((PACKAGE_ROOT / "assets").glob("*manifest.json"))
    paths += manifests + list((PACKAGE_ROOT / "assets").glob("*.onnx"))
    for manifest in manifests:
        description = json.loads(manifest.read_text())
        vendor = description.get("vendor_directory")
        if vendor:
            paths += [ROOT / ".vendor" / vendor / row["path"] for row in description.get("files", [])
                      if (ROOT / ".vendor" / vendor / row["path"]).is_file()]
    # Hash native source inputs, including generated model XML and pinned third
    # party headers, but never build output or executables. Python remains an
    # oracle in the same tree; both runtime snapshots use the same source list.
    for directory in ("src", "include", "tests", "third_party"):
        paths += [path for path in (CPP_ROOT / directory).rglob("*")
                  if path.is_file() and path.suffix in (".cpp", ".cu", ".hpp", ".h")]
    paths += [path for path in (CPP_ROOT / "assets").rglob("*")
              if path.is_file() and path.suffix in (".json", ".xml")]
    paths += [path for path in (CPP_ROOT / "CMakeLists.txt", CPP_ROOT / "third_party" / "manifest.json")
              if path.is_file()]
    return {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(set(paths))}


def scan_digest(xyz, offsets, lines):
    digest = hashlib.sha256()
    for values, dtype in ((xyz, "<f4"), (offsets, "<u4"), (lines, "u1")):
        digest.update(np.asarray(values, dtype=dtype).tobytes(order="C"))
    return digest.hexdigest()


def imu_digest_update(digest, stamp_ns, angular, acceleration, orientation):
    digest.update(struct.pack("<Q", stamp_ns))
    digest.update(np.asarray([*angular, *acceleration, *orientation], dtype="<f8").tobytes())
