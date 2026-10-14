"""Evidence files, source snapshots and independent Python payload digests."""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
import struct
import numpy as np

from project import ROOT, REFERENCE_ROOT

SCRIPT_DIR = ROOT / "tools"
CPP_WORKER = ROOT / "build" / "native" / "simple-rbt-native"


def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def digest_json(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def source_fingerprints():
    # Include both runtime implementations, their checks and the canonical inputs.
    # Build outputs are excluded; native and Python workers hash this same list.
    paths = list(SCRIPT_DIR.glob("*.py")) + list(SCRIPT_DIR.glob("*.sh"))
    paths += list((REFERENCE_ROOT / "simple_rbt_mujoco").rglob("*.py"))
    paths += list((REFERENCE_ROOT / "tests").rglob("*.py"))
    paths += [path for path in (ROOT / "assets").rglob("*") if path.is_file()]
    manifests = list((ROOT / "assets" / "manifests").glob("*manifest.json"))
    for manifest in manifests:
        description = json.loads(manifest.read_text())
        vendor = description.get("vendor_directory")
        if vendor:
            paths += [ROOT / ".vendor" / vendor / row["path"] for row in description.get("files", [])
                      if (ROOT / ".vendor" / vendor / row["path"]).is_file()]
    for directory in ("src", "include", "tests", "third_party"):
        paths += [path for path in (ROOT / directory).rglob("*")
                  if path.is_file() and path.suffix in (".cpp", ".cu", ".hpp", ".h")]
    paths += [path for path in (ROOT / "ros2_interfaces").rglob("*")
              if path.is_file() and (path.suffix in (".msg", ".xml", ".sh")
                                     or path.name == "CMakeLists.txt")]
    paths += [path for path in (ROOT / "CMakeLists.txt", ROOT / "third_party" / "manifest.json",
              ROOT / "setup.sh", ROOT / "run.sh", REFERENCE_ROOT / "run.sh",
              REFERENCE_ROOT / "pyproject.toml", REFERENCE_ROOT / "requirements.lock")
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
