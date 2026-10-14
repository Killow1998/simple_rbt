"""Shared locations for the retained Python oracle and its frozen assets."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ASSETS = ROOT / "assets"
MANIFESTS = ASSETS / "manifests"
POLICIES = ASSETS / "policies"
SCAN_CSV = ASSETS / "lidar" / "mid360-real-centr.csv"
