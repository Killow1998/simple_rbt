"""Repository locations for standalone maintenance and validation tools."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
REFERENCE_ROOT = ROOT / "reference" / "python"
sys.path.insert(0, str(REFERENCE_ROOT))
