import sys
from pathlib import Path

# The spike is intentionally not part of the drone_cv package yet.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
