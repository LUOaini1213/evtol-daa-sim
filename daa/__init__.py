"""eVTOL detect-and-avoid alerting and avoidance simulation (personal project, synthetic encounters)."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def load_config(path=None):
    p = Path(path) if path else ROOT / "config" / "default.json"
    with open(p, encoding="utf-8") as f:
        return json.load(f)
