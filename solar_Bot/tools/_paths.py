"""Shared paths for standalone tools.

The tools in this folder are meant to run directly from any working directory.
Keep this tiny helper local to avoid repeating sys.path and data-dir setup in
each diagnostic script.
"""

from __future__ import annotations

import sys
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = TOOLS_DIR.parent
PATH_DIR = PROJECT_ROOT / "paths"


def ensure_project_root() -> Path:
    root = str(PROJECT_ROOT)
    if root not in sys.path:
        sys.path.insert(0, root)
    return PROJECT_ROOT


ensure_project_root()
