"""Compatibility import for the canonical, bundled SO101 protocol helpers."""
import sys
from project_paths import arm_source_path

_source = arm_source_path()
if _source not in sys.path:
    sys.path.insert(0, _source)
from so101_bringup.servo_protocol import *  # noqa: E402,F401,F403
