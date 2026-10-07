"""Expose shared path helpers to tools executed directly from this directory."""
import sys
from pathlib import Path

_package_root = str(Path(__file__).resolve().parents[2] /
                    'qianli_ws/src/qianli_vision')
if _package_root not in sys.path:
    sys.path.insert(0, _package_root)
from qianli_vision.runtime_paths import (  # noqa: E402,F401
    arm_source_path, calibration_path, camera_source, default_arm_port,
    default_camera, driver_params_path, parts_path, project_path, project_root,
    so101_path, robot_urdf_path,
)
