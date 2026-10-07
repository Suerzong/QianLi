"""ROS-independent resource paths shared by nodes and standalone tools.

Explicit overrides never silently fall back to a different robot or calibration.
Importing this module does not create directories, open devices or import ROS.
"""
from __future__ import annotations

import os
from pathlib import Path


def project_root() -> Path:
    override = os.environ.get('QI_PROJECT_ROOT')
    if override:
        return Path(override).expanduser().resolve()
    for parent in Path(__file__).resolve().parents:
        if (parent / 'qianli_ws').is_dir() and (parent / 'dual_twin').is_dir():
            return parent
    raise RuntimeError('Cannot locate QianLi; set QI_PROJECT_ROOT to the checkout')


def project_path(relative: str = '') -> str:
    return str(project_root() / relative)


def so101_path(relative: str = '') -> str:
    override = os.environ.get('QI_SO101_PKG')
    if override:
        root = Path(override).expanduser()
    else:
        try:
            from ament_index_python.packages import get_package_share_directory
            root = Path(get_package_share_directory('so101_bringup'))
        except (ImportError, LookupError):
            root = project_root() / 'dual_twin'
    return str(root / relative)


def parts_path(relative: str = '') -> str:
    root = Path(os.environ.get('QI_PARTS_DIR', project_path('dual_twin/mj_parts')))
    return str(root.expanduser() / relative)


def robot_urdf_path() -> str:
    """ROS safety/TF model includes the repository's measured TCP frame.

    twin_runtime selects the historical training URDF with an explicit
    QI_SO101_PKG, preserving old policies and kinematics.
    """
    if os.environ.get('QI_SO101_PKG'):
        return so101_path('urdf/so101.urdf')
    try:
        from ament_index_python.packages import get_package_share_directory
        return str(Path(get_package_share_directory('so101_bringup'))/'urdf/so101.urdf')
    except (ImportError, LookupError):
        return project_path('qianli_ws/src/qianli_description/urdf/so101.urdf')


def arm_source_path(relative: str = '') -> str:
    """Resolve old so101_bringup source paths to the canonical overlay files."""
    if relative.startswith('so101_bringup/'):
        relative = 'so101_overlay/' + relative[len('so101_bringup/'):]
    if relative == 'config/driver_params.yaml':
        relative = 'so101_overlay/driver_params.yaml'
    return project_path('qianli_ws/src/qianli_arm/' + relative)


def driver_params_path() -> str:
    override = os.environ.get('QI_DRIVER_CONFIG')
    if override:
        return str(Path(override).expanduser())
    package_config = Path(so101_path('config/driver_params.yaml'))
    if package_config.is_file() or os.environ.get('QI_SO101_PKG'):
        return str(package_config)
    return arm_source_path('so101_overlay/driver_params.yaml')


def calibration_path(filename: str = '') -> str:
    root = Path(os.environ.get('QI_CALIB_DIR', project_path('calib'))).expanduser()
    return str(root / filename)


def camera_source(value):
    """OpenCV accepts both integer indices and stable V4L2 device paths."""
    if isinstance(value, int):
        return value
    value = os.fspath(value)
    return int(value) if value.isdecimal() else value


def default_camera():
    return camera_source(os.environ.get('QI_CAMERA', '0'))


def open_video_capture(source=None, *args, **kwargs):
    """Open a camera with an optional machine-specific transport format.

    QI_CAMERA_FOURCC=MJPG avoids raw YUYV USB bandwidth failures in VMware.
    With no override, OpenCV retains its existing backend and format defaults.
    Video files/URLs retain their decoder settings.
    """
    import cv2
    source = default_camera() if source is None else camera_source(source)
    fourcc = os.environ.get('QI_CAMERA_FOURCC', '')
    device = isinstance(source, int) or str(source).startswith('/dev/')
    if device and fourcc and len(fourcc) != 4:
        raise ValueError('QI_CAMERA_FOURCC must contain four characters, e.g. MJPG')
    capture = cv2.VideoCapture(source, *args, **kwargs)
    if device and fourcc and capture.isOpened():
        if not capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc)):
            capture.release()
            raise RuntimeError(f'Camera {source} cannot select format {fourcc}')
    return capture


def default_arm_port() -> str:
    return os.environ.get('QI_ARM_PORT',
                          '/dev/qianli_arm' if Path('/dev/qianli_arm').exists()
                          else '/dev/ttyACM0')
