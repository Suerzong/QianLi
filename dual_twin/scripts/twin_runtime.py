"""Portable resource setup, inherited by spawned training workers."""
import os
from pathlib import Path
import sys

HERE = str(Path(__file__).resolve().parent)
ROOT = Path(os.environ.get('QI_PROJECT_ROOT', Path(HERE).parents[1])).expanduser().resolve()
DUAL = str(ROOT/'dual_twin')
LOCAL_SCRIPTS = str(ROOT/'qianli_ws/src/qianli_vision/scripts')
for path in (LOCAL_SCRIPTS, HERE):
    if path not in sys.path:
        sys.path.insert(0, path)
os.environ.setdefault('QI_PROJECT_ROOT', str(ROOT))
os.environ.setdefault('QI_SO101_PKG', DUAL)
os.environ.setdefault('QI_PARTS_DIR', str(Path(DUAL)/'mj_parts'))
URDF_PATH = str(Path(os.environ['QI_SO101_PKG'])/'urdf/so101.urdf')
PARTS_DIR = os.environ['QI_PARTS_DIR']


def report():
    print(f'URDF = {URDF_PATH}\nConvex parts = {PARTS_DIR}\nScripts = {LOCAL_SCRIPTS}')


if __name__ == '__main__':
    report()
