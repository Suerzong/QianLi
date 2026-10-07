"""ROS console entry points for installed standalone scripts."""
import runpy
import sys
from pathlib import Path


def safety_gate():
    from ament_index_python.packages import get_package_share_directory
    scripts = Path(get_package_share_directory('qianli_vision')) / 'scripts'
    sys.path.insert(0, str(scripts))
    runpy.run_path(str(scripts / 'safety_gate.py'), run_name='__main__')
