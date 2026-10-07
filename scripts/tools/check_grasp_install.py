"""Import installed ROS packages without creating nodes or hardware clients."""
import json
from pathlib import Path
from rcl_interfaces.srv import GetParameters
import qianli_vision.object_localizer as vision
import so101_bringup.ik_node as native

assert hasattr(vision.ObjectLocalizer,'_warn_throttled')
assert hasattr(native.So101IkNode,'_receive_hardware_limits')
assert hasattr(native.So101IkNode,'_orientation_error_deg')
request = GetParameters.Request(names=['mode','zero_raw','direction','raw_min','raw_max'])
print(json.dumps(dict(vision_module=str(Path(vision.__file__).resolve()),
                      native_ik_module=str(Path(native.__file__).resolve()),
                      parameter_request_fields=list(request.names),imports_passed=True)))
