"""Load the same URDF meshes in MuJoCo with or without a ROS installation."""
from pathlib import Path
from xml.etree import ElementTree as ET


def load_mujoco_spec(urdf_path):
    import mujoco

    path = Path(urdf_path).resolve()
    root = ET.parse(path).getroot()
    meshes = root.findall('.//mesh')
    if not any(mesh.get('filename', '').startswith('package://') for mesh in meshes):
        # Preserve MuJoCo's relative-path handling for historical training URDFs.
        return mujoco.MjSpec.from_file(str(path))

    for mesh in meshes:
        filename = mesh.get('filename', '')
        if filename.startswith('package://'):
            package, relative = filename[len('package://'):].split('/', 1)
            resource = Path(relative)
            if resource.is_absolute() or '..' in resource.parts:
                raise ValueError(f'Invalid URDF resource: {filename}')
            # so101_bringup installs a self-contained copy in urdf/assets;
            # qianli_description uses its own meshes directory in the checkout.
            candidates = [path.parent/'assets'/resource.name,
                          path.parent.parent/resource]
            resolved = next((p for p in candidates if p.is_file()), None)
            if resolved is None:
                from ament_index_python.packages import get_package_share_directory
                resolved = Path(get_package_share_directory(package))/resource
        else:
            resolved = Path(filename)
            if not resolved.is_absolute():
                resolved = path.parent/resolved
        if not resolved.is_file():
            raise FileNotFoundError(f'URDF mesh {filename}: {resolved}')
        mesh.set('filename', str(resolved.resolve()))
    return mujoco.MjSpec.from_string(ET.tostring(root, encoding='unicode'))
