from glob import glob
from pathlib import Path
from setuptools import find_packages, setup

# colcon requires relative data_files. Reuse the repository's canonical assets
# without checking a second copy into this package.
MODEL = Path('../qianli_description')
PACKAGE = 'so101_bringup'

setup(
    name=PACKAGE,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
        ('share/' + PACKAGE, ['package.xml', 'migration_provenance.json']),
        ('share/' + PACKAGE + '/launch', glob('launch/*.launch.py')),
        ('share/' + PACKAGE + '/config',
         ['so101_overlay/driver_params.yaml'] + glob('config/*.rviz') + glob('config/*.yaml')),
        ('share/' + PACKAGE + '/urdf', [str(MODEL / 'urdf/so101.urdf')]),
        ('share/' + PACKAGE + '/urdf/assets', glob(str(MODEL / 'meshes/*.stl'))),
    ],
    install_requires=['setuptools', 'ikpy', 'numpy', 'pyserial'],
    tests_require=['pytest'],
    zip_safe=False,
    maintainer='Suerzong',
    maintainer_email='suerzong2007@gmail.com',
    description='QianLi SO-ARM101 driver and IK',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'driver_node = so101_bringup.driver_node:main',
        'ik_node = so101_bringup.ik_node:main',
    ]},
)
