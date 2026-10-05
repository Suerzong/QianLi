from setuptools import find_packages, setup

package_name = 'qianli_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Suerzong',
    maintainer_email='suerzong2007@gmail.com',
    description='QianLi 视觉感知：RGB 相机物块检测 + 网格纸标定 + 物理坐标定位',
    license='MIT',
    entry_points={
        'console_scripts': [
            'object_localizer = qianli_vision.object_localizer:main',
            'grab_bridge = qianli_vision.grab_bridge:main',
        ],
    },
)
