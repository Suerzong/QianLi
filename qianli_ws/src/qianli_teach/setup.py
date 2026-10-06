from setuptools import find_packages, setup

package_name = 'qianli_teach'

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
    description='QianLi 拖动示教节点：录制真实关节轨迹并回放',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teach_node = qianli_teach.teach_node:main',
        ],
    },
)
