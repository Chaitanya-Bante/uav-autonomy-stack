from setuptools import setup
import os
from glob import glob

package_name = 'px4_drone_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') if os.path.exists('config') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chaitu',
    maintainer_email='chaitu@example.com',
    description='PX4 drone SLAM integration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_bridge = px4_drone_slam.camera_bridge:main',
            'slam_controller = px4_drone_slam.slam_controller:main',
            'camera_discovery = px4_drone_slam.camera_discovery:main',
        ],
    },
)
