from setuptools import setup
from glob import glob
import os

package_name = 'px4_2d_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chaitu',
    maintainer_email='chaitu@example.com',
    description='2D SLAM with PX4 drone',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tf_publisher = px4_2d_slam.tf_publisher:main',
            'odometry_publisher = px4_2d_slam.odometry_publisher:main',
        ],
    },
)
