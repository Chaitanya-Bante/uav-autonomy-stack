from setuptools import setup
import os
from glob import glob

package_name = 'px4_custom_nodes'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chaitu',
    maintainer_email='chaitu@example.com',
    description='Custom PX4 control nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'square_flight = px4_custom_nodes.square_flight_node:main',
            'test_connection = px4_custom_nodes.test_connection:main',
            'waypoint_navigator = px4_custom_nodes.waypoint_navigator:main',
        ],
    },
)