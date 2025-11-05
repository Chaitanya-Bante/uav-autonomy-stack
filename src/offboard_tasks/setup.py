from setuptools import setup
import os
from glob import glob

package_name = 'offboard_tasks'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chaitu',
    maintainer_email='your@email.com',
    description='Custom offboard tasks for PX4 simulation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'offboard_circle = offboard_tasks.offboard_circle:main',
        ],
    },
)
