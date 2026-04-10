import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'so101_web_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'web'), glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nimrod',
    maintainer_email='nimicu21@gmail.com',
    description='IMU web teleop for SO101',
    license='MIT',
    entry_points={
        'console_scripts': [
            'web_teleop_node = so101_web_teleop.web_teleop_node:main',
        ],
    },
)
