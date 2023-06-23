"""Setup file for package."""

import os
from glob import glob
from setuptools import setup

PACKAGE_NAME = 'flexbe_widget'

setup(
    name=PACKAGE_NAME,
    version='2.0.0',
    packages=[PACKAGE_NAME],
    data_files=[
        (os.path.join('share', PACKAGE_NAME), glob('launch/*.launch.py')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phil',
    maintainer_email='philsplus@gmail.com',
    description='flexbe_widget implements some smaller scripts for the behavior engine.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'behavior_launcher = flexbe_widget.behavior_launcher',
            'behavior_action_server = flexbe_widget.behavior_action_server',
        ],
    },
)
