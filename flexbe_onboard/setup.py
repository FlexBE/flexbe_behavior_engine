"""Setup script for flexbe_onboard package."""

import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'flexbe_onboard'

setup(
    name=package_name,
    version='2.3.3',
    packages=find_packages(),
    data_files=[
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('tests/*.py')),
        (os.path.join('share', package_name), glob('tests/flexbe_onboard_test_data/*.py')),
        (os.path.join('share', package_name, "tests", "flexbe_onboard_test_data"),
            glob('tests/flexbe_onboard_test_data/*.py')),  # No * here due to __pycache__ folder
        (os.path.join('share', package_name, "tests", "flexbe_onboard_test_data"),
            glob('tests/flexbe_onboard_test_data/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phil',
    maintainer_email='philsplus@gmail.com',
    description='flexbe_onboard implements the robot-side of the behavior engine from where all behaviors are started.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flexbe_onboard = flexbe_onboard.flexbe_onboard',
            'start_behavior = flexbe_onboard.start_behavior:main',
        ],
    },
)
