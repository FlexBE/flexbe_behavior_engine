"""Setup script for flexbe_onboard package."""

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'flexbe_onboard'

setup(
    name=package_name,
    version='4.1.3',
    packages=find_packages(),
    data_files=[
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('test/*.py')),
        (os.path.join('share', package_name), glob('test/flexbe_onboard_test_data/*.py')),
        (os.path.join('share', package_name, 'test', 'flexbe_onboard_test_data'),
            glob('test/flexbe_onboard_test_data/*.py')),  # No * here due to __pycache__ folder
        (os.path.join('share', package_name, 'test', 'flexbe_onboard_test_data'),
            glob('test/flexbe_onboard_test_data/*.xml')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    author='phil',
    author_email='philsplus@gmail.com',
    maintainer='David Conner',
    maintainer_email='robotics@cnu.edu',
    description='flexbe_onboard implements the robot-side of the behavior engine from where all behaviors are started.',
    license='BSD',
    entry_points={
        'console_scripts': [
            'start_behavior = flexbe_onboard.start_behavior:main',
        ],
    },
)
