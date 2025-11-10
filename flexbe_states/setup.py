"""Set up for flexbe_states package."""

from glob import glob

from setuptools import find_packages
from setuptools import setup

PACKAGE_NAME = 'flexbe_states'

setup(
    name=PACKAGE_NAME,
    version='4.0.3',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        ('share/' + PACKAGE_NAME + '/tests', glob('tests/*.test')),
        ('share/' + PACKAGE_NAME + '/launch', glob('tests/*.launch.py')),
        # ros2 bag issues - ('share/' + PACKAGE_NAME + '/tests/bags', glob('tests/bags/*.bag')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    author='phil',
    author_email='philsplus@gmail.com',
    maintainer='David Conner',
    maintainer_email='robotics@cnu.edu',
    description='flexbe_states provides a collection of predefined states.',
    license='BSD',
    entry_points={
        'console_scripts': [
        ],
    },
)
