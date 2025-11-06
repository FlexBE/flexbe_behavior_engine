"""Setup for flexbe_testing package."""
import os
from glob import glob

from setuptools import setup

package_name = 'flexbe_testing'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name), glob('tests/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='phil',
    author_email='philsplus@gmail.com',
    maintainer='David Conner',
    maintainer_email='robotics@cnu.edu',
    description='flexbe_testing provides a framework for unit testing states.',
    license='BSD',
)
