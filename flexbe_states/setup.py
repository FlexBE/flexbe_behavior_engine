"""Set up for flexbe_states package."""

from glob import glob

from setuptools import setup
from setuptools import find_packages

PACKAGE_NAME = 'flexbe_states'

setup(
    name=PACKAGE_NAME,
    version='2.3.2',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        ('share/' + PACKAGE_NAME + "/tests", glob('tests/*.test')),
        ('share/' + PACKAGE_NAME + "/launch", glob('tests/*.launch.py')),
        # ros2 bag issues - ('share/' + PACKAGE_NAME + "/tests/bags", glob('tests/bags/*.bag')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phil',
    maintainer_email='philsplus@gmail.com',
    description='flexbe_states provides a collection of predefined states.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
