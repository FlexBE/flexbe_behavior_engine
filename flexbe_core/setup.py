"""Setup package for flexbe_core package."""
from setuptools import find_packages
from setuptools import setup

PACKAGE_NAME = 'flexbe_core'

setup(
    name=PACKAGE_NAME,
    version='4.0.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    author='phil',
    author_email='philsplus@gmail.com',
    maintainer='David Conner',
    maintainer_email='robotics@cnu.edu',
    description='flexbe_core provides the core components for the FlexBE behavior engine.',
    license='BSD',
)
