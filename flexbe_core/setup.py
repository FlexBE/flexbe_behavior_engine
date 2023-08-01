"""Setup package for flexbe_core package."""
from setuptools import setup
from setuptools import find_packages

PACKAGE_NAME = 'flexbe_core'

setup(
    name=PACKAGE_NAME,
    version='2.3.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phil',
    maintainer_email='philsplus@gmail.com',
    description='flexbe_core provides the core components for the FlexBE behavior engine.',
    license='BSD',
    tests_require=['pytest'],
)
