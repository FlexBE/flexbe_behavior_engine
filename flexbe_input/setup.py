"""Setup for flexbe_input package."""
from setuptools import setup
from setuptools import find_packages

PACKAGE_NAME = 'flexbe_input'

setup(
    name=PACKAGE_NAME,
    version='2.3.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        # No tests yet ('share/' + PACKAGE_NAME + "/tests", glob('tests/*.test')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phil',
    maintainer_email='philsplus@gmail.com',
    description='flexbe_input enables to send data to onboard behavior when required.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flexbe_input = flexbe_input.bin.flexbe_input:main',
            'input_action_server = flexbe_input.input_action_server:main'
        ]
    },
)
