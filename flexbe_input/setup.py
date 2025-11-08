"""Setup for flexbe_input package."""
from setuptools import find_packages
from setuptools import setup

PACKAGE_NAME = 'flexbe_input'

setup(
    name=PACKAGE_NAME,
    version='4.0.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        # No tests yet ('share/' + PACKAGE_NAME + '/tests', glob('tests/*.test')),
    ],
    install_requires=['setuptools', 'PySide6'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    author='phil',
    author_email='philsplus@gmail.com',
    maintainer='David Conner',
    maintainer_email='robotics@cnu.edu',
    description='flexbe_input enables to send data to onboard behavior when required.',
    license='BSD',
    entry_points={
        'console_scripts': [
            'flexbe_input = flexbe_input.bin.flexbe_input:main',
            'input_action_server = flexbe_input.input_action_server:main'
        ]
    },
)
