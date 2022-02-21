import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'flexbe_onboard'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(),
    data_files=[
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phil',
    maintainer_email='philsplus@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flexbe_onboard = flexbe_onboard.flexbe_onboard',
            'start_behavior = flexbe_onboard.start_behavior:main'
        ],
    },
)
