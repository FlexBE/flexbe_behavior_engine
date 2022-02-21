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
    # entry_points={
    #     'console_scripts': [
    #         'my_node = my_package.my_node:main',
    #     ],
    # },
)
