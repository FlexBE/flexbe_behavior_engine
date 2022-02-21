from setuptools import setup
from setuptools import find_packages

package_name = 'flexbe_mirror'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
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
            'flexbe_mirror = flexbe_mirror.flexbe_mirror',
            'mirror_state = flexbe_mirror.mirror_state',
            'behavior_mirror_sm = flexbe_mirror.behavior_mirror_sm:main'
        ],
    },
)
