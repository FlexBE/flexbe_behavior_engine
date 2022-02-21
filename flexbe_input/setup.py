from setuptools import setup
from setuptools import find_packages

package_name = 'flexbe_input'

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
            'complex_action_server = flexbe_input.complex_action_server',
            'flexbe_input = flexbe_input.flexbe_input',
            'behavior_input = flexbe_input.bin.behavior_input:main'
        ],
    },
)
