from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'security_tests'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jhonnie',
    maintainer_email='the@mail.muh',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'robot_controller = security_tests.robot_controller:main',
        'command_injector = security_tests.command_injector:main',
        'odom_spoofer = security_tests.odom_spoofer:main',
        'lidar_spoofer = security_tests.lidar_spoofer:main',
        'topic_flooder = security_tests.topic_flooder:main',
    ],
},
)
