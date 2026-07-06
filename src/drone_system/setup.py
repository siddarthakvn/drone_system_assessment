import os
from glob import glob
from setuptools import setup

package_name = 'drone_system'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*'))),
        (os.path.join('share', package_name, 'models', 'car'), glob(os.path.join('models', 'car', '*'))),
        (os.path.join('share', package_name, 'models', 'drone'), glob(os.path.join('models', 'drone', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kvn',
    maintainer_email='kvn@todo.todo',
    description='Assessment package for a ROS 2 drone-following-car system.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'car_simulator = drone_system.car_simulator:main',
            'follower_node = drone_system.follower_node:main',
            'drone_manager = drone_system.drone_manager:main',
            'px4_manager = drone_system.px4_manager:main',
            'health_monitor = drone_system.health_monitor:main',
            'telemetry_recorder = drone_system.telemetry_recorder:main',
        ],
    },
)
