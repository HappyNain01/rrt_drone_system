from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'rrt_drone'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dipanshu',
    maintainer_email='dnain9143@gmail.com',
    description='Distributed RRT path planning - Ubuntu Server optimized',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command_executor = rrt_drone.nodes.command_executor_node:main',
            'health_monitor = rrt_drone.nodes.health_monitor_node:main',
        ],
    },
)
