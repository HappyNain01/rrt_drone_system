from setuptools import setup
import os
from glob import glob

package_name = 'rrt_drone'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name, 
              f'{package_name}.nodes',
              f'{package_name}.algorithm',
              f'{package_name}.utils'],
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
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Distributed RRT path planning system - Fixed Version',
    license='MIT',
    entry_points={
        'console_scripts': [
            'path_planner = rrt_drone.nodes.path_planner_node:main',
            ]
    },
)
