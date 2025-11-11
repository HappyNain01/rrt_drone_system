
#!/usr/bin/env python3
"""
Launch file for Raspberry Pi system
Starts all onboard nodes
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('rrt_drone')
    config_dir = os.path.join(pkg_dir, 'config')
    
    rrt_params = os.path.join(config_dir, 'rrt_params.yaml')
    mavros_params = os.path.join(config_dir, 'mavros_config.yaml')
    
    return LaunchDescription([
        # Health Monitor (start first)
        Node(
            package='rrt_drone',
            executable='health_monitor',
            name='health_monitor',
            output='screen',
            parameters=[rrt_params]
        ),
        
        # MAVROS (delay 2 seconds)
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='mavros',
                    executable='mavros_node',
                    name='mavros',
                    output='screen',
                    parameters=[mavros_params]
                )
            ]
        ),
        
        # Command Executor (delay 4 seconds)
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='rrt_drone',
                    executable='command_executor',
                    name='command_executor',
                    output='screen',
                    parameters=[rrt_params]
                )
            ]
        ),
        
        # YDLiDAR (delay 6 seconds - optional, skip if no LiDAR)
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='ydlidar_ros2_driver',
                    executable='ydlidar_ros2_driver_node',
                    name='ydlidar_node',
                    output='screen',
                    parameters=[{
                        'port': '/dev/ttyUSB0',
                        'baudrate': 230400,
                        'frame_id': 'laser_frame',
                        'frequency': 8.0,
                        'range_max': 8.0,
                        'range_min': 0.1
                    }]
                )
            ]
        ),
    ])
