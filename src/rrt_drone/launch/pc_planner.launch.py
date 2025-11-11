from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('rrt_drone')
    config_dir = os.path.join(pkg_dir, 'config')
    rrt_params = os.path.join(config_dir, 'rrt_params.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument('goal_x', default_value='5.0'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),
        DeclareLaunchArgument('goal_z', default_value='2.0'),
        
        Node(
            package='rrt_drone',
            executable='path_planner',
            name='path_planner',
            output='screen',
            parameters=[
                rrt_params,
                {
                    'planning.goal_x': LaunchConfiguration('goal_x'),
                    'planning.goal_y': LaunchConfiguration('goal_y'),
                    'planning.goal_z': LaunchConfiguration('goal_z'),
                }
            ]
        ),
    ])
