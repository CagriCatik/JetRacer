import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    nav_dir = get_package_share_directory('jetracer_navigation')

    # Include the SLAM Nav2 backbone
    slam_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_dir, 'launch', 'slam_nav.launch.py'))
    )

    # Boot the Frontier Exploration node
    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[{
            'robot_base_frame': 'base_footprint',
            'costmap_topic': '/global_costmap/costmap',
            'costmap_updates_topic': '/global_costmap/costmap_updates',
            'visualize': True,
            'planner_frequency': 0.33,
            'progress_timeout': 30.0,
            'potential_scale': 3.0,
            'orientation_scale': 0.0,
            'gain_scale': 1.0,
            'transform_tolerance': 0.3,
            'min_frontier_size': 0.5,
        }]
    )

    return LaunchDescription([
        slam_nav_launch,
        explore_node
    ])
