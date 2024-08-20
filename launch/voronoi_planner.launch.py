"""
VoronoiPlanner app launch file.

Lorenzo Bianchi <lnz.bnc@gmail.com>

January 13, 2024
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Builds a LaunchDescription for the VoronoiPlanner app"""
    ld = LaunchDescription()

    # Build config file path
    config = os.path.join(
        get_package_share_directory('voronoi_planner'), 'config', 'voronoi_planner.yaml')

    # Declare launch arguments
    cf = LaunchConfiguration('cf')
    use_rviz = LaunchConfiguration('rviz')
    cf_launch_arg = DeclareLaunchArgument('cf', default_value=config)
    use_rviz_launch_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    ld.add_action(cf_launch_arg)
    ld.add_action(use_rviz_launch_arg)

    # Create node launch description
    node = Node(
        package='voronoi_planner',
        executable='voronoi_planner_app',
        shell=False,
        emulate_tty=True,
        output='both',
        log_cmd=True,
        parameters=[config]
    )
    ld.add_action(node)

    # Launch RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('voronoi_planner'), 'rviz', 'voronoi.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    ld.add_action(rviz)

    return ld
