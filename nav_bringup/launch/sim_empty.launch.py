import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    pkg_nav_bringup = get_package_share_directory('nav_bringup')

    tb3_pkg = get_package_share_directory('turtlebot3_gazebo')

    rviz_config = os.path.join(
        pkg_nav_bringup,
        'config',
        'rviz',
        'nav.rviz'
    )

    # Launch oficial do TurtleBot3
    tb3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                tb3_pkg,
                'launch',
                'empty_world.launch.py'
            )
        )
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        tb3_launch,
        rviz,
    ])