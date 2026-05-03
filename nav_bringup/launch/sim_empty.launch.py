import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
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
    world = LaunchConfiguration('world')

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            pkg_nav_bringup,
            'worlds',
            'empty.world'
        ),
        description='World file'
)

    tb3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                tb3_pkg,
                'launch',
                'empty_world.launch.py'
            )
        ),
        launch_arguments={
            'world': world
        }.items()
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
        declare_world,
        tb3_launch,
        rviz,
    ])