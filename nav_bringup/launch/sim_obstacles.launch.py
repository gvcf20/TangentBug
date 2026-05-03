import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_nav_bringup = get_package_share_directory('nav_bringup')
    world_file = os.path.join(pkg_nav_bringup, 'worlds', 'obstacles_simple.world')

    sim_empty = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav_bringup, 'launch', 'sim_empty.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    return LaunchDescription([sim_empty])