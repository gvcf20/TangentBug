import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('nav_bringup')
    pkg_tb = get_package_share_directory('nav_tangent_bug')

    # Sobe simulação com obstáculos
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'sim_obstacles.launch.py')
        )
    )

    # Nó do Tangent Bug (servidor de action)
    tangent_bug = Node(
        package='nav_tangent_bug',
        executable='tangent_bug',
        name='tangent_bug',
        output='screen',
        parameters=[
            os.path.join(pkg_tb, 'config', 'tangent_bug.yaml')
        ]
    )

    return LaunchDescription([
        sim,
        tangent_bug,
    ])