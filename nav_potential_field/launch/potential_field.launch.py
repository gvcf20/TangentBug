import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('nav_bringup')
    pkg_pf = get_package_share_directory('nav_potential_field')

    # Sobe simulação com obstáculos
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'sim_obstacles.launch.py')
        )
    )

    # Nó do campo potencial
    potential_field = Node(
        package='nav_potential_field',
        executable='potential_field',
        name='potential_field',
        output='screen',
        parameters=[
            os.path.join(pkg_pf, 'config', 'potential_params.yaml')
        ]
    )

    return LaunchDescription([
        sim,
        potential_field,
    ])