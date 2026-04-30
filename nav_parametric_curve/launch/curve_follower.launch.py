import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('nav_bringup')
    pkg_curve = get_package_share_directory('nav_parametric_curve')

    # Sobe a simulação com mundo vazio (sem obstáculos para o exercício 2)
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'sim_empty.launch.py')
        )
    )

    # Sobe o nó do curve follower com os parâmetros do YAML
    curve_follower = Node(
        package='nav_parametric_curve',
        executable='curve_follower',
        name='curve_follower',
        output='screen',
        parameters=[
            os.path.join(pkg_curve, 'config', 'curve_params.yaml')
        ]
    )

    return LaunchDescription([
        sim,
        curve_follower,
    ])