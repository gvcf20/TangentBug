import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def spawn_controllers(context, *args, **kwargs):
    """Cria N instâncias do multi_robot_node."""
    pkg_multi = get_package_share_directory('nav_multi_robot')
    n_robots = int(LaunchConfiguration('n_robots').perform(context))
    params_file = os.path.join(pkg_multi, 'config', 'multi_robot_params.yaml')

    nodes = []
    for i in range(n_robots):
        node = Node(
            package='nav_multi_robot',
            executable='multi_robot_node',
            name=f'multi_robot_{i}',
            output='screen',
            parameters=[
                params_file,
                {'robot_id': i, 'n_robots': n_robots}
            ],
            # Remap para que o YAML 'multi_robot_common' funcione
            remappings=[]
        )
        nodes.append(node)
    return nodes


def generate_launch_description():
    pkg_bringup = get_package_share_directory('nav_bringup')

    declare_n = DeclareLaunchArgument(
        'n_robots', default_value='2')

    # Sobe Gazebo + N robôs + bridges
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'sim_multi_robot.launch.py')
        ),
        launch_arguments={'n_robots': LaunchConfiguration('n_robots')}.items()
    )

    # Sobe N controladores
    controllers = OpaqueFunction(function=spawn_controllers)

    return LaunchDescription([
        declare_n,
        sim,
        controllers,
    ])