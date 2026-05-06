import os
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    OpaqueFunction, GroupAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def spawn_robots(context, *args, **kwargs):
    """Gera ações para spawnar N robôs com namespaces."""
    pkg_bringup = get_package_share_directory('nav_bringup')
    n_robots = int(LaunchConfiguration('n_robots').perform(context))

    # Posições iniciais dos robôs (espalhados ao redor da origem)
    import math
    spawn_positions = []
    for i in range(n_robots):
        angle = 2 * math.pi * i / n_robots
        x = 3.0 * math.cos(angle)  # raio 3m, fora da lemniscata (a=2)
        y = 3.0 * math.sin(angle)
        spawn_positions.append((x, y))

    # Lê o URDF base
    urdf_path = os.path.join(pkg_bringup, 'urdf', 'turtlebot3_burger.urdf')
    with open(urdf_path, 'r') as f:
        urdf_base = f.read()

    actions = []

    for i in range(n_robots):
        ns = f'robot_{i}'
        x, y = spawn_positions[i]

        # Modifica o URDF para este robô: prefixa os tópicos dos plugins
        urdf_robot = urdf_base
        urdf_robot = urdf_robot.replace(
            '<topic>cmd_vel</topic>',
            f'<topic>{ns}/cmd_vel</topic>')
        urdf_robot = urdf_robot.replace(
            '<odom_topic>odom</odom_topic>',
            f'<odom_topic>{ns}/odom</odom_topic>')
        urdf_robot = urdf_robot.replace(
            '<tf_topic>tf</tf_topic>',
            f'<tf_topic>{ns}/tf</tf_topic>')
        urdf_robot = urdf_robot.replace(
            '<topic>joint_states</topic>',
            f'<topic>{ns}/joint_states</topic>')
        urdf_robot = urdf_robot.replace(
            '<topic>scan</topic>',
            f'<topic>{ns}/scan</topic>')

        # Robot state publisher (no namespace)
        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'rsp_{ns}',
            output='screen',
            parameters=[{
                'robot_description': urdf_robot,
                'frame_prefix': f'{ns}/',
                'use_sim_time': True,
            }]
        )

        # Spawn no Gazebo
        spawn = Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_{ns}',
            arguments=[
                '-string', urdf_robot,
                '-name', ns,
                '-x', str(x),
                '-y', str(y),
                '-z', '0.01',
            ],
            output='screen'
        )

        # Bridge para este robô
        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'bridge_{ns}',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                f'/{ns}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                f'/{ns}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                f'/{ns}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                f'/{ns}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                f'/{ns}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            ]
        )

        actions.extend([rsp, spawn, bridge])

    return actions


def generate_launch_description():
    pkg_bringup = get_package_share_directory('nav_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(pkg_bringup, 'worlds', 'obstacles_multi.world')
    rviz_config = os.path.join(pkg_bringup, 'config', 'rviz', 'nav.rviz')

    declare_n_robots = DeclareLaunchArgument(
        'n_robots', default_value='2',
        description='Número de robôs')

    # Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 2 {world_file}'}.items()
    )

    # Clock bridge (global)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Spawn dos robôs (via OpaqueFunction para ser dinâmico)
    spawn_action = OpaqueFunction(function=spawn_robots)

    return LaunchDescription([
        declare_n_robots,
        gz_sim,
        clock_bridge,
        rviz,
        spawn_action,
    ])