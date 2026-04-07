import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_nav_bringup = get_package_share_directory('nav_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Caminhos para arquivos do pacote
    urdf_file = os.path.join(pkg_nav_bringup, 'urdf', 'diff_robot.urdf.xacro')
    world_file = os.path.join(pkg_nav_bringup, 'worlds', 'empty.world')
    bridge_config = os.path.join(pkg_nav_bringup, 'config', 'bridge_params.yaml')

    # Argumento opcional: usar sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Processa o xacro para URDF em memória
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    robot_description = {'robot_description': robot_description_content,
                         'use_sim_time': use_sim_time}

    # Sobe o robot_state_publisher: publica /tf dos links do URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Inclui o launch do Gazebo que vem com ros_gz_sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items()
    )

    # Spawna o robô no Gazebo a partir do tópico /robot_description
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'diff_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
        ],
        output='screen'
    )

    # Bridge ROS <-> Gazebo usando o YAML
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': use_sim_time,
        }],
        output='screen'
    )

    rviz_config = os.path.join(pkg_nav_bringup, 'config', 'rviz', 'nav.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
        rviz,
    ])