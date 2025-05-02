from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # === Rutas ===
    pkg_f450 = get_package_share_directory('f450_description')
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')

    urdf_path = os.path.join(pkg_f450, 'urdf', 'f450.urdf')
    world_path = os.path.join(pkg_f450, 'worlds', 'empty_world.world')

    # Leer URDF directamente
    with open(urdf_path, 'r') as f:
        robot_urdf = f.read()

    # === Nodos ===
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_urdf}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )

    # === Lanzar Gazebo con el mundo que ya incluye el dron ===
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': world_path
        }.items()
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_launch
    ])
