from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from ros_gz_bridge.actions import RosGzBridge  # Importando el ros_gz_bridge
import os
import xacro

def generate_launch_description():
    # Declarar los argumentos para el bridge y el archivo de configuración YAML
    bridge_name = LaunchConfiguration('bridge_name')
    config_file = LaunchConfiguration('config_file')

    declare_bridge_name_cmd = DeclareLaunchArgument(
        'bridge_name', description='Name of ros_gz_bridge node'
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', description='Path to YAML config file'
    )

    # === Rutas ===
    pkg_f450 = get_package_share_directory('f450_base')
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')
    pkg_ros_gz_bridge = get_package_share_directory('ros_gz_bridge')  # Paquete ros_gz_bridge
    world_path = os.path.join(pkg_f450, 'worlds', 'empty_world.world')  # Mundo vacío que ya contiene el modelo f450
    urdf_path = os.path.join(pkg_f450, 'urdf', 'f450b.urdf')  # Ruta a tu archivo URDF ya existente

    # Leer el archivo URDF y cargarlo como string
    with open(urdf_path, 'r') as urdf_file:
        robot_urdf = urdf_file.read()

    # === Nodos ROS 2 ===
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description':robot_urdf}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # === Lanzar Gazebo con mundo personalizado usando gz_args ===
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': world_path
        }.items()
    )

    # === Agregar el ros_gz_bridge ===
    ros_gz_bridge_node = RosGzBridge(
        bridge_name=bridge_name,
        config_file=config_file
    )

    # === Finalizar lanzamiento ===
    return LaunchDescription([
        declare_bridge_name_cmd,
        declare_config_file_cmd,
        gazebo_launch,  # Inicia Gazebo
        robot_state_publisher_node,  # Publica la descripción del robot
        joint_state_publisher_node,  # Publica el estado de las articulaciones
        ros_gz_bridge_node  # Conecta Gazebo con ROS 2 a través del bridge
    ])
