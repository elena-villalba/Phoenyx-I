from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declarar argumentos
    simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='true',
        description='Si se ejecuta en modo simulación'
    )
    
    # Obtener el valor del argumento
    simulation = LaunchConfiguration('simulation')
    
    # Ruta al launch de simulación de Gazebo
    sim_gazebo_launch = os.path.join(
        get_package_share_directory('osr_bringup'),
        'launch',
        'maze_simulation.launch.py'
    )
    
    # Incluir el launch de la simulación
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_gazebo_launch)
    )
    
    # Nodo de navegación del laberinto
    maze_navigation_node = Node(
        package='osr_control_challenge',
        executable='maze_navigation',
        name='maze_navigation',
        output='screen',
        parameters=[{
            'simulation': simulation
        }]
    )
    
    return LaunchDescription([
        simulation_arg,
        gazebo_launch,
        maze_navigation_node
    ])