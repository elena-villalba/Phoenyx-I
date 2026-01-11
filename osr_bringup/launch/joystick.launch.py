from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory 
import os 

def generate_launch_description():

    # Path to the joystick configuration file
    joy_params = os.path.join(
        get_package_share_directory('osr_bringup'),
        'config',
        'joystick.yaml'
    )

    # Joystick driver nod
    joy_node = Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            emulate_tty=True,
            respawn=True,
            parameters=[joy_params],
    )

    # Joystick teleoperation node (Twist commands)
    teleop_twist_joy_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            emulate_tty=True,
            respawn=True,
            parameters=[joy_params]
    )

    # Launch both joystick-related nodes
    return LaunchDescription([
        joy_node,
        teleop_twist_joy_node
    ])