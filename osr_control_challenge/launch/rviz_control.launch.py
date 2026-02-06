import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    osr_urdf_path = os.path.join(get_package_share_directory('osr_gazebo'))

    xacro_file = os.path.join(osr_urdf_path, 'urdf', 'osr_simplified.urdf.xacro')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)

    params = {
        'robot_description': doc.toxml(),
        'use_sim_time': False,
    }

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    # Si en real NO publicas /joint_states, activa esto:
    # jsp = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     output='screen',
    # )

    rviz_config = os.path.join(osr_urdf_path, 'rviz', 'osr_gazebo.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        rsp,
        # jsp,  # descomenta si lo necesitas
        rviz,
    ])
