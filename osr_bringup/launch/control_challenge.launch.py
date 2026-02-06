import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # =========================
    # Launch arguments
    # =========================

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )
    rviz = LaunchConfiguration('rviz')

    # =========================
    # OSR bringup (REAL)
    # =========================

    osr_mod_launch = os.path.join(
        get_package_share_directory('osr_bringup'),
        'launch',
        'osr_mod_launch.py'
    )

    osr_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(osr_mod_launch)
    )

    # =========================
    # RPLIDAR launch (REAL) after 10s
    # =========================

    rplidar_launch_path = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch',
        'rplidar.launch.py'   # <-- ajusta si tu archivo se llama diferente
    )

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch_path)
    )

    rplidar_delayed = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg='[osr_control_real] Waiting 10 seconds before starting RPLIDAR...'),
            rplidar_launch
        ]
    )

    # =========================
    # Maze navigation node (REAL)
    # =========================

    maze_navigation_node = Node(
        package='osr_control_challenge',
        executable='maze_navigation',
        name='maze_navigation',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # =========================
    # RViz (optional)
    # =========================

    rviz_config = os.path.join(
        get_package_share_directory('osr_bringup'),
        'rviz',
        'nav_config.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(rviz),
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
    )

    # =========================
    # Launch description
    # =========================

    return LaunchDescription([
        rviz_arg,

        LogInfo(msg='[osr_control_real] Starting OSR bringup (REAL)...'),
        osr_bringup,

        rplidar_delayed,

        LogInfo(msg='[osr_control_real] Starting maze_navigation (REAL)...'),
        maze_navigation_node,

        rviz_node,
    ])
