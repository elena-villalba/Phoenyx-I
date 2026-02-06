import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # --- Paths to launch files ---
    osr_bringup_dir = get_package_share_directory('osr_bringup')
    perception_dir = get_package_share_directory('osr_perception_challenge')

    osr_mod_launch = os.path.join(
        osr_bringup_dir,
        'launch',
        'osr_mod_launch.py'
    )

    perception_launch = os.path.join(
        perception_dir,
        'launch',
        'perception.launch.py'
    )

    # --- Include OSR launch ---
    osr = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(osr_mod_launch)
    )

    # --- Include perception launch after delay ---
    perception = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg='[osr_bringup] Waiting 10 seconds before starting perception...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(perception_launch)
            )
        ]
    )

    return LaunchDescription([
        LogInfo(msg='[osr_bringup] Starting OSR bringup...'),
        osr,
        perception
    ])
