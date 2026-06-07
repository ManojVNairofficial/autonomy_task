"""
Save the current SLAM map to ~/maps/<map_name>.{pgm,yaml}.

IMPORTANT: exploration_slam.launch.py must be running in another terminal first.

  Terminal 1:  ros2 launch tb3_exploration_slam exploration_slam.launch.py
  Terminal 2:  ros2 launch tb3_exploration_slam save_map.launch.py
               ros2 launch tb3_exploration_slam save_map.launch.py map_name:=office_map
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

_MAPS_DIR = os.path.expanduser('~/maps')
os.makedirs(_MAPS_DIR, exist_ok=True)


def generate_launch_description():
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='exploration_map',
        description='Output base name (no extension); saved to ~/maps/<map_name>.pgm/.yaml',
    )
    map_name = LaunchConfiguration('map_name')

    save_map = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', [_MAPS_DIR + '/', map_name],
            '--ros-args', '-p', 'save_map_timeout:=10.0',
        ],
        output='screen',
    )

    return LaunchDescription([
        map_name_arg,
        save_map,
    ])
