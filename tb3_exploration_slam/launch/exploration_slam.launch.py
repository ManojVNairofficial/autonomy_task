"""
Section 1: Exploration and SLAM.

Launches Gazebo + slam_toolbox + Nav2 + frontier explorer + RViz.

Usage:
  ros2 launch tb3_exploration_slam exploration_slam.launch.py
  ros2 launch tb3_exploration_slam exploration_slam.launch.py world:=turtlebot3_world x_pose:=1.0 y_pose:=0.0
  ros2 launch tb3_exploration_slam exploration_slam.launch.py world:=office x_pose:=0.0 y_pose:=0.0

Save the map while running (in a second terminal):
  ros2 launch tb3_exploration_slam save_map.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _gazebo_for_world(context, *args, **kwargs):

    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    world = LaunchConfiguration('world').perform(context)
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    if world == 'office':
        office_world = os.path.join(
            get_package_share_directory('tb3_semantic_mapping'),
            'worlds', 'office_building.world')
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
                launch_arguments={'world': office_world}.items()),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py'))),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')),
                launch_arguments={'use_sim_time': 'true'}.items()),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')),
                launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items()),
        ]

    launch_file = ('turtlebot3_world.launch.py'
                   if world == 'turtlebot3_world'
                   else 'turtlebot3_house.launch.py')
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_gazebo, 'launch', launch_file)),
            launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items()),
    ]


def generate_launch_description():
    pkg = get_package_share_directory('tb3_exploration_slam')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    nav2_params_file = os.path.join(pkg, 'config', 'nav2_params.yaml')
    slam_params_file = os.path.join(pkg, 'config', 'slam_params.yaml')


    try:
        office_models = os.path.join(
            get_package_share_directory('tb3_semantic_mapping'), 'models')
    except Exception:
        office_models = ''

    # ------------------------------------------------------------------ robot model

    set_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')
    set_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        office_models + ':' + os.environ.get('GAZEBO_MODEL_PATH', ''),
    )

    # ------------------------------------------------------------------ args
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='turtlebot3_house',
        description='Gazebo world: turtlebot3_house | turtlebot3_world | office',
    )
    x_arg = DeclareLaunchArgument('x_pose', default_value='-2.0')
    y_arg = DeclareLaunchArgument('y_pose', default_value='-0.5')

    # ------------------------------------------------------------------ Gazebo 
    gazebo = OpaqueFunction(function=_gazebo_for_world)

    # ------------------------------------------------------------------ SLAM toolbox (online async mapping)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': 'True',
        }.items(),
    )

    # ------------------------------------------------------------------ Nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': nav2_params_file,
            'autostart': 'True',
        }.items(),
    )

    # ------------------------------------------------------------------ Frontier explorer
    frontier_explorer = Node(
        package='tb3_exploration_slam',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'min_frontier_size': 5,
            'goal_timeout_sec': 30.0,
            'tried_goal_radius': 0.3,
            'min_frontier_dist': 0.20,
            'max_stuck_resets': 8,
            'explore_hz': 1.0,
        }],
    )

    # ------------------------------------------------------------------ RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg, 'rviz', 'exploration.rviz')],
        parameters=[{'use_sim_time': True}],
    )

    nav2_delayed = TimerAction(period=10.0, actions=[nav2])
    rviz_delayed = TimerAction(period=8.0, actions=[rviz])
    explorer_delayed = TimerAction(period=30.0, actions=[frontier_explorer])

    return LaunchDescription([
        set_model,
        set_model_path,
        world_arg,
        x_arg,
        y_arg,
        gazebo,
        slam,
        nav2_delayed,
        rviz_delayed,
        explorer_delayed,
    ])
