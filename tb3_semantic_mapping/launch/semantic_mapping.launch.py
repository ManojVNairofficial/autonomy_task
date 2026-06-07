"""
Section 2: Agentic Semantic Reasoning.

Usage:
  export TURTLEBOT3_MODEL=waffle_pi
  ros2 launch tb3_semantic_mapping semantic_mapping.launch.py

Then, in a second terminal, to query a place:
  ros2 service call /query_place tb3_semantic_interfaces/srv/QueryPlace "{query: 'Where is the toilet?'}"

"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('tb3_semantic_mapping')
    slam_pkg = get_package_share_directory('tb3_exploration_slam')
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    world_file = os.path.join(pkg, 'worlds', 'office_building.world')
    nav2_params = os.path.join(slam_pkg, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(slam_pkg, 'config', 'slam_params.yaml')
    rviz_cfg = os.path.join(pkg, 'rviz', 'semantic.rviz')
    models_dir = os.path.join(pkg, 'models')

    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    # ------------------------------------ env ----------------------------------- #
    set_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi')
    set_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        models_dir + ':' + os.environ.get('GAZEBO_MODEL_PATH', ''),
    )

    # ----------------------------------- args ----------------------------------- #
    x_arg = DeclareLaunchArgument('x_pose', default_value='0.0')
    y_arg = DeclareLaunchArgument('y_pose', default_value='0.0')

    # ---------------------------------- Gazebo ---------------------------------- #
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_file}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
    )
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
    )

    # ----------------------------------- SLAM ----------------------------------- #
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')),
        launch_arguments={'slam_params_file': slam_params,
                          'use_sim_time': 'True'}.items(),
    )

    # ----------------------------------- Nav2 ----------------------------------- #
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': 'True',
                          'params_file': nav2_params,
                          'autostart': 'True'}.items(),
    )

    # ---------------------------- Section 1 explorer ---------------------------- #
    explorer = Node(
        package='tb3_exploration_slam',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'min_frontier_size': 5,
            'goal_timeout_sec': 30.0,
            'tried_goal_radius': 0.3,
            'min_frontier_dist': 0.25,
            'max_stuck_resets': 8,
            'explore_hz': 1.0,
        }],
    )

    # ------------------------------ semantic nodes ------------------------------ #
    tagger = Node(
        package='tb3_semantic_mapping',
        executable='semantic_tagger',
        name='semantic_tagger',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'vlm_backend': 'mock',
            'camera_topic': '/camera/image_raw',
            'process_period': 0.5,
            'min_confidence': 0.65,
        }],
    )
    query = Node(
        package='tb3_semantic_mapping',
        executable='semantic_query',
        name='semantic_query',
        output='screen',
        parameters=[{'use_sim_time': True, 'auto_navigate': True}],
    )

    # ----------------------------------- RViz ----------------------------------- #
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': True}],
    )

    nav2_delayed = TimerAction(period=10.0, actions=[nav2])
    rviz_delayed = TimerAction(period=8.0, actions=[rviz])
    explorer_delayed = TimerAction(period=22.0, actions=[explorer])
    semantic_delayed = TimerAction(period=12.0, actions=[tagger, query])

    return LaunchDescription([
        set_model,
        set_model_path,
        x_arg,
        y_arg,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_robot,
        slam,
        nav2_delayed,
        rviz_delayed,
        semantic_delayed,
        explorer_delayed,
    ])
