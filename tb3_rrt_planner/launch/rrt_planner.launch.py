import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    pkg = get_package_share_directory('tb3_rrt_planner')

    # ------------------------------------------------------------------ args
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg, 'maps', 'office_map.yaml'),
        description='Full path to the map YAML file (YAML + PGM pair)',
    )
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use Gazebo simulation clock',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file     = LaunchConfiguration('map')

    # -------------------------------------------------------------- map server
    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[{
            'yaml_filename': map_file,
            'use_sim_time':  use_sim_time,
        }],
    )

    # Lifecycle manager activates map_server automatically
    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart':    True,
            'node_names':   ['map_server'],
        }],
    )

    # -------------------------------------------------------------- RRT node
    rrt_node = Node(
        package='tb3_rrt_planner',
        executable='rrt_planner_node',
        name='rrt_planner_node',
        output='screen',
        parameters=[
            os.path.join(pkg, 'config', 'rrt_params.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    # --------------------------------------------------------------- RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg, 'rviz', 'rrt.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    map_delayed = TimerAction(period=3.0, actions=[map_server])

    return LaunchDescription([
        map_arg,
        sim_time_arg,
        lifecycle_mgr,
        rviz_node,
        rrt_node,
        map_delayed,
    ])
