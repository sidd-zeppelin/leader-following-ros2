#!/usr/bin/env python3
"""
Launch nav2 stack for TB3_1 (leader) with proper namespacing.
Provides map_server, AMCL, and full nav2 planning.
Send goals via RViz to navigate the leader autonomously.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    leader_follower_dir = os.path.join(
        os.path.expanduser('~'), 'turtlebot_ws', 'src', 'leader_follower_bringup'
    )

    # Map file for turtlebot3_world
    map_file = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'), 'map', 'map.yaml'
    )

    # Our custom nav2 params
    params_file = os.path.join(leader_follower_dir, 'config', 'nav2_leader_params.yaml')

    namespace = 'TB3_1'

    # TF: map -> TB3_1/odom (published by AMCL)
    # TF: TB3_1/odom -> TB3_1/base_footprint (published by Gazebo)
    # But nav2 uses unqualified frame names internally, so we need
    # a static transform from "map" to namespace or remap frames.

    # The trick: nav2 with namespace auto-prefixes topics but NOT frames.
    # We need frame remapping. The Gazebo plugin publishes:
    #   TB3_1/odom -> TB3_1/base_footprint
    # But nav2 expects:
    #   odom -> base_footprint (within namespace)
    # Solution: use tf_remap or set frame params to TB3_1/ prefixed versions

    return LaunchDescription([
        # Static transform: map -> TB3_1/map (identity)
        # This bridges the global "map" frame to the namespaced one
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_ns_map',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'TB3_1/map'],
            output='screen',
        ),

        # Map server (global, not namespaced)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'yaml_filename': map_file,
                'frame_id': 'map',
            }],
        ),

        # AMCL (namespaced)
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'base_frame_id': 'TB3_1/base_footprint',
                'global_frame_id': 'map',
                'odom_frame_id': 'TB3_1/odom',
                'scan_topic': 'scan',
                'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
                'set_initial_pose': True,
                'initial_pose': {'x': -1.5, 'y': -0.5, 'yaw': 0.0},
                'max_particles': 2000,
                'min_particles': 500,
                'tf_broadcast': True,
            }],
        ),

        # Nav2 controller server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace=namespace,
            output='screen',
            parameters=[params_file],
            remappings=[
                ('cmd_vel', 'cmd_vel'),
                ('odom', 'odom'),
            ],
        ),

        # Nav2 planner
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace=namespace,
            output='screen',
            parameters=[params_file],
        ),

        # Nav2 behaviors (recovery)
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            namespace=namespace,
            output='screen',
            parameters=[params_file],
        ),

        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            namespace=namespace,
            output='screen',
            parameters=[params_file],
        ),

        # Smoother
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            namespace=namespace,
            output='screen',
            parameters=[params_file],
        ),

        # Velocity smoother
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            namespace=namespace,
            output='screen',
            remappings=[
                ('cmd_vel', 'cmd_vel'),
                ('cmd_vel_smoothed', 'cmd_vel'),
            ],
            parameters=[params_file],
        ),

        # Lifecycle manager to activate all nodes
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'controller_server',
                    'smoother_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'velocity_smoother',
                ],
            }],
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(leader_follower_dir, 'config', 'leader_nav2.rviz')],
            parameters=[{'use_sim_time': True}],
        ),
    ])
