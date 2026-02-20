#!/usr/bin/env python3
"""
ArUco Follower World Launch
- Empty world
- TB3_1 (leader) at (1.0, 0) facing +x, with ArUco marker on back
- TB3_2 (follower) at (-0.5, 0) facing +x, camera sees leader's marker
"""

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def add_aruco_marker_to_sdf(root):
    """Inject ArUco marker link onto the leader robot's back."""
    model = root.find('.//model')
    if model is None:
        model = root

    marker_link = ET.SubElement(model, 'link')
    marker_link.set('name', 'aruco_marker_link')

    pose = ET.SubElement(marker_link, 'pose')
    pose.text = '-0.18 0 0.20 0 -1.5708 0'

    inertial = ET.SubElement(marker_link, 'inertial')
    mass = ET.SubElement(inertial, 'mass')
    mass.text = '0.001'
    inertia = ET.SubElement(inertial, 'inertia')
    for tag, val in [('ixx','1e-7'),('ixy','0'),('ixz','0'),
                     ('iyy','1e-7'),('iyz','0'),('izz','1e-7')]:
        el = ET.SubElement(inertia, tag)
        el.text = val

    visual = ET.SubElement(marker_link, 'visual')
    visual.set('name', 'aruco_visual')
    geom = ET.SubElement(visual, 'geometry')
    box = ET.SubElement(geom, 'box')
    size = ET.SubElement(box, 'size')
    size.text = '0.15 0.15 0.001'

    material = ET.SubElement(visual, 'material')
    script = ET.SubElement(material, 'script')
    uri1 = ET.SubElement(script, 'uri')
    uri1.text = 'model://aruco_marker_42/materials/scripts'
    uri2 = ET.SubElement(script, 'uri')
    uri2.text = 'model://aruco_marker_42/materials/textures'
    mat_name = ET.SubElement(script, 'name')
    mat_name.text = 'ArUcoMarker/Marker42'

    joint = ET.SubElement(model, 'joint')
    joint.set('name', 'aruco_marker_joint')
    joint.set('type', 'fixed')
    parent = ET.SubElement(joint, 'parent')
    parent.text = 'base_link'
    child = ET.SubElement(joint, 'child')
    child.text = 'aruco_marker_link'


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    model = os.environ.get('TURTLEBOT3_MODEL', 'waffle')
    model_folder = os.path.join(turtlebot3_gazebo, 'models', f'turtlebot3_{model}')
    sdf_file = os.path.join(model_folder, 'model.sdf')
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'urdf',
        f'turtlebot3_{model}.urdf')

    # Robot configs: name, x, y, yaw, is_leader
    robots = [
        ('TB3_1', 1.0, 0.0, 0.0, True),    # leader, facing +x
        ('TB3_2', -0.5, 0.0, 0.0, False),   # follower, facing +x (sees leader's back)
    ]

    ld = LaunchDescription()

    # ── Gazebo: empty world ──
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': os.path.join(
            turtlebot3_gazebo, 'worlds', 'empty_world.world')}.items(),
    )
    ld.add_action(gazebo)

    for i, (ns, x, y, yaw, is_leader) in enumerate(robots):
        # Read and modify SDF
        tree = ET.parse(sdf_file)
        root = tree.getroot()

        # Fix frame names for namespace
        for frame_tag in root.iter('frame_id'):
            frame_tag.text = f'{ns}/{frame_tag.text}'
        for child_tag in root.iter('child'):
            if child_tag.text and 'frame' in child_tag.text.lower():
                child_tag.text = f'{ns}/{child_tag.text}'
        # Remap odom
        for plugin in root.iter('plugin'):
            for odom_frame in plugin.iter('odometry_frame'):
                odom_frame.text = f'{ns}/odom'
            for robot_base in plugin.iter('robot_base_frame'):
                robot_base.text = f'{ns}/base_footprint'

        # Add ArUco marker to leader only
        if is_leader:
            add_aruco_marker_to_sdf(root)

        # Write tmp SDF
        tmp_sdf = os.path.join(model_folder, f'tmp{i+1}.sdf')
        tree.write(tmp_sdf)

        # Robot state publisher
        with open(urdf_path, 'r') as f:
            urdf_content = f.read()

        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=ns,
            parameters=[{
                'use_sim_time': True,
                'robot_description': urdf_content,
                'frame_prefix': f'{ns}/',
            }],
            output='screen',
        )

        # Spawn with delay to avoid race
        spawn = TimerAction(
            period=float(i * 3),
            actions=[Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                namespace=ns,
                arguments=[
                    '-entity', f'{model}_{i+1}',
                    '-file', tmp_sdf,
                    '-x', str(x), '-y', str(y), '-z', '0.01',
                    '-Y', str(yaw),
                    '-robot_namespace', ns,
                ],
                output='screen',
            )],
        )

        ld.add_action(rsp)
        ld.add_action(spawn)

    return ld
