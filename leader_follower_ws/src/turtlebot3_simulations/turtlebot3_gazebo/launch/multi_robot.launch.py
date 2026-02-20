#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool, HyunGyu Kim

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace


# ★ ArUco marker injection for the leader robot
def add_aruco_marker_to_sdf(root):
    model = root.find('.//model')
    if model is None:
        model = root

    # ── ArUco marker link ──
    marker_link = ET.SubElement(model, 'link')
    marker_link.set('name', 'aruco_marker_link')

    # ABSOLUTE pose in model frame (no relative_to — Gazebo Classic doesn't support it)
    # Waffle: back edge ~x=-0.14, top plate ~z=0.14
    # So x=-0.18 clears the back, z=0.20 sits above the top plate
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

    # Use direct URI to the texture image instead of Ogre material script
    material = ET.SubElement(visual, 'material')
    script = ET.SubElement(material, 'script')
    uri1 = ET.SubElement(script, 'uri')
    uri1.text = 'model://aruco_marker_42/materials/scripts'
    uri2 = ET.SubElement(script, 'uri')
    uri2.text = 'model://aruco_marker_42/materials/textures'
    mat_name = ET.SubElement(script, 'name')
    mat_name.text = 'ArUcoMarker/Marker42'

    # ── Fixed joint ──
    joint = ET.SubElement(model, 'joint')
    joint.set('name', 'aruco_marker_joint')
    joint.set('type', 'fixed')
    parent = ET.SubElement(joint, 'parent')
    parent.text = 'base_link'
    child = ET.SubElement(joint, 'child')
    child.text = 'aruco_marker_link'


def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    number_of_robots = 2
    namespace = 'TB3'
    # pose = [[1.0, 0.0], [-0.5, 0.0], [2, 0.5], [-0.5, 2]]
    pose = [[-1.5, -0.5], [-2.5, -0.5], [2, 0.5], [-0.5, 2]]
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )
    save_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'tmp'
    )
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        # 'empty_world.world'
        'turtlebot3_world.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd_list = []

    for count in range(number_of_robots):
        robot_state_publisher_cmd_list.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'frame_prefix': f'{namespace}_{count+1}'
                    }.items()
            )
        )

    spawn_turtlebot_cmd_list = []

    for count in range(number_of_robots):
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        for odom_frame_tag in root.iter('odometry_frame'):
            odom_frame_tag.text = f'{namespace}_{count+1}/odom'
        for base_frame_tag in root.iter('robot_base_frame'):
            base_frame_tag.text = f'{namespace}_{count+1}/base_footprint'
        for scan_frame_tag in root.iter('frame_name'):
            scan_frame_tag.text = f'{namespace}_{count+1}/base_scan'

        # ★ ADD ARUCO MARKER TO THE LEADER (TB3_1 = count 0) ONLY
        if count == 0:
            add_aruco_marker_to_sdf(root)

        urdf_modified = ET.tostring(tree.getroot(), encoding='unicode')
        urdf_modified = '<?xml version="1.0" ?>\n'+urdf_modified
        with open(f'{save_path}{count+1}.sdf', 'w') as file:
            file.write(urdf_modified)

        spawn_turtlebot_cmd_list.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_file_dir, 'multi_spawn_turtlebot3.launch.py')
                ),
                launch_arguments={
                        'x_pose': str(pose[count][0]),
                        'y_pose': str(pose[count][1]),
                        'robot_name': f'{TURTLEBOT3_MODEL}_{count+1}',
                        'namespace': f'{namespace}_{count+1}',
                        'sdf_path': f'{save_path}{count+1}.sdf'
                }.items()
            )
        )

    ld = LaunchDescription()
    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(RegisterEventHandler(
        OnShutdown(
            on_shutdown=lambda event,
            context: [os.remove(f'{save_path}{count+1}.sdf') for count in range(number_of_robots)]
        )
    ))
    for count, spawn_turtlebot_cmd in enumerate(spawn_turtlebot_cmd_list, start=1):
        ld.add_action(GroupAction([PushRosNamespace(f'{namespace}_{count}'),
                                  robot_state_publisher_cmd_list[count-1],
                                  spawn_turtlebot_cmd]))

    return ld
