#!/usr/bin/env python3
"""
Patch TurtleBot3 Waffle SDF to Add ArUco Marker
═════════════════════════════════════════════════

This script takes the ORIGINAL TurtleBot3 waffle model.sdf and creates
a PATCHED copy with an ArUco marker attached to the back of the robot.

It works by injecting an additional <link> and <joint> into the SDF XML,
right before the closing </model> tag.

Usage:
    python3 patch_leader_sdf.py

    # Or with custom paths:
    python3 patch_leader_sdf.py \
        --input ~/turtlebot_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf \
        --output ~/turtlebot_ws/src/leader_follower_bringup/models/leader_waffle/model.sdf

What it adds:
    - A 10cm × 10cm thin plate on the back of the robot (x=-0.10, z=0.08)
    - The plate is textured with ArUco marker ID 42 (DICT_4X4_50)
    - It faces backward (-X direction) so the follower can see it from behind
    - It's attached via a fixed joint to base_link

    ┌──────────────────────────────────┐
    │  TOP VIEW of leader (waffle)     │
    │                                  │
    │         front (+X)               │
    │            ▲                     │
    │     ┌──────┼──────┐             │
    │     │      │      │             │
    │     │   [robot]   │             │
    │     │      │      │             │
    │  ▓▓▓├──────┘      │  ▓▓▓=ArUco │
    │     └─────────────┘    marker   │
    │         back (-X)               │
    └──────────────────────────────────┘
"""

import argparse
import os
import shutil
import sys


# ── The SDF snippet to inject ──────────────────────────────────────
# This gets inserted right before </model> in the waffle SDF.
# The marker is a thin textured box, attached to base_link via fixed joint.
#
# Position: x=-0.10 (behind center), z=0.08 (above ground)
# Orientation: rotated so the textured face points backward (-X)

ARUCO_MARKER_SDF_SNIPPET = """
    <!-- ═══════ ArUco Marker (added by patch_leader_sdf.py) ═══════ -->
    <link name="aruco_marker_link">
      <pose relative_to="base_link">-0.10 0 0.08 0 -1.5708 0</pose>
      <inertial>
        <mass>0.001</mass>
        <inertia>
          <ixx>1e-7</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>1e-7</iyy><iyz>0</iyz><izz>1e-7</izz>
        </inertia>
      </inertial>
      <visual name="aruco_visual">
        <geometry>
          <box>
            <size>0.10 0.10 0.001</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>model://aruco_marker_42/materials/scripts</uri>
            <uri>model://aruco_marker_42/materials/textures</uri>
            <name>ArUcoMarker/Marker42</name>
          </script>
        </material>
      </visual>
    </link>

    <joint name="aruco_marker_joint" type="fixed">
      <parent>base_link</parent>
      <child>aruco_marker_link</child>
    </joint>
    <!-- ═══════ End ArUco Marker ═══════ -->
"""


def find_waffle_sdf():
    """Try common locations for the TurtleBot3 waffle model.sdf."""
    candidates = [
        # Source workspace (user's setup)
        os.path.expanduser(
            "~/turtlebot_ws/install/turtlebot3_gazebo/share/"
            "turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf"
        ),
        # Apt-installed
        "/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf",
        # Another common source path
        os.path.expanduser(
            "~/turtlebot_ws/src/turtlebot3_simulations/turtlebot3_gazebo/"
            "models/turtlebot3_waffle/model.sdf"
        ),
    ]
    for path in candidates:
        if os.path.isfile(path):
            return path
    return None


def patch_sdf(input_path, output_path):
    """Read original SDF, inject ArUco marker, write patched SDF."""

    with open(input_path, "r") as f:
        sdf_content = f.read()

    # Verify it's a valid TurtleBot3 SDF
    if "base_link" not in sdf_content:
        print(f"ERROR: {input_path} doesn't look like a TurtleBot3 SDF (no base_link found)")
        sys.exit(1)

    # Check if already patched
    if "aruco_marker_link" in sdf_content:
        print(f"WARNING: {input_path} already contains an ArUco marker. Overwriting output anyway.")

    # Inject the marker snippet right before the LAST </model> tag
    # (There should only be one, but let's be safe)
    last_model_close = sdf_content.rfind("</model>")
    if last_model_close == -1:
        print("ERROR: No </model> tag found in SDF file!")
        sys.exit(1)

    patched = (
        sdf_content[:last_model_close]
        + ARUCO_MARKER_SDF_SNIPPET
        + "\n  "
        + sdf_content[last_model_close:]
    )

    # Create output directory if needed
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    with open(output_path, "w") as f:
        f.write(patched)

    return True


def main():
    parser = argparse.ArgumentParser(
        description="Patch TurtleBot3 waffle SDF to add ArUco marker on the back"
    )
    parser.add_argument(
        "--input", "-i",
        help="Path to original TurtleBot3 waffle model.sdf (auto-detected if not given)",
    )
    parser.add_argument(
        "--output", "-o",
        help="Path for patched output SDF (default: leader_waffle.sdf in current dir)",
        default=None,
    )
    args = parser.parse_args()

    # Find input SDF
    if args.input:
        input_path = args.input
    else:
        input_path = find_waffle_sdf()
        if input_path is None:
            print("ERROR: Could not auto-detect TurtleBot3 waffle model.sdf")
            print("Please specify with: --input /path/to/model.sdf")
            print("")
            print("Common locations:")
            print("  ~/turtlebot_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf")
            print("  /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf")
            sys.exit(1)

    if not os.path.isfile(input_path):
        print(f"ERROR: Input file not found: {input_path}")
        sys.exit(1)

    # Determine output path
    if args.output:
        output_path = args.output
    else:
        # Default: put it next to the script or in current dir
        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_path = os.path.join(script_dir, "..", "models", "leader_waffle", "leader_waffle.sdf")
        output_path = os.path.normpath(output_path)

    print(f"Input  (original) : {input_path}")
    print(f"Output (patched)  : {output_path}")
    print("")

    # Also copy the original model.config if it exists
    input_dir = os.path.dirname(input_path)
    output_dir = os.path.dirname(output_path)
    config_src = os.path.join(input_dir, "model.config")
    if os.path.isfile(config_src):
        os.makedirs(output_dir, exist_ok=True)
        config_dst = os.path.join(output_dir, "model.config")
        if not os.path.isfile(config_dst):
            shutil.copy2(config_src, config_dst)
            print(f"Copied model.config to {config_dst}")

    # Patch!
    patch_sdf(input_path, output_path)

    print("")
    print("SUCCESS! Patched SDF created.")
    print("")
    print("What was added:")
    print("  - ArUco marker (ID 42, DICT_4X4_50, 10cm × 10cm)")
    print("  - Position: back of robot (x=-0.10, z=0.08)")
    print("  - Faces backward so follower camera can see it")
    print("")
    print("Next steps:")
    print("  1. Make sure GAZEBO_MODEL_PATH includes the aruco_marker_42 model dir")
    print("  2. Update your multi_robot.launch.py to use this SDF for the leader")
    print(f"     Change the leader's SDF path to: {output_path}")
    print("")
    print("  Or test with direct spawn:")
    print(f"    ros2 run gazebo_ros spawn_entity.py -entity leader -file {output_path} -x 0 -y 0 -z 0.01")


if __name__ == "__main__":
    main()
