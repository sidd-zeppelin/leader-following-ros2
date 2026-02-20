# ArUco Marker Setup for Leader-Follower (SDF Approach)

## Your Setup

Based on your terminal output, you have:
- **Workspace:** `~/turtlebot_ws`
- **Multi-robot launch:** spawns two waffle bots as `TB3_1` and `TB3_2`
- **SDF-based spawning** from `tmp1.sdf` / `tmp2.sdf`
- **Topics:** `/TB3_1/cmd_vel`, `/TB3_1/camera/camera_info`, `/TB3_2/camera/image_raw`, etc.
- **ROS 2 Humble** + Gazebo Classic 11

The previous xacro approach failed because `turtlebot3_description` URDFs
aren't at the expected path. **This approach modifies the SDF directly** —
no xacro needed, works with your existing multi-robot launch.

---

## Files in This Package

```
leader_follower_bringup/
├── models/
│   └── aruco_marker_42/                # Gazebo model for the texture
│       ├── model.config
│       ├── model.sdf
│       └── materials/
│           ├── scripts/aruco_marker.material
│           └── textures/aruco_marker_42.png
├── scripts/
│   ├── patch_leader_sdf.py             # ★ Patches waffle SDF to add marker
│   ├── aruco_detector_test.py          # ★ Camera detection test node
│   └── generate_aruco_marker.py        # Regenerate marker image if needed
└── README.md
```

---

## Step-by-Step Setup

### Step 1: Extract into your workspace

```bash
cd ~/turtlebot_ws/src
tar xzf leader_follower_bringup.tar.gz
```

You should now have `~/turtlebot_ws/src/leader_follower_bringup/`.

### Step 2: Set GAZEBO_MODEL_PATH

Gazebo needs to find the ArUco marker texture model. Add this to your `~/.bashrc`:

```bash
# Add to ~/.bashrc:
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/turtlebot_ws/src/leader_follower_bringup/models
```

Then source it:
```bash
source ~/.bashrc
```

Verify:
```bash
ls ~/turtlebot_ws/src/leader_follower_bringup/models/aruco_marker_42/
# Should show: materials  model.config  model.sdf
```

### Step 3: Patch the leader's SDF

This is the key step. The script reads the original TurtleBot3 waffle `model.sdf`
and creates a patched copy with the ArUco marker added.

```bash
cd ~/turtlebot_ws/src/leader_follower_bringup

python3 scripts/patch_leader_sdf.py \
    --input ~/turtlebot_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf \
    --output ~/turtlebot_ws/src/leader_follower_bringup/models/leader_waffle/leader_waffle.sdf
```

This creates `models/leader_waffle/leader_waffle.sdf` — a copy of the waffle
model with an ArUco marker plate attached to the back.

### Step 4: Update your multi_robot.launch.py

In your launch file, change the LEADER robot's SDF path to use the patched file.

Find the line where `tmp1.sdf` (or the leader's SDF) is referenced:

```python
# BEFORE (somewhere in your multi_robot.launch.py):
# The leader loads from the original model:
'-file', os.path.join(model_path, 'turtlebot3_waffle', 'tmp1.sdf'),

# AFTER:
# The leader loads from the patched model with ArUco marker:
'-file', os.path.join(
    os.path.expanduser('~'),
    'turtlebot_ws/src/leader_follower_bringup/models/leader_waffle/leader_waffle.sdf'
),
```

The FOLLOWER (TB3_2) keeps using the original unmodified SDF.

### Step 5: Test — Launch and Verify

```bash
# Terminal 1: Source and launch
source /opt/ros/humble/setup.bash
source ~/turtlebot_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/turtlebot_ws/src/leader_follower_bringup/models

ros2 launch turtlebot3_gazebo multi_robot.launch.py
```

**In Gazebo:** Look at the leader (TB3_1). You should see a small square plate
on the back of the robot with the ArUco marker pattern.

- If you see a **white square** → GAZEBO_MODEL_PATH is not set correctly
- If you see the **black/white ArUco pattern** → texture is loading correctly

### Step 6: Test Detection

First, position the follower (TB3_2) behind the leader so its camera can see
the marker. You may need to teleop:

```bash
# Terminal 2: Teleop the follower to face the leader's back
ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r cmd_vel:=/TB3_2/cmd_vel
```

Then run the detector:

```bash
# Terminal 3: Run ArUco detector on follower's camera
cd ~/turtlebot_ws/src/leader_follower_bringup
python3 scripts/aruco_detector_test.py --ros-args \
    -r image_raw:=/TB3_2/camera/image_raw \
    -r camera_info:=/TB3_2/camera/camera_info
```

```bash
# Terminal 4: View the debug image
ros2 run rqt_image_view rqt_image_view /aruco_debug_image
```

**Expected output in Terminal 3:**
```
[INFO] ArUco detector started — looking for marker ID 42
[INFO] Camera intrinsics received: fx=554.3, fy=554.3, cx=320.0, cy=240.0
[INFO] Marker 42: dist=0.847m, bearing=3.2deg, tvec=[0.047, -0.021, 0.846]
[INFO] Marker 42: dist=0.851m, bearing=3.4deg, tvec=[0.050, -0.019, 0.850]
...
```

**Expected debug image:** Green box around the detected marker, RGB axes drawn
on it, distance and bearing text overlay.

---

## Quick Test (Without Modifying Launch File)

If you just want to test quickly without editing your launch file:

```bash
# Terminal 1: Launch Gazebo with your normal multi_robot launch
ros2 launch turtlebot3_gazebo multi_robot.launch.py

# Terminal 2: Spawn a THIRD entity — just the ArUco marker plate
# Place it where the leader is (adjust x,y to match TB3_1's position)
ros2 run gazebo_ros spawn_entity.py \
    -entity aruco_test \
    -file ~/turtlebot_ws/src/leader_follower_bringup/models/aruco_marker_42/model.sdf \
    -x 0.0 -y 0.0 -z 0.15

# Terminal 3: Teleop TB3_2 to face the marker, then run detector
```

This spawns a standalone marker plate to test that detection works,
without modifying any launch files.

---

## What the Detector Publishes

| Topic | Type | Content |
|-------|------|---------|
| `/aruco_debug_image` | sensor_msgs/Image | Camera feed with detection overlay |
| `/detected_aruco_pose` | geometry_msgs/PoseStamped | 6-DOF pose of marker in camera frame |

The PoseStamped message gives you:
```
position.x  → left/right offset (positive = right)
position.y  → up/down offset (positive = down, camera convention)
position.z  → depth (distance forward from camera)
orientation → full 3D rotation of the marker
```

---

## Troubleshooting

### "No ArUco markers detected"
- Is TB3_2's camera actually facing the marker? Check with:
  `ros2 run rqt_image_view rqt_image_view /TB3_2/camera/image_raw`
- Is the marker too far away? (>3m gets unreliable with default camera res)
- Is Gazebo lighting too dark? (Gazebo's default lighting should be fine)

### White square instead of ArUco pattern
- GAZEBO_MODEL_PATH is not set. Re-export and restart Gazebo:
  ```bash
  export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/turtlebot_ws/src/leader_follower_bringup/models
  ```
  **You must restart Gazebo** after changing this — it reads model paths at startup.

### Marker visible but detector says "No markers detected"
- The Gazebo camera might be publishing in a different encoding
- Check: `ros2 topic echo /TB3_2/camera/image_raw --once | head -5`
- Verify the topic names match (TB3_1 vs TB3_2 vs leader vs follower)

### patch_leader_sdf.py can't find model.sdf
- Specify the path explicitly:
  ```bash
  # Find it:
  find ~/turtlebot_ws -name "model.sdf" -path "*/turtlebot3_waffle/*" 2>/dev/null

  # Then use that path:
  python3 scripts/patch_leader_sdf.py --input /path/you/found/model.sdf
  ```

---

## Next Steps After ArUco Works

1. **Kalman filter** — Smooth the pose estimates, handle occlusion
2. **TF transform** — Convert camera-frame pose to odom-frame
3. **Goal publisher** — Compute follow-behind goal, publish at 10Hz
4. **MPPI controller** — Set up Nav2 local planner for follower
5. **State machine** — SEARCH / FOLLOW / TOO_CLOSE modes
