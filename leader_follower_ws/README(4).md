# TurtleBot3 ArUco Leader-Follower

Vision-based leader-follower using two TurtleBot3 Waffle robots. Follower tracks an ArUco marker on the leader's back using OpenCV.

---

## Setup (inside your Docker container)

### 1. Fix OpenCV and NumPy

```bash
pip install opencv-contrib-python "numpy<2"
```

### 2. Clone leader_follower_bringup into your workspace

```bash
cd ~/leader_follower_ws/src
git clone -b develop https://github.com/sidd-zeppelin/leader-following-ros2.git /tmp/lf_repo
cp -r /tmp/lf_repo/leader_follower_ws/src/leader_follower_bringup ~/leader_follower_ws/src/
rm -rf /tmp/lf_repo
```

### 3. Move turtlebot3_simulations to src

The container already has `turtlebot3_simulations` installed via apt. Copy it into your workspace so we can modify the launch file:

```bash
cd ~/leader_follower_ws/src
cp -r /opt/ros/humble/share/turtlebot3_gazebo .
cp -r /opt/ros/humble/share/turtlebot3_simulations .
```

### 4. Replace multi_robot.launch.py

Open `~/leader_follower_ws/src/turtlebot3_gazebo/launch/multi_robot.launch.py` and replace the entire contents with the version from this repo:

[multi_robot.launch.py](https://github.com/sidd-zeppelin/leader-following-ros2/blob/develop/leader_follower_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/multi_robot.launch.py)

### 5. Environment variables + Build

```bash
echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/leader_follower_ws/src/leader_follower_bringup/models' >> ~/.bashrc
source ~/.bashrc

cd ~/leader_follower_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Running (3 terminals)

### Terminal 1 — Gazebo

```bash
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source ~/leader_follower_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/leader_follower_ws/src/leader_follower_bringup/models
ros2 launch turtlebot3_gazebo multi_robot.launch.py
```

### Terminal 2 — Follower (`docker exec -it ros2-humble-container bash`)

```bash
source /opt/ros/humble/setup.bash
source ~/leader_follower_ws/install/setup.bash
python3 ~/leader_follower_ws/src/leader_follower_bringup/scripts/aruco_follower.py
```

### Terminal 3 — Teleop Leader (`docker exec -it ros2-humble-container bash`)

```bash
source /opt/ros/humble/setup.bash
source ~/leader_follower_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r cmd_vel:=/TB3_1/cmd_vel
```

Drive with `w/a/s/d/x`. Follower tracks automatically.
