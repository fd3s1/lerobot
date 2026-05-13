# Orin NX Handoff: VLADrone + SmolVLA + ROS2 PX4Ctrl

This file is the handoff summary for moving the current development branch and working context to the Orin NX.

## Current Git State

- Repository: `https://github.com/fd3s1/lerobot.git`
- Branch: `drone-smolvla`
- Main local workspace: `/home/user/vla_drone/lerobot`

Recommended commit scope before moving to the Orin NX:

```bash
cd /home/user/vla_drone/lerobot
git add ORIN_NX_HANDOFF.md src/lerobot/robots/vla_drone ref_code/vla_px4ctrl_ros2
git status --short
git commit -m "Add VLADrone ROS2 PX4 control workspace"
git push origin drone-smolvla
```

Note: `ref_code/vla_px4ctrl_ros2/build`, `install`, and `log` are ignored by `ref_code/vla_px4ctrl_ros2/.gitignore` and should be rebuilt on the Orin NX.

## Important Local Dirty State

At the time of writing, these old reference directories show dirty nested-git state:

```text
ref_code/lesson_ws_od/src/yolov5_trt/pynodes/yolov5
ref_code/lesson_ws_od/src/yolov5_trt/pynodes/yolov5_d435i_detection
```

They are unrelated to the VLADrone ROS2 px4ctrl work. If their local contents matter, back them up or commit them inside those nested repositories separately.

## Main Code Added

### LeRobot robot wrapper

```text
src/lerobot/robots/vla_drone/
├── __init__.py
├── config_vla_drone.py
├── ros2_bridge.py
└── vla_drone.py
```

Purpose:

- Defines `robot.type=vla_drone`.
- Uses two cameras, Nokov pose state, and two Feetech STS3215 gripper motors.
- Action/state convention:
  - `x`
  - `y`
  - `z`
  - `yaw`
  - `gripper_left.pos`
  - `gripper_right.pos`

Units:

- `x/y/z`: meters
- `yaw`: radians
- gripper: normalized `0..100`

### ROS2 PX4Ctrl workspace

```text
ref_code/vla_px4ctrl_ros2/
├── README.md
├── shflies/
│   ├── takeoff.sh
│   ├── land.sh
│   └── run_ctrl.sh
└── src/
    ├── quadrotor_msgs/
    ├── uav_utils/
    └── px4ctrl/
```

Purpose:

- ROS2/Humble version of the old px4ctrl-style controller.
- Keeps the old file structure close to FAST-Drone/px4ctrl style.
- Sends PX4/MAVROS position + yaw setpoints.
- Publishes/receives the familiar takeoff/land command.
- Includes a Feetech gripper ROS2 node.

Key config file:

```text
ref_code/vla_px4ctrl_ros2/src/px4ctrl/config/ctrl_param_fpv.yaml
```

Important topics:

```text
/drone6/position_cmd
/drone6/gripper/command
/drone6/px4ctrl/takeoff_land
/drone6/mavros/setpoint_raw/local
/drone6/mavros/vision_pose/pose
/drone6/traj_start_trigger
```

The test target requires `limits.x_max >= 7.0`; current value is `8.0`.

## Feetech Gripper

ROS2 node:

```text
ref_code/vla_px4ctrl_ros2/src/px4ctrl/scripts/feetech_gripper_node.py
```

Behavior:

- Subscribes to `/drone6/gripper/command`.
- Message type: `std_msgs/msg/Float64`.
- `0.0`: closed.
- `100.0`: open.
- Default Feetech serial port: `/dev/ttyACM0`.
- Default motor IDs:
  - left: `1`
  - right: `2`

The node uses LeRobot's Feetech bus and the motor min/max already configured in the Windows Feetech tool.

## Test Flight Node

ROS2 node:

```text
ref_code/vla_px4ctrl_ros2/src/px4ctrl/scripts/fly_x_gripper_test.py
```

Behavior:

- Reads Nokov/VRPN pose directly.
- Default `--mocap-topic auto` auto-discovers topics like:
  - `/Tracker0/pose`
  - `/vrpn_client_node/<tracker_name>/pose`
- Supports:
  - `geometry_msgs/msg/PoseStamped`
  - `nav_msgs/msg/Odometry`
- Default target is absolute mocap coordinate `x=7.0`.
- `y/z` default to current Nokov position.
- The drone flies toward mocap +X.
- During flight:
  - yaw sweeps `+1 rad`, then `-1 rad`, then returns to initial yaw.
  - gripper closes/opens three times.
- Before landing:
  - gripper is forced open.
  - position command publishing stops for 1.2 s so px4ctrl can return from `CMD_CTRL` to `AUTO_HOVER`.
  - then it publishes LAND.

Run:

```bash
ros2 run px4ctrl fly_x_gripper_test.py
```

If multiple Nokov rigid bodies exist:

```bash
ros2 run px4ctrl fly_x_gripper_test.py --mocap-topic /Tracker0/pose
```

If Nokov publishes odometry:

```bash
ros2 run px4ctrl fly_x_gripper_test.py \
  --mocap-topic /vrpn_client_node/Tracker0/odom \
  --mocap-msg-type odometry
```

## Orin NX Setup

Check the Orin NX Ubuntu version first:

```bash
lsb_release -a
```

Recommended:

- Ubuntu 22.04 / JetPack 6: use ROS2 Humble.
- Ubuntu 20.04 / JetPack 5: Humble is not the native target; use ROS2 Foxy or a container if needed.

For this workspace, the current tested target is ROS2 Humble.

Install baseline dependencies on the Orin NX:

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-mavros \
  ros-humble-mavros-extras
```

Source ROS2:

```bash
source /opt/ros/humble/setup.bash
```

Do not run ROS2 nodes with conda Python. The ROS2 scripts use:

```text
#!/usr/bin/python3
```

because Humble `rclpy` is built for system Python.

## Pull Branch On Orin NX

```bash
cd ~
git clone https://github.com/fd3s1/lerobot.git
cd lerobot
git checkout drone-smolvla
```

If the repo already exists:

```bash
cd ~/lerobot
git fetch origin
git checkout drone-smolvla
git pull origin drone-smolvla
```

## Two-Machine Sync Scripts

Use these scripts when development stays on this desktop and deployment/testing happens on the Orin NX.

### On this development machine

After modifying code here:

```bash
cd /home/user/vla_drone/lerobot
scripts/vla_dev_push.sh "Describe the fix"
```

This script:

- stages changes,
- commits them,
- pushes the current branch to `origin`,
- excludes the old YOLO nested-git reference folders,
- excludes `ref_code/vla_px4ctrl_ros2/build`, `install`, and `log`.

If no files changed, it only pushes existing local commits.

### On the Orin NX

To pull the latest code:

```bash
cd ~/lerobot
scripts/vla_nx_pull.sh
```

To pull and rebuild the ROS2 px4ctrl workspace:

```bash
cd ~/lerobot
scripts/vla_nx_pull.sh --build
```

This script:

- stashes local uncommitted changes on the NX,
- pulls the current branch with fast-forward only,
- re-applies the stash,
- optionally runs:

```bash
cd ~/lerobot/ref_code/vla_px4ctrl_ros2
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

Environment overrides:

```bash
VLA_SYNC_REMOTE=origin VLA_SYNC_BRANCH=drone-smolvla scripts/vla_dev_push.sh "message"
VLA_SYNC_REMOTE=origin VLA_SYNC_BRANCH=drone-smolvla scripts/vla_nx_pull.sh --build
```

## Build ROS2 PX4Ctrl Workspace On Orin NX

```bash
cd ~/lerobot/ref_code/vla_px4ctrl_ros2
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

Check the test node is installed:

```bash
ros2 run px4ctrl fly_x_gripper_test.py --help
```

## Runtime Sequence

Terminal 1:

```bash
cd ~/lerobot/ref_code/vla_px4ctrl_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch px4ctrl run_ctrl.launch.py
```

Terminal 2, after MAVROS/PX4/Nokov are running:

```bash
cd ~/lerobot/ref_code/vla_px4ctrl_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
./shflies/takeoff.sh
```

Terminal 2, after takeoff and hover are stable:

```bash
ros2 run px4ctrl fly_x_gripper_test.py
```

Use explicit Nokov topic if auto-discovery finds multiple rigid bodies:

```bash
ros2 run px4ctrl fly_x_gripper_test.py --mocap-topic /Tracker0/pose
```

## Nokov Notes

Nokov/XINGYING ROS2 documentation tested against Foxy, but the messages are standard ROS2 topics and should work in Humble if the package builds.

Important Nokov/XINGYING settings:

- VRPN broadcast must be enabled.
- The rigid body / Markerset name becomes the ROS2 topic name, for example `/Tracker0/pose`.
- Use rigid body output if yaw/orientation is required.
- Ensure VRPN units are meters. If XINGYING broadcasts millimeters, the test will command the wrong scale.
- Confirm the mocap X axis direction before running the 7 m flight.

Useful checks:

```bash
ros2 topic list | grep -E 'Tracker|vrpn|nokov|mocap'
ros2 topic echo /Tracker0/pose
ros2 topic info /Tracker0/pose
```

## Safety Checklist Before Flight

- Props area clear.
- PX4 OFFBOARD path tested at low altitude.
- RC hover/command mode set correctly.
- RC kill switch or manual override ready.
- Nokov pose is stable and in meters.
- Feetech gripper test passed independently.
- `ctrl_param_fpv.yaml` bounds match the flight area.
- For the 7 m test, `limits.x_max` is at least `7.0`.
