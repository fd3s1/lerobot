# VLADrone PX4 Control ROS2 Workspace

This workspace is a ROS2 port of the px4ctrl-style onboard controller used in
the reference code. The original `lesson_ws_od` and `august_ws_od` folders are
kept unchanged for comparison.

## Build

```bash
cd /home/user/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2
colcon build
source install/setup.bash
```

## Run

```bash
ros2 launch px4ctrl run_ctrl.launch.py
```

Takeoff and land commands:

```bash
bash shflies/takeoff.sh
bash shflies/land.sh
```

The controller publishes MAVROS `PositionTarget` setpoints containing only
position and yaw. The gripper node uses the same Feetech API path as
`lerobot/examples/feetech_gripper_test.py`.
