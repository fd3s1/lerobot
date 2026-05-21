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

Start VRPN, MAVROS, the VRPN-to-MAVROS vision bridge, and `px4ctrl_node`:

```bash
bash shflies/run_mocap_mavros.sh
```

Defaults:

```text
VRPN_SERVER=10.1.1.198
VRPN_PORT=3883
VRPN_SOURCE_TOPIC=/vla_drone1/pose
MAVROS_VISION_TOPIC=/mavros/vision_pose/pose
FCU_URL=/dev/ttyACM0:921600
GCS_URL=udp://@10.1.1.198:14550
BRIDGE_RESTAMP=false
PX4CTRL_PARAMS_FILE=install/px4ctrl/share/px4ctrl/config/ctrl_param_fpv.yaml
START_PX4CTRL=true
```

Set `START_PX4CTRL=false` if you only want the mocap/MAVROS chain. The script
does not start `feetech_gripper_node.py`, so LeRobot can own the Feetech serial
port during data collection. `GCS_URL` forwards MAVLink from MAVROS to QGC on
the Windows mocap computer.

Start px4ctrl and the Feetech gripper node for standalone gripper testing:

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
