# hls_gripper

Project-owned tools and runtime code for the Feetech HLS3625 two-finger gripper.

Keep the upstream Feetech SDK and examples under `lerobot/ref_code/FT-servo`.
This package should contain the aircraft-specific gripper calibration data,
ROS2 launch/config files, and the future flight gripper force-control node.

## Gravity Calibration

The calibration tool has three subcommands:

```bash
python3 src/hls_gripper/scripts/gripper_gravity_calibration.py collect --help
python3 src/hls_gripper/scripts/gripper_gravity_calibration.py collect-full --help
python3 src/hls_gripper/scripts/gripper_gravity_calibration.py fit --help
```

For field use, prefer the workspace wrapper because it follows the existing
`shflies` convention and sources ROS2 before collection:

```bash
bash shflies/gripper_gravity_calibration.sh collect --help
bash shflies/gripper_gravity_calibration.sh collect-full --help
bash shflies/gripper_gravity_calibration.sh fit --help
```

Project defaults:

```text
gripper serial: /dev/ttyACM1
MAVROS FCU serial: /dev/ttyACM0:921600
attitude topic: /mavros/imu/data
```

Override the gripper serial for one run with:

```bash
GRIPPER_PORT=/dev/ttyUSB0 bash shflies/gripper_gravity_calibration.sh collect ...
```

`collect` sweeps the empty gripper through calibrated open/close limits and
records position, current, load, temperature, and optional ROS2 attitude. Keep
using it only for smoke tests or legacy two-point calibration.

`collect-full` is the field calibration path for the mounted gripper. It
collects:

```text
both_clear       both fingers sweep open -> clear without self-contact
left_extension   left finger sweeps clear -> max while right stays open
right_extension  right finger sweeps clear -> max while left stays open
```

Use `clear` for the near-vertical no-self-contact position. Use `max` for the
actual inward travel limit. The fitted JSON stores `close_pos=max`; the
runtime close ratio is computed over `open -> max`.

`fit` bins the raw CSV by close ratio, roll, and pitch, then writes a JSON table
that the runtime gripper controller can use for gravity/friction compensation.

Example output locations:

```text
config/calibration/raw/gravity_empty_level.csv
config/gravity_compensation.json
config/gravity_compensation_curve.csv
```

Do not use a compensation table for flight until it has been collected on the
mounted aircraft with props removed and validated against a low-force grasp test.

## Runtime Topics

The HLS force-control node keeps the old LeRobot-compatible gripper interface:

```text
/gripper/command       std_msgs/Float64
/gripper/command_pair  quadrotor_msgs/GripperCommandPair
/gripper/feedback      quadrotor_msgs/GripperFeedback
```

For Simulink and other external tools, HLS-specific status is intentionally
published as separate standard ROS messages instead of a custom status message.
The default `status_topic=/hls_gripper/status` parameter is treated as a prefix,
so the actual status topics are:

```text
/hls_gripper/state                    std_msgs/String
/hls_gripper/fault_reason             std_msgs/String
/hls_gripper/left_contact             std_msgs/Bool
/hls_gripper/right_contact            std_msgs/Bool
/hls_gripper/both_contact             std_msgs/Bool
/hls_gripper/centered                 std_msgs/Bool
/hls_gripper/safe_to_lift             std_msgs/Bool
/hls_gripper/fault                    std_msgs/Bool
/hls_gripper/single_contact_need_motion std_msgs/Bool
/hls_gripper/left_at_close_limit      std_msgs/Bool
/hls_gripper/right_at_close_limit     std_msgs/Bool
/hls_gripper/left_close_ratio         std_msgs/Float64
/hls_gripper/right_close_ratio        std_msgs/Float64
/hls_gripper/center_error_ratio       std_msgs/Float64
/hls_gripper/center_error_m           std_msgs/Float64
/hls_gripper/single_contact_direction std_msgs/Float64
/hls_gripper/left_current             std_msgs/Float64
/hls_gripper/right_current            std_msgs/Float64
/hls_gripper/left_current_baseline    std_msgs/Float64
/hls_gripper/right_current_baseline   std_msgs/Float64
/hls_gripper/left_current_residual    std_msgs/Float64
/hls_gripper/right_current_residual   std_msgs/Float64
/hls_gripper/roll_deg                 std_msgs/Float64
/hls_gripper/pitch_deg                std_msgs/Float64
```

Change all status topic names together with:

```bash
HLS_STATUS_TOPIC=/my_hls/status bash shflies/handheld_hls_grasp_test.sh
```

This publishes the standard status fields under `/my_hls/...`.
