# hls_gripper

Project-owned tools and runtime code for the Feetech HLS3625 two-finger gripper.

Keep the upstream Feetech SDK and examples under `lerobot/ref_code/FT-servo`.
This package should contain the aircraft-specific gripper calibration data,
ROS2 launch/config files, and the future flight gripper force-control node.

## Gravity Calibration

The calibration tool has five subcommands:

```bash
python3 src/hls_gripper/scripts/gripper_gravity_calibration.py read-limits --help
python3 src/hls_gripper/scripts/gripper_gravity_calibration.py collect --help
python3 src/hls_gripper/scripts/gripper_gravity_calibration.py collect-full --help
python3 src/hls_gripper/scripts/gripper_gravity_calibration.py calibrate-session --help
python3 src/hls_gripper/scripts/gripper_gravity_calibration.py fit --help
```

For field use, prefer the workspace wrapper because it follows the existing
`shflies` convention and sources ROS2 before collection:

```bash
bash shflies/gripper_gravity_calibration.sh read-limits --help
bash shflies/gripper_gravity_calibration.sh collect --help
bash shflies/gripper_gravity_calibration.sh collect-full --help
bash shflies/gripper_gravity_calibration.sh calibrate-session --help
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

`read-limits` is a read-only helper for checking HLS IDs, present positions,
current, temperature, and stored min/max angle limit registers:

```bash
GRIPPER_PORT=/dev/ttyACM1 bash shflies/gripper_gravity_calibration.sh read-limits
```

To watch live positions while manually moving the fingers:

```bash
GRIPPER_PORT=/dev/ttyACM1 bash shflies/gripper_gravity_calibration.sh read-limits --watch
```

The min/max registers reflect the limits configured in the Windows tool, but
they do not tell which endpoint is physically open or closed. Use
`present_pos` while placing the fingers at the actual mechanical open, clear,
and max postures to fill `--left-open`, `--left-clear`, `--left-max`,
`--right-open`, `--right-clear`, and `--right-max`.

The mounted gripper limits are saved in:

```text
src/hls_gripper/config/hls_gripper_limits.json
```

Current saved raw positions:

```text
left:  open=2050 clear=952 close/max=675
right: open=2050 clear=964 close/max=679
```

The commanded open position is kept inside the HLS angle limit register. The
manual back-driven present position can read slightly above this limit, but
calibration and runtime commands must use a reachable target.

`collect-full` loads this file by default, so the normal field command does not
need explicit open/clear/max arguments. Command-line values still override the
JSON for one run.

`collect` sweeps the empty gripper through calibrated open/close limits and
records position, current, load, temperature, and optional ROS2 attitude. Keep
using it only for smoke tests or legacy two-point calibration.

`collect-full` is the field calibration path for the mounted gripper. It
collects:

```text
both_clear       both fingers sweep open -> clear without self-contact
left_full        left finger sweeps open -> max while right stays open
right_full       right finger sweeps open -> max while left stays open
```

Use `clear` for the near-vertical no-self-contact position. Use `max` for the
actual inward travel limit. The fitted JSON stores `close_pos=max`; the
runtime close ratio is computed over `open -> max`.

Before and after each empty segment, the script parks both fingers at open with
low-speed transition commands. Those transition moves are not recorded in the
CSV, so current spikes from changing segment geometry do not pollute the
no-load baseline. During empty calibration, each commanded point must settle
within `--position-tolerance` before samples are written; otherwise the command
fails instead of saving moving-current data. Increase `--move-timeout` for very
slow smoke tests, or increase speed/acceleration after confirming the motion is
safe.

`fit` bins the raw CSV by close ratio, roll, and pitch, then writes a JSON table
that the runtime gripper controller can use for gravity/friction compensation.

`calibrate-session` is the preferred full field workflow when contact detection
must also be calibrated. It first collects the empty no-load table over multiple
speed/acceleration/torque profiles, then opens the gripper and waits for plain
terminal prompts while you place or change objects. The terminal does not
refresh while waiting for input.

Default object labels are `foam,bottle,box`; override them with your own names:

```bash
GRIPPER_PORT=/dev/ttyACM1 ATTITUDE_SOURCE=ros-imu \
bash shflies/gripper_gravity_calibration.sh calibrate-session \
  --object-labels foam,bottle,small_box \
  --contact-trials-per-object 3 \
  --profiles 5:3:90,10:5:120,20:8:150 \
  --max-current 300 \
  --max-temp 60
```

If the empty baseline is already trusted, collect only contact samples and
append contact thresholds to a new JSON:

```bash
GRIPPER_PORT=/dev/ttyACM1 ATTITUDE_SOURCE=ros-imu \
bash shflies/gripper_gravity_calibration.sh calibrate-session \
  --skip-empty \
  --baseline-json src/hls_gripper/config/gravity_compensation.json \
  --object-labels bottle,box \
  --contact-trials-per-object 3
```

The final JSON contains `contact_detection.left/right` with
`metric_sign`, `enter_threshold`, `exit_threshold`, and `strong_threshold`.
The runtime node uses those values by default; ROS parameters can still override
them for field debugging.

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
/hls_gripper/left_contact_metric      std_msgs/Float64
/hls_gripper/right_contact_metric     std_msgs/Float64
/hls_gripper/roll_deg                 std_msgs/Float64
/hls_gripper/pitch_deg                std_msgs/Float64
```

Change all status topic names together with:

```bash
HLS_STATUS_TOPIC=/my_hls/status bash shflies/handheld_hls_grasp_test.sh
```

This publishes the standard status fields under `/my_hls/...`.
