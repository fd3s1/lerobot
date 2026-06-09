# Simulink Reference / Actual Signal Topics

This note lists the standard ROS2 messages that expose the px4ctrl reference,
actual odometry, tracking error, and controller output for Simulink.

All topics below use existing standard message types. No custom message package
is required.

## Recommended Subscriptions

| Purpose | Topic | Type | Source |
| --- | --- | --- | --- |
| Controller reference state | `/px4ctrl/simulink/reference_state` | `nav_msgs/msg/Odometry` | `px4ctrl_node` |
| Actual UAV state used by controller | `/px4ctrl/simulink/actual_state` | `nav_msgs/msg/Odometry` | `px4ctrl_node` |
| Reference minus actual error | `/px4ctrl/simulink/tracking_error` | `nav_msgs/msg/Odometry` | `px4ctrl_node` |
| Controller output setpoint | `/px4ctrl/simulink/attitude_target` | `nav_msgs/msg/Odometry` | `px4ctrl_node` |
| UDE internal debug vector | `/px4ctrl/simulink/ude_debug` | `std_msgs/msg/Float64MultiArray` | `px4ctrl_node` |
| Online UDE Kp/Kd/T command | `/px4ctrl/ude_tune` | `std_msgs/msg/Float64MultiArray` | Simulink or ROS2 CLI |
| Online UDE Kp/Kd/T status | `/px4ctrl/ude_tune_status` | `std_msgs/msg/Float64MultiArray` | `px4ctrl_node` |
| Online UDE Kp/Kd/T status text | `/px4ctrl/ude_tune_status_text` | `std_msgs/msg/String` | `px4ctrl_node` |
| Legacy reference pose only | `/px4ctrl/expert_pose` | `geometry_msgs/msg/PoseStamped` | `px4ctrl_node` |

For checking how far the reference and actual signals differ, subscribe to:

```text
/px4ctrl/simulink/reference_state
/px4ctrl/simulink/actual_state
/px4ctrl/simulink/tracking_error
```

The error topic is already `reference - odom`, so Simulink can plot it directly.

## `/px4ctrl/simulink/reference_state`

Type: `nav_msgs/msg/Odometry`

This is the clamped desired state that is actually sent into the controller
calculation in the current px4ctrl loop.

```text
header.frame_id              = px4ctrl frame_id, normally map
child_frame_id               = reference_state
pose.pose.position.{x,y,z}   = desired position des.p
pose.pose.orientation        = desired yaw as quaternion
twist.twist.linear.{x,y,z}   = desired velocity des.v
twist.twist.angular.z        = desired yaw_rate
```

Notes:

- Roll and pitch in `pose.pose.orientation` are zero; only yaw is encoded.
- If command feedforward is disabled, desired velocity is normally zero even
  during position hold or position-command tracking.
- The reference is published only when odom and IMU feedback are fresh enough
  for px4ctrl to compute a control output.

## `/px4ctrl/simulink/actual_state`

Type: `nav_msgs/msg/Odometry`

This is the actual odometry copied from the same `Odom_Data_t` sample used by
px4ctrl as the control feedback source. It is published from inside
`px4ctrl_node` so Simulink can subscribe to reference, actual, error, and UDE
debug from one namespace.

```text
header.frame_id              = px4ctrl frame_id, normally map
child_frame_id               = actual_state_world_velocity
pose.pose.position.{x,y,z}   = actual position odom.p
pose.pose.orientation        = actual attitude odom.q
twist.twist.linear.{x,y,z}   = actual velocity odom.v
twist.twist.angular.{x,y,z}  = actual angular velocity odom.w
```

For UDE tests, this remains the control odom source. Do not replace the
controller feedback with vision pose just for plotting. The raw MAVROS topic
`/mavros/local_position/odom` is still available for cross-checking.

## `/px4ctrl/simulink/tracking_error`

Type: `nav_msgs/msg/Odometry`

This is computed inside `px4ctrl_node` from the same desired state and odom
sample used by the controller loop.

```text
header.frame_id              = px4ctrl frame_id, normally map
child_frame_id               = tracking_error_des_minus_odom
pose.pose.position.{x,y,z}   = desired position - actual position
pose.pose.orientation        = desired yaw - actual yaw, encoded as quaternion
twist.twist.linear.{x,y,z}   = desired velocity - actual velocity
twist.twist.angular.z        = desired yaw_rate - actual yaw_rate
```

Sign convention:

```text
positive error = reference is larger than actual in that axis
negative error = reference is smaller than actual in that axis
```

Example:

```text
tracking_error.pose.pose.position.x = +0.10
```

means the reference x is 10 cm ahead of the current odom x.

## `/px4ctrl/simulink/attitude_target`

Type: `nav_msgs/msg/Odometry`

This mirrors the final MAVROS `AttitudeTarget` command in a standard message so
Simulink can subscribe without MAVROS custom message support.

```text
pose.pose.orientation        = commanded attitude quaternion
twist.twist.angular.{x,y,z}  = commanded body rates
twist.twist.linear.x         = normalized thrust command
twist.twist.linear.y         = AttitudeTarget type_mask
twist.twist.linear.z         = output mode: 1 bodyrate, 0 attitude
child_frame_id               = bodyrate_setpoint or attitude_setpoint
```

This topic is for controller output inspection. It is not the position tracking
reference.

## `/px4ctrl/simulink/ude_debug`

Type: `std_msgs/msg/Float64MultiArray`

The layout label is `ude_debug_v1`. The vector is:

```text
0      stamp seconds
1      px4ctrl FSM state enum
2:4    desired position des.p
5:7    odom position odom.p
8:10   position error e = des.p - odom.p
11:13  desired velocity des.v
14:16  odom velocity odom.v
17:19  velocity error e_dot
20:22  u0 = Kp*e + Kd*e_dot
23:25  integral_u0
26:28  f_hat
29:31  u_acc = u0 - f_hat
32:34  tilt-limited thrust acceleration
35:37  feedforward bodyrates
38:40  attitude feedback bodyrates
41:43  final bodyrate command
44     normalized thrust command
45     desired yaw
46     odom yaw
47     yaw error
48     controller dt
```

## Online UDE Tuning

Only `ude.Kp_diag`, `ude.Kd_diag`, and `ude.T_diag` are accepted online. Other
controller, attitude, thrust, limit, and estimator parameters are intentionally
not writable through this tuning interface.

Publish `std_msgs/msg/Float64MultiArray` to `/px4ctrl/ude_tune`:

```text
0      seq
1      reset_control, 1 reset integral/UDE state after applying params, 0 keep state
2:4    Kp_diag [x,y,z], use NaN to leave unchanged
5:7    Kd_diag [x,y,z], use NaN to leave unchanged
8:10   T_diag  [x,y,z], use NaN to leave unchanged
```

Example:

```bash
ros2 topic pub --once /px4ctrl/ude_tune std_msgs/msg/Float64MultiArray \
  "{data: [1, 1, 0.9, 0.9, 1.1, 1.4, 1.4, 1.8, 0.8, 0.8, 1.0]}"
```

`/px4ctrl/ude_tune_status` uses layout label `ude_tune_status_v1`:

```text
0      seq
1      accepted, 1 accepted, 0 rejected
2      code, 1 accepted, 0 rejected, -1 malformed
3:5    current Kp_diag
6:8    current Kd_diag
9:11   current T_diag
```

`/px4ctrl/ude_tune_status_text` carries the same result as readable text.

## Quick Terminal Checks

After starting px4ctrl:

```bash
ros2 topic info /px4ctrl/simulink/reference_state
ros2 topic info /px4ctrl/simulink/actual_state
ros2 topic info /px4ctrl/simulink/tracking_error
ros2 topic info /px4ctrl/simulink/ude_debug
ros2 topic echo --once /px4ctrl/simulink/tracking_error
```

To compare by hand:

```bash
ros2 topic echo --once /px4ctrl/simulink/reference_state
ros2 topic echo --once /px4ctrl/simulink/actual_state
ros2 topic echo --once /px4ctrl/simulink/tracking_error
```

## Simulink Setup

Use ROS Toolbox Subscribe blocks with:

```text
Message type: nav_msgs/Odometry
Topic names:
  /px4ctrl/simulink/reference_state
  /px4ctrl/simulink/actual_state
  /px4ctrl/simulink/tracking_error
  /px4ctrl/simulink/attitude_target
  /px4ctrl/simulink/ude_debug
  /px4ctrl/ude_tune_status
```

For tracking plots, use `/px4ctrl/simulink/tracking_error` first. Use the
reference and actual odometry topics when you need to inspect raw signals or
verify the error calculation.
