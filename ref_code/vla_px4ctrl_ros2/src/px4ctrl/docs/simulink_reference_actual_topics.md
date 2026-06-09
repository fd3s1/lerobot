# Simulink Reference / Actual Signal Topics

This note lists the standard ROS2 messages that expose the px4ctrl reference,
actual odometry, tracking error, and controller output for Simulink.

All topics below use existing standard message types. No custom message package
is required.

## Recommended Subscriptions

| Purpose | Topic | Type | Source |
| --- | --- | --- | --- |
| Controller reference state | `/px4ctrl/simulink/reference_state` | `nav_msgs/msg/Odometry` | `px4ctrl_node` |
| Actual UAV state used by controller | `/mavros/local_position/odom` | `nav_msgs/msg/Odometry` | MAVROS / PX4 local odom |
| Reference minus actual error | `/px4ctrl/simulink/tracking_error` | `nav_msgs/msg/Odometry` | `px4ctrl_node` |
| Controller output setpoint | `/px4ctrl/simulink/attitude_target` | `nav_msgs/msg/Odometry` | `px4ctrl_node` |
| Legacy reference pose only | `/px4ctrl/expert_pose` | `geometry_msgs/msg/PoseStamped` | `px4ctrl_node` |

For checking how far the reference and actual signals differ, subscribe to:

```text
/px4ctrl/simulink/reference_state
/mavros/local_position/odom
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

## `/mavros/local_position/odom`

Type: `nav_msgs/msg/Odometry`

This is the actual odometry used by px4ctrl as the control feedback source.

```text
pose.pose.position.{x,y,z}   = actual position odom.p
pose.pose.orientation        = actual attitude odom.q
twist.twist.linear.{x,y,z}   = actual velocity odom.v
twist.twist.angular.{x,y,z}  = actual angular velocity odom.w
```

For UDE tests, this remains the control odom source. Do not replace the
controller feedback with vision pose just for plotting.

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

## Quick Terminal Checks

After starting px4ctrl:

```bash
ros2 topic info /px4ctrl/simulink/reference_state
ros2 topic info /px4ctrl/simulink/tracking_error
ros2 topic echo --once /px4ctrl/simulink/tracking_error
```

To compare by hand:

```bash
ros2 topic echo --once /px4ctrl/simulink/reference_state
ros2 topic echo --once /mavros/local_position/odom
ros2 topic echo --once /px4ctrl/simulink/tracking_error
```

## Simulink Setup

Use ROS Toolbox Subscribe blocks with:

```text
Message type: nav_msgs/Odometry
Topic names:
  /px4ctrl/simulink/reference_state
  /mavros/local_position/odom
  /px4ctrl/simulink/tracking_error
  /px4ctrl/simulink/attitude_target
```

For tracking plots, use `/px4ctrl/simulink/tracking_error` first. Use the
reference and actual odometry topics when you need to inspect raw signals or
verify the error calculation.
