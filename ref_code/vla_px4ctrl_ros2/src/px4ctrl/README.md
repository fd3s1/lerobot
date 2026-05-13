# px4ctrl ROS2

This package keeps the reference px4ctrl file layout while targeting ROS2 and
MAVROS `setpoint_raw/local`. The controller outputs position and yaw only.

The active control path is:

```text
RC / odom / cmd / takeoff_land
  -> input.*
  -> PX4CtrlFSM.*
  -> controller.*
  -> mavros_msgs/msg/PositionTarget
```

Yaw is always in radians. Odom and cmd yaw are extracted from
`PoseStamped.pose.orientation`.
