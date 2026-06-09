# PX4Ctrl UDE Bodyrate / Attitude Controller Data Flow

This document describes the current ROS2 controller data flow in:

- `src/controller.cpp`
- `src/controller.h`

The main entry point is:

```cpp
Controller_Output_t LinearControl::calculateControl(
  const Desired_State_t &des,
  const Odom_Data_t &odom,
  const Imu_Data_t &imu,
  const rclcpp::Time &now)
```

The controller output computed by `calculateControl()` is:

```text
bodyrates = [p_rate, q_rate, r_rate]
q         = desired attitude quaternion
thrust    = normalized collective thrust
```

PX4 receives `mavros_msgs/msg/AttitudeTarget` on `/mavros/setpoint_raw/attitude`.
The final setpoint mode is selected by `use_bodyrate_ctrl`:

```text
true  -> body_rate + thrust, IGNORE_ATTITUDE
false -> orientation + thrust, IGNORE_ROLL_RATE | IGNORE_PITCH_RATE | IGNORE_YAW_RATE
```

The same setpoint is also mirrored for Simulink as a standard
`nav_msgs/msg/Odometry` on `/px4ctrl/simulink/attitude_target`:

```text
pose.pose.orientation = commanded quaternion
twist.twist.angular   = commanded body_rate
twist.twist.linear.x  = normalized thrust
twist.twist.linear.y  = AttitudeTarget type_mask
twist.twist.linear.z  = output mode, 1 bodyrate, 0 attitude
```

For Simulink reference/actual tracking diagnostics, px4ctrl also publishes
standard topics:

```text
/px4ctrl/simulink/reference_state
  pose.pose.position = desired position used by the controller
  pose.pose.orientation = desired yaw as quaternion
  twist.twist.linear = desired velocity
  twist.twist.angular.z = desired yaw_rate

/px4ctrl/simulink/actual_state
  pose.pose.position = odom position used by the controller
  pose.pose.orientation = odom attitude
  twist.twist.linear = odom velocity in the controller/world frame
  twist.twist.angular = odom angular velocity

/px4ctrl/simulink/tracking_error
  pose.pose.position = desired position - odom position
  pose.pose.orientation = desired yaw - odom yaw as quaternion
  twist.twist.linear = desired velocity - odom velocity
  twist.twist.angular.z = desired yaw_rate - odom yaw_rate

/px4ctrl/simulink/ude_debug
  nav_msgs/Odometry packing key UDE debug values into scalar fields

/px4ctrl/ude_tune
  std_msgs/Float64MultiArray for online Kp/Kd/T tuning only
```

See `simulink_reference_actual_topics.md` for the complete field mapping.

## Top Level Flow

```mermaid
flowchart TD
  DES["Desired_State_t des<br/>p, yaw<br/>j=0, yaw_rate=0 by default"]
  ODOM["Odom_Data_t odom<br/>p, v_ENU, q, w"]
  IMU["Imu_Data_t imu<br/>q, w, a"]
  NOW["now"]
  PARAM["Parameter_t param<br/>ude, attitude, controller, thrust_model"]

  CALC["LinearControl::calculateControl(des, odom, imu, now)"]

  DES --> CALC
  ODOM --> CALC
  IMU --> CALC
  NOW --> CALC
  PARAM --> CALC

  CALC --> TIME["Timing block<br/>dt = now - last_control_time_<br/>reset if dt invalid or too large"]
  TIME --> UDE["UDE outer loop<br/>position and velocity feedback"]
  UDE --> LIMIT_ACC["computeLimitedTotalAcc()<br/>limit thrust direction tilt"]
  LIMIT_ACC --> FLAT["computeFlatInput()<br/>thrust_acc + yaw -> desired_attitude + feedforward_bodyrates"]
  FLAT --> ATT_FB["computeFeedBackControlBodyrates()<br/>desired_attitude vs odom.q"]
  ATT_FB --> BODYRATE_SUM["bodyrates = feedforward + feedback<br/>axis clamp"]
  LIMIT_ACC --> THRUST["computeDesiredCollectiveThrustSignal()<br/>current attitude projection"]

  BODYRATE_SUM --> OUT["Controller_Output_t u<br/>u.bodyrates, u.q, u.thrust"]
  THRUST --> OUT
  OUT --> PUB["PX4CtrlFSM::publish_ctrl()<br/>select bodyrate or attitude output by use_bodyrate_ctrl"]
```

## UDE Outer Loop

This replaces the old `computePIDErrorAcc()` path.

```mermaid
flowchart TD
  P_DES["des.p"] --> E["e = des.p - odom.p"]
  P_ODOM["odom.p"] --> E
  V_ODOM["odom.v"] --> EDOT["e_dot = -odom.v"]

  KP["Kp = diag(param.ude.Kp_diag)"] --> U0
  KD["Kd = diag(param.ude.Kd_diag)"] --> U0
  E --> U0["u0 = Kp*e + Kd*e_dot"]
  EDOT --> U0

  DT["dt"] --> INT["integral_u0_ += u0 * dt<br/>only when dt > 0 and u0 is finite"]
  U0 --> INT

  T["T = diag(param.ude.T_diag)"] --> FHAT
  V_ODOM --> FHAT["f_hat_i = (odom.v_i - integral_u0_i) / T_i"]
  INT --> FHAT
  FHAT --> FHAT_LIMIT["clampVectorByAxis(f_hat, max_f_hat)"]

  U0 --> UACC["u_acc = u0 - f_hat"]
  FHAT_LIMIT --> UACC
  UACC --> UACC_LIMIT["clampVectorByAxis(u_acc, max_u_acc)"]

  G["gravity"] --> THRUST_ACC
  UACC_LIMIT --> THRUST_ACC["thrust_acc = u_acc + [0,0,g]"]
  THRUST_ACC --> LIMITED["computeLimitedTotalAcc(thrust_acc)"]
```

Old controller comparison:

```text
old:
  pid_error_acc = computePIDErrorAcc(odom, des, param)
  total_acc = pid_error_acc + des.a + gravity

current:
  u0 = Kp*e + Kd*e_dot
  f_hat = T^-1 * (odom.v - integral_u0)
  u_acc = u0 - f_hat
  thrust_acc = u_acc + gravity
```

## Tilt Limit

```mermaid
flowchart TD
  IN["ref_acc = thrust_acc"] --> CHECK["finite and large enough?"]
  CHECK -- "no" --> FALLBACK["return [0,0,g] or min_collective_acc * world_z"]
  CHECK -- "yes" --> DIR["desired_z = normalize(ref_acc)"]
  DIR --> ANGLE["angle = acos(world_z dot desired_z)"]
  ANGLE -- "angle <= max_angle" --> KEEP["return ref_acc"]
  ANGLE -- "angle > max_angle" --> ROT["rotate world_z toward desired_z by max_angle"]
  ROT --> SCALE["scale by vertical_acc / cos(max_angle)"]
  SCALE --> RETURN["return limited thrust_acc"]
```

`computeLimitedTotalAcc()` limits the direction of the requested thrust vector.
It does not use the old roll/pitch small-angle equations.

## Flat Input To Desired Attitude

`computeFlatInput()` is where `desired_attitude` is computed.

```mermaid
flowchart TD
  A["thrust_acc"] --> NZB["normalizeWithGrad(thrust_acc, jerk)"]
  J["des.j"] --> NZB
  NZB --> ZB["zb = desired body z axis<br/>zbd = derivative of zb"]

  YAW["des.yaw"] --> XC["xc = [cos(yaw), sin(yaw), 0]"]
  YAWR["des.yaw_rate"] --> XCD["xcd = d(xc)/dt"]

  ZB --> YC["yc = zb cross xc"]
  XC --> YC
  ZB --> YCD["ycd = zbd cross xc + zb cross xcd"]
  XC --> YCD
  XCD --> YCD

  YC --> NYB["normalizeWithGrad(yc, ycd)"]
  YCD --> NYB
  NYB --> YB["yb = desired body y axis<br/>ybd = derivative of yb"]

  YB --> XB["xb = yb cross zb"]
  ZB --> XB
  YB --> XBD["xbd = ybd cross zb + yb cross zbd"]
  ZB --> XBD

  XB --> ROT["rot = [xb yb zb]"]
  YB --> ROT
  ZB --> ROT
  ROT --> ATT["desired_attitude = Quaternion(rot)"]

  ZB --> FF["feedforward_bodyrates from xb/yb/zb derivatives"]
  YB --> FF
  XB --> FF

  NZB -- "fail" --> FALLBACK["desired_attitude = odom.q<br/>feedforward_bodyrates = 0"]
  NYB -- "fail" --> FALLBACK
```

Important notes:

- `desired_attitude` is initialized to `odom.q` before calling
  `computeFlatInput()`.
- On success, `computeFlatInput()` overwrites it with `Quaternion(rot)`.
- On failure, the controller keeps `desired_attitude = odom.q` and sets
  feedforward bodyrates to zero.

## Bodyrate Feedback

```mermaid
flowchart TD
  DESQ["desired_attitude<br/>passed as des_q"] --> QERR["q_error = odom.q.inverse() * desired_attitude"]
  ESTQ["odom.q<br/>passed as est_q"] --> QERR
  QERR --> NORM["normalize q_error"]
  NORM --> SIGN["sign = q_error.w >= 0 ? 1 : -1"]
  KANG["KAng = diag(param.attitude.KAng_diag)"] --> FB
  SIGN --> FB["feedback_bodyrates = sign * 2 * KAng * q_error.xyz"]
  NORM --> FB

  FF["feedforward_bodyrates"] --> SUM["bodyrates = feedforward + feedback"]
  FB --> SUM
  SUM --> CLAMP["clamp x/y/z by max_bodyrate_x/y/z"]
```

## Setpoint Publishing

```mermaid
flowchart TD
  U["Controller_Output_t u<br/>q, bodyrates, thrust"] --> MODE{"use_bodyrate_ctrl?"}
  MODE -- "true" --> BR["type_mask = IGNORE_ATTITUDE<br/>body_rate = u.bodyrates<br/>thrust = u.thrust"]
  MODE -- "false" --> ATT["type_mask = IGNORE_ROLL_RATE | IGNORE_PITCH_RATE | IGNORE_YAW_RATE<br/>orientation = u.q<br/>body_rate = 0<br/>thrust = u.thrust"]
  BR --> MSG["mavros_msgs/msg/AttitudeTarget"]
  ATT --> MSG
  MSG --> PX4["/mavros/setpoint_raw/attitude"]
  MSG --> SIM["/px4ctrl/simulink/attitude_target<br/>nav_msgs/msg/Odometry"]
  DESREF["Desired_State_t safe_des"] --> REF["/px4ctrl/simulink/reference_state<br/>nav_msgs/msg/Odometry"]
  DESREF --> ERR["/px4ctrl/simulink/tracking_error<br/>nav_msgs/msg/Odometry<br/>des - odom"]
  ODOM["Odom_Data_t odom"] --> ERR
```

Variable mapping:

```text
computeFeedBackControlBodyrates(des_q, est_q)

des_q = desired_attitude
est_q = odom.q
```

## Thrust Mapping

```mermaid
flowchart TD
  ACC["limited thrust_acc"] --> DOT["project onto current body z axis"]
  ODOMQ["odom.q"] --> BZ["body_z = odom.q * [0,0,1]"]
  BZ --> DOT
  THR2ACC["thr2acc_"] --> THRUST["thrust = dot(thrust_acc, body_z) / thr2acc_"]
  DOT --> THRUST
  THRUST --> CLAMP["clamp to min_thrust / max_thrust"]
  CLAMP --> QUEUE["push (now, thrust) into timed_thrust_<br/>used by estimateThrustModel()"]
```

This is different from the old `des_acc.z / thr2acc` mapping. The current
mapping uses the current attitude projection:

```text
thrust = dot(thrust_acc, current_body_z) / thr2acc
```

## Runtime State Variables

```mermaid
flowchart TD
  RESET["resetControlState()"] --> ZERO_INT["integral_u0_ = 0"]
  RESET --> ZERO_TIME["last_control_time_ = 0"]
  RESET --> INIT_FALSE["control_initialized_ = false"]
  RESET --> CLEAR_QUEUE["clear timed_thrust_"]

  CALC["calculateControl()"] --> UPDATE_TIME["last_control_time_ = now"]
  CALC --> UPDATE_INT["update integral_u0_ when dt is valid"]
  CALC --> UPDATE_QUEUE["push thrust sample"]

  FSM["PX4CtrlFSM::change_state()"] --> RESET
  TIMEOUT["odom/imu timeout or recovery"] --> RESET
  MANUAL["MANUAL_CTRL or landing idle output"] --> RESET
```

Mode switching effect:

- `AUTO_TAKEOFF -> AUTO_HOVER` resets UDE state.
- `AUTO_HOVER -> CMD_CTRL` resets UDE state.
- `CMD_CTRL -> AUTO_HOVER` resets UDE state.
- Feedback timeout and recovery also reset UDE state.

Therefore `integral_u0_` is not shared across these mode transitions.

## Short Function Map

```text
calculateControl()
  dt / reset guard
  UDE outer loop
    diagVector()
    clampVectorByAxis()
  computeLimitedTotalAcc()
  computeFlatInput()
    normalizeWithGrad()
  computeFeedBackControlBodyrates()
    diagVector()
  computeDesiredCollectiveThrustSignal()
  update timed_thrust_

estimateThrustModel()
  consumes timed_thrust_
  consumes imu.a
  updates thr2acc_
  rejects out-of-range imu.a.z / thrust samples
  limits each thr2acc_ update step to avoid sudden thrust-model jumps
```
