# Atomic Physical Wrench And WLS Allocation

## Control paths

The standard path remains available at all times:

```text
/mavros/setpoint_raw/attitude
  -> SET_ATTITUDE_TARGET
  -> PX4 normalized torque/thrust
  -> stock control allocation
```

The physical path adds one atomic sample:

```text
/mavros/tunnel/in (mavros_msgs/msg/Tunnel)
  -> MAVLink TUNNEL message 385, payload_type 42001
  -> mavlink_tunnel uORB
  -> mc_rate_control physical attitude torque controller
  -> vehicle_physical_wrench_setpoint
  -> control_allocator WLS
  -> measured F(V,u) inverse model
  -> actuator_motors Motor1..Motor4
```

No custom ROS or MAVLink message is required. The two new messages are internal
PX4 uORB topics used for atomic handoff and logging.

## Payload versions

All multibyte fields are little-endian. PX4 accepts both versions. ROS2
Offboard publishes version 2 by default; version 1 remains accepted for
backward compatibility.

| Offset | Size | Field |
|---:|---:|---|
| 0 | 4 | ASCII magic `PCTL` |
| 4 | 1 | version: `1` or `2` |
| 5 | 1 | mode: `0` attitude, `1` bodyrate |
| 6 | 2 | valid flags: quaternion, bodyrate, physical thrust, and for v2 angular acceleration |
| 8 | 4 | monotonically increasing sequence |
| 12 | 8 | ROS source time in microseconds |
| 20 | 16 | desired quaternion `w,x,y,z`, NED/FRD |
| 36 | 12 | desired body rate `x,y,z`, FRD, rad/s |
| 48 | 4 | positive total rotor thrust, N |
| 52 | 12 | v2 only: desired body angular acceleration `x,y,z`, FRD, rad/s2 |

Version 1 uses 52 bytes. Version 2 uses 64 bytes. The rest of the standard
128-byte TUNNEL payload is zero. PX4 rejects an unknown version, a mismatched
length, a missing v2 angular-acceleration valid flag, non-finite values, or
angular acceleration beyond `[120,120,60] rad/s2`.

In attitude mode the body-rate field is the desired-body-frame flatness
feedforward rate. It does not contain the outer attitude-error feedback. In
bodyrate mode it is the complete current-body-frame bodyrate command. This
distinction prevents the PX4 physical attitude controller from applying the
same attitude feedback twice.

In attitude mode, v2 angular velocity and angular acceleration are expressed
in the desired body frame by px4ctrl. PX4 transports both into the current
body frame, adds the existing PX4 attitude-feedback rate, and supplies the
transported angular acceleration as feedforward to the physical torque
controller. In bodyrate mode v2 currently carries a valid zero angular
acceleration; no numerical differentiation of the full feedback command is
performed.

The attitude-mode quaternion and feedforward have separate sources:

- `q_d` is generated from the outer-loop feedback/UDE corrected thrust vector.
- body-rate feedforward is generated only from
  `gravity + trajectory acceleration`, trajectory jerk and yaw derivatives.
- angular-acceleration feedforward is generated only from
  `gravity + trajectory acceleration`, trajectory jerk/snap and yaw
  derivatives.

The pure trajectory vectors are rotated into the body frame associated with
the corrected `q_d` before packing. This coordinate representation does not
add feedback content to the feedforward.

The ROS controller applies the same ENU/FLU to NED/FRD transforms as MAVROS
before packing the payload. PX4 validates target system/component, payload
length, magic, version, flags, finite values, quaternion norm, limits, age and
sequence ordering.

## State alignment and trajectory derivatives

The control state keeps mocap position/velocity and FCU attitude/angular rate.
At each control update the mocap position is projected to the FCU odometry
timestamp with the mocap velocity when the signed timestamp offset is within
`[-5,60] ms`. Invalid timing or a source change blends the position correction
back to zero over the configured alignment time constant. This alignment path
does not reset or modify the outer UDE integral.

The figure-eight is a degree-five periodic B-spline evaluated through its fourth
spatial derivative. Main-trajectory scalar speed changes use a seventh-order
velocity blend whose acceleration, jerk and snap are zero at each transition
boundary. Transfer and yaw-alignment phases use the ninth-order endpoint blend.
The resulting `p/v/a/jerk/snap/yaw/yaw_rate/yaw_acceleration` fields share one
time law; configured jerk and snap limits remain final safety clamps.

## Physical thrust

The outer controller computes the force component currently producible along
the rotor axis:

```text
b3 = odom.q * [0, 0, 1]
T_d = mass * max(0, thrust_acc dot b3)
```

The configured takeoff mass is 1.75 kg. `T_d` is independent of the adaptive
`thr2acc` estimate. `thr2acc` still controls the normalized
`SET_ATTITUDE_TARGET` backup.

When `MC_PA_MODE=ACTIVE`, the normalized command is not the command applied to
the motors, so online `thr2acc` is no longer directly identifiable from the
paired IMU and normalized-backup samples. For initial ACTIVE tests, use a
verified fixed `hover_thrust` and disable online thrust estimation with a ROS
parameter override. In OFF, the original estimator remains meaningful
because the normalized path still drives the motors.

## WLS geometry and motor model

Logical motor order is Motor1, Motor2, Motor3, Motor4. AUX connector assignment
does not reorder this vector. With radial arm length `L=0.102 m`,
`a=L/sqrt(2)` and `k=0.01192094 m`:

```text
[ T  ]   [ 1   1   1   1 ] [F1]
[tx  ] = [-a  +a  +a  -a] [F2]
[ty  ]   [+a  -a  +a  -a] [F3]
[tz  ]   [+k  +k  -k  -k] [F4]
```

The allocator first uses the analytic unconstrained inverse. If any force is
outside `[0,Fmax(V)]`, it enumerates all 81 free/lower/upper active sets and
solves the weighted least-squares problem. The default priority is
`[thrust,roll,pitch,yaw]=[1,4,4,0.5]`, normalized by axis capability. Each
resulting force is converted to `u` using the measured `F(V,u)` table. Achieved
yaw torque is evaluated with the full measured `Q(V,u)` table.

## Battery voltage sag

The custom Pixhawk 6C firmware uses the existing PX4 battery parameters instead
of defining duplicate physical-allocation parameters:

```text
BAT1_N_CELLS=6
BAT1_CAPACITY=2700
BAT1_R_INTERNAL=0.009 Ohm per cell
```

Six cells in series give `R_pack=6*0.009=0.054 Ohm`. The current sensor is not used
by physical allocation. From the previous actually selected four motor commands,
the calibrated `I(V,u)` table estimates the present motor current:

```text
I_hat(k) = sum_i I(V_bus(k), u_i(k-1))
V_source_hat(k) = V_bus(k) + R_pack * I_hat(k)
```

For the next requested four forces, the motor model then iterates
`V_bus(k+1)=V_source_hat(k)-R_pack*I_hat(k+1)` together with `F(V,u)` and `I(V,u)`.
This predicts the next loaded bus voltage, current, and commands without battery
current telemetry. `BAT1_R_INTERNAL` is editable in QGC and requires a reboot.
Capacity affects PX4 battery state and remaining-time estimation; it does not
directly change the WLS equations.

`physical_allocation_status.current_valid` reports only whether the unused current
sensor happens to be valid. `model_current_valid` and `estimated_current_a` report
the current reconstructed from the motor model. The status also records source and
predicted bus voltage, next predicted current, per-cell and pack resistance, cell
count, and battery-model validity.

## QGC parameters and gates

`MC_PA_MODE` is latched while disarmed:

- `0 OFF`: exact stock normalized allocation.
- `2 ACTIVE`: allow physical motor commands only when every gate is valid.

The legacy value `1` is treated as OFF and is not an available mode.

ACTIVE requires all of the following:

- armed rotary-wing Offboard control with control allocation enabled;
- `MC_TQ_CTRL_EN=1`;
- exactly four nonreversible motors and no handled motor failure;
- fresh valid atomic TUNNEL setpoint and battery voltage;
- a valid current estimate reconstructed from the previously selected motor commands;
- valid `BAT1_N_CELLS` and nonnegative `BAT1_R_INTERNAL`;
- valid motor-model and WLS output;
- multirotor geometry, `DSHOT_MIN=0.055`, `DSHOT_3D_ENABLE=0`,
  `THR_MDL_FAC=0`, and `MC_BAT_SCALE_EN=0`.

Changing `MC_PA_MODE` in flight is deferred until disarm. Setting
`MC_TQ_CTRL_EN=0` remains the emergency in-air route to the normalized PX4
controller. Loss of a physical setpoint is held until `MC_PA_SP_AGE` and then
blended to the normalized candidate over `MC_PA_BLEND_T` (default 0.30 s).

The normalized backup is continuously generated by the stock PX4 rate
controller. `MC_TQ_MAX_X/Y/Z` does not scale the ACTIVE physical WLS path,
which uses N m directly and is limited by
`MC_PA_TQ_LIM_X/Y/Z` plus motor-force constraints.

## Bring-up sequence

1. Keep props removed and set `MC_PA_MODE=0`. Verify the old controller.
2. Disarm, set `MC_PA_MODE=2`, reboot if required, and run the ROS publisher at
   100 Hz over a MAVLink 2 link.
3. Inspect ULog topics `vehicle_physical_wrench_setpoint`,
   `physical_allocation_status`, `telemetry_status`, `actuator_motors`, and
   `control_allocator_status`.
4. Verify sequence loss is zero or rare, `configuration_valid=true`, voltage
   is valid, motor signs match pure roll/pitch/yaw tests, and commands remain in
   `[0,1]`.
5. Confirm ACTIVE selects the physical source and remains within limits without
   props, then use a restrained low-height flight.

Do not use the telemetry MAVLink 1 link for TUNNEL message 385. The verified
Pixhawk USB link is MAVLink 2 and is the required link for this experiment.

## Manual modes

Acro, Stabilized and Altitude can be tested without ROS2, MAVROS or mocap, but
they do not exercise ACTIVE physical WLS because the physical path intentionally
requires Offboard plus a complete atomic TUNNEL sample.

- Acro generates a body-rate setpoint directly from the sticks. The current
  implementation uses the stock PX4 rate controller and normalized allocation.
- Stabilized generates attitude and body-rate setpoints in `mc_att_control` and
  uses the stock PX4 rate controller with normalized allocation.
- Altitude has the same attitude path as Stabilized while PX4 altitude control
  supplies collective thrust; allocation remains normalized.

These modes are valid regression tests for RC response, mode switching and the
normalized fallback. An ACTIVE WLS test still needs Offboard data; this can be
a minimal constant hover publisher
and does not need to execute an automatic waypoint trajectory.
