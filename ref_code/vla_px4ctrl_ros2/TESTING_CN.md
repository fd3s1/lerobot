# VLADrone ROS2 + MAVROS + Feetech 夹爪测试流程

本文档用于在 Orin NX 上按步骤测试整条链路：

```text
遥控器 / Nokov / 测试程序
  -> MAVROS / px4ctrl
  -> /gripper/command
  -> feetech_gripper_node.py
  -> 两个 Feetech STS3215 夹爪舵机
```

当前约定：

- MAVROS RC 输入：`/mavros/rc/in`
- MAVROS vision pose：`/mavros/vision_pose/pose`
- px4ctrl 位置指令：`/position_cmd`
- px4ctrl 专家动作：`/px4ctrl/expert_pose`
- px4ctrl 起降指令：`/px4ctrl/takeoff_land`
- 夹爪命令：`/gripper/command`
- 夹爪定义：`100 = 全开`，`0 = 全关`
- 遥控器第 10 通道：低位打开，高位关闭
- CH10 只在 PX4 模式为 `POSCTL` 或 `OFFBOARD` 时控制夹爪。
- `ALTCTL`、`STABILIZED`、`MANUAL`、降落、已落地、未解锁或未知模式下，夹爪会自动保持全开。
- 新夹爪安全高度按 `20 cm` 处理，当前配置会在 mocap 高度 `z <= 0.20 m` 时强制全开，避免接近地面时夹爪触地。

安全前提：

- 第一次测试必须拆桨。
- 夹爪悬空，避免触地、夹到线束或机体。
- 第一次真实舵机测试时，不要启动飞行测试程序。
- 每一步只验证一条链路，确认无误后再进入下一步。

## 1. 同步代码并构建 ROS2 workspace

作用：确保 NX 上是 GitHub 最新代码，并重新安装 ROS2 节点到 `install/`。

```bash
cd ~/vla_drone/lerobot
scripts/vla_nx_pull.sh --build
```

每个新终端都需要 source：

```bash
cd ~/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
```

检查 package 是否存在：

```bash
ros2 pkg list | grep -E "px4ctrl|quadrotor_msgs|uav_utils"
```

期望看到：

```text
px4ctrl
quadrotor_msgs
uav_utils
```

## 2. 检查 Feetech Python 依赖

作用：确认 ROS2 系统 Python 可以直接控制 Feetech 舵机。这里不要依赖 conda，也不要依赖 LeRobot / torch。

```bash
python3 -c "import scservo_sdk; print('scservo ok')"
```

如果报错，安装：

```bash
python3 -m pip install --user feetech-servo-sdk pyserial
```

检查串口：

```bash
ls -l /dev/ttyACM1
```

如果没有 `/dev/ttyACM1`，查看可用串口：

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
```

## 3. 单独测试夹爪节点 dry run

作用：不打开串口、不控制舵机，只确认 ROS2 topic 能到达夹爪节点。

终端 A：

```bash
cd ~/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run px4ctrl feetech_gripper_node.py --ros-args -p dry_run:=true
```

终端 B：

```bash
source /opt/ros/humble/setup.bash
source ~/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2/install/setup.bash

ros2 topic pub --once /gripper/command std_msgs/msg/Float64 "{data: 100.0}"
ros2 topic pub --once /gripper/command std_msgs/msg/Float64 "{data: 0.0}"
```

终端 A 期望看到：

```text
dry_run gripper target: 100.0
dry_run gripper target: 0.0
```

## 4. 单独测试真实 Feetech 舵机

作用：绕过 px4ctrl、MAVROS、遥控器，只验证夹爪舵机本身。

终端 A：

```bash
cd ~/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run px4ctrl feetech_gripper_node.py --ros-args -p port:=/dev/ttyACM1
```

启动后会读取两个舵机的 Windows 标定范围，期望看到类似：

```text
gripper_left: id=1, min=..., max=..., homing_offset=..., direction=normal
gripper_right: id=2, min=..., max=..., homing_offset=..., direction=normal
Listening for gripper commands on /gripper/command
```

终端 B：

```bash
source /opt/ros/humble/setup.bash
source ~/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2/install/setup.bash

ros2 topic pub --once /gripper/command std_msgs/msg/Float64 "{data: 100.0}"
ros2 topic pub --once /gripper/command std_msgs/msg/Float64 "{data: 0.0}"
```

期望结果：

- `100.0`：夹爪全开
- `0.0`：夹爪全关

当前夹爪约定是 `100.0 = 全开`，`0.0 = 全关`。`feetech_gripper_node.py` 和 LeRobot 录制默认都使用软件反向，让无参启动时也保持这个语义。如果单独运行 ROS 夹爪节点时方向仍然反了，不要改 Windows 标定，先显式指定软件反向：

```bash
ros2 run px4ctrl feetech_gripper_node.py --ros-args \
  -p port:=/dev/ttyACM1 \
  -p left_inverted:=true \
  -p right_inverted:=true
```

如果只有一个舵机方向反，就只改对应参数。

## 5. 启动 MAVROS

作用：让 ROS2 能读到 PX4 飞控状态、遥控器通道，并能向 PX4 发送 setpoint。

先查看 MAVROS 的 launch 参数：

```bash
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch --show-args
```

常见串口连接写法如下。请根据你的飞控实际串口修改 `fcu_url`。

USB 串口示例：

```bash
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:57600
```

TELEM 串口示例：

```bash
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600
```

UDP 示例：

```bash
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557
```

当前 MAVROS 不使用额外 namespace，topic 应该是 `/mavros/...`。如果你的启动方式生成了其他 namespace，需要同步修改 `ctrl_param_fpv.yaml` 中的 MAVROS topic。

检查 MAVROS 是否连接：

```bash
ros2 topic echo /mavros/state
```

期望看到：

```text
connected: true
```

检查 RC 输入：

```bash
ros2 topic echo /mavros/rc/in
```

拨动遥控器第 10 通道，观察 `channels` 数组第 10 个值，也就是 `channels[9]`。

期望：

- 低位约 `1000~1300`
- 高位约 `1700~2000`

## 6. 检查 Nokov / mocap vision pose

作用：确认 PX4/MAVROS 能收到外部定位。px4ctrl 默认从 `/mavros/vision_pose/pose` 读取当前位置。

如果 Nokov/VRPN 的原始 topic 是 `/vla_drone1/pose`，不要直接把它 remap 到 `/mavros/vision_pose/pose`。当前 `vrpn_mocap` 发布端通常是 `BEST_EFFORT` QoS，而 MAVROS `vision_pose` 订阅端是 `RELIABLE` QoS，二者可能不兼容。使用桥接节点转换 QoS：

推荐启动方式：一个脚本同时启动 VRPN client、MAVROS 和 QoS 桥接节点。

```bash
cd ~/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2
bash shflies/run_mocap_mavros.sh
```

脚本默认参数：

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

如果要临时覆盖参数，例如飞控串口临时变成 `/dev/ttyUSB0`：

```bash
FCU_URL=/dev/ttyUSB0:921600 bash shflies/run_mocap_mavros.sh
```

脚本会通过 `GCS_URL` 把 MAVLink 转发给 QGC。当前默认 QGC 在 Windows/mocap 主机 `10.1.1.198`，端口 `14550`。如果 QGC 不显示连接，检查 Windows 防火墙是否允许 QGC 接收 UDP 14550。

如果只想启动定位链路，不启动 `px4ctrl_node`：

```bash
START_PX4CTRL=false bash shflies/run_mocap_mavros.sh
```

脚本只启动 `px4ctrl_node`，不会启动 `feetech_gripper_node.py`。这样 LeRobot 采集时可以自己打开 Feetech 串口，不会和 ROS 夹爪节点抢 `/dev/ttyACM1`。

如果只想手动分终端启动，使用下面的命令。

终端 A：启动 VRPN client，保留原始 topic。

```bash
source /opt/ros/humble/setup.bash
ros2 run vrpn_mocap client_node --ros-args \
  -p server:=10.1.1.198 \
  -p port:=3883
```

终端 B：启动 QoS 桥接节点。

```bash
cd ~/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run px4ctrl vrpn_to_mavros_vision_bridge.py --ros-args \
  -p source_topic:=/vla_drone1/pose \
  -p target_topic:=/mavros/vision_pose/pose
```

桥接节点默认：

- 订阅 `/vla_drone1/pose`：`BEST_EFFORT`
- 发布 `/mavros/vision_pose/pose`：`RELIABLE`
- 保留 VRPN 输入消息的 `header.stamp`
- 保留输入消息的 `frame_id`

时间戳注意事项：

- 默认不要设置 `restamp:=true`，因为 EKF 应该看到 mocap pose 的原始产生时间。
- 如果桥接节点提示 `Input header.stamp is zero`，或者 stamp age 明显异常，才临时测试：

```bash
ros2 run px4ctrl vrpn_to_mavros_vision_bridge.py --ros-args \
  -p source_topic:=/vla_drone1/pose \
  -p target_topic:=/mavros/vision_pose/pose \
  -p restamp:=true
```

检查 QoS 是否正确：

```bash
ros2 topic info /mavros/vision_pose/pose -v
```

期望 `/mavros/vision_pose/pose` 的 publisher 端是桥接节点且为 `RELIABLE`，subscriber 端是 `/mavros/vision_pose` 且为 `RELIABLE`。

检查 vision pose：

```bash
ros2 topic echo /mavros/vision_pose/pose
```

期望：

- `position.x/y/z` 随无人机移动变化
- `orientation` 不全是 0
- 数据频率稳定

查看频率：

```bash
ros2 topic hz /mavros/vision_pose/pose
```

如果这里没有数据，说明 Nokov 到 MAVROS 的 vision pose 桥接还没有启动或 topic 名不一致。先修这一步，不要继续飞行测试。

## 7. 检查 px4ctrl 专家动作 topic

作用：确认手动飞行时，px4ctrl 已经把内部 `hover_pose` 发布成可供 LeRobot 记录的专家 action。

终端 A：启动 MAVROS，并确认 `/mavros/vision_pose/pose` 和 `/mavros/rc/in` 有数据。

终端 B：启动 px4ctrl：

```bash
cd ~/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run px4ctrl px4ctrl_node --ros-args --params-file install/px4ctrl/share/px4ctrl/config/ctrl_param_fpv.yaml
```

终端 C：监听专家动作：

```bash
source /opt/ros/humble/setup.bash
source ~/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2/install/setup.bash
ros2 topic echo /px4ctrl/expert_pose
```

在 `AUTO_HOVER` 手动飞行时，`/px4ctrl/expert_pose` 应持续发布 `PoseStamped`：

- `pose.position.x/y/z` 是 px4ctrl 当前目标位置，不是 mocap 当前测量位置。
- `pose.orientation` 中的 yaw 是 px4ctrl 当前目标 yaw。
- `header.stamp` 与同一控制周期发给 `/mavros/setpoint_raw/local` 的 setpoint 使用同一时间戳。

检查频率：

```bash
ros2 topic hz /px4ctrl/expert_pose
```

期望频率接近 `ctrl_freq_max`，当前默认约 `60 Hz`。

位置限幅检查：

`AUTO_HOVER` 下 `/px4ctrl/expert_pose` 是 px4ctrl 的目标位置，目标位置会被 `ctrl_param_fpv.yaml` 里的 `limits` 限制。当前默认范围：

```yaml
limits:
  x_min: -12.0
  x_max: 8.0
  y_min: -3.5
  y_max: 3.5
  z_min: -0.3
  z_max: 3.0
```

如果飞机实际 mocap 位置已经在限幅外，例如 `x < x_min`，切入 `AUTO_HOVER` 后目标点会被夹到边界，表现为某些方向打杆没有反应、只能往场地内部方向移动。飞行前应确认采集区域完全落在 `limits` 内，修改后需要重新 `colcon build --packages-select px4ctrl` 并重启 `run_mocap_mavros.sh`。

## 8. 单独测试 RC 第 10 通道到夹爪命令 topic

作用：暂时不接舵机，只确认遥控器 CH10 会在允许模式下被 px4ctrl 转成 `/gripper/command`。

终端 A：启动 MAVROS，并确认 `/mavros/rc/in` 和 `/mavros/state` 有数据。

终端 B：监听夹爪命令：

```bash
source /opt/ros/humble/setup.bash
source ~/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2/install/setup.bash
ros2 topic echo /gripper/command
```

终端 C：启动 px4ctrl，不启动真实夹爪节点：

```bash
cd ~/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run px4ctrl px4ctrl_node --ros-args --params-file install/px4ctrl/share/px4ctrl/config/ctrl_param_fpv.yaml
```

先确认当前 PX4 模式：

```bash
ros2 topic echo /mavros/state
```

拨动遥控器第 10 通道。

当前配置：

```yaml
gripper:
  rc_channel: 10
  pwm_open: 1300
  pwm_close: 1700
  open_position: 100.0
  closed_position: 0.0
  force_open_below_z: 0.20
```

期望：

- 在 `POSCTL` 或 `OFFBOARD` 下，CH10 低位：`/gripper/command` 输出 `100.0`
- 在 `POSCTL` 或 `OFFBOARD` 下，CH10 高位：`/gripper/command` 输出 `0.0`
- 在 `POSCTL` 或 `OFFBOARD` 下，CH10 中间：不发布新命令
- 在 `ALTCTL`、`STABILIZED`、`MANUAL`、降落、未解锁、未知模式或 `z <= 0.20 m` 时，无论 CH10 位置如何，`/gripper/command` 都应发布或保持 `100.0`

如果允许模式下没有 CH10 输出，检查：

```bash
ros2 topic echo /mavros/rc/in
ros2 topic echo /mavros/state
```

确认 `channels[9]` 是否真的变化，并确认 `mode` 是 `POSCTL` 或 `OFFBOARD`。

## 9. 测试 RC 第 10 通道真实控制夹爪

作用：测试完整链路：遥控器 CH10 -> MAVROS -> px4ctrl -> gripper topic -> Feetech 舵机。

终端 A：启动 MAVROS。

终端 B：启动 px4ctrl。

```bash
cd ~/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run px4ctrl px4ctrl_node --ros-args --params-file install/px4ctrl/share/px4ctrl/config/ctrl_param_fpv.yaml
```

终端 C：启动真实夹爪节点。

```bash
cd ~/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run px4ctrl feetech_gripper_node.py --ros-args -p port:=/dev/ttyACM1
```

这个节点默认已经是 `left_inverted:=true`、`right_inverted:=true`，因此 `/gripper/command=100.0` 应该对应物理全开。

确认 PX4 处于 `POSCTL` 或 `OFFBOARD` 后，拨动遥控器第 10 通道。

期望：

- `POSCTL` 或 `OFFBOARD` 下，CH10 低位：夹爪全开
- `POSCTL` 或 `OFFBOARD` 下，CH10 高位：夹爪全关
- 切到 `ALTCTL`、`STABILIZED`、`MANUAL`、触发降落或高度低于 `0.20 m` 后，无论 CH10 位置如何，夹爪自动全开

如果单独运行 ROS 夹爪节点时方向仍然反了，停止夹爪节点，用反向参数重启：

```bash
ros2 run px4ctrl feetech_gripper_node.py --ros-args \
  -p port:=/dev/ttyACM1 \
  -p left_inverted:=true \
  -p right_inverted:=true
```

## 10. 用 launch 同时启动 px4ctrl 和夹爪节点

作用：确认正式启动方式可用。`run_ctrl.launch.py` 会同时启动：

- `px4ctrl_node`
- `feetech_gripper_node.py`

命令：

```bash
cd ~/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch px4ctrl run_ctrl.launch.py
```

此时再拨动遥控器第 10 通道，夹爪应能开合。

## 11. 起飞和降落脚本测试

作用：确认 px4ctrl 能收到起飞/降落命令。

起飞：

```bash
cd ~/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
bash shflies/takeoff.sh
```

降落：

```bash
cd ~/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
bash shflies/land.sh
```

也可以直接发布：

```bash
ros2 topic pub --once /px4ctrl/takeoff_land quadrotor_msgs/msg/TakeoffLand "{takeoff_land_cmd: 1}"
ros2 topic pub --once /px4ctrl/takeoff_land quadrotor_msgs/msg/TakeoffLand "{takeoff_land_cmd: 2}"
```

含义：

- `1`：takeoff
- `2`：land

## 12. 最后测试 fly_x_gripper_test.py

作用：在已经完成 MAVROS、Nokov、px4ctrl、夹爪节点测试后，执行整机自动测试。

默认行为：

- 读取 Nokov / mocap pose
- 飞向 mocap 坐标系的 `x = 7.0`
- 飞行中 yaw 左右摆动 `1 rad`
- 飞行中夹爪开合 3 次
- 降落前强制夹爪全开

先查看帮助：

```bash
cd ~/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run px4ctrl fly_x_gripper_test.py --help
```

推荐第一次用保守参数：

```bash
ros2 run px4ctrl fly_x_gripper_test.py \
  --target-x 1.0 \
  --target-y 0.0 \
  --target-z 1.0 \
  --duration 20.0 \
  --yaw-amplitude 0.3 \
  --final-hold 3.0
```

确认小范围测试正常后，再使用 7 米目标：

```bash
ros2 run px4ctrl fly_x_gripper_test.py \
  --target-x 7.0 \
  --target-y 0.0 \
  --target-z 1.0 \
  --duration 35.0 \
  --yaw-amplitude 1.0 \
  --final-hold 3.0
```

如果你的 Nokov topic 不是自动识别到的 topic，显式指定：

```bash
ros2 run px4ctrl fly_x_gripper_test.py \
  --mocap-topic /your/nokov/pose/topic \
  --mocap-msg-type pose \
  --target-x 7.0 \
  --target-y 0.0 \
  --target-z 1.0
```

如果只想测试动作发布，不想自动降落：

```bash
ros2 run px4ctrl fly_x_gripper_test.py --no-land
```

## 13. LeRobot 手动飞行数据采集

作用：手动飞行时，LeRobot 记录 `/px4ctrl/expert_pose` 作为专家 action，同时记录 mocap 状态、夹爪状态和两路相机。

采集链路：

```text
遥控器手动飞行
  -> px4ctrl 内部 hover_pose
  -> /px4ctrl/expert_pose
  -> ros_expert_pose teleop
  -> lerobot-record 保存 action

遥控器 CH10
  -> px4ctrl
  -> /gripper/command
  -> ros_expert_pose teleop
  -> lerobot-record
  -> vla_drone robot
  -> Feetech 夹爪
```

重要规则：

- 手动采集时不要让 LeRobot 把 pose action 反写到 `/position_cmd`，否则会干扰 px4ctrl 的 RC hover 控制。因此必须设置 `--robot.send_pose_actions=false`。
- 手动采集时不要启动 `feetech_gripper_node.py`，因为 `vla_drone` robot 会直接打开 `/dev/ttyACM1` 控制 Feetech 夹爪。
- 夹爪仍然用遥控器 CH10 控制，但只有 `POSCTL` 或 `OFFBOARD` 下 CH10 生效。其它模式、降落、未解锁或 `z <= 0.20 m` 时 px4ctrl 会强制 `/gripper/command=100.0`。LeRobot 会记录 `/gripper/command`，并把该 action 发送给 `vla_drone` robot 执行。
- 当前相机约定：前视使用 `FRONT_CAMERA`，夹爪/下视使用 `DOWN_CAMERA`。现场固定路径写在 `shflies/record_camera_paths.env`。

### 13.1 启动飞行和定位链路

终端 A：启动 VRPN、MAVROS、vision bridge 和 px4ctrl。

```bash
cd ~/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2
bash shflies/run_mocap_mavros.sh
```

默认使用轻量 MAVROS 插件列表，只加载采集需要的插件，减少 ROS topic 数量和 NX 负载。保留的 MAVROS 功能包括：

- `/mavros/state`、`/mavros/extended_state`、`/mavros/battery`
- `/mavros/rc/in`
- `/mavros/set_mode`、`/mavros/cmd/arming`、`/mavros/cmd/command`
- `/mavros/setpoint_raw/local`
- `/mavros/vision_pose/pose`
- `/mavros/local_position/*` 和 `/mavros/imu/*` 诊断 topic

如果需要临时恢复 MAVROS 原始全插件配置：

```bash
cd ~/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2
MAVROS_LIGHT=false bash shflies/run_mocap_mavros.sh
```

确认以下 topic 正常：

```bash
source /opt/ros/humble/setup.bash
source ~/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2/install/setup.bash

ros2 topic hz /mavros/vision_pose/pose
ros2 topic hz /px4ctrl/expert_pose
ros2 topic echo /gripper/command
```

期望：

- `/mavros/vision_pose/pose` 有稳定 mocap 数据。
- `/px4ctrl/expert_pose` 持续发布。
- `POSCTL` 或 `OFFBOARD` 下拨动 CH10 时，`/gripper/command` 在 `100.0` 和 `0.0` 之间变化。
- `ALTCTL`、`STABILIZED`、`MANUAL`、降落、未解锁或 `z <= 0.20 m` 时，`/gripper/command` 为 `100.0`。

### 13.2 启动 LeRobot 录制

终端 B：使用采集脚本。脚本会自动 source ROS2 环境、进入 LeRobot 仓库、以当前时间生成数据集名字，并启动 `lerobot-record`。

```bash
cd ~/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2
bash shflies/record_vla_dataset.sh
```

默认行为：

- 数据集名：`vla_drone_grasp_YYYYmmdd_HHMMSS`
- 保存位置：`~/vla_drone/data/vla_drone_grasp_YYYYmmdd_HHMMSS`
- episode 数量：`1`
- 每条 episode 时长：`30 s`
- 开始门控：等待 `/px4ctrl/state` 连续 `AUTO_HOVER` `3.0 s` 后才开始正式录制
- 正式计时前预热：`2` 轮 observation/action 读取，不发送 action，不写入数据集
- reset 时长：`10 s`
- 采集频率：`20 fps`
- 相机 warmup：`3 s`
- 图像保存：开启，两路相机，当前默认 `front=/dev/video2`、`down=/dev/video0`
- 视频编码：`h264`
- 上传 Hugging Face Hub：关闭

常用覆盖示例：

```bash
cd ~/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2

# 采集前确认相机标签；如果设备号变化，可临时覆盖 FRONT_CAMERA/DOWN_CAMERA
ffplay /dev/video2   # 应为前视
ffplay /dev/video0   # 应为夹爪/下视

# 推荐固定相机路径，避免每次上电 /dev/videoN 改变
ls -l /dev/v4l/by-path/
cp shflies/record_camera_paths.env.example shflies/record_camera_paths.env
# 编辑 record_camera_paths.env，把 FRONT_CAMERA 和 DOWN_CAMERA 改成 /dev/v4l/by-path/... 路径

# 稳定优先：默认 20 fps，录 3 条，每条 30 秒
NUM_EPISODES=3 EPISODE_TIME_S=30 bash shflies/record_vla_dataset.sh

# 关闭 AUTO_HOVER 开始门控，仅用于地面调试
START_GATE_TOPIC="" bash shflies/record_vla_dataset.sh

# 质量优先：恢复 30 fps，但对 NX 压力更大
DATASET_FPS=30 CAMERA_FPS=30 TELEOP_MAX_POSE_AGE_S=0.5 \
  bash shflies/record_vla_dataset.sh

# 只做 10 秒调试，并关闭视频
DATASET_PREFIX=debug_vla_drone EPISODE_TIME_S=10 RESET_TIME_S=1 DATASET_VIDEO=false \
  bash shflies/record_vla_dataset.sh

# 临时修改任务描述
TASK="Fly to the target cube and close the gripper" bash shflies/record_vla_dataset.sh

# 续录指定数据集：NUM_EPISODES 表示本次追加几条 episode
RESUME_DATASET=true DATASET_NAME=vla_drone_grasp_YYYYmmdd_HHMMSS \
  NUM_EPISODES=3 bash shflies/record_vla_dataset.sh

# 交互选择已有数据集续录
RESUME_DATASET=true bash shflies/record_vla_dataset.sh

# 快速续录最新数据集，仅在确认最新目录就是目标数据集时使用
RESUME_LATEST=true NUM_EPISODES=3 bash shflies/record_vla_dataset.sh
```

### 13.3 录制脚本参数解释

自动命名相关参数：

- `RUN_ID`：默认使用当前时间，例如 `20260521_143012`。
- `DATASET_PREFIX`：数据集名前缀，默认 `vla_drone_grasp`。
- `DATASET_NAME`：完整数据集名，默认 `${DATASET_PREFIX}_${RUN_ID}`。
- `DATASET_BASE_DIR`：数据集根目录，默认 `${HOME}/vla_drone/data`。
- `DATASET_ROOT`：本地保存路径，默认 `${DATASET_BASE_DIR}/${DATASET_NAME}`。
- `REPO_OWNER`：repo_id 的用户名前缀，默认 `fd3s1`。
- `REPO_ID`：数据集 ID，默认 `${REPO_OWNER}/${DATASET_NAME}`。即使 `PUSH_TO_HUB=false`，LeRobot 仍需要一个 repo_id 作为数据集标识。
- `RESUME_DATASET`：默认 `false`。设为 `true` 时续录已有数据集，`NUM_EPISODES` 表示本次追加的 episode 数量。
- `RESUME_LATEST`：默认 `false`。设为 `true` 时自动续录 `${DATASET_BASE_DIR}` 下最新的 `${DATASET_PREFIX}_*` 数据集。
- `CONDA_ENV`：要激活的 conda 环境，默认 `vla-drone-v044`。
- `CONDA_SH`：conda 初始化脚本，默认 `${HOME}/miniforge3/etc/profile.d/conda.sh`。
- 脚本内部会执行 `source /opt/ros/humble/setup.bash`，让 conda Python 能 import ROS2 的 `rclpy`。
- 脚本内部会执行 `source install/setup.bash`，让当前 ROS2 workspace 的消息和节点环境生效。

Robot 参数：

- `NOKOV_POSE_TOPIC`：默认 `/mavros/vision_pose/pose`。从 MAVROS vision pose 读取当前 mocap 位姿，作为 observation state 的 `x, y, z, yaw` 来源。
- `ROBOT_MAX_POSE_AGE_S`：默认 `2.0`。允许 LeRobot 在采集过程中短暂等待最新 mocap pose。NX 同时读两路相机和写数据时偶发调度延迟，`0.5 s` 容易误判为 pose 超时。
- `MAVROS_SETPOINT_TOPIC`：默认 `/position_cmd`。推理阶段向 px4ctrl 发送目标位置的 topic。手动采集时保留该配置，但脚本固定使用 `--robot.send_pose_actions=false`，不会发送 pose action。
- `GRIPPER_PORT`：默认 `/dev/ttyACM1`。Feetech 舵机总线串口。
- `GRIPPER_LEFT_INVERTED` / `GRIPPER_RIGHT_INVERTED`：默认 `true`。录制脚本会显式传给 LeRobot，保证 `100.0 = 物理全开`、`0.0 = 物理全关`，避免受旧环境默认值影响。
- `SAFE_OPEN_GRIPPER_AFTER_EPISODE`：默认 `true`。每条 episode 到时结束后，立即在保存/编码视频前直接向 Feetech 舵机写入全开位置。这个动作不写入 dataset，也不发布 action。
- `SAFE_OPEN_GRIPPER_ON_DISCONNECT`：默认 `true`。录制进程退出并进入 robot disconnect 时，在关闭 `/dev/ttyACM1` 前再次直接向 Feetech 舵机写入全开位置，避免串口关闭后夹爪停在闭合位置。
- `DISCONNECT_GRIPPER_OPEN_POSITION`：默认 `100.0`。disconnect 前写入的夹爪全开目标。保持数据语义 `100 = 全开`。
- `DISCONNECT_GRIPPER_REPEATS`：默认 `3`。disconnect 前重复写入全开目标的次数，降低单次串口写入失败的风险。
- `DISCONNECT_GRIPPER_SETTLE_S`：默认 `0.5`。写入全开目标后等待舵机动作完成，再关闭串口和扭矩。
- `FRONT_CAMERA`：默认 `/dev/video2`，前视相机。
- `DOWN_CAMERA`：默认 `/dev/video0`，夹爪/下视相机。
- `CAMERA_PATHS_FILE`：默认 `shflies/record_camera_paths.env`。如果该文件存在，脚本会先读取它。建议在里面写 `/dev/v4l/by-path/...` 稳定路径，避免每次上电 `/dev/video0/2` 顺序变化。
- `CAMERA_WIDTH`、`CAMERA_HEIGHT`、`CAMERA_FPS`：默认 `640`、`480`、`20`。需要 30fps 时可设置 `CAMERA_FPS=30`。
- `CAMERA_WARMUP_S`：默认 `3`。相机连接后先读取几秒再进入 episode，减少第一次 record loop 因相机预热导致的低频 warning。

Teleop 参数：

- `EXPERT_POSE_TOPIC`：默认 `/px4ctrl/expert_pose`。读取 px4ctrl 发布的专家目标位姿，保存为 action 的 `x, y, z, yaw`。
- `GRIPPER_TOPIC`：默认 `/gripper/command`。读取 CH10 产生的夹爪命令，保存为 action 的 `gripper_left.pos, gripper_right.pos`。
- `TELEOP_STARTUP_TIMEOUT_S`：默认 `2.0`。录制刚开始时等待第一帧 `/px4ctrl/expert_pose` 的最长时间。
- `TELEOP_MAX_POSE_AGE_S`：默认 `0.5`。允许专家 pose 的最大年龄。如果 `/px4ctrl/expert_pose` 超过该时间没更新，录制会报错，避免保存动作和图像严重错位的数据。之前 `0.2 s` 在 NX 高负载时容易因为 ROS 回调线程被抢占而误触发。

Dataset 参数：

- `DATASET_FPS`：默认 `20`。LeRobot 保存数据的目标频率。默认降到 20fps 是为了降低 NX 上双相机、写盘和 ROS 回调竞争。需要 30fps 时可设置 `DATASET_FPS=30 CAMERA_FPS=30`。
- `NUM_EPISODES`：默认 `1`。本次连续采集的 episode 数量。
- `EPISODE_TIME_S`：默认 `30`。每条 episode 最长 30 秒。
- `RECORD_PREWARM_STEPS`：默认 `2`。每条 episode 正式计时前先读取若干轮 observation 和 teleop action，用来预热相机、ROS pose、专家 pose 和夹爪读取路径。预热阶段不会调用 `robot.send_action()`，不会发布位置 action 或夹爪 action，也不会调用 `dataset.add_frame()`，因此不会污染数据集。
- `START_GATE_TOPIC`：默认 `/px4ctrl/state`。正式录制前等待的 ROS2 String topic。设为空字符串可关闭门控。
- `START_GATE_VALUE`：默认 `AUTO_HOVER`。只有 topic 内容等于该值时才开始稳定计时。
- `START_GATE_STABLE_S`：默认 `3.0`。`/px4ctrl/state` 必须连续保持 `AUTO_HOVER` 的时间。切出 AUTO_HOVER 会清零重新计时。
- `START_GATE_TIMEOUT_S`：默认 `0.0`。`0.0` 表示无限等待。现场飞行建议保持无限等待，准备好后切入 AUTO_HOVER 即可。
- `DATASET_STATUS_TOPIC`：默认空。自动采集脚本使用时设为 `/lerobot_record/status`，record 完成相机、夹爪和 ROS 初始化并等待 gate 后会发布 `WAITING_GATE`。
- `RESET_TIME_S`：默认 `10`。两条 episode 之间留 10 秒复位时间。
- `TASK`：默认 `Fly to the target and operate the gripper`。本批数据的任务描述。
- `PUSH_TO_HUB`：默认 `false`。采集后只保存到本地，不自动上传 Hugging Face Hub。
- `DATASET_VIDEO`：默认 `true`。正式训练 SmolVLA 时必须保留图像；调试时可以临时设置为 `false`。
- `DATASET_VCODEC`：默认 `h264`。比 `libsvtav1` 编码压力更低，文件会更大一些，但更适合 NX 现场采集。
- `STREAMING_ENCODING`：默认 `false`。保持先写临时图像、episode 后再编码，避免实时编码抢占飞行采集主循环。
- `ENCODER_THREADS`：默认 `2`。限制视频编码线程数，减少编码阶段对系统的冲击。
- `IMAGE_WRITER_PROCESSES`：默认 `0`。使用线程写图，不额外开子进程。
- `IMAGE_WRITER_THREADS_PER_CAMERA`：默认 `2`。两路相机共 4 个写图线程。原默认每相机 4 个线程在 NX 上容易和 ROS 回调、相机读取抢 CPU。
- `PLAY_SOUNDS`：默认 `false`。关闭录制提示音，避免 NX 环境缺少音频设备时报错。

### 13.4 飞行中开始采集流程

正式飞行采集时，不需要在地面提前进入 AUTO_HOVER 再等待录制启动。推荐流程：

1. 终端 A 启动 `run_mocap_mavros.sh`。
2. 终端 B 启动 `record_vla_dataset.sh`。
3. 录制脚本完成相机、ROS、夹爪连接后，会等待 `/px4ctrl/state == AUTO_HOVER`，此时还没有开始写入 dataset。
4. 手动使用 POSCTL 起飞到合适位置。
5. CH5 切入 auto hover。
6. `/px4ctrl/state` 连续保持 `AUTO_HOVER` `3.0 s` 后，脚本执行 `RECORD_PREWARM_STEPS` 预热，然后正式开始 episode 计时和写入数据。

检查命令：

```bash
ros2 topic echo /px4ctrl/state
ros2 topic echo /px4ctrl/expert_pose
ros2 topic echo /mavros/state
```

注意：

- AUTO_HOVER 门控依据的是 px4ctrl 内部 FSM 状态 `/px4ctrl/state`，不是 PX4 的 `/mavros/state.mode`。
- 未进入 AUTO_HOVER 前可以没有 `/px4ctrl/expert_pose`，录制脚本不会在门控通过前读取 expert action。
- 门控和预热阶段都不会调用 `dataset.add_frame()`，不会污染数据集。
- 门控和预热阶段都不会调用 `robot.send_action()`，不会发布 `/position_cmd`，也不会主动驱动夹爪。
- px4ctrl 终端中进入 AUTO_HOVER 的状态切换日志为绿色，离开 AUTO_HOVER 的状态切换日志为红色，便于飞行中快速判断状态变化。

### 13.4.1 自动抓取/放置采集

如果手动 AUTO_HOVER 不跟手，可以使用自动采集入口。该入口不会覆盖手动脚本：`record_vla_dataset.sh` 默认仍然是手动采集，自动流程只通过总控脚本临时设置 record gate，并启动一个独立的 gripper manager 独占 `/dev/ttyACM1`。

默认 VRPN 刚体 topic：

- 被抓目标：`/strawberry_bear/pose`
- 放置盒子：`/box1/pose`

启动前确认：

```bash
ros2 topic echo /strawberry_bear/pose
ros2 topic echo /box1/pose
ros2 topic echo /mavros/vision_pose/pose
ros2 topic echo /px4ctrl/state
```

自动脚本订阅目标、盒子和无人机位姿时使用 `BEST_EFFORT` 订阅，兼容 VRPN 原始刚体 topic，也能接收 MAVROS/bridge 的可靠发布。若终端一直显示 `Still waiting for fresh poses`，优先检查 topic 名字和 QoS：

```bash
ros2 topic info /strawberry_bear/pose -v
ros2 topic info /box1/pose -v
ros2 topic echo /strawberry_bear/pose
ros2 topic echo /box1/pose
```

自动采集脚本不会启动 VRPN/MAVROS/px4ctrl。先在第一个终端启动 mocap 和飞控链路：

```bash
cd ~/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2
bash shflies/run_mocap_mavros.sh
```

确认 `/mavros/vision_pose/pose`、`/strawberry_bear/pose`、`/box1/pose` 都有数据后，把 CH5 和 CH6 都拨到高位，在第二个终端运行：

```bash
cd ~/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2
bash shflies/auto_record_grasp_place.sh
```

`auto_record_grasp_place.sh` 启动 record 前会先用 `ros2 topic echo --once` 做一次位姿 topic 预检查，默认等待 `6s`。这个检查只作为提示：如果某个 topic 因 DDS 发现延迟没有被一次性命令捕获，脚本会打印 warning 并继续。真正的自动任务节点会持续等待 `/mavros/vision_pose/pose`、目标和盒子三个位姿都新鲜后才起飞。

相关参数：

```bash
# 默认：预检查失败只报警，后续自动节点继续等待
POSE_PREFLIGHT_TIMEOUT_S=6 bash shflies/auto_record_grasp_place.sh

# 如果希望预检查失败就退出
POSE_PREFLIGHT_REQUIRED=true bash shflies/auto_record_grasp_place.sh

# 如果不想做这一步一次性预检查
SKIP_POSE_PREFLIGHT=true bash shflies/auto_record_grasp_place.sh
```

如果 VRPN 刚体名字变化，可以临时改 topic：

```bash
TARGET_POSE_TOPIC=/new_target/pose \
BOX_POSE_TOPIC=/new_box/pose \
bash shflies/auto_record_grasp_place.sh
```

第一次飞行建议使用更保守速度。草莓熊约 `300 g`，对当前飞机属于明显带载，优先降低失控风险，不追求第一次就夹得很紧：

```bash
MAX_SPEED=0.5 APPROACH_SPEED=0.25 LIFT_SPEED=0.35 \
PAYLOAD_LIFT_SPEED=0.06 PAYLOAD_TRANSFER_SPEED=0.10 \
POST_GRASP_SETTLE_S=2.0 POST_LIFT_SETTLE_S=2.0 \
bash shflies/auto_record_grasp_place.sh
```

默认几何假设：

- 夹爪夹持中心在无人机 mocap 刚体中心下方 `0.25 m`：`GRIPPER_Z_OFFSET_M=0.25`。
- 被抓目标高度 `0.30 m`：`TARGET_HEIGHT_M=0.30`。
- 目标刚体 z 默认表示目标几何中心：`TARGET_POSE_Z_REFERENCE=center`。
- 夹爪默认夹在目标底部上方 `0.17 m` 处：`TARGET_GRASP_HEIGHT_M=0.17`。
- 周转箱尺寸默认 `0.65 x 0.41 x 0.14 m`：`BOX_LENGTH_M=0.65`、`BOX_WIDTH_M=0.41`、`BOX_HEIGHT_M=0.14`。
- 周转箱刚体中心默认在箱子上沿平面、XY 为箱体几何中心。自动脚本会把玩具放到箱子中心附近。
- 夹取高度和放置高度分别计算，不使用同一个飞行高度。放置高度按“箱底 + 目标夹持高度 + 释放余量”换算到无人机中心高度。
- 脚本会用箱体长宽做中心放置余量检查；默认把目标高度 `0.30 m` 作为保守占地尺寸估计。
- 可对目标和箱子的规划位置施加 mocap/map 坐标系偏置。偏置单位是米，方向与 mocap 的 `x/y/z` 完全一致，不随无人机 yaw 旋转：
  - `TARGET_OFFSET_X/Y/Z`
  - `BOX_OFFSET_X/Y/Z`

如果夹爪下偏不是 `0.25 m`，临时覆盖：

```bash
GRIPPER_Z_OFFSET_M=0.22 bash shflies/auto_record_grasp_place.sh
```

如果你说的 `0.25 cm` 是真实尺寸，而不是 `0.25 m`，应使用：

```bash
GRIPPER_Z_OFFSET_M=0.0025 bash shflies/auto_record_grasp_place.sh
```

如果需要手动修正目标或箱子位置，例如抓取点相对草莓熊刚体向 `+x` 偏 `5 cm`、放置点相对箱子中心向 `-y` 偏 `4 cm`：

```bash
TARGET_OFFSET_X=0.05 BOX_OFFSET_Y=-0.04 \
bash shflies/auto_record_grasp_place.sh
```

自动流程：

1. 总控脚本先启动 `feetech_gripper_node.py`，由它独占 `/dev/ttyACM1`，发布 `/gripper/feedback`，接收 `/gripper/command_pair` 和兼容旧流程的 `/gripper/command`。
2. 总控脚本再启动 `record_vla_dataset.sh`，并临时设置 `USE_ROS_GRIPPER=true`。此时 LeRobot record 不再打开串口，只从 `/gripper/feedback` 读取夹爪状态。
3. record 完成相机、夹爪反馈、ROS bridge 初始化后发布 `/lerobot_record/status = WAITING_GATE`，此时还没有起飞，也没有写 dataset。
4. 自动任务脚本读取 `/strawberry_bear/pose`、`/box1/pose`、`/mavros/vision_pose/pose`，确认新鲜稳定。
5. 自动发布夹爪全开 `100.0`，然后发布 `/px4ctrl/takeoff_land` 起飞。
6. 等 `/px4ctrl/state = AUTO_HOVER`，表示 `AUTO_TAKEOFF` 已完成。
7. 自动发布当前位置 hold 的 `/position_cmd`，让 px4ctrl 进入 `CMD_CTRL`。
8. 进入 `CMD_CTRL` 后发布 `/auto_grasp_dataset/record_gate = START`，record 才开始写 dataset。
9. 第一帧保持任务起点悬停，下一帧开始向目标飞，避免数据集中包含起飞后的长时间悬停。

目标/盒子位姿更新策略：

- 自动任务启动前会先确认目标、盒子、无人机三者位姿新鲜稳定，但不会只使用这一刻的位置跑完整个任务。
- 飞向草莓熊上方时，脚本会持续读取 `/strawberry_bear/pose`，实时刷新目标上方 waypoint。
- 到达目标上方后，进入下降、闭合夹爪、抬升阶段，这些阶段可能遮挡草莓熊刚体；脚本会锁存最后一次新鲜目标位姿，避免遮挡导致 waypoint 跳变。
- 飞向盒子上方时，脚本会持续读取 `/box1/pose`，实时刷新盒子上方 waypoint。
- 到达盒子上方后，进入下降投放阶段，可能遮挡盒子刚体；脚本会锁存最后一次新鲜盒子位姿。
- 如果接近阶段短暂看不到目标或盒子，脚本会继续使用锁存位置，并打印 `Using latched ... pose`。如果从未获得过可用锁存位姿，则会报错退出。

轨迹速度：

- `/position_cmd` 是位置目标，但自动脚本按 `20 Hz` 逐点插值发布，不直接跳到目标点。
- `px4ctrl` 会从连续 `/position_cmd` 估计速度/加速度前馈，并写入 MAVROS `PositionTarget`。当前配置限幅为 `cmd_feedforward.max_velocity=0.8 m/s`、`cmd_feedforward.max_acceleration=1.5 m/s^2`。
- 默认开启 `SMOOTH_TRAJECTORY=true`，轨迹采用平滑起停，避免段起点/终点速度突变激发草莓熊摆动。
- `MAX_SPEED` 默认 `0.6 m/s`，建议范围 `0.5-1.0 m/s`。
- `APPROACH_SPEED` 默认 `0.3 m/s`，用于下降接近目标和盒子。
- `LIFT_SPEED` 默认 `0.4 m/s`，用于抓取后抬升和释放后抬升。
- `PAYLOAD_LIFT_SPEED` 默认 `0.10 m/s`，用于夹住草莓熊后的带载抬升。
- `PAYLOAD_TRANSFER_SPEED` 默认 `0.16 m/s`，用于带载飞向箱子。
- `POST_GRASP_SETTLE_S` 默认 `1.5 s`，夹住后原地等待，让负载先稳定。
- `POST_LIFT_SETTLE_S` 默认 `1.5 s`，抬升后原地等待，降低摆振后再横移。
- `TAKEOFF_FORWARD_COMP_M` 默认 `0.0 m`，起飞完成进入 `CMD_CTRL` 后、record 开始前，沿无人机当前机头方向做前向补偿。用于抵消机体后重导致的起飞后后窜。
- `PAYLOAD_LIFT_FORWARD_COMP_M` 默认 `0.0 m`，夹住草莓熊后抬升时，沿无人机当前机头方向同步做前向补偿。用于抵消带载抬升阶段后窜。
- `TAKEOFF_COMP_X/Y/Z` 默认 `0.0 m`，起飞后按 mocap/map 坐标系直接补偿位置，不依赖无人机 yaw。
- `PAYLOAD_LIFT_COMP_X/Y/Z` 默认 `0.0 m`，夹住草莓熊后抬升时按 mocap/map 坐标系补偿位置。
- `RETREAT_SPEED` 默认 `0.6 m/s`，用于放置后向前撤离。

如果夹起草莓熊后摆动明显，先使用更保守的带载参数：

```bash
PAYLOAD_LIFT_SPEED=0.06 \
PAYLOAD_TRANSFER_SPEED=0.10 \
POST_GRASP_SETTLE_S=2.0 \
POST_LIFT_SETTLE_S=2.0 \
bash shflies/auto_record_grasp_place.sh
```

如果机体后重导致起飞或带载抬升时明显往后窜，可先小量补偿，不建议一开始超过 `0.10 m`：

```bash
TAKEOFF_FORWARD_COMP_M=0.05 \
PAYLOAD_LIFT_FORWARD_COMP_M=0.05 \
bash shflies/auto_record_grasp_place.sh
```

如果发现 `TAKEOFF_FORWARD_COMP_M` 或 `PAYLOAD_LIFT_FORWARD_COMP_M` 方向不对，说明 mocap 刚体 yaw 和实际机头方向可能不一致。此时优先用 map 坐标系补偿，例如希望向 mocap `+X` 方向补 `5 cm`：

```bash
TAKEOFF_FORWARD_COMP_M=0.0 \
PAYLOAD_LIFT_FORWARD_COMP_M=0.0 \
TAKEOFF_COMP_X=0.05 \
PAYLOAD_LIFT_COMP_X=0.05 \
bash shflies/auto_record_grasp_place.sh
```

如果需要向 mocap `-X`、`+Y` 或 `-Y` 方向补偿，分别设置负号或对应轴，例如 `TAKEOFF_COMP_X=-0.05`、`TAKEOFF_COMP_Y=0.05`。

夹爪软夹持：

- 当前 STS3215 仍使用位置伺服模式，不是真正硬件力控。
- 自动脚本不再从 `100.0` 硬闭合到 `0.0`，而是通过 `/gripper/command_pair` 让左右夹爪按小步低速闭合。
- gripper manager 以 `/gripper/feedback` 发布左右位置、load、current、位置误差；自动脚本用这些反馈判断接触。
- gripper manager 退出时默认再次写入全开位置：`open_on_shutdown=true`、`shutdown_open_position=100.0`、`shutdown_open_repeats=3`。
- 抓取阶段 `z/yaw` 保持，`x/y` 做顺从保持：允许无人机在小半径内让开夹爪反作用力，避免位置环硬拉导致机体倾斜放大。
- 如果抓取阶段平面漂移超过 `GRASP_ABORT_DRIFT_M`，脚本会打开夹爪并退出。
- 软夹持目标是“刚好抓住”，不是把草莓熊强行拖到几何中心。抓到后会做少量左右负载均衡。

默认参数：

```bash
GRASP_STEP_SIZE=3.0
GRASP_STEP_SETTLE_S=0.10
GRASP_OPEN_TIMEOUT_S=2.0
GRASP_OPEN_TOLERANCE=5.0
GRASP_STEP_TIMEOUT_S=0.80
GRASP_GOAL_TOLERANCE=3.0
GRASP_CLOSE_MIN=15.0
GRASP_CONTACT_CURRENT_DELTA=250
GRASP_CONTACT_LOAD_DELTA=800
GRASP_POSITION_ERROR_THRESHOLD=20.0
GRASP_ANGLE_CONTACT_DELTA=20.0
GRASP_STALL_DELTA=0.25
GRASP_CONTACT_MIN_CLOSE_DELTA=15.0
GRASP_CONTACT_CONFIRM_STEPS=2
GRASP_BALANCE_LOAD_DIFF=60
GRASP_BALANCE_STEP=1.5
GRASP_MAX_BALANCE_STEPS=8
GRASP_ANGLE_BALANCE_DIFF=5.0
GRASP_COMPLIANCE_RADIUS_M=0.14
GRASP_ABORT_DRIFT_M=0.24
```

这些参数集中在 `shflies/grasp_params.env`。`auto_record_grasp_place.sh` 和手持调参脚本都会 source 同一个文件，所以你在这里稳定下来的参数会默认同步到真实自动飞行。命令行环境变量优先级更高，只影响本次运行：

```bash
GRASP_STEP_SIZE=2.0 GRASP_CLOSE_MIN=25.0 bash shflies/auto_record_grasp_place.sh
```

如果夹取仍然扰动大，先让夹爪更保守：

```bash
GRASP_STEP_SIZE=1.0 \
GRASP_STEP_SETTLE_S=0.25 \
GRASP_CLOSE_MIN=30.0 \
GRASP_COMPLIANCE_RADIUS_M=0.16 \
GRASP_ABORT_DRIFT_M=0.24 \
bash shflies/auto_record_grasp_place.sh
```

### 13.4.2 手持夹爪软夹持调参

该入口用于手持无人机调夹爪，不录制数据集、不起飞、不发布 `/position_cmd`，也不发布 `/px4ctrl/takeoff_land`。它仍然读取无人机、草莓熊、box 位姿和 gripper feedback，并用 CH10 触发与真实自动飞行一致的软夹持动作。

启动前先运行基础链路，保证 MAVROS、VRPN、px4ctrl 已经工作：

```bash
cd ~/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2
bash shflies/run_mocap_mavros.sh
```

第二个终端运行手持调参：

```bash
cd ~/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2
bash shflies/handheld_grasp_tune.sh
```

行为：

- 脚本默认启动 `feetech_gripper_node.py` 并独占 `/dev/ttyACM1`。
- gripper manager 的 scalar `/gripper/command` 会被重映射到 `/handheld_grasp_tune/ignore_scalar_command`，避免 px4ctrl 的 CH10 直接闭合命令和测试节点的 pair command 抢夹爪。
- 测试节点只发布 `/gripper/command_pair`。
- CH10 低位：自动软夹持生效。触发时终端会打印 `CH10 low: starting automatic soft-grasp sequence.`。
- CH10 高位：手动超控自动夹取，直接发布左右夹爪目标 `MANUAL_OVERRIDE_POS`，默认 `15.0`。高位期间自动软夹持暂停；回到低位后重新进入自动软夹持。
- 自动软夹持每一步会等待舵机跟上目标，默认最多 `GRASP_STEP_TIMEOUT_S=0.80s`；角度滞后只有在这一步超时后才会当成接触，避免把正常跟随延迟误判成夹到物体。
- 自动软夹持开始前会先发布全开，并等待左右夹爪接近 `GRIPPER_OPEN`，避免从手动闭合位置直接进入自动闭合导致跳变。
- 终端会持续显示无人机相对草莓熊抓取点、box 放置点的 `dx/dy/dz`，以及当前左右夹爪位置、goal、load、current。
- 如果感觉像 CH10 在直接手动控制夹爪，而不是软夹持，先确认没有旧的 `feetech_gripper_node.py` 仍在监听 `/gripper/command`。手持脚本启动时会检查并提示已有 gripper 节点。

常用调参方式：

```bash
# 临时更保守，只影响本次手持测试
GRASP_STEP_SIZE=1.5 GRASP_CLOSE_MIN=25.0 bash shflies/handheld_grasp_tune.sh

# 临时更大胆，只影响本次手持测试
GRASP_STEP_SIZE=4.0 GRASP_CLOSE_MIN=10.0 bash shflies/handheld_grasp_tune.sh

# 高位手动超控如果只想打开，可设成 100
MANUAL_OVERRIDE_POS=100.0 bash shflies/handheld_grasp_tune.sh

# 如果确实要测试硬闭合到 0，需要明确指定；容易触发过载，不建议带物体使用
MANUAL_OVERRIDE_POS=0.0 bash shflies/handheld_grasp_tune.sh
```

如果某组参数手持测试效果好，把它写入：

```bash
vim shflies/grasp_params.env
```

之后运行真实自动采集：

```bash
bash shflies/auto_record_grasp_place.sh
```

会默认使用同一套参数，不需要再手动复制。

放置后撤离：

- 物体释放后，脚本会再次发布夹爪全开 `100.0`。
- 然后从放置点上升 `0.3 m`：`RELEASE_RETREAT_UP_M=0.3`。
- 再沿当前 yaw 的机头前方飞 `2.0 m`：`RELEASE_RETREAT_FORWARD_M=2.0`。
- 撤离完成后默认不触发 px4ctrl `AUTO_LAND`，而是继续在 `CMD_CTRL` 下用 `/position_cmd` 限速下降。
- `LANDING_MODE=cmd` 为默认值。CMD 降落目标高度默认 `CMD_LAND_Z=-0.3`。
- `CMD_LAND_SPEED` 默认 `0.25 m/s`。降落过程中持续发布夹爪全开 `100.0`。
- CMD 降落会进入 dataset，用于记录完整任务收尾；但它不会自动 disarm，落地后需要手动切模式/上锁，或后续再加自动 disarm 策略。
- `EPISODE_TIME_S` 需要足够覆盖“起飞后任务开始、抓取、放置、撤离、降落”全过程；如果 episode 太短，降落后半段不会被保存。

如需修改：

```bash
RELEASE_RETREAT_UP_M=0.4 RELEASE_RETREAT_FORWARD_M=1.5 \
bash shflies/auto_record_grasp_place.sh
```

如需指定 CMD 降落高度或改回 px4ctrl 自动降落：

```bash
CMD_LAND_Z=-0.25 CMD_LAND_SPEED=0.2 bash shflies/auto_record_grasp_place.sh

LANDING_MODE=auto bash shflies/auto_record_grasp_place.sh
```

注意：

- 目标和盒子的 mocap 位姿只用于自动脚本规划，不进入 LeRobot dataset features。
- record 只记录飞机 observation、相机、夹爪状态和 `/px4ctrl/expert_pose` action。自动流程中夹爪状态来自 `/gripper/feedback`，record 不直接打开 `/dev/ttyACM1`。
- 默认 CMD 降落会进入 LeRobot dataset；`EPISODE_TIME_S` 应覆盖完整任务，包括降落段。

夹爪安全策略：

- PX4 `POSCTL` 和 `OFFBOARD` 模式下允许 CH10 控制夹爪。
- PX4 掉到 `ALTCTL`、`STABILIZED`、`MANUAL` 等非允许模式时，px4ctrl 会强制发布夹爪全开。
- RC 超时、mocap/odom 超时、未解锁、降落、自动起飞/降落、低于 `force_open_below_z` 时，px4ctrl 也会强制发布夹爪全开。
- 强制全开不是只发布一次；安全条件持续存在时会周期性重发全开命令，降低串口或舵机漏掉单次命令的风险。

如果录制中出现：

```text
Expert pose on /px4ctrl/expert_pose is stale: 0.520s > 0.500s
```

这通常表示 px4ctrl 超过 `0.5 s` 没有发布 `/px4ctrl/expert_pose`。当前代码只有在 `odom_is_received()` 为真时才发布 expert pose，因此常见根因是 mocap/网络/VRPN/MAVROS vision pose 短时中断。这个报错不是单纯录制阈值过紧；它表示 action 已经不再新鲜，继续保存会污染图像和动作对齐。

### 13.5 时间戳对齐规则

- `/px4ctrl/expert_pose.header.stamp` 来自 px4ctrl 控制周期，和同周期控制目标一致。
- `ros_expert_pose` teleop 每帧读取最新 `/px4ctrl/expert_pose`，如果 age 超过 `--teleop.max_pose_age_s` 就停止记录并报错。
- `vla_drone` robot 在一次 `get_observation()` 中读取相机、夹爪和 mocap pose，并把它们保存成同一帧 observation。
- `/gripper/command` 是保持型目标命令，不是连续流；teleop 会记录最后一次夹爪目标。没有收到夹爪命令时默认记录 `100.0`，即全开。
- 每条 episode 到时结束后，`vla_drone` robot 会立即直接把 Feetech 夹爪打开到 `100.0`，然后再保存/编码视频。录制进程退出进入 disconnect 时会再开一次。这个动作不会保存进 dataset，也不会发布 `/position_cmd`。

### 13.6 每次采集前检查

```bash
ros2 topic hz /mavros/vision_pose/pose
ros2 topic hz /px4ctrl/expert_pose
ros2 topic echo /mavros/state --once
ros2 topic echo /gripper/command
```

同时确认：

- QGC 没有 `yaw_estimate_error`。
- Position 模式悬停稳定。
- 当前 mocap 位置在 `ctrl_param_fpv.yaml` 的 `limits` 范围内，特别是大场地负 X 方向不要小于 `x_min`。
- `POSCTL/OFFBOARD` 下 CH10 能实际控制夹爪。
- `ALTCTL/STABILIZED/MANUAL/AUTO_LAND/未解锁/z <= 0.20 m` 下夹爪自动全开。
- `/dev/video0` 和 `/dev/video2` 都能被 LeRobot 找到。
- 没有单独运行 `feetech_gripper_node.py`。

### 13.7 复制数据集到服务器并可视化

在 NX 上选择要复制的数据集。快速复制最新数据集时用：

```bash
conda activate vla-drone-v044
cd ~/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2

DATASET_ROOT="$(ls -td ~/vla_drone/data/vla_drone_grasp_* | head -1)"
DATASET_NAME="$(basename "$DATASET_ROOT")"
REPO_ID="fd3s1/${DATASET_NAME}"
```

如果要复制指定数据集，手动填 `DATASET_NAME`：

```bash
conda activate vla-drone-v044
cd ~/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2

DATASET_NAME="vla_drone_grasp_YYYYmmdd_HHMMSS"
DATASET_ROOT="${HOME}/vla_drone/data/${DATASET_NAME}"
REPO_ID="fd3s1/${DATASET_NAME}"
```

如果不确定名字，可以先列出所有本地数据集，再复制选中的那个：

```bash
ls -td ~/vla_drone/data/vla_drone_grasp_*

DATASET_ROOT="/home/user/vla_drone/data/vla_drone_grasp_YYYYmmdd_HHMMSS"
DATASET_NAME="$(basename "$DATASET_ROOT")"
REPO_ID="fd3s1/${DATASET_NAME}"
```

确认路径：

```bash
echo "DATASET_ROOT=${DATASET_ROOT}"
echo "DATASET_NAME=${DATASET_NAME}"
echo "REPO_ID=${REPO_ID}"
```

推荐用 `rsync` 复制到服务器：

```bash
ssh user@10.1.1.35 "mkdir -p ~/vla_drone/data"
rsync -av "$DATASET_ROOT" user@10.1.1.35:~/vla_drone/data/
```

如果出现：

```text
ssh: connect to host 10.1.1.35 port 22: Connection refused
```

说明服务器 SSH 服务没有启动或没有安装。在服务器上执行：

```bash
sudo systemctl status ssh
sudo apt install -y openssh-server
sudo systemctl enable --now ssh
```

如果暂时不用 SSH，也可以先打包再拷贝：

```bash
tar -czf ~/vla_drone/${DATASET_NAME}.tar.gz -C "$(dirname "$DATASET_ROOT")" "$DATASET_NAME"
```

在服务器上解压：

```bash
mkdir -p ~/vla_drone/data
tar -xzf ~/vla_drone/${DATASET_NAME}.tar.gz -C ~/vla_drone/data
```

服务器上建议单独建一个只用于可视化的环境：

```bash
conda create -n vla-drone-viz python=3.10 -y
conda activate vla-drone-viz
cd ~/vla_drone/lerobot
pip install -e .
pip install rerun-sdk opencv-python av torchvision
```

注意 Python 包名是 `av`，不是 `pyav`。

用 LeRobot/Rerun 可视化：

```bash
conda activate vla-drone-viz
cd ~/vla_drone/lerobot

DATASET_ROOT="$(ls -td ~/vla_drone/data/vla_drone_grasp_* | head -1)"
DATASET_NAME="$(basename "$DATASET_ROOT")"
REPO_ID="fd3s1/${DATASET_NAME}"

lerobot-dataset-viz \
  --repo-id "$REPO_ID" \
  --root "$DATASET_ROOT" \
  --episode-index 0 \
  --num-workers 0 \
  --batch-size 8 \
  --tolerance-s 0.01
```

如果 Rerun 播放一卡一卡，先不要直接判断数据集坏了。`lerobot-dataset-viz` 是逐帧解码后再写入 Rerun，低功耗机器或远程显示会更容易卡。可以先直接播放原始视频：

```bash
find "$DATASET_ROOT" -type f | grep -E '\.(mp4|avi|mkv)$'

ffplay "$DATASET_ROOT/videos/observation.images.front/chunk-000/file-000.mp4"
ffplay "$DATASET_ROOT/videos/observation.images.down/chunk-000/file-000.mp4"
```

对已经编码好的 `.mp4`，`ffplay` 不需要加 `-framerate`。如果 `ffplay` 流畅，通常说明视频本身没问题，卡顿主要来自 Rerun 可视化链路。

也可以把前视和下视合成一个预览视频：

```bash
ffmpeg \
  -i "$DATASET_ROOT/videos/observation.images.front/chunk-000/file-000.mp4" \
  -i "$DATASET_ROOT/videos/observation.images.down/chunk-000/file-000.mp4" \
  -filter_complex hstack \
  -c:v libx264 -crf 20 -preset veryfast \
  /tmp/vla_drone_preview.mp4

ffplay /tmp/vla_drone_preview.mp4
```

检查数据集帧数和时间戳：

```bash
DATASET_ROOT="$DATASET_ROOT" REPO_ID="$REPO_ID" python - <<'PY'
from lerobot.datasets.lerobot_dataset import LeRobotDataset
import numpy as np
import os

root = os.environ["DATASET_ROOT"]
repo_id = os.environ["REPO_ID"]
ds = LeRobotDataset(repo_id, root=root)
ts = np.array([ds[i]["timestamp"].item() for i in range(ds.num_frames)])

print("frames:", ds.num_frames)
print("episodes:", ds.num_episodes)
print("duration:", ts[-1] - ts[0])
print("mean dt:", np.diff(ts).mean())
print("max dt:", np.diff(ts).max())
print("features:", ds.features)
PY
```

20 fps 的正常数据通常 `mean dt` 接近 `0.05`，`max dt` 也应接近 `0.05`。如果 Rerun 卡但 `ffplay` 流畅、时间戳正常，优先按可视化性能问题处理。

如果发现 `front/down` 画面标签反了，优先检查 NX 上 `shflies/record_camera_paths.env` 的固定摄像头路径。历史旧数据集不建议自动改标签，避免和已有 episode 混淆。

## 14. 常用排错命令

查看所有相关 topic：

```bash
ros2 topic list | grep -E "mavros|gripper|px4ctrl|vision"
```

检查 RC：

```bash
ros2 topic echo /mavros/rc/in
```

检查飞控连接：

```bash
ros2 topic echo /mavros/state
```

检查定位：

```bash
ros2 topic echo /mavros/vision_pose/pose
ros2 topic hz /mavros/vision_pose/pose
```

检查夹爪命令：

```bash
ros2 topic echo /gripper/command
```

检查专家动作：

```bash
ros2 topic echo /px4ctrl/expert_pose
ros2 topic hz /px4ctrl/expert_pose
```

手动打开夹爪：

```bash
ros2 topic pub --once /gripper/command std_msgs/msg/Float64 "{data: 100.0}"
```

手动关闭夹爪：

```bash
ros2 topic pub --once /gripper/command std_msgs/msg/Float64 "{data: 0.0}"
```

检查 px4ctrl 参数：

```bash
ros2 param list /px4ctrl
ros2 param get /px4ctrl gripper.rc_channel
ros2 param get /px4ctrl gripper.pwm_open
ros2 param get /px4ctrl gripper.pwm_close
```

## 15. 测试通过标准

夹爪单独测试通过：

- `100.0` 全开
- `0.0` 全关
- 退出节点没有异常 traceback

MAVROS 测试通过：

- `/mavros/state` 中 `connected: true`
- `/mavros/rc/in` 有 RC 通道数据
- 第 10 通道拨动时 `channels[9]` 明显变化

RC 夹爪测试通过：

- `POSCTL/OFFBOARD` 下 CH10 低位发布 `100.0`
- `POSCTL/OFFBOARD` 下 CH10 高位发布 `0.0`
- `ALTCTL/STABILIZED/MANUAL/AUTO_LAND/未解锁/z <= 0.20 m` 下发布或保持 `100.0`
- 真实夹爪方向正确：`100.0` 全开，`0.0` 全关

定位测试通过：

- `/mavros/vision_pose/pose` 有稳定数据
- 位置单位为米
- yaw / orientation 随机体转动变化

专家动作测试通过：

- `/px4ctrl/expert_pose` 持续发布。
- 手动拨杆时，`/px4ctrl/expert_pose.pose.position` 按目标位置变化。
- `/px4ctrl/expert_pose` 频率稳定，采集时不会超过 `max_pose_age_s`。

整机测试通过：

- 起飞脚本能触发 takeoff
- 降落脚本能触发 land
- `fly_x_gripper_test.py` 能读取 mocap pose
- 飞行过程中 yaw 和夹爪动作符合预期
- 降落前夹爪保持全开
- 降落过程中和落地后夹爪保持全开
