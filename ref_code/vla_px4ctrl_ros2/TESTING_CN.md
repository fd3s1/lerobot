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
- 夹爪比起落架低约 `15 cm`，当前配置会在 mocap 高度 `z <= 0.15 m` 时强制全开，避免接近地面时夹爪触地。

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

`feetech_gripper_node.py` 默认保持原始方向。如果单独运行 ROS 夹爪节点时方向反了，不要改 Windows 标定，先用软件反向：

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

期望频率接近 `ctrl_freq_max`，默认约 `100 Hz`。

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
  force_open_below_z: 0.15
```

期望：

- 在 `POSCTL` 或 `OFFBOARD` 下，CH10 低位：`/gripper/command` 输出 `100.0`
- 在 `POSCTL` 或 `OFFBOARD` 下，CH10 高位：`/gripper/command` 输出 `0.0`
- 在 `POSCTL` 或 `OFFBOARD` 下，CH10 中间：不发布新命令
- 在 `ALTCTL`、`STABILIZED`、`MANUAL`、降落、未解锁、未知模式或 `z <= 0.15 m` 时，无论 CH10 位置如何，`/gripper/command` 都应发布或保持 `100.0`

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

确认 PX4 处于 `POSCTL` 或 `OFFBOARD` 后，拨动遥控器第 10 通道。

期望：

- `POSCTL` 或 `OFFBOARD` 下，CH10 低位：夹爪全开
- `POSCTL` 或 `OFFBOARD` 下，CH10 高位：夹爪全关
- 切到 `ALTCTL`、`STABILIZED`、`MANUAL`、触发降落或高度低于 `0.15 m` 后，无论 CH10 位置如何，夹爪自动全开

如果单独运行 ROS 夹爪节点时方向反了，停止夹爪节点，用反向参数重启：

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
- 夹爪仍然用遥控器 CH10 控制，但只有 `POSCTL` 或 `OFFBOARD` 下 CH10 生效。其它模式、降落、未解锁或 `z <= 0.15 m` 时 px4ctrl 会强制 `/gripper/command=100.0`。LeRobot 会记录 `/gripper/command`，并把该 action 发送给 `vla_drone` robot 执行。
- 当前相机约定：`/dev/video0` 是前视相机，`/dev/video2` 是夹爪/下视相机。

### 13.1 启动飞行和定位链路

终端 A：启动 VRPN、MAVROS、vision bridge 和 px4ctrl。

```bash
cd ~/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2
bash shflies/run_mocap_mavros.sh
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
- `ALTCTL`、`STABILIZED`、`MANUAL`、降落、未解锁或 `z <= 0.15 m` 时，`/gripper/command` 为 `100.0`。

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
- reset 时长：`10 s`
- 图像保存：开启，两路相机 `/dev/video0` 和 `/dev/video2`
- 上传 Hugging Face Hub：关闭

常用覆盖示例：

```bash
cd ~/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2

# 录 3 条，每条 30 秒
NUM_EPISODES=3 EPISODE_TIME_S=30 bash shflies/record_vla_dataset.sh

# 只做 10 秒调试，并关闭视频
DATASET_PREFIX=debug_vla_drone EPISODE_TIME_S=10 RESET_TIME_S=1 DATASET_VIDEO=false \
  bash shflies/record_vla_dataset.sh

# 临时修改任务描述
TASK="Fly to the target cube and close the gripper" bash shflies/record_vla_dataset.sh
```

### 13.3 录制脚本参数解释

自动命名相关参数：

- `RUN_ID`：默认使用当前时间，例如 `20260521_143012`。
- `DATASET_PREFIX`：数据集名前缀，默认 `vla_drone_grasp`。
- `DATASET_NAME`：完整数据集名，默认 `${DATASET_PREFIX}_${RUN_ID}`。
- `DATASET_ROOT`：本地保存路径，默认 `${HOME}/vla_drone/data/${DATASET_NAME}`。
- `REPO_OWNER`：repo_id 的用户名前缀，默认 `fd3s1`。
- `REPO_ID`：数据集 ID，默认 `${REPO_OWNER}/${DATASET_NAME}`。即使 `PUSH_TO_HUB=false`，LeRobot 仍需要一个 repo_id 作为数据集标识。
- `CONDA_ENV`：要激活的 conda 环境，默认 `vla-drone-v044`。
- `CONDA_SH`：conda 初始化脚本，默认 `${HOME}/miniforge3/etc/profile.d/conda.sh`。
- 脚本内部会执行 `source /opt/ros/humble/setup.bash`，让 conda Python 能 import ROS2 的 `rclpy`。
- 脚本内部会执行 `source install/setup.bash`，让当前 ROS2 workspace 的消息和节点环境生效。

Robot 参数：

- `NOKOV_POSE_TOPIC`：默认 `/mavros/vision_pose/pose`。从 MAVROS vision pose 读取当前 mocap 位姿，作为 observation state 的 `x, y, z, yaw` 来源。
- `ROBOT_MAX_POSE_AGE_S`：默认 `2.0`。允许 LeRobot 在采集过程中短暂等待最新 mocap pose。NX 同时读两路相机和写数据时偶发调度延迟，`0.5 s` 容易误判为 pose 超时。
- `MAVROS_SETPOINT_TOPIC`：默认 `/position_cmd`。推理阶段向 px4ctrl 发送目标位置的 topic。手动采集时保留该配置，但脚本固定使用 `--robot.send_pose_actions=false`，不会发送 pose action。
- `GRIPPER_PORT`：默认 `/dev/ttyACM1`。Feetech 舵机总线串口。
- `FRONT_CAMERA`：默认 `/dev/video0`，前视相机。
- `DOWN_CAMERA`：默认 `/dev/video2`，夹爪/下视相机。
- `CAMERA_WIDTH`、`CAMERA_HEIGHT`、`CAMERA_FPS`：默认 `640`、`480`、`30`。

Teleop 参数：

- `EXPERT_POSE_TOPIC`：默认 `/px4ctrl/expert_pose`。读取 px4ctrl 发布的专家目标位姿，保存为 action 的 `x, y, z, yaw`。
- `GRIPPER_TOPIC`：默认 `/gripper/command`。读取 CH10 产生的夹爪命令，保存为 action 的 `gripper_left.pos, gripper_right.pos`。
- `TELEOP_STARTUP_TIMEOUT_S`：默认 `2.0`。录制刚开始时等待第一帧 `/px4ctrl/expert_pose` 的最长时间。
- `TELEOP_MAX_POSE_AGE_S`：默认 `0.2`。允许专家 pose 的最大年龄。如果 `/px4ctrl/expert_pose` 超过该时间没更新，录制会报错，避免保存动作和图像严重错位的数据。

Dataset 参数：

- `DATASET_FPS`：默认 `30`。LeRobot 保存数据的目标频率。这里和两路相机 `30 fps` 对齐。
- `NUM_EPISODES`：默认 `1`。本次连续采集的 episode 数量。
- `EPISODE_TIME_S`：默认 `30`。每条 episode 最长 30 秒。
- `RESET_TIME_S`：默认 `10`。两条 episode 之间留 10 秒复位时间。
- `TASK`：默认 `Fly to the target and operate the gripper`。本批数据的任务描述。
- `PUSH_TO_HUB`：默认 `false`。采集后只保存到本地，不自动上传 Hugging Face Hub。
- `DATASET_VIDEO`：默认 `true`。正式训练 SmolVLA 时必须保留图像；调试时可以临时设置为 `false`。
- `PLAY_SOUNDS`：默认 `false`。关闭录制提示音，避免 NX 环境缺少音频设备时报错。

### 13.4 时间戳对齐规则

- `/px4ctrl/expert_pose.header.stamp` 来自 px4ctrl 控制周期，和同周期控制目标一致。
- `ros_expert_pose` teleop 每帧读取最新 `/px4ctrl/expert_pose`，如果 age 超过 `--teleop.max_pose_age_s` 就停止记录并报错。
- `vla_drone` robot 在一次 `get_observation()` 中读取相机、夹爪和 mocap pose，并把它们保存成同一帧 observation。
- `/gripper/command` 是保持型目标命令，不是连续流；teleop 会记录最后一次夹爪目标。没有收到夹爪命令时默认记录 `100.0`，即全开。

### 13.5 每次采集前检查

```bash
ros2 topic hz /mavros/vision_pose/pose
ros2 topic hz /px4ctrl/expert_pose
ros2 topic echo /mavros/state --once
ros2 topic echo /gripper/command
```

同时确认：

- QGC 没有 `yaw_estimate_error`。
- Position 模式悬停稳定。
- `POSCTL/OFFBOARD` 下 CH10 能实际控制夹爪。
- `ALTCTL/STABILIZED/MANUAL/AUTO_LAND/未解锁/z <= 0.15 m` 下夹爪自动全开。
- `/dev/video0` 和 `/dev/video2` 都能被 LeRobot 找到。
- 没有单独运行 `feetech_gripper_node.py`。

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
- `ALTCTL/STABILIZED/MANUAL/AUTO_LAND/未解锁/z <= 0.15 m` 下发布或保持 `100.0`
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
