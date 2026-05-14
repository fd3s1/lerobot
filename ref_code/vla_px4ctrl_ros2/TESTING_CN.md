# VLADrone ROS2 + MAVROS + Feetech 夹爪测试流程

本文档用于在 Orin NX 上按步骤测试整条链路：

```text
遥控器 / Nokov / 测试程序
  -> MAVROS / px4ctrl
  -> /drone6/gripper/command
  -> feetech_gripper_node.py
  -> 两个 Feetech STS3215 夹爪舵机
```

当前约定：

- 飞控命名空间：`/drone6`
- MAVROS RC 输入：`/drone6/mavros/rc/in`
- MAVROS vision pose：`/drone6/mavros/vision_pose/pose`
- px4ctrl 位置指令：`/drone6/position_cmd`
- px4ctrl 起降指令：`/drone6/px4ctrl/takeoff_land`
- 夹爪命令：`/drone6/gripper/command`
- 夹爪定义：`100 = 全开`，`0 = 全关`
- 遥控器第 10 通道：低位打开，高位关闭

安全前提：

- 第一次测试必须拆桨。
- 夹爪悬空，避免触地、夹到线束或机体。
- 第一次真实舵机测试时，不要启动飞行测试程序。
- 每一步只验证一条链路，确认无误后再进入下一步。

## 1. 同步代码并构建 ROS2 workspace

服务器上传代码：
cd /home/user/vla_drone/lerobot
scripts/vla_dev_push.sh "Add Chinese VLADrone test procedure"


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
ls -l /dev/ttyACM0
```

如果没有 `/dev/ttyACM0`，查看可用串口：

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

ros2 topic pub --once /drone6/gripper/command std_msgs/msg/Float64 "{data: 100.0}"
ros2 topic pub --once /drone6/gripper/command std_msgs/msg/Float64 "{data: 0.0}"
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
ros2 run px4ctrl feetech_gripper_node.py --ros-args -p port:=/dev/ttyACM0
```

启动后会读取两个舵机的 Windows 标定范围，期望看到类似：

```text
gripper_left: id=1, min=..., max=..., homing_offset=..., direction=normal
gripper_right: id=2, min=..., max=..., homing_offset=..., direction=normal
Listening for gripper commands on /drone6/gripper/command
```

终端 B：

```bash
source /opt/ros/humble/setup.bash
source ~/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2/install/setup.bash

ros2 topic pub --once /drone6/gripper/command std_msgs/msg/Float64 "{data: 100.0}"
ros2 topic pub --once /drone6/gripper/command std_msgs/msg/Float64 "{data: 0.0}"
```

期望结果：

- `100.0`：夹爪全开
- `0.0`：夹爪全关

如果方向反了，不要改 Windows 标定，先用软件反向：

```bash
ros2 run px4ctrl feetech_gripper_node.py --ros-args \
  -p port:=/dev/ttyACM0 \
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
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:57600 namespace:=drone6
```

TELEM 串口示例：

```bash
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600 namespace:=drone6
```

UDP 示例：

```bash
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557 namespace:=drone6
```

如果你的 MAVROS launch 文件不支持 `namespace:=drone6`，先按 `--show-args` 里显示的参数名启动。最终必须确认 topic 是 `/drone6/mavros/...`。如果实际 topic 是 `/mavros/...`，要么调整 MAVROS namespace，要么修改 `ctrl_param_fpv.yaml` 中的 topic。

检查 MAVROS 是否连接：

```bash
ros2 topic echo /drone6/mavros/state
```

期望看到：

```text
connected: true
```

检查 RC 输入：

```bash
ros2 topic echo /drone6/mavros/rc/in
```

拨动遥控器第 10 通道，观察 `channels` 数组第 10 个值，也就是 `channels[9]`。

期望：

- 低位约 `1000~1300`
- 高位约 `1700~2000`

## 6. 检查 Nokov / mocap vision pose

作用：确认 PX4/MAVROS 能收到外部定位。px4ctrl 默认从 `/drone6/mavros/vision_pose/pose` 读取当前位置。

检查 vision pose：

```bash
ros2 topic echo /drone6/mavros/vision_pose/pose
```

期望：

- `position.x/y/z` 随无人机移动变化
- `orientation` 不全是 0
- 数据频率稳定

查看频率：

```bash
ros2 topic hz /drone6/mavros/vision_pose/pose
```

如果这里没有数据，说明 Nokov 到 MAVROS 的 vision pose 桥接还没有启动或 topic 名不一致。先修这一步，不要继续飞行测试。

## 7. 单独测试 RC 第 10 通道到夹爪命令 topic

作用：暂时不接舵机，只确认遥控器 CH10 会被 px4ctrl 转成 `/drone6/gripper/command`。

终端 A：启动 MAVROS，并确认 `/drone6/mavros/rc/in` 有数据。

终端 B：监听夹爪命令：

```bash
source /opt/ros/humble/setup.bash
source ~/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2/install/setup.bash
ros2 topic echo /drone6/gripper/command
```

终端 C：启动 px4ctrl，不启动真实夹爪节点：

```bash
cd ~/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run px4ctrl px4ctrl_node --ros-args --params-file install/px4ctrl/share/px4ctrl/config/ctrl_param_fpv.yaml
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
```

期望：

- CH10 低位：`/drone6/gripper/command` 输出 `100.0`
- CH10 高位：`/drone6/gripper/command` 输出 `0.0`
- CH10 中间：不发布新命令

如果没有输出，检查：

```bash
ros2 topic echo /drone6/mavros/rc/in
```

确认 `channels[9]` 是否真的变化。

## 8. 测试 RC 第 10 通道真实控制夹爪

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
ros2 run px4ctrl feetech_gripper_node.py --ros-args -p port:=/dev/ttyACM0
```

拨动遥控器第 10 通道。

期望：

- CH10 低位：夹爪全开
- CH10 高位：夹爪全关

如果方向反了，停止夹爪节点，用反向参数重启：

```bash
ros2 run px4ctrl feetech_gripper_node.py --ros-args \
  -p port:=/dev/ttyACM0 \
  -p left_inverted:=true \
  -p right_inverted:=true
```

## 9. 用 launch 同时启动 px4ctrl 和夹爪节点

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

## 10. 起飞和降落脚本测试

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
ros2 topic pub --once /drone6/px4ctrl/takeoff_land quadrotor_msgs/msg/TakeoffLand "{takeoff_land_cmd: 1}"
ros2 topic pub --once /drone6/px4ctrl/takeoff_land quadrotor_msgs/msg/TakeoffLand "{takeoff_land_cmd: 2}"
```

含义：

- `1`：takeoff
- `2`：land

## 11. 最后测试 fly_x_gripper_test.py

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

## 12. 常用排错命令

查看所有相关 topic：

```bash
ros2 topic list | grep -E "drone6|mavros|gripper|px4ctrl|vision"
```

检查 RC：

```bash
ros2 topic echo /drone6/mavros/rc/in
```

检查飞控连接：

```bash
ros2 topic echo /drone6/mavros/state
```

检查定位：

```bash
ros2 topic echo /drone6/mavros/vision_pose/pose
ros2 topic hz /drone6/mavros/vision_pose/pose
```

检查夹爪命令：

```bash
ros2 topic echo /drone6/gripper/command
```

手动打开夹爪：

```bash
ros2 topic pub --once /drone6/gripper/command std_msgs/msg/Float64 "{data: 100.0}"
```

手动关闭夹爪：

```bash
ros2 topic pub --once /drone6/gripper/command std_msgs/msg/Float64 "{data: 0.0}"
```

检查 px4ctrl 参数：

```bash
ros2 param list /px4ctrl
ros2 param get /px4ctrl gripper.rc_channel
ros2 param get /px4ctrl gripper.pwm_open
ros2 param get /px4ctrl gripper.pwm_close
```

## 13. 测试通过标准

夹爪单独测试通过：

- `100.0` 全开
- `0.0` 全关
- 退出节点没有异常 traceback

MAVROS 测试通过：

- `/drone6/mavros/state` 中 `connected: true`
- `/drone6/mavros/rc/in` 有 RC 通道数据
- 第 10 通道拨动时 `channels[9]` 明显变化

RC 夹爪测试通过：

- CH10 低位发布 `100.0`
- CH10 高位发布 `0.0`
- 真实夹爪跟随开合

定位测试通过：

- `/drone6/mavros/vision_pose/pose` 有稳定数据
- 位置单位为米
- yaw / orientation 随机体转动变化

整机测试通过：

- 起飞脚本能触发 takeoff
- 降落脚本能触发 land
- `fly_x_gripper_test.py` 能读取 mocap pose
- 飞行过程中 yaw 和夹爪动作符合预期
- 降落前夹爪保持全开
