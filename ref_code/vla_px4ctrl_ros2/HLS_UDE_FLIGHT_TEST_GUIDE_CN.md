# HLS-UDE 自动飞行抓取测试说明

本文档说明 `shflies/auto_hls_ude_grasp_place_test.sh` 的完整飞行测试流程、推荐命令、运行阶段、安全保护，以及每个常用参数的含义。该脚本用于一终端启动 mocap/VRPN、MAVROS、vision bridge、`px4ctrl_node`、HLS 力控夹爪和自动抓取放置流程。

相关入口：

- `shflies/auto_hls_ude_grasp_place_test.sh`：一键飞行测试入口。
- `shflies/auto_hls_grasp_place.sh`：自动 HLS 抓取放置入口，负责启动 HLS 节点并运行自动抓放节点。
- `src/px4ctrl/scripts/auto_hls_grasp_place.py`：自动抓放节点，负责起飞、到点、HLS close、body-Y 居中、起吊、放置和命令降落。
- `src/hls_gripper/scripts/hls_gripper_node.py`：HLS 力控夹爪状态机。

## 重要安全原则

本测试会实际控制飞机和夹爪。每次上桨飞行前，应先完成无桨检查。

- 起飞、飞向目标、下降和实际到位 settle 前，夹爪会持续保持 open。
- 只有 `Pre-grasp actual settle` 完成后，脚本才允许 HLS close。
- CH10 低位、离开 `CMD_CTRL`、HLS 状态失效或 HLS fault 都会停止 close 并发布 open。`/mavros/rc/in` 超时默认只告警，避免把 ROS 话题回调间隔误判为遥控器丢失。
- 释放到 box 后，撤离和命令降落阶段会持续保持 open。
- 默认降落是 `CMD_CTRL` 位置命令下降到 `CMD_LAND_Z=-0.3`，不是 PX4 autoland；结束后仍需人工确认安全和必要时 disarm。

## 推荐完整测试流程

### 1. 物理准备

1. 确认飞机、HLS 夹爪、串口 `/dev/ttyACM1`、飞控串口 `/dev/ttyACM0` 接线正确。
2. 确认 mocap 中有三个刚体：
   - 无人机：默认 `/vla_drone1/pose`
   - 抓取目标：默认 `/strawberry_bear/pose`
   - 放置 box：默认 `/box1/pose`
3. 确认 QGC 或遥控器能切到 px4ctrl 需要的飞行模式。
4. 确认 CH10 低位是安全释放，开始自动任务前 CH10 不能在低位。
5. 首次测试必须无桨运行；确认逻辑正确后再上桨低风险飞行。

### 2. 编译和环境

```bash
cd /home/user/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

如果只改了脚本但没有改 C++ 包，通常不需要重新编译；如果 `install/setup.bash` 不存在，必须先 `colcon build`。

### 3. 无桨 topic 检查

单独检查 mocap、MAVROS、RC 和 px4ctrl 状态：

```bash
cd /home/user/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo --once --qos-reliability best_effort /vla_drone1/pose
ros2 topic echo --once --qos-reliability best_effort /strawberry_bear/pose
ros2 topic echo --once --qos-reliability best_effort /box1/pose
ros2 topic echo --once --qos-reliability best_effort /mavros/rc/in
ros2 topic echo --once /px4ctrl/state
ros2 topic echo --once /mavros/state
```

一键脚本也会做这些检查。如果 `POSE_PREFLIGHT_REQUIRED=true`，任何关键 topic 检查失败都会退出。

### 4. 单独 UDE 手动起飞检查

如果只想验证和以前一致的 UDE 起飞/悬停控制链，使用：

```bash
cd /home/user/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2

bash shflies/test_ude_takeoff_hover.sh
```

这个脚本默认不会发布 `/px4ctrl/takeoff_land`，也不会要求按 Enter 或输入 `TAKEOFF` 来触发起飞。你仍按以前的流程手动起飞；脚本只等待 px4ctrl 进入 `AUTO_HOVER`，然后进入状态查看/降落菜单。

只有在明确需要恢复“helper 代发 TAKEOFF”的旧流程时，才使用：

```bash
TEST_PUBLISH_TAKEOFF=true \
TEST_AUTO_CONFIRM=true \
bash shflies/test_ude_takeoff_hover.sh
```

其中 `TEST_PUBLISH_TAKEOFF=true` 表示由 helper 发布 `TakeoffLand.TAKEOFF`；`TEST_AUTO_CONFIRM=true` 只在这个模式下生效，表示不再输入 `TAKEOFF` 确认。

### 5. 无桨自动流程检查

默认会在自动节点内部等待确认：脚本先完成 topic 检查、目标/box/无人机 pose 稳定检查并打印锁定快照，然后暂停 ROS 回调刷新；这时按一次 Enter 会立即发布 `TAKEOFF`。px4ctrl 收到 `TAKEOFF` 后先执行水平姿态的电机加速推力渐增，避免未离地时产生水平倾角；加速结束后进入 UDE 竖直爬升。

```bash
cd /home/user/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2

TARGET_OFFSET_X=0.00 \
TARGET_OFFSET_Y=0.00 \
TARGET_OFFSET_Z=0.00 \
BOX_OFFSET_X=0.00 \
BOX_OFFSET_Y=0.00 \
BOX_OFFSET_Z=0.00 \
bash shflies/auto_hls_ude_grasp_place_test.sh
```

检查终端输出中这些项目：

- `drone pose is live`
- `target pose is live`
- `box pose is live`
- `RC input is live`
- `px4ctrl state is live`
- `MAVROS state is live`
- `CH10 safety: ... open<=1300`
- `takeoff confirmation: inner_confirm=true outer_wait=false`
- `Pre-takeoff pose snapshot is locked`

在按 Enter 前，确认打印出的 drone/target/box 坐标合理、CH10 不在低位、遥控器姿态安全，必要时保持随时切 CH10 低位释放。按下 Enter 后不再继续刷新起飞前快照，会直接发 `TAKEOFF`。

### 6. CH10 安全释放检查

无桨时做两类检查：

1. CH10 低位启动：脚本应拒绝自动起飞或自动抓取。
2. 自动抓取中切 CH10 低位：HLS 应立即 open，自动任务中止，不再继续 close。

观察话题：

```bash
ros2 topic echo /hls_gripper/state
ros2 topic echo /hls_gripper/safe_to_lift
ros2 topic echo /hls_gripper/fault
ros2 topic echo /hls_gripper/centering_offset_m
ros2 topic echo /hls_gripper/single_contact_need_motion
```

### 7. 上桨低风险飞行测试

推荐顺序：

1. 空目标或非常轻目标测试：确认起飞、到目标上方、下降、命令降落流程。
2. 固定目标测试：确认到位后才夹，夹爪进入 `SEARCH_OBJECT -> LEFT/RIGHT_CONTACT -> BOTH_CONTACT/CENTERING -> FINAL_GRIP -> LIFT_READY`。
3. 偏左/偏右目标测试：确认单侧接触时飞机只在 `single_contact_need_motion=true` 后低速 body-Y 让位。
4. 放置测试：确认飞到 box 上方、下降、open 释放、撤离、命令下降到 `z=-0.3`。

## 推荐一键命令

最小推荐命令：

```bash
cd /home/user/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2

TARGET_OFFSET_X=0.00 \
TARGET_OFFSET_Y=0.00 \
TARGET_OFFSET_Z=0.00 \
BOX_OFFSET_X=0.00 \
BOX_OFFSET_Y=0.00 \
BOX_OFFSET_Z=0.00 \
bash shflies/auto_hls_ude_grasp_place_test.sh
```

带高度偏置示例：

```bash
cd /home/user/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2

TARGET_OFFSET_X=0.00 \
TARGET_OFFSET_Y=0.00 \
TARGET_OFFSET_Z=0.00 \
TARGET_GRASP_Z_OFFSET=-0.02 \
TARGET_HOVER_Z_OFFSET=0.45 \
BOX_OFFSET_X=0.00 \
BOX_OFFSET_Y=0.00 \
BOX_OFFSET_Z=0.00 \
BOX_PLACE_Z_OFFSET=0.20 \
BOX_HOVER_Z_OFFSET=0.70 \
bash shflies/auto_hls_ude_grasp_place_test.sh
```

如果 mocap 刚体名字不同：

```bash
VRPN_SOURCE_TOPIC=/my_drone/pose \
TARGET_POSE_TOPIC=/my_target/pose \
BOX_POSE_TOPIC=/my_box/pose \
bash shflies/auto_hls_ude_grasp_place_test.sh
```

## 自动流程阶段

| 阶段 | 飞机动作 | 夹爪动作 | 保护逻辑 |
| --- | --- | --- | --- |
| 启动检查 | 启动 stack，检查 pose/RC/state | 不 close | topic 缺失可拒绝启动 |
| 起飞前确认 | 锁定起飞前 pose 快照，暂停回调等待一次 Enter | 保持 open | Enter 后重新检查 CH10，低位不会起飞 |
| 自动起飞 | 发布 `TakeoffLand.TAKEOFF`，加速期水平姿态，随后 UDE 竖直爬升 | 发布 open | 等待 `AUTO_HOVER` |
| 进入 `CMD_CTRL` | 发布当前位置 `/position_cmd` | 保持 open | 进入失败则退出 |
| 飞到目标上方 | 跟随目标 live waypoint | 保持 open | 未到位不夹 |
| 下降到抓取点 | 到目标抓取高度 | 保持 open | 到位误差和 settle 检查 |
| HLS 抓取 | body-Y 小幅辅助居中 | 持续 close/追夹 | CH10、CMD_CTRL、HLS 状态持续检查；RC topic stale 默认只告警 |
| `LIFT_READY` | 提起到目标上方 | 保持 close/追夹 | HLS safe 后才起吊 |
| 转运到 box | 飞向 box 上方 | 保持 close/追夹 | 避免中途松开 |
| 下降到 box | 到放置高度 | 保持 close/追夹 | 到位后才释放 |
| 释放和撤离 | open、上撤、前撤 | 持续 open | 释放后不再 close |
| 命令降落 | 下降到 `CMD_LAND_Z` | 持续 open | 不发送 autoland |

## 参数总览

### 一键启动和进程管理参数

| 参数 | 默认值 | 作用 | 什么时候修改 |
| --- | --- | --- | --- |
| `START_STACK` | `true` | 是否由一键脚本启动 mocap、MAVROS、vision bridge、px4ctrl。 | 已手动启动这些节点时设为 `false`。 |
| `START_PX4CTRL` | `true` | 传给 `run_mocap_mavros.sh`，决定是否启动 `px4ctrl_node`。 | 只想用已有 px4ctrl 时设为 `false`。 |
| `STACK_STARTUP_WAIT_S` | `8` | 启动底层 stack 后等待的秒数。 | 电脑慢或 MAVROS 启动慢时增大。 |
| `WAIT_FOR_ENTER` | `false` | 一键 shell 外层是否额外等待 Enter。 | 默认 `false`，避免两次 Enter；通常不改。 |
| `CONFIRM_BEFORE_TAKEOFF` | `true` | 自动节点完成 pose 锁定后是否等待一次 Enter 再发 `TAKEOFF`。 | 真机建议保持 `true`；自动化回归可设 `false`。 |
| `KEEP_STACK_ON_INTERRUPT` | `false` | Ctrl+C 后是否保留 stack。 | 空中调试时可临时设 `true`，避免误杀控制链。 |
| `CLEANUP_STACK_ON_EXIT` | `true` | 脚本退出时是否关闭由它启动的 stack。 | 想保留 MAVROS/px4ctrl 继续观察时设 `false`。 |

### mocap、MAVROS 和 px4ctrl 参数

| 参数 | 默认值 | 作用 | 说明 |
| --- | --- | --- | --- |
| `VRPN_SERVER` | `10.1.1.198` | VRPN 服务器 IP。 | 在 `run_mocap_mavros.sh` 中使用。 |
| `VRPN_PORT` | `3883` | VRPN 端口。 | 通常不改。 |
| `VRPN_SOURCE_TOPIC` | `/vla_drone1/pose` | 无人机 VRPN pose 话题。 | 对应 mocap 中无人机刚体名。 |
| `MAVROS_VISION_TOPIC` | `/mavros/vision_pose/pose` | vision bridge 输出给 MAVROS 的位姿话题。 | 给 MAVROS/EKF 输入动捕位姿。 |
| `DRONE_POSE_TOPIC` | `/mavros/local_position/odom` | 自动抓放节点读取的无人机 odom。 | 必须使用 MAVROS odom，和 `test_ude_takeoff_hover.sh` 的稳定控制坐标源保持一致；目标和 box 仍用 mocap 刚体话题。 |
| `TARGET_POSE_TOPIC` | `/strawberry_bear/pose` | 目标物 mocap pose。 | 如果目标刚体名不同，需要修改。 |
| `BOX_POSE_TOPIC` | `/box1/pose` | 放置 box mocap pose。 | 如果 box 刚体名不同，需要修改。 |
| `FCU_URL` | `/dev/ttyACM0:921600` | MAVROS 到飞控的串口。 | 飞控端口变化时修改。 |
| `GCS_URL` | `udp://@10.1.1.198:14550` | MAVROS 到地面站/QGC 的 UDP。 | QGC 主机变化时修改。 |
| `MAVROS_LIGHT` | `true` | 使用轻量 MAVROS pluginlist。 | 需要完整 MAVROS 插件时设 `false`。 |
| `BRIDGE_RESTAMP` | `false` | vision bridge 是否重写时间戳。 | 一般保持默认。 |
| `PX4CTRL_PARAMS_FILE` | `install/px4ctrl/share/px4ctrl/config/ctrl_param_fpv.yaml` | px4ctrl 参数文件。 | 测不同参数文件时修改。 |

### preflight 检查参数

| 参数 | 默认值 | 作用 | 说明 |
| --- | --- | --- | --- |
| `POSE_PREFLIGHT_TIMEOUT_S` | `6` | 每个 topic echo 的等待时间。 | topic 首帧慢时增大。 |
| `POSE_PREFLIGHT_REQUIRED` | `true` | preflight 失败是否直接退出。 | 真机建议保持 `true`。 |
| `SKIP_POSE_PREFLIGHT` | `false` | 是否完全跳过一键脚本的 topic 检查。 | 只在明确知道 topic 正常时使用。 |
| `PX4CTRL_STATE_TOPIC` | `/px4ctrl/state` | px4ctrl 状态话题。 | 自动流程需要看到 `AUTO_HOVER` 和 `CMD_CTRL`。 |
| `MAVROS_STATE_TOPIC` | `/mavros/state` | MAVROS 状态话题。 | 用于启动前确认 MAVROS 在线。 |

### CH10 和任务安全参数

| 参数 | 默认值 | 作用 | 调参建议 |
| --- | --- | --- | --- |
| `RC_TOPIC` | `/mavros/rc/in` | RC 输入话题。 | 通常不改。 |
| `RC_TIMEOUT_S` | `0.5` | 自动节点判定 `/mavros/rc/in` 是否新鲜的阈值。 | 起飞前必须 fresh；飞行中默认 stale 只告警。 |
| `RC_STALE_ACTION` | `warn` | 飞行中 RC topic 超时后的动作。 | `warn` 表示继续任务，只在日志警告；`abort` 表示 open 并中止。注意这不是飞控真实遥控器 failsafe。 |
| `CH10_INDEX` | `9` | CH10 在 MAVROS `channels[]` 中的 0-based 索引。 | CH10 是第 10 通道，所以默认 9。 |
| `CH10_OPEN_PWM` | `1300` | CH10 小于等于该 PWM 时认为低位/安全释放。 | 按遥控器实际 PWM 调整。 |
| `CH10_CLOSE_PWM` | `1700` | CH10 高于等于该 PWM 时认为未触发释放。 | 自动流程不由高位直接 close，只作为安全条件。 |
| `FORCE_OPEN_BELOW_Z` | `0.20` | 抓取/携带阶段，如果无人机 z 低于该高度，强制 open 并 abort。 | 防止贴地或降落阶段误 close。必须在飞行 z 限制内。 |

### mocap 坐标偏置参数

这些偏置是在 mocap/map 坐标系下直接加到目标或 box 刚体位置上的，单位米。

| 参数 | 默认值 | 作用 | 示例 |
| --- | --- | --- | --- |
| `TARGET_OFFSET_X` | `0.0` | 目标 x 偏置。 | mocap 刚体中心不等于夹取中心时使用。 |
| `TARGET_OFFSET_Y` | `0.0` | 目标 y 偏置。 | 目标夹取点偏左/右时使用。 |
| `TARGET_OFFSET_Z` | `0.0` | 目标 z 偏置。 | 目标刚体高度标定有误时使用。 |
| `BOX_OFFSET_X` | `0.0` | box x 偏置。 | 放置点不是 box 刚体中心时使用。 |
| `BOX_OFFSET_Y` | `0.0` | box y 偏置。 | 放置点偏左/右时使用。 |
| `BOX_OFFSET_Z` | `0.0` | box z 偏置。 | box 刚体 z 标定有误时使用。 |

### 高度覆盖参数

这些参数如果为空，则使用几何模型自动计算；如果设置，则直接使用 `object.z + offset` 作为无人机中心高度。

| 参数 | 默认值 | 作用 | 说明 |
| --- | --- | --- | --- |
| `TARGET_HOVER_Z_OFFSET` | 空 | 目标上方悬停高度覆盖。 | 设置后 `target_hover_drone_z = target.z + offset`。 |
| `TARGET_GRASP_Z_OFFSET` | 空 | 抓取高度覆盖。 | 设置后 `target_grasp_drone_z = target.z + offset`。 |
| `BOX_HOVER_Z_OFFSET` | 空 | box 上方悬停高度覆盖。 | 设置后 `box_hover_drone_z = box.z + offset`。 |
| `BOX_PLACE_Z_OFFSET` | 空 | box 放置高度覆盖。 | 设置后 `box_place_drone_z = box.z + offset`。 |

### 几何参数

| 参数 | 默认值 | 作用 | 说明 |
| --- | --- | --- | --- |
| `GRIPPER_X_OFFSET_M` | `0.0` | 夹爪接触中心相对无人机刚体中心的 body-X 偏移。 | 正值表示机体前方。 |
| `GRIPPER_Y_OFFSET_M` | `0.0` | 夹爪接触中心相对无人机刚体中心的 body-Y 偏移。 | 正值表示机体左方。 |
| `GRIPPER_Z_OFFSET_M` | `0.25` | 无人机刚体中心到夹爪接触中心的 z 偏移。 | 用于从夹爪目标高度反算无人机中心高度。 |
| `TARGET_HEIGHT_M` | `0.30` | 目标高度估计。 | 用于计算目标底部/顶部/抓取高度。 |
| `TARGET_GRASP_HEIGHT_M` | `0.17` | 从目标底部算起的抓取高度。 | 抓取点越高，该值越大。 |
| `TARGET_POSE_Z_REFERENCE` | `center` | 目标 mocap z 表示的位置。 | 可选 `base`、`center`、`top`、`grasp`。 |
| `TARGET_HOVER_CLEARANCE_M` | `0.45` | 抓取点上方悬停余量。 | 未设置 `TARGET_HOVER_Z_OFFSET` 时使用。 |
| `BOX_LENGTH_M` | `0.65` | box 长度。 | 用于检查目标是否能放进 box。 |
| `BOX_WIDTH_M` | `0.41` | box 宽度。 | 用于检查目标是否能放进 box。 |
| `BOX_HEIGHT_M` | `0.14` | box 高度。 | 用于自动计算放置高度。 |
| `BOX_HOVER_GRIPPER_CLEARANCE_M` | `0.55` | 夹爪相对 box 顶部/中心的悬停余量。 | 未设置 `BOX_HOVER_Z_OFFSET` 时使用。 |
| `BOX_PLACE_BOTTOM_CLEARANCE_M` | `0.03` | 放置时目标底部离 box 底部的余量。 | 控制释放高度。 |

### 飞行速度和轨迹参数

| Python 参数 | 默认值 | 作用 | 调参建议 |
| --- | --- | --- | --- |
| `--rate-hz` | `20.0` | 自动节点发布 `/position_cmd` 的频率。 | 通常不改。 |
| `--max-speed` | `0.6` | 飞到目标上方的最大速度。 | 首次飞行可降低。 |
| `--approach-speed` | `0.3` | 下降到抓取点、下降到 box 的速度。 | 目标附近建议保守。 |
| `--lift-speed` | `0.4` | 释放后向上撤离速度。 | 过快会带来摆动。 |
| `--payload-lift-speed` | `0.10` | 抓住物体后起吊速度。 | 携带阶段建议慢。 |
| `--payload-transfer-speed` | `0.16` | 携带物体飞到 box 的速度。 | 太快会摆动。 |
| `--post-grasp-settle-s` | `1.5` | HLS 报 safe 后、起吊前的等待。 | 让夹爪和目标进入稳态。 |
| `--post-lift-settle-s` | `1.5` | 起吊后转运前的等待。 | 抑制摆动。 |
| `--smooth-trajectory` / `--no-smooth-trajectory` | `true` | 是否使用平滑五次曲线。 | 真机建议保持开启。 |

脚本环境变量目前没有显式包装所有速度参数；如果要改这些基础自动节点参数，可以直接在 `auto_hls_grasp_place.sh` 中补充，或直接运行 Python 节点传入对应 `--xxx` 参数。常规测试优先只改一键脚本已暴露的偏置、HLS 力控和安全参数。

### 携带补偿参数

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `--payload-lift-forward-comp-m` | `0.0` | 起吊目标上方点沿 yaw 前向补偿。 |
| `--payload-lift-comp-x/y/z` | `0.0` | 起吊目标上方点在 mocap/map 坐标系补偿。 |

### 实际到位判定参数

| 参数 | 默认值 | 作用 | 调参建议 |
| --- | --- | --- | --- |
| `WAYPOINT_ARRIVAL_TOLERANCE_M` | `0.08` | 实际无人机位置到命令点的容许误差。 | 越小越严格，太小可能等不到；真机初测不建议低于 `0.08`。 |
| `WAYPOINT_ARRIVAL_SETTLE_S` | `0.4` | 误差进入容差后必须持续稳定的时间。 | 越大越稳但流程变慢。 |
| `WAYPOINT_ARRIVAL_TIMEOUT_S` | `15.0` | 等待实际到位的最长时间。 | 控制响应慢时可加大；超时会触发 open 并执行安全下降。 |
| `POSE_TIMEOUT_S` / `--pose-timeout-s` | `0.5` | pose 新鲜度阈值。 | mocap 丢帧时会触发 stale。 |
| `STABLE_DURATION_S` / `--stable-duration-s` | `0.5` | 起飞前等待目标/box/drone 姿态稳定时间。 | 目标抖动大时增大。 |
| `STABLE_POS_TOLERANCE_M` / `--stable-pos-tolerance-m` | `0.03` | 稳定判定的位置波动容许值。 | mocap 噪声大时适当放宽。 |

### 降落和退出参数

| 参数 | 默认值 | 作用 | 说明 |
| --- | --- | --- | --- |
| `LANDING_MODE` | `cmd` | 任务结束后的降落方式。 | `cmd` 为位置命令下降；`auto` 为发 LAND；`none` 不降落。 |
| `CMD_LAND_Z` | `-0.3` | `cmd` 降落的目标 z。 | 本测试默认到场地下限，不 autoland。 |
| `CMD_LAND_SPEED` | `0.25` | 命令下降速度。 | 太快有风险。 |
| `CMD_LAND_Z_OFFSET_M` | `0.0` | 如果不设置 `CMD_LAND_Z`，使用起飞前 z 加该偏移。 | 当前一键脚本显式设置 `CMD_LAND_Z=-0.3`。 |
| `NO_LAND` | `false` | 是否跳过降落。 | 特殊调试才设 `true`。 |
| `--release-retreat-up-m` | `0.25` | 释放后向上撤离距离。 | box 附近有障碍时增大。 |
| `--release-retreat-forward-m` | `0.25` | 释放后沿 yaw 前方撤离距离。 | 根据场地空间调整。 |
| `--retreat-speed` | `0.6` | 释放后前撤速度。 | 首次可降低。 |

## HLS 力控参数详解

以下参数最终会传给 `hls_gripper_node.py`，控制夹爪状态机、接触检测、力控保持和追夹。

### HLS 节点和硬件参数

| 参数 | 默认值 | 作用 | 说明 |
| --- | --- | --- | --- |
| `START_GRIPPER_MANAGER` | `true` | 是否由脚本启动 HLS 节点。 | 已有 HLS 节点运行时设 `false`。 |
| `GRIPPER_MANAGER_TYPE` | `hls` | 夹爪管理器类型。 | 本脚本要求 `hls`。 |
| `GRIPPER_MANAGER_PORT` | `/dev/ttyACM1` | HLS 舵机串口。 | 串口变化时修改。 |
| `HLS_SDK_ROOT` | 空 | FTServo Python SDK 路径。 | 默认候选路径找不到 SDK 时再指定。 |
| `HLS_DRY_RUN` | `false` | 干跑模式，不实际写舵机。 | 软件验证可设 `true`；真机必须 `false`。 |
| `HLS_GRAVITY_COMP_PATH` | `src/hls_gripper/config/gravity_compensation.json` | 重力补偿、限位、接触阈值和运动 profile 文件。 | 推荐保持当前标定文件。 |
| `HLS_LEFT_OPEN/CLEAR/CLOSE` | 空 | 左夹爪 open/clear/close 原始舵机位置覆盖。 | 通常由补偿 JSON 提供，不手动填。 |
| `HLS_RIGHT_OPEN/CLEAR/CLOSE` | 空 | 右夹爪 open/clear/close 原始舵机位置覆盖。 | 同上。 |
| `HLS_LEFT_CURRENT_INWARD_SIGN` | `1` | 左侧电流正方向是否表示向内夹。 | 当前实测为 `1`。方向错会导致接触/力控异常。 |
| `HLS_RIGHT_CURRENT_INWARD_SIGN` | `1` | 右侧电流正方向是否表示向内夹。 | 当前实测为 `1`。 |

### HLS 运动 profile 和搜索参数

| 参数 | 当前一键默认 | 作用 | 调参影响 |
| --- | --- | --- | --- |
| `HLS_MOTION_PROFILE` | `p3` | 从补偿 JSON 选择运动 profile。 | 决定默认搜索速度、加速度、力矩。 |
| `HLS_MOTION_PROFILE_INDEX` | 空 | 用 profile index 选择运动 profile。 | 与 `HLS_MOTION_PROFILE` 二选一，通常不用。 |
| `HLS_SEARCH_SPEED` | `10` | `SEARCH_OBJECT` 和未接触侧继续闭合的速度。 | 第一次下夹速度主要看它；太大冲击，太小初始夹持慢。 |
| `HLS_SEARCH_ACC` | `4` | 搜索闭合加速度。 | 控制第一次下夹柔和程度。 |
| `HLS_SEARCH_TORQUE_LIMIT` | `145` | 搜索闭合位置控制力矩上限。 | 太大搜索阶段冲击大；太小可能碰不到或闭合无力。 |

### 开爪参数

| 参数 | 默认值 | 作用 | 调参影响 |
| --- | --- | --- | --- |
| `HLS_OPEN_SPEED` | `24` | open 位置控制速度。 | 影响释放速度。 |
| `HLS_OPEN_ACC` | `6` | open 位置控制加速度。 | 太大开爪动作硬。 |
| `HLS_OPEN_TORQUE_LIMIT` | `120` | open 位置控制力矩上限。 | 只用于释放，不建议过大。 |
| `OPEN_COMMAND` | `100.0` | 自动节点发给 HLS 的 open 标量命令。 | 标量命令中 100 表示全开。 |
| `CLOSE_COMMAND` | `0.0` | 自动节点发给 HLS 的 close 标量命令。 | 标量命令中 0 表示进入抓取/闭合。 |

### 电流保持和最终夹持参数

| 参数 | 当前一键默认 | 作用 | 调参影响 |
| --- | --- | --- | --- |
| `HLS_LOW_CURRENT` | `28` | 基础低保持电流。 | 越大越稳但更压物体；越小更轻但容易松。 |
| `HLS_CENTER_HOLD_CURRENT` | `28` | 居中阶段非推力侧保持电流。 | 通常接近 `LOW_CURRENT`。 |
| `HLS_CENTER_PUSH_CURRENT` | `66` | 居中/推力侧电流。 | 影响夹爪自己修正偏差和被推开后的基础补偿力度。 |
| `HLS_GRIP_CHASE_MIN_CURRENT` | `66` | 追夹阶段电流地板。 | 影响保持夹持和被推开补偿。 |
| `HLS_LIFT_CURRENT` | `76` | `LIFT_READY` 阶段保持电流。 | 影响起吊可靠性；太低可能提起时滑落。 |
| ROS 参数 `final_grip_ramp_s` | 节点默认 `1.2` | 从低电流爬升到 lift 电流的时间。 | 一键脚本目前未显式暴露，必要时可在 wrapper 中补。 |

### 位置脉冲追夹参数

追夹由“电流保持 + 短时位置脉冲”组成。被推开或接触变弱时，节点会短时间向 close 位置写位置命令，同时保持电流。

| 参数 | 当前一键默认 | 作用 | 调参影响 |
| --- | --- | --- | --- |
| `HLS_GRIP_CHASE_POSITION_ENABLE` | `true` | 是否启用位置脉冲追夹。 | 保持夹持建议开启。 |
| `HLS_GRIP_CHASE_POSITION_SPEED` | `16` | 追夹位置脉冲速度。 | 被推开后补得慢可小幅增加；太大可能冲击。 |
| `HLS_GRIP_CHASE_POSITION_ACC` | `6` | 追夹位置脉冲加速度。 | 太大动作硬，太小补偿慢。 |
| `HLS_GRIP_CHASE_POSITION_TORQUE_LIMIT` | `145` | 追夹位置脉冲力矩上限。 | 影响被推开补偿最大力度。 |
| `HLS_GRIP_CHASE_SLIP_RATIO` | `0.03` | 被推开多少 close ratio 后触发追夹。 | 越小越敏感，太小可能频繁触发。 |
| `HLS_GRIP_CHASE_POSITION_PERIOD_S` | `0.05` | 追夹脉冲最小重复周期。 | 节点内部至少约 `0.08s`。 |
| `HLS_GRIP_CHASE_POSITION_PULSE_S` | `0.80` | 单次追夹脉冲持续时间。 | 太短追不上，太长容易一直顶。 |

### HLS 居中和单侧接触参数

| 参数 | 当前一键默认 | 作用 | 说明 |
| --- | --- | --- | --- |
| `HLS_CENTER_ERROR_GAIN` | `2.0` | HLS 内部左右闭合比例误差放大倍数。 | 影响 `/hls_gripper/centering_offset_m`。 |
| `HLS_CENTER_GAIN_M_PER_RATIO` | `0.0` | 手动覆盖 close ratio 到米的换算。 | `0` 表示按夹爪几何自动计算。 |
| `HLS_CENTER_BIAS` | `0.0` | 居中误差偏置。 | 用于补偿左右结构不对称。 |
| `HLS_CENTER_SIGN` | `1.0` | HLS 居中误差符号。 | 方向反了才改成 `-1.0`。 |
| `HLS_CENTER_TIMEOUT_ACTION` | `final_grip` | 居中超时后的动作。 | `final_grip` 表示继续最终夹持；`fault` 表示失败释放。 |
| `HLS_SINGLE_CONTACT_OFFSET_LIMIT_M` | `0.12` | HLS 对单侧接触偏移估计的最大输出。 | 防止估计值过大。 |
| `HLS_SINGLE_CONTACT_TIMEOUT_S` | `20.0` | 单侧接触最长等待时间。 | 超时会进入故障。 |
| `HLS_SINGLE_CONTACT_LIMIT_RATIO` | `0.97` | 单侧已接触指接近 close 极限时判定需要飞机让位。 | 越小越早请求 body-Y。 |

### HLS 接触检测和保护参数

| 参数 | 默认值 | 作用 | 说明 |
| --- | --- | --- | --- |
| `HLS_MAX_TEMP` | `85.0` | 舵机温度故障阈值。 | 达到后 fault。 |
| `HLS_TEMP_WARN_THRESHOLD` | `75.0` | 温度警告阈值。 | 到达后应暂停散热。 |
| ROS 参数 `contact_current_threshold` | 节点默认 `-1` | 接触进入电流阈值覆盖。 | `-1` 表示用补偿 JSON；一键脚本未显式暴露。 |
| ROS 参数 `contact_exit_threshold` | 节点默认 `-1` | 接触退出阈值覆盖。 | 通常不改。 |
| ROS 参数 `contact_strong_threshold` | 节点默认 `-1` | 强接触阈值覆盖。 | 通常不改。 |
| ROS 参数 `contact_confirm_cycles` | 节点默认 `3` | 接触确认所需连续周期。 | 越大越抗噪，越慢。 |
| ROS 参数 `max_current` | 节点默认 `600` | 最大电流保护。 | 通常不改。 |
| ROS 参数 `feedback_timeout_s` | 节点默认 `0.5` | 舵机反馈超时。 | 超时会保护。 |
| ROS 参数 `attitude_timeout_s` | 节点默认 `0.5` | IMU 姿态超时。 | 影响重力补偿。 |
| `HLS_DRY_RUN_CONTACT_PATTERN` | `both` | dry-run 中模拟接触模式。 | 只用于软件测试。 |

## 飞机与夹爪协同居中参数

这些参数属于自动飞行节点，不是夹爪节点。夹爪继续负责 close/追夹，飞机只做小幅 body-Y 让位或居中。

| 参数 | 默认值 | 作用 | 调参影响 |
| --- | --- | --- | --- |
| `HLS_GRASP_TIMEOUT_S` | `12.0` | HLS 抓取从 close 到 `safe_to_lift` 的最长时间。 | 目标难抓时可增大。 |
| `HLS_STATUS_TIMEOUT_S` | `0.8` | HLS 状态话题新鲜度阈值。 | 太小可能误报 stale，太大安全响应变慢。 |
| `CENTER_DEADBAND_M` | `0.005` | body-Y 居中死区。 | 小误差不移动，避免抖动。 |
| `CENTER_KP` | `0.8` | body-Y 偏移伺服比例增益。 | 越大越快，太大可能来回抢控制。 |
| `CENTER_VMAX_MPS` | `0.03` | 双侧接触后居中的最大 body-Y 速度。 | 应低于夹爪追夹响应。 |
| `CENTER_OFFSET_MAX_M` | `0.08` | 双侧居中最大 body-Y 偏移。 | 防止大幅横移。 |
| `CENTER_COMMAND_SIGN` | `1.0` | 双侧居中方向符号。 | 方向反了才改成 `-1.0`。 |
| `SINGLE_CONTACT_VMAX_MPS` | `0.015` | 单侧接触让位最大 body-Y 速度。 | 比双侧居中更慢，避免硬压物体。 |
| `SINGLE_CONTACT_OFFSET_MAX_M` | `0.10` | 单侧让位最大 body-Y 偏移。 | 防止持续侧推。 |
| `SINGLE_CONTACT_BODY_Y_SIGN` | `1.0` | 单侧让位方向符号。 | 单侧接触时飞机移动方向反了才改。 |
| `ABORT_RISE_M` | `0.25` | 抓取失败且未起吊前上升避让高度。 | 避免夹爪继续压目标。 |
| `ABORT_RISE_SPEED` | `0.12` | 抓取失败上升避让速度。 | 保守即可。 |

### 协同居中状态说明

| HLS 状态 | 飞机 body-Y 行为 | 夹爪行为 |
| --- | --- | --- |
| `SEARCH_OBJECT` | 不移动，保持抓取点 | 双指搜索闭合 |
| `LEFT_CONTACT` / `RIGHT_CONTACT` | 只有 `single_contact_need_motion=true` 才按 `single_contact_direction` 低速移动 | 已接触侧追夹，未接触侧继续搜索 |
| `BOTH_CONTACT` / `CENTERING` / `CENTERED` / `FINAL_GRIP` | 按 `centering_offset_m` 小幅居中 | 持续 close、追夹、最终夹持 |
| `LIFT_READY` | 停止抓取循环，冻结最后偏移并进入起吊 | 进入可起吊保持 |

## 常用调参方向

### 夹持力度略大

优先小幅降低：

- `HLS_CENTER_PUSH_CURRENT`
- `HLS_GRIP_CHASE_MIN_CURRENT`
- `HLS_LIFT_CURRENT`
- `HLS_GRIP_CHASE_POSITION_TORQUE_LIMIT`

不要一次降太多。当前轻力可靠组合是 `66/66/76/145`。

### 第一次下夹慢

优先调：

- `HLS_SEARCH_SPEED`

当前已调到 `10`。再增大前应观察初始接触是否冲击过大。

### 被推开后追夹慢

优先调：

- `HLS_GRIP_CHASE_POSITION_SPEED`
- `HLS_GRIP_CHASE_POSITION_ACC`
- `HLS_GRIP_CHASE_POSITION_TORQUE_LIMIT`
- `HLS_GRIP_CHASE_SLIP_RATIO`

如果只是补得慢，先小幅提高 speed 或 acc；如果明显没力，再提高 torque 或电流。

### 飞机居中方向反了

分情况改：

- 双侧接触后居中方向反：改 `CENTER_COMMAND_SIGN=-1.0`。
- 单侧接触让位方向反：改 `SINGLE_CONTACT_BODY_Y_SIGN=-1.0`。
- HLS 自身发布的 offset 方向反：检查 `HLS_CENTER_SIGN`。

每次只改一个符号，避免方向问题叠加。

### 未到位就夹

检查：

- `WAYPOINT_ARRIVAL_TOLERANCE_M` 是否过大。
- `WAYPOINT_ARRIVAL_SETTLE_S` 是否过小。
- `DRONE_POSE_TOPIC` 是否和控制器使用的位姿一致。
- 日志中是否出现 `Pre-grasp actual settle: actual drone arrived`。

### 到目标上方后自动 open 并下降

如果日志出现：

```text
Automatic HLS sequence failed: Target hover actual settle: actual drone did not arrive before timeout
Emergency CMD_CTRL descent with gripper open
```

含义是飞机还没有满足实际到位判定，脚本为了避免未到位夹持而中止，并按安全路径 open 后命令下降到 `CMD_LAND_Z`。这通常不是 CH10 触发，也不是 HLS fault。

处理顺序：

- 看 `final err=...m`。如果只差十几厘米且仍在收敛，优先增大 `WAYPOINT_ARRIVAL_TIMEOUT_S`。
- 如果悬停误差长期稳定在 8cm 以上，可把 `WAYPOINT_ARRIVAL_TOLERANCE_M` 小幅放宽，但不要大到导致 `Pre-grasp actual settle` 过早通过。
- 确认 `DRONE_POSE_TOPIC=/mavros/local_position/odom`，目标/box 仍用 mocap 刚体话题。
- 确认目标和 box 没有被遮挡导致长时间使用 latched pose。

### CH10 低位没有立即释放

检查：

- `ros2 topic echo --once --qos-reliability best_effort /mavros/rc/in`
- CH10 是否真的是 `channels[9]`。
- 低位 PWM 是否小于等于 `CH10_OPEN_PWM=1300`。
- `RC_TIMEOUT_S` 是否过大，或 `/mavros/rc/in` 是否在拨动 CH10 时有新消息。

## 关键日志和验收标准

飞行中重点看这些日志：

- `CH10 safety ready`
- `Publishing TAKEOFF`
- `px4ctrl state reached AUTO_HOVER`
- `px4ctrl state reached CMD_CTRL`
- `Target hover actual settle`
- `Pre-grasp actual settle`
- `HLS force grasp requested`
- `HLS state=... phase=single/center`
- `HLS reports safe_to_lift`
- `Box place actual settle`
- `Opening gripper to release`
- `CMD_CTRL landing`

验收标准：

- 起飞前、目标上方、下降和 settle 前夹爪一直 open。
- 未出现 `Pre-grasp actual settle` 前不会 close。
- 抓取中切 CH10 低位会立即 open 并中止。
- 单侧接触时，飞机只做低速 body-Y 让位，不大幅横跳。
- `LIFT_READY` 后才起吊。
- 放置后夹爪保持 open，撤离和命令降落不再 close。
