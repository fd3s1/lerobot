# HLS 夹爪调试和使用说明

本文档说明当前 HLS 双指夹爪的调试过程、推荐使用方法、状态机逻辑，以及 `shflies/handheld_hls_grasp_test.sh` 和 `shflies/auto_hls_grasp_place.sh` 中每个参数的含义。

当前主要测试入口是：

- `shflies/handheld_hls_grasp_test.sh`：手持 CH10 拨杆测试，只控制夹爪，不发布 `/position_cmd`，用于安全调夹爪。
- `shflies/auto_hls_grasp_place.sh`：自动抓取放置流程，会控制无人机位置命令，调夹爪前应先用手持脚本确认参数。

## 当前推荐结论

当前效果最好的调参方向是：

- 第一次下夹速度继续降低：`HLS_SEARCH_SPEED=7`。
- 搜索加速度保持柔和：`HLS_SEARCH_ACC=4`。
- 搜索力矩继续降低：`HLS_SEARCH_TORQUE_LIMIT=100`。
- 回中夹爪主动推力增强、追夹电流更轻、起吊阶段使用很高保持力：`HLS_CENTER_HOLD_CURRENT=24`、`HLS_CENTER_PUSH_CURRENT=115`、`HLS_GRIP_CHASE_MIN_CURRENT=36`、`HLS_LIFT_CURRENT=1000`、`HLS_MAX_CURRENT=1500`。
- 追夹位置脉冲保持原来效果好的速度：`HLS_GRIP_CHASE_POSITION_SPEED=16`、`HLS_GRIP_CHASE_POSITION_ACC=6`。
- 追夹位置脉冲力矩只小幅下降：`HLS_GRIP_CHASE_POSITION_TORQUE_LIMIT=145`。
- CH10 开爪使用稳定判定，不追求零延迟：`OPEN_MODE_STABLE_S=0.8`。
- RC 短时丢包保持原状态：`RC_STALE_MODE=hold`。

上述参数已经写入脚本默认值。常规测试时可以直接运行脚本；只有临时覆盖参数时才需要在命令行显式传入环境变量。

## 推荐手持测试命令

先确保当前目录和 ROS 环境正确：

```bash
cd /home/user/vla_drone/lerobot/ref_code/vla_px4ctrl_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
```

推荐测试命令：

```bash
bash shflies/handheld_hls_grasp_test.sh
```

## 操作方式

`handheld_hls_grasp_test.sh` 使用遥控器 CH10 控制夹爪：

- CH10 高位：进入抓取模式，发布 close 命令，夹爪搜索物体、居中、最终夹持。
- CH10 低位：进入开爪模式，发布 open 命令，夹爪从当前位置释放到全开。
- CH10 中位或 RC 短时不稳定：按脚本参数保持上一有效模式，避免抖动。

该脚本只控制夹爪：

- 不发布 `/position_cmd`。
- 不起飞、不降落。
- 不启动 LeRobot 录制。

## 调试过程记录

### 1. 建立可用的轻力抓取

最初有效版本使用：

- `HLS_SEARCH_SPEED=8`
- `HLS_SEARCH_ACC=4`
- `HLS_SEARCH_TORQUE_LIMIT=150`
- `HLS_CENTER_PUSH_CURRENT=70`
- `HLS_GRIP_CHASE_MIN_CURRENT=70`
- `HLS_LIFT_CURRENT=80`
- `HLS_GRIP_CHASE_POSITION_SPEED=16`
- `HLS_GRIP_CHASE_POSITION_ACC=6`
- `HLS_GRIP_CHASE_POSITION_TORQUE_LIMIT=150`

该版本抓取保持效果很好，但整体力度略大。

### 2. 降低夹持力

为了让所有阶段更轻，逐步降低了：

- 低保持电流。
- 居中推力电流。
- 追夹最小电流。
- 最终 lift 电流。
- 位置追夹力矩。

过度降低后会出现“被推开后补得慢”的现象，因此最终只做小幅下降，保持追夹动态不变。

### 3. 开爪安全修正

曾观察到 CH10 从高位切到低位时，夹爪有回中再打开的危险动作。现在开爪逻辑是：

- 节点收到 open 命令后退出当前抓取状态。
- 从当前位置释放到全开。
- 不先做居中。
- fault/open 状态也使用直接开爪路径。

### 4. 追夹速度恢复

后来将追夹位置速度/加速度调得太激进或太保守都不合适。实际效果最稳的是恢复：

- `HLS_GRIP_CHASE_POSITION_SPEED=16`
- `HLS_GRIP_CHASE_POSITION_ACC=6`

这两个参数主要影响被推开后的位置脉冲追夹，不应和第一次下夹搜索速度混淆。

### 5. 第一次下夹速度微调

第一次或第二次被推开时追夹略弱，后面会变好。合理原因是追夹记忆和接触判定刚进入稳定阶段：

- `grip_best_close_ratio` 需要先记录夹持后的最佳闭合比例。
- 接触判定需要多帧确认。
- `FINAL_GRIP -> LIFT_READY` 的电流爬升刚完成时，残差电流和物体姿态仍在进入稳态。

后续为了进一步降低第一次下夹冲击，当前默认搜索速度回到 `8`；追夹逻辑不变。

### 6. 单侧先接触时的追夹修正

后来发现一个结构性问题：如果左侧先碰到物体，飞机继续向左推时，左侧夹爪会停在第一次接触位置，只做电流保持，不会像进入稳定夹持后那样主动追夹；右侧先接触时也一样。

原因是早期代码只在 `CENTERING`、`CENTERED`、`FINAL_GRIP`、`LIFT_READY` 状态启用位置追夹，而 `LEFT_CONTACT` / `RIGHT_CONTACT` 单侧接触阶段的已接触侧只写保持电流。修改后，单侧接触阶段的已接触侧也会更新追夹记忆，并在被推开超过 `HLS_GRIP_CHASE_SLIP_RATIO` 后触发同一套位置脉冲追夹；未接触侧仍然继续按搜索速度闭合。

如果仍偶发“第一次单侧接触不补偿”，原因通常是单侧接触后还没有形成 `slipped_open` 或 `contact_lost` 判定，因此没有触发第一下位置脉冲。进一步修正为：刚进入 `LEFT_CONTACT` / `RIGHT_CONTACT` 时，自动对已接触侧预激活一次 `HLS_GRIP_CHASE_POSITION_PULSE_S` 时长的位置追夹窗口，相当于自动做一次轻微追夹唤醒，不需要人工碰一下或晃动飞机。

## HLS 节点状态机

`hls_gripper_node.py` 的主要状态如下：

| 状态 | 含义 | 主要动作 |
| --- | --- | --- |
| `OPEN` | 开爪 | 周期性写全开位置。 |
| `SEARCH_OBJECT` | 搜索物体 | 两侧向 close 位置运动，直到检测接触。 |
| `LEFT_CONTACT` / `RIGHT_CONTACT` | 单侧接触 | 接触侧保持电流并允许被推开时位置追夹，未接触侧继续闭合。 |
| `BOTH_CONTACT` | 双侧接触 | 进入居中前的过渡状态。 |
| `CENTERING` | 居中 | 根据左右闭合比例差异，差分调节左右电流。 |
| `CENTERED` | 居中完成 | 短暂保持后进入最终夹持。 |
| `FINAL_GRIP` | 最终夹持爬升 | 电流从低保持逐步爬升到 lift 电流。 |
| `LIFT_READY` | 可提升 | 持续保持和追夹，状态话题 `safe_to_lift` 为真时可认为可提升。 |
| `FAULT` | 失效保护 | 进入故障后开爪释放。 |

## 追夹逻辑说明

追夹由两部分组成：

1. 电流保持：持续给左右舵机写入保持电流，防止物体滑出。
2. 位置脉冲追夹：当检测到某侧被推开或接触丢失时，短时间给该侧写 close 位置，让夹爪主动追回。

位置脉冲追夹现在覆盖两类阶段：

- 单侧接触阶段：刚进入状态时对已接触侧预激活一次追夹；之后只对已经接触的一侧启用追夹，另一侧继续搜索闭合。
- 双侧接触后的阶段：对左右两侧都启用追夹，包括 `CENTERING`、`CENTERED`、`FINAL_GRIP`、`LIFT_READY`。

相关参数：

- `HLS_GRIP_CHASE_MIN_CURRENT` 控制追夹时最低保持电流。
- `HLS_GRIP_CHASE_POSITION_ENABLE` 控制是否启用位置脉冲。
- `HLS_GRIP_CHASE_POSITION_SPEED` 控制位置脉冲速度。
- `HLS_GRIP_CHASE_POSITION_ACC` 控制位置脉冲加速度。
- `HLS_GRIP_CHASE_POSITION_TORQUE_LIMIT` 控制位置脉冲最大力矩。
- `HLS_GRIP_CHASE_SLIP_RATIO` 控制被推开多少比例后触发追夹。
- `HLS_GRIP_CHASE_POSITION_PERIOD_S` 控制位置脉冲最小重复间隔。
- `HLS_GRIP_CHASE_POSITION_PULSE_S` 控制一次位置脉冲持续时间。

## 手持测试脚本参数详解

本节解释 `shflies/handheld_hls_grasp_test.sh` 的每一个环境变量。

### ROS 话题参数

| 参数 | 默认值 | 含义 | 调参建议 |
| --- | --- | --- | --- |
| `RC_TOPIC` | `/mavros/rc/in` | 遥控器 RC 输入话题。 | 通常不改。若 MAVROS 命名空间变化才改。 |
| `HLS_STATUS_TOPIC` | `/hls_gripper/status` | HLS 状态前缀兼容话题。脚本会派生读取 `/hls_gripper/status_snapshot` 和 `/hls_gripper/state` 等状态。 | 通常不改。自动飞行优先使用原子 snapshot，手持和调试仍可看分散话题。 |
| `HLS_COMMAND_TOPIC` | `/handheld_hls_grasp_test/command` | 手持测试节点发布给 HLS 节点的标量命令话题。 | 为避免和自动脚本冲突，手持测试使用独立话题。 |
| `HLS_COMMAND_PAIR_TOPIC` | `/handheld_hls_grasp_test/ignore_pair` | 双侧命令话题，本手持测试基本不用。 | 保持默认即可。 |
| `GRIPPER_FEEDBACK_TOPIC` | `/gripper/feedback` | 夹爪反馈话题，包含位置、电流、温度等。 | 通常不改。 |
| `ATTITUDE_TOPIC` | `/mavros/imu/data` | 机体姿态话题，用于重力补偿表计算。 | 若没有 MAVROS 或话题名不同需修改。 |

### HLS 管理器参数

| 参数 | 默认值 | 含义 | 调参建议 |
| --- | --- | --- | --- |
| `START_GRIPPER_MANAGER` | `true` | 是否由脚本启动 `hls_gripper_node.py`。 | 单独手持测试用 `true`；若已有节点运行，用 `false`。 |
| `GRIPPER_MANAGER_PORT` | `/dev/ttyACM1` | HLS 舵机串口。 | 若设备枚举变化，改成实际端口。 |
| `HLS_GRAVITY_COMP_PATH` | `${WORKSPACE_DIR}/install/hls_gripper/share/hls_gripper/config/gravity_compensation.json` | 重力补偿和限位 JSON 路径。 | 默认使用安装目录中的当前标定文件。 |
| `HLS_SDK_ROOT` | 空 | FTServo Python SDK 路径。 | SDK 在默认候选路径时可空；找不到 SDK 时再指定。 |
| `HLS_DRY_RUN` | `false` | 是否干跑，不实际写舵机。 | 调软件逻辑可设 `true`；真机必须 `false`。 |

### 开爪参数

| 参数 | 默认值 | 含义 | 调参建议 |
| --- | --- | --- | --- |
| `HLS_OPEN_SPEED` | `24` | 开爪位置控制速度。 | 太小开爪慢；太大可能冲击。当前值较安全。 |
| `HLS_OPEN_ACC` | `6` | 开爪位置控制加速度。 | 控制开爪柔和程度。 |
| `HLS_OPEN_TORQUE_LIMIT` | `160` | 开爪位置控制力矩上限。 | 只负责释放；当前加大用于保证释放可靠，不影响夹持阶段持续力。 |

### 夹持电流参数

| 参数 | 默认值 | 含义 | 调参建议 |
| --- | --- | --- | --- |
| `HLS_LOW_CURRENT` | `28` | 低保持电流，居中前后轻力保持的基础值。 | 越大越稳但越容易压物体；越小越容易松。 |
| `HLS_LIFT_CURRENT` | `1000` | `LIFT_READY` 阶段的最终保持电流。 | 直接影响提起时夹持力；当前用于防止起吊时被拔出，力度很大，必须注意温度、电流保护和压物体风险。 |
| `HLS_MAX_CURRENT` | `1500` | HLS 节点反馈电流故障阈值，对应 ROS 参数 `max_current`。 | 因 `HLS_LIFT_CURRENT` 已明显超过节点默认 `600`，这里同步抬高，避免起吊阶段误触发 current limit fault。 |
| `HLS_CENTER_HOLD_CURRENT` | `24` | 居中阶段非推力侧电流。 | 比低保持更轻，减少非推力侧持续压迫。 |
| `HLS_CENTER_PUSH_CURRENT` | `115` | 居中和追夹时推力侧电流。 | 当前进一步增强夹爪主动回中推力；太大居中有冲击，太小补偿弱。 |
| `HLS_GRIP_CHASE_MIN_CURRENT` | `36` | 追夹时两侧最低电流地板。 | 当前继续降低追夹持续电流，让被推开后的补偿更轻。 |

### 追夹位置脉冲参数

| 参数 | 默认值 | 含义 | 调参建议 |
| --- | --- | --- | --- |
| `HLS_GRIP_CHASE_POSITION_ENABLE` | `true` | 是否启用位置脉冲追夹。 | 需要保持夹持时建议打开。 |
| `HLS_GRIP_CHASE_POSITION_SPEED` | `16` | 追夹位置脉冲速度。 | 被推开后追得慢可增加；太大可能冲击或抖动。 |
| `HLS_GRIP_CHASE_POSITION_ACC` | `6` | 追夹位置脉冲加速度。 | 太大响应硬，太小补偿慢。 |
| `HLS_GRIP_CHASE_POSITION_TORQUE_LIMIT` | `145` | 追夹位置脉冲力矩上限。 | 控制被推开补偿的最大力；当前比强力版略小。 |
| `HLS_GRIP_CHASE_SLIP_RATIO` | `0.03` | 触发追夹的位置滑移比例阈值。 | 越小越敏感；太小可能误触发。 |
| `HLS_GRIP_CHASE_POSITION_PERIOD_S` | `0.05` | 追夹脉冲最小重复周期。代码内部会限制到至少约 `0.08s`。 | 一般不需要改。 |
| `HLS_GRIP_CHASE_POSITION_PULSE_S` | `0.80` | 一次追夹位置脉冲持续时间。 | 太短追不上，太长可能持续顶住物体。 |

### 温度和故障保护参数

| 参数 | 默认值 | 含义 | 调参建议 |
| --- | --- | --- | --- |
| `HLS_CENTER_TIMEOUT_ACTION` | `final_grip` | 居中超时后的动作。`final_grip` 表示继续最终夹持，`fault` 表示故障释放。 | 当前推荐 `final_grip`，避免轻微未居中导致任务失败。 |
| `HLS_CENTER_TIMEOUT_S` | `5.0` | HLS 在 `CENTERING` 中等待真正居中的最长时间。 | 如果夹爪已经差不多居中但迟迟不起吊，优先缩短这个值。 |
| `HLS_CENTER_STABLE_TIME_S` | `0.25` | 中心误差进入 deadband 后需要连续稳定的时间。 | 太大容易因接触抖动反复重置；太小可能过早进入最终夹持。 |
| `HLS_FINAL_GRIP_RAMP_S` | `0.8` | `FINAL_GRIP` 中从低电流爬升到 `HLS_LIFT_CURRENT` 的时间。 | 当前 lift 电流很大，不建议再明显缩短。 |
| `HLS_MAX_TEMP` | `85.0` | 舵机温度故障阈值，单位摄氏度。 | 达到后进入故障保护。 |
| `HLS_TEMP_WARN_THRESHOLD` | `75.0` | 舵机温度警告阈值。 | 高温时应暂停测试散热。 |

### 电流方向和限位参数

| 参数 | 默认值 | 含义 | 调参建议 |
| --- | --- | --- | --- |
| `HLS_LEFT_CURRENT_INWARD_SIGN` | `1` | 左侧向内夹持电流符号。 | 当前实测用 `1`。方向错会越夹越松或接触判断异常。 |
| `HLS_RIGHT_CURRENT_INWARD_SIGN` | `1` | 右侧向内夹持电流符号。 | 当前实测用 `1`。 |
| `HLS_LEFT_OPEN` | 空 | 左舵机全开位置，覆盖补偿文件中的标定。 | 只有重新标定或补偿文件缺失时才填。 |
| `HLS_LEFT_CLEAR` | 空 | 左舵机安全清空/过渡位置。 | 通常来自补偿文件。 |
| `HLS_LEFT_CLOSE` | 空 | 左舵机全闭位置。 | 不建议手动覆盖，避免撞限位。 |
| `HLS_RIGHT_OPEN` | 空 | 右舵机全开位置。 | 同上。 |
| `HLS_RIGHT_CLEAR` | 空 | 右舵机安全清空/过渡位置。 | 同上。 |
| `HLS_RIGHT_CLOSE` | 空 | 右舵机全闭位置。 | 同上。 |

### 搜索和运动 profile 参数

| 参数 | 默认值 | 含义 | 调参建议 |
| --- | --- | --- | --- |
| `HLS_MOTION_PROFILE` | `p3` | 使用补偿 JSON 中的命名 motion profile，例如 `p3`。 | 默认使用当前实测 profile。 |
| `HLS_MOTION_PROFILE_INDEX` | 空 | 按索引选择 motion profile。 | 一般不用；优先用名称。 |
| `HLS_SEARCH_SPEED` | `7` | 第一次下夹搜索物体的闭合速度。 | 当前继续降低搜索速度；只影响初始搜索，不等同于追夹速度。 |
| `HLS_SEARCH_ACC` | `4` | 第一次下夹搜索物体的加速度。 | 当前保持柔和。 |
| `HLS_SEARCH_TORQUE_LIMIT` | `100` | 第一次下夹搜索物体的力矩上限。 | 当前继续降低搜索冲击；太小接触不稳定，太大初夹冲击大。 |

### 居中和单侧接触参数

| 参数 | 默认值 | 含义 | 调参建议 |
| --- | --- | --- | --- |
| `HLS_CENTER_GAIN_M_PER_RATIO` | `0.0` | 闭合比例差到米的转换增益。`0.0` 表示自动根据几何估算。 | 通常保持 `0.0`。 |
| `HLS_CENTER_ERROR_GAIN` | `2.0` | 居中误差放大系数。 | 越大越积极修正左右偏差，太大可能左右摆动。 |
| `HLS_CENTER_BIAS` | `0.0` | 居中偏置。 | 如果长期偏向一侧，可小幅修正。 |
| `HLS_CENTER_SIGN` | `1.0` | 居中方向符号。 | 方向反了会越居中越偏。 |
| `HLS_SINGLE_CONTACT_OFFSET_LIMIT_M` | `0.12` | 单侧接触时允许估计的最大横向偏移。 | 防止单侧接触补偿过大。 |
| `HLS_SINGLE_CONTACT_TIMEOUT_S` | `20.0` | 单侧接触最长等待时间。 | 太短会误故障；太长会卡状态。 |
| `HLS_SINGLE_CONTACT_LIMIT_RATIO` | `0.97` | 未接触侧接近闭合极限的比例阈值。 | 用于判断是否需要无人机/夹爪继续偏移寻找另一侧。 |
| `HLS_DRY_RUN_CONTACT_PATTERN` | `both` | 干跑时模拟接触模式。 | 只在 `HLS_DRY_RUN=true` 时有意义。 |

### 手持 RC 参数

| 参数 | 默认值 | 含义 | 调参建议 |
| --- | --- | --- | --- |
| `RATE_HZ` | `20` | 手持测试节点循环频率。 | 通常不改。 |
| `RC_TIMEOUT_S` | `0.8` | RC 输入超时时间。 | 太短容易误判丢包。 |
| `STATUS_TIMEOUT_S` | `2.0` | HLS 状态超时时间。 | 超时日志会显示 status stale。 |
| `CH10_INDEX` | `9` | CH10 在 RC channel 数组中的索引，0 基。 | 第 10 通道即 `9`。 |
| `CH10_OPEN_PWM` | `1300` | 低位/open 判定阈值附近 PWM。 | 遥控器行程不同才改。 |
| `CH10_CLOSE_PWM` | `1700` | 高位/grasp 判定阈值附近 PWM。 | 遥控器行程不同才改。 |
| `OPEN_COMMAND` | `100.0` | 发布给 HLS 的开爪命令值。 | 保持默认。 |
| `CLOSE_COMMAND` | `0.0` | 发布给 HLS 的抓取命令值。 | 保持默认。 |
| `PUBLISH_PERIOD_S` | `0.5` | 重复发布命令的周期。 | 太短日志多；太长状态切换慢。 |
| `GRASP_MODE_STABLE_S` | `0.3` | CH10 高位需要稳定多久才确认 grasp。 | 太小易误触发，太大响应慢。 |
| `OPEN_MODE_STABLE_S` | `0.8` | CH10 低位需要稳定多久才确认 open。 | 当前用稍长延迟，避免抖动或丢包导致危险开爪。 |
| `HOLD_MODE_TIMEOUT_S` | `1.0` | CH10 中位/hold 保持上一模式的最长时间。 | 当前推荐 `1.0`。 |
| `RC_STALE_MODE` | `hold` | RC 丢包时的处理。`hold` 保持上一模式，`open` 进入开爪。 | 手持夹持测试推荐 `hold`，避免短时丢包打断抓取。 |
| `CSV_PATH` | 空 | 状态日志 CSV 输出路径。 | 需要离线分析时指定，例如 `/tmp/hls_test.csv`。 |

## 自动抓放脚本补充参数

`auto_hls_grasp_place.sh` 复用上面所有 HLS 参数，并额外包含无人机抓放流程参数。本节只解释自动脚本新增的参数。

### 自动飞行中的协同居中逻辑

手持测试脚本不会发布 `/position_cmd`，所以手持测试里的 centering 只由 HLS 夹爪内部的电流差和追夹脉冲完成；飞机不会主动移动。

自动抓放脚本会发布 `/position_cmd`，抓取阶段采用分阶段保守协同：

1. `SEARCH_OBJECT`：飞机保持预抓取位置不横移，只让夹爪搜索闭合。
2. `LEFT_CONTACT` / `RIGHT_CONTACT`：夹爪继续追夹。只有 HLS 发布 `single_contact_need_motion=true` 时，飞机才按 `single_contact_direction` 做低速 body-Y 让位，速度上限由 `SINGLE_CONTACT_VMAX_MPS` 控制。
3. `BOTH_CONTACT` / `CENTERING` / `CENTERED` / `FINAL_GRIP`：飞机以 `/hls_gripper/status_snapshot` 中的 `center_error_m` 为主做小幅 body-Y 居中修正，速度上限由 `CENTER_VMAX_MPS` 控制，同时每个控制循环继续向 HLS 发布 close command，保证移动过程中夹爪仍然追夹物体。HLS 节点内部仍会用差动电流和追夹脉冲主动配合居中。
4. `LIFT_READY`：冻结当前 body-Y 偏移，进入后续提起流程。

如果 body-Y 偏移真的走到 `SINGLE_CONTACT_OFFSET_MAX_M` 或 `CENTER_OFFSET_MAX_M` 附近，自动脚本不会立刻开爪中止，而是短暂保持 close command 等待 HLS 变为 `safe_to_lift`；如果持续卡在上限，才会执行开爪并上升撤离。

### 自动脚本话题参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `TARGET_POSE_TOPIC` | `/strawberry_bear/pose` | 被抓目标位姿话题。 |
| `BOX_POSE_TOPIC` | `/box1/pose` | 放置盒子位姿话题。 |
| `DRONE_POSE_TOPIC` | `/mavros/local_position/odom` | 无人机控制位姿话题。自动抓取用它生成 `/position_cmd`，和 UDE 单独测试保持一致，不要改成 vision pose。 |
| `ARRIVAL_POSE_TOPIC` | `/mavros/vision_pose/pose` | 无人机实际到位判定位姿话题。只用于判断是否到达目标/抓取/box 关键点。 |
| `CMD_TOPIC` | `/position_cmd` | 发布给 px4ctrl 的位置命令。 |
| `GRIPPER_TOPIC` | `/gripper/command` | 自动流程给夹爪发标量命令的话题。 |
| `GRIPPER_COMMAND_PAIR_TOPIC` | `/gripper/command_pair` | 自动流程给夹爪发双侧命令的话题。 |
| `TAKEOFF_LAND_TOPIC` | `/px4ctrl/takeoff_land` | 起降命令话题。 |
| `PX4CTRL_STATE_TOPIC` | `/px4ctrl/state` | px4ctrl 状态话题。 |
| `TAKEOFF_MODE` | `auto` | 默认发布 `TakeoffLand.TAKEOFF` 自动起飞；临时设为 `manual` 时不发布 `TAKEOFF`，只等待手动起飞后的 `AUTO_HOVER`。 |
| `CONFIRM_BEFORE_TAKEOFF` | HLS-UDE 一键脚本默认 `true` | 自动节点完成 pose 稳定检查并锁定快照后，是否等待一次 Enter 再继续。 |
| `RC_TOPIC` | `/mavros/rc/in` | 自动 HLS 流程读取 CH10 安全释放的话题。 |
| `RC_TIMEOUT_S` | `0.5` | 自动 HLS 流程判断 RC topic 新鲜度的阈值。起飞前要求 fresh，飞行中默认 stale 只告警。 |
| `RC_STALE_ACTION` | `warn` | 飞行中 RC topic 超时后的动作。`warn` 只报警继续；`abort` 会 open 并中止。 |
| `CH10_INDEX` | `9` | CH10 在 RC channel 数组中的索引，0 基。 |
| `CH10_OPEN_PWM` | `1300` | CH10 低位/open 判定阈值。飞行中只要收到低位会立即 open 并中止。 |
| `CH10_CLOSE_PWM` | `1700` | 起飞前安全检查中的 CH10 高位阈值。 |

### 自动脚本管理参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `GRASP_PARAMS_FILE` | `shflies/grasp_params.env` | 自动脚本启动前加载的参数文件。 |
| `GRIPPER_MANAGER_TYPE` | `hls` | 夹爪管理器类型。当前脚本要求 `hls`。 |
| `START_GRIPPER_MANAGER` | `true` | 是否由自动脚本启动夹爪节点。 |
| `GRIPPER_MANAGER_PORT` | `/dev/ttyACM1` | HLS 串口。 |

### 飞行速度和流程时间参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `RATE_HZ` | `20` | 自动流程控制循环频率。 |
| `MAX_SPEED` | `0.6` | 全局最大移动速度，单位 m/s。 |
| `APPROACH_SPEED` | `0.15` | 接近目标速度。 |
| `LIFT_SPEED` | `0.4` | 空载提升速度。 |
| `PAYLOAD_LIFT_SPEED` | `0.15` | 夹住物体后的提升速度。 |
| `PAYLOAD_TRANSFER_SPEED` | `0.15` | 带载转移速度。 |
| `POST_GRASP_SETTLE_S` | `0.3` | 抓取完成后的等待时间。 |
| `POST_LIFT_SETTLE_S` | `1.0` | 提升后的等待时间。 |
| `SMOOTH_TRAJECTORY` | `true` | 是否使用平滑轨迹。 |

### 带载补偿参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `PAYLOAD_LIFT_FORWARD_COMP_M` | `0.0` | 带载提升前向补偿，旧式单轴参数。 |
| `PAYLOAD_LIFT_COMP_X` | `0.0` | 带载提升 X 补偿。 |
| `PAYLOAD_LIFT_COMP_Y` | `0.0` | 带载提升 Y 补偿。 |
| `PAYLOAD_LIFT_COMP_Z` | `0.0` | 带载提升 Z 补偿。 |

### 夹爪几何和目标参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `GRIPPER_X_OFFSET_M` | `0.0` | 夹爪相对机体/控制点的 X 偏移。 |
| `GRIPPER_Y_OFFSET_M` | `0.0` | 夹爪相对机体/控制点的 Y 偏移。 |
| `GRIPPER_Z_OFFSET_M` | `0.25` | 夹爪相对机体/控制点的 Z 偏移。 |
| `TARGET_HEIGHT_M` | `0.30` | 目标物体高度。 |
| `TARGET_GRASP_HEIGHT_M` | `0.17` | 抓取高度。 |
| `TARGET_POSE_Z_REFERENCE` | `center` | 目标位姿 Z 的参考点。 |
| `TARGET_HOVER_CLEARANCE_M` | `0.45` | 目标上方悬停净空。 |
| `TARGET_OFFSET_X` | `0.0` | 抓取目标 X 修正。 |
| `TARGET_OFFSET_Y` | `0.0` | 抓取目标 Y 修正。 |
| `TARGET_OFFSET_Z` | `0.0` | 抓取目标 Z 修正。 |

### 放置盒参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `BOX_LENGTH_M` | `0.65` | 盒子长度。 |
| `BOX_WIDTH_M` | `0.41` | 盒子宽度。 |
| `BOX_HEIGHT_M` | `0.14` | 盒子高度。 |
| `BOX_HOVER_GRIPPER_CLEARANCE_M` | `0.55` | 盒子上方悬停时夹爪净空。 |
| `BOX_PLACE_BOTTOM_CLEARANCE_M` | `0.24` | 放置时物体底部相对盒底净空。 |
| `BOX_OFFSET_X` | `0.0` | 放置点 X 修正。 |
| `BOX_OFFSET_Y` | `0.0` | 放置点 Y 修正。 |
| `BOX_OFFSET_Z` | `0.0` | 放置点 Z 修正。 |

### 释放和降落参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `RELEASE_RETREAT_UP_M` | `0.3` | 释放后向上撤离距离。 |
| `RELEASE_RETREAT_FORWARD_M` | `1.0` | 释放后前向撤离距离。 |
| `RETREAT_SPEED` | `0.4` | 撤离速度。 |
| `LANDING_MODE` | `cmd` | 降落模式。 |
| `CMD_LAND_SPEED` | `0.25` | 命令式降落速度。 |
| `CMD_LAND_Z` | `-0.3` | 命令式降落目标 Z。 |
| `CMD_LAND_Z_OFFSET_M` | `0.0` | 命令式降落 Z 偏移。 |
| `NO_LAND` | `false` | 是否跳过降落。 |

### 自动抓取判定和补偿参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `HLS_GRASP_TIMEOUT_S` | `12.0` | 等待 HLS 抓取完成的最长时间。 |
| `HLS_STATUS_TIMEOUT_S` | `0.8` | 自动流程认为 HLS 状态新鲜的超时时间。 |
| `CENTER_DEADBAND_M` | `0.04` | 自动居中死区。 |
| `CENTER_KP` | `1.0` | 自动居中比例控制增益。 |
| `CENTER_VMAX_MPS` | `0.04` | 双侧接触后根据 HLS `center_error_m` 居中时的最大 body-Y 速度。 |
| `CENTER_OFFSET_MAX_M` | `0.30` | 双侧接触后自动居中允许的最大 body-Y 偏移量。 |
| `HLS_STATUS_CENTER_MISMATCH_TOL_M` | `0.015` | 双侧接触时 `center_error_m` 与 `centering_offset_m` 的一致性容差。不同号或差值过大时，本周期 body-Y 居中会被拒绝。 |
| `CENTER_COMMAND_SIGN` | `1.0` | 双侧居中指令方向符号。若飞机越修越偏，优先检查这个符号。 |
| `SINGLE_CONTACT_VMAX_MPS` | `0.015` | 单侧接触且 HLS 请求 `single_contact_need_motion` 时，飞机让位的最大 body-Y 速度。该值应小于或等于 `CENTER_VMAX_MPS`。 |
| `SINGLE_CONTACT_OFFSET_MAX_M` | `0.10` | 单侧接触让位允许的最大 body-Y 偏移。 |
| `SINGLE_CONTACT_BODY_Y_SIGN` | `1.0` | 单侧接触让位方向符号。若左/右单侧接触时飞机让位方向反了，优先改这个符号。 |
| `ABORT_RISE_M` | `0.25` | 抓取失败中止时上升距离。 |
| `ABORT_RISE_SPEED` | `0.12` | 抓取失败中止时上升速度。 |
| `OPEN_COMMAND` | `100.0` | 自动流程开爪命令值。 |
| `CLOSE_COMMAND` | `0.0` | 自动流程抓取命令值。 |

### 航点和预检参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `WAYPOINT_ARRIVAL_TOLERANCE_M` | `0.12` | 航点到达位置容差。 |
| `WAYPOINT_ARRIVAL_SETTLE_S` | `0.4` | 到达航点后的稳定等待时间。 |
| `WAYPOINT_ARRIVAL_TIMEOUT_S` | `30.0` | 航点到达超时。 |
| `POSE_PREFLIGHT_TIMEOUT_S` | `6` | 启动前检查位姿话题的超时时间。 |
| `POSE_PREFLIGHT_REQUIRED` | `false` | 位姿预检失败是否直接退出。 |
| `SKIP_POSE_PREFLIGHT` | `false` | 是否跳过位姿预检。 |

## 调参指南

### 第一次下夹慢

优先调：

1. `HLS_SEARCH_SPEED`：当前回到 `8`，优先降低第一次下夹冲击。
2. `HLS_SEARCH_ACC`：若仍慢，可从 `4` 小幅增加到 `5`，但要观察冲击。
3. `HLS_SEARCH_TORQUE_LIMIT`：只有明显夹不住或闭合受阻时才增加。

不要优先调追夹参数，因为追夹只在被推开或接触丢失后起作用，不是第一次搜索闭合。

### 被推开后追夹慢

优先调：

1. `HLS_GRIP_CHASE_POSITION_SPEED`
2. `HLS_GRIP_CHASE_POSITION_ACC`
3. `HLS_GRIP_CHASE_POSITION_TORQUE_LIMIT`
4. `HLS_GRIP_CHASE_MIN_CURRENT`

如果只是第一次或第二次推开略弱，后面很好，通常是追夹记忆和接触状态冷启动，不一定需要改参数。

### 夹持力度太大

优先小幅降低：

1. `HLS_CENTER_PUSH_CURRENT`
2. `HLS_GRIP_CHASE_MIN_CURRENT`
3. `HLS_LIFT_CURRENT`
4. `HLS_GRIP_CHASE_POSITION_TORQUE_LIMIT`

每次建议只降 `3-5` 个单位，不要一次大幅下降。

### 夹持松或提起掉落

优先小幅增加：

1. `HLS_LIFT_CURRENT`
2. `HLS_GRIP_CHASE_MIN_CURRENT`
3. `HLS_CENTER_PUSH_CURRENT`

若被推开后补偿慢，再看追夹位置参数。

### 开爪不安全

重点检查：

- CH10 低位是否稳定。
- `OPEN_MODE_STABLE_S` 是否太短。
- `RC_STALE_MODE` 是否被设为 `open`。
- 是否有多个节点同时发布夹爪命令。

当前推荐 `OPEN_MODE_STABLE_S=0.8`、`RC_STALE_MODE=hold`。

## 常用状态观察

运行时重点看日志中的字段：

- `state`：HLS 状态机状态。
- `pos=(left,right)`：左右舵机原始位置。
- `seg=(left,right)`：左右闭合段比例。
- `contact=(left,right)`：左右接触判定。
- `limit=(left,right)`：是否接近闭合极限。
- `chase=(left,right)`：是否正在触发追夹位置脉冲。
- `safe=1`：进入 `LIFT_READY` 且双侧接触，适合提升。
- `cur=(left,right)`：当前电流。
- `res=(left,right)`：扣除重力补偿后的残差电流，用于接触判定。

也可以直接 echo 状态话题：

```bash
ros2 topic echo /hls_gripper/state
ros2 topic echo /hls_gripper/safe_to_lift
ros2 topic echo /hls_gripper/left_chase_position_pulse
ros2 topic echo /hls_gripper/right_chase_position_pulse
ros2 topic echo /gripper/feedback
```

## 安全注意事项

- 真机测试前确认 `GRIPPER_MANAGER_PORT` 指向正确设备。
- 初次上电不要把手放在夹爪闭合路径中。
- 若温度超过 `HLS_TEMP_WARN_THRESHOLD`，应暂停测试散热。
- 若发现开爪不是从当前位置直接释放，应立即停止测试，检查是否有旧节点或其他脚本在发布命令。
- 自动抓放前必须先用手持脚本确认夹爪动作稳定。
- 自动脚本会发布 `/position_cmd`，不要在地面或人员附近误运行。

## 版本维护建议

若后续重新调参，建议每次只改一类参数：

1. 先调第一次搜索：`HLS_SEARCH_SPEED`、`HLS_SEARCH_ACC`、`HLS_SEARCH_TORQUE_LIMIT`。
2. 再调夹持力：`HLS_CENTER_PUSH_CURRENT`、`HLS_GRIP_CHASE_MIN_CURRENT`、`HLS_LIFT_CURRENT`。
3. 最后调追夹：`HLS_GRIP_CHASE_POSITION_*` 和 `HLS_GRIP_CHASE_SLIP_RATIO`。

每次测试记录终端第一段配置输出，尤其是：

- `current:`
- `chase position:`
- `search override:`
- `rc latch:`
- `HLS gripper node started:`

这些输出能确认实际运行参数，不要只看命令行或脚本默认值。
