# HLS3625 双指夹爪柔顺夹持控制方案

本文整理 Feetech HLS3625 舵机用于无人机末端双指夹爪的控制思路。目标是：夹爪在未知物体位置时低速低扭矩搜索，低力夹住偏心物体；无人机根据夹爪反馈移动，使物体逐渐回到夹爪中心；居中后再增加夹紧力并起吊。

## 1. 需求理解

夹爪安装在无人机上，不能在物体偏心时用大力强行推物体回中，否则会给无人机带来较大的反作用力矩，造成姿态扰动甚至事故。

理想流程：

```text
夹爪不知道物体在哪
  -> 两指慢速、低扭矩向内搜索
  -> 一边先接触物体，保持低扭矩贴住
  -> 另一边继续搜索，直到也接触
  -> 两边都低扭矩夹住
  -> 根据左右舵机角度差/闭合量差估计偏心
  -> 无人机移动，让夹爪逐渐居中
  -> 夹爪保持低扭矩配合物体移动
  -> 居中稳定后增加夹紧力
  -> 无人机起吊
```

核心原则：

- 偏心阶段，夹爪只做柔顺夹持和偏心测量，不主动用大力纠正物体。
- 回中动作由无人机上层控制器完成。
- 夹爪在搜索和回中阶段保持低扭矩，居中后才增加扭矩。

## 2. HLS3625 是否支持

HLS 系列 SDK 显示支持以下模式：

```text
ServoMode: 位置模式
WheelMode: 恒速模式
EleMode: 恒力/恒电流模式
```

Linux/C++ SDK 中相关接口：

```cpp
WritePosEx(ID, Position, Speed, ACC, Torque)
```

用于：

```text
位置目标 + 速度 + 加速度 + 扭矩限制
```

适合搜索阶段：让舵机朝闭合位置慢慢走，但最大输出扭矩很小。

```cpp
EleMode(ID)
WriteEle(ID, Torque)
```

用于：

```text
恒力/恒电流保持
```

适合两边都接触后的低力夹持、无人机回中阶段，以及居中后的夹紧力提升。

结论：

```text
搜索阶段：位置模式 + 低速度 + 低扭矩限制
保持/回中阶段：恒力模式 + 低扭矩
起吊前：恒力模式下缓慢增加扭矩
```

## 3. Python SDK 与 C++ SDK 对比

Python SDK 可以实现该方案，但原版 HLS 封装不如 C++ 完整。

Python SDK 原版已支持：

```python
packetHandler.WritePosEx(id, position, speed, acc, torque)
```

这可以用于低速低扭矩搜索。

Python SDK 原版对 `EleMode` 和 `WriteEle` 可能没有直接封装，但底层寄存器读写能力存在，可以直接写寄存器实现：

```python
# 进入恒力/恒电流模式
packetHandler.write1ByteTxRx(id, HLS_MODE, 2)

# 写目标扭矩，负值需要先编码
torque_raw = packetHandler.scs_toscs(torque, 15)
packetHandler.write2ByteTxRx(id, HLS_GOAL_TORQUE_L, torque_raw)
```

C++ SDK 相对更适合正式飞行内环，原因：

- HLS 接口封装完整，`EleMode`、`WriteEle`、`ReadCurrent` 等接口直接可用。
- 串口读写时序更确定。
- C++ 串口层会清输入缓存，异常通信后更不容易受残留字节影响。
- 控制循环抖动更小。

建议取舍：

```text
地面验证、状态机调试：Python SDK 可以
无人机正式飞行、安全内环：优先 C++ 或独立控制器
```

如果使用 Python 做正式控制，建议补齐：

- 输入缓存清理。
- `EleMode`、`WriteEle`、`ReadCurrent` 等薄封装。
- 通信错误重试。
- watchdog。
- 软限位。
- 超温、过流、越界卸力。

## 4. 角度统一后的闭合量定义

如果已经在舵机寄存器中统一了左右两边的最大/最小角度，可以把每个舵机位置抽象成统一的闭合量。

定义：

```text
open_pos  = 张开位置
close_pos = 闭合极限位置
```

闭合比例：

```text
close_ratio = (present_pos - open_pos) / (close_pos - open_pos)
```

这样无论左右舵机原始方向是否相反，都可以得到：

```text
0.0 = 完全张开
1.0 = 完全闭合
```

左右闭合量：

```text
left_close_ratio
right_close_ratio
```

偏心误差：

```text
center_error = left_close_ratio - right_close_ratio
```

理想居中时：

```text
left_close_ratio ~= right_close_ratio
center_error ~= 0
```

注意：寄存器统一最大最小角度只统一了运动范围，不一定完全统一指尖几何。如果两指连杆、安装角度、手指长度有差异，最好进一步把舵机角度标定为指尖位置，再计算偏心。

## 5. 推荐状态机

### OPEN

夹爪张开到安全初始位置。

输出：

```text
state = OPEN
safe_to_lift = false
```

### SEARCH_OBJECT

两指同时慢速、低扭矩向内闭合。

控制方式：

```text
ServoMode
WritePosEx(close_limit, low_speed, low_acc, low_torque_limit)
```

目标不是精确到闭合极限，而是用很低的速度和扭矩限制去找物体。

### LEFT_CONTACT / RIGHT_CONTACT

一侧先接触物体。

先接触侧：

```text
切到 EleMode
保持很小的 inward torque
```

另一侧：

```text
继续低速、低扭矩搜索
```

这样可以避免先接触的一侧继续强推物体，把扰动力传给无人机。

### BOTH_CONTACT

两边都检测到接触后，两侧都进入低恒扭矩保持。

控制方式：

```text
left_cmd  = left_inward_sign  * T_low
right_cmd = right_inward_sign * T_low
```

左右符号必须根据实际安装方向实测。

### CENTERING

夹爪保持低扭矩不变，持续输出：

```text
left_pos
right_pos
left_close_ratio
right_close_ratio
center_error
```

无人机根据 `center_error` 做横向移动，使夹爪逐渐居中。

夹爪不要主动用大力把物体推回中心。

### CENTERED

居中判定建议：

```text
abs(center_error) < center_threshold
持续 0.5~1.0 s
且 两侧仍保持接触
```

### FINAL_GRIP

居中稳定后，夹爪从低扭矩缓慢增加到起吊扭矩。

```text
T_low -> T_lift
```

建议 ramp 时间：

```text
0.5~2.0 s
```

不要阶跃增加扭矩。

### LIFT_READY

夹紧力达到起吊值，并且反馈正常。

输出：

```text
safe_to_lift = true
```

## 6. 接触检测

不要只看电流或负载，建议组合判断：

```text
接触 = 电流/负载残差超过阈值
    且 位置变化速度明显变小
    且 状态持续 N 个控制周期
```

示例：

```text
abs(current_residual) > I_contact
abs(position_delta) < pos_delta_threshold
持续 100~300 ms
```

其中 `current_residual` 最好扣除空载重力/摩擦基线：

```text
current_residual = measured_current - no_load_current_baseline
```

这样可以避免手指自重、摩擦、线缆阻力被误判为接触。

## 7. 无人机上层控制接口

夹爪模块建议向上层输出：

```text
grip_state
left_pos
right_pos
left_close_ratio
right_close_ratio
center_error
left_contact
right_contact
both_contact
centered
safe_to_lift
fault
```

无人机上层控制器根据 `center_error` 生成横向速度或位置修正：

```text
uav_lateral_velocity = Kp * center_error + Kd * error_rate
```

必须做：

- 限速。
- 低通滤波。
- 死区。
- 最大移动距离限制。
- 回中超时保护。

## 8. 重力补偿

小扭矩阶段可能需要重力补偿，尤其是：

- 舵机转轴接近水平。
- 手指在竖直平面内摆动。
- 无人机俯仰/横滚变化明显。
- 低扭矩设定值与手指自重/摩擦产生的电流同量级。

如果舵机转轴竖直、手指主要在水平面内开合，重力对开合方向力矩影响较小，可能不需要复杂补偿。

建议先做空载标定：

```text
无物体
低速扫过整个开合范围
记录每个角度下的 present_current 或 present_load
分别记录左、右舵机
最好在几种姿态下记录：水平、前倾、后倾、左倾、右倾
```

得到基线：

```text
I_gravity_left(pos, attitude)
I_gravity_right(pos, attitude)
```

运行时接触检测使用残差：

```text
I_contact_left  = I_measured_left  - I_gravity_left(pos, attitude)
I_contact_right = I_measured_right - I_gravity_right(pos, attitude)
```

恒力保持时加入重力前馈：

```text
left_cmd  = left_inward_sign  * T_low + T_gravity_left
right_cmd = right_inward_sign * T_low + T_gravity_right
```

简化动力学模型：

```text
T_g = m * g * l_com * sin(theta - theta_zero)
```

再换算为舵机命令：

```text
cmd_g = T_g / K_servo
```

实际工程中更推荐查表法，因为摩擦、连杆、线缆、装配误差都会影响结果。

是否必须补偿的经验判断：

```text
空载电流峰值 > T_low 的 20%~30%：建议补偿
空载电流峰值 > T_low 的 50%：基本必须补偿
```

## 9. 安全保护

无人机夹爪必须保留软件安全边界，即使舵机寄存器里已经设置最大最小角度。

建议保护项：

- 最大电流限制。
- 最大温度限制。
- 舵机软限位。
- 通信丢失立即卸力。
- 搜索超时。
- 回中超时。
- 单边接触超时。
- 起吊前居中稳定时间。
- 扭矩 ramp，禁止阶跃加力。
- 故障状态下禁止 `safe_to_lift`。

故障处理建议：

```text
轻微异常：降到 T_low 或保持当前低扭矩
严重异常：WriteEle(0) 或 DisableTorque
通信丢失：立即卸力，由上层进入安全策略
```

## 10. 推荐实现架构

正式系统建议分层：

```text
无人机上层控制器
  - 接收 center_error
  - 移动无人机回中
  - 判断是否起吊

夹爪控制内环
  - 舵机通信
  - 状态机
  - 接触检测
  - 恒低扭矩保持
  - 重力补偿
  - 安全保护

HLS3625 舵机
  - 内部位置/速度/电流控制
```

正式飞行建议：

```text
夹爪内环：C++ 或独立控制器
上层任务逻辑/调试：Python 可用
```

原型验证可以先用 Python 快速完成状态机和日志记录；确认参数后，再移植到 C++。

## 11. 参数初始建议

以下只是起步范围，必须根据机构和被夹物实测调整。

```text
搜索速度：尽量低，先从 SDK 示例速度的 10%~30% 开始
搜索扭矩限制：从很小值开始，逐步增加到刚好能稳定接触
低保持扭矩 T_low：只需防止物体脱离，不应明显扰动无人机
起吊扭矩 T_lift：通过地面拉力实验确定
接触确认时间：100~300 ms
居中稳定时间：0.5~1.0 s
扭矩提升时间：0.5~2.0 s
控制周期：20~50 Hz 起步
```

## 12. 地面测试顺序

建议测试顺序：

```text
1. 单舵机确认正负方向
2. 单舵机确认低扭矩搜索是否能安全停止
3. 双舵机空载开合，确认左右闭合量定义
4. 空载记录电流/负载基线
5. 夹固定假物体，测试单边接触检测
6. 测试双边低扭矩保持
7. 人工移动夹爪或物体，验证 center_error 方向
8. 模拟无人机回中，验证夹爪能柔顺跟随
9. 居中后 ramp 到起吊扭矩
10. 加入所有故障保护后，再考虑上机
```

## 13. 参考资料

Feetech 官方资料入口：

- [2026 年资料汇总](http://doc.feetech.cn/#/prodinfodownload?srcType=FTServo-emanual-ab4947a479784a71a8cb7928)

说明：该链接为 Feetech 官方文档站的资料下载/资料汇总入口，可用于在其他机器上查找 HLS 系列舵机手册、SDK、调试工具和相关资料。

## 14. 一句话总结

该夹爪方案应设计为：

```text
低速低扭矩搜索
低恒扭矩柔顺夹持
用左右闭合量差估计偏心
由无人机移动回中
居中稳定后再增加夹紧力起吊
```

夹爪越柔顺，无人机越安全；夹爪越想在偏心阶段主动纠正物体，越容易把扰动力传给机体。
