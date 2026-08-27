# 单次轨迹自动采集使用说明（实时流式写入）

## 1. 写入方式与安全说明

本管线通过**实时流式写入**驱动灵巧手：ROS 节点按 `write_rate_hz`（默认 50 Hz）逐点写
`MAIN.ROS_A`（16 通道手部目标值），每次写入同时置 `CurrentJob=210` 触发手部运动，
与 `teleoperation` 节点的写入方法一致。轨迹不再预上传到 `MyTRAJ` 数组，也不使用
`CurrentJob=114` 轨迹触发。

节点默认只生成轨迹：

```yaml
stream_to_plc: false
```

默认模式不会建立 ADS 连接。`stream_to_plc:=true` 即开始真实运动（流式写入本身就是
运动），没有"只上传不触发"的中间档位。第一次实机流式必须满足：

- 急停可用；
- 操作者在现场；
- 工作空间清空；
- 使用短时、低频、小幅值轨迹；
- 明确轨迹单位（`MAIN.ROS_A` 手部目标值单位）和限位；
- 已确认 ADS 地址与 NetId 参数正确（配置默认留空，防止误连）。

连接后节点会写 `MAIN.ROSControl=true`（可用 `execution/ros_control` 关闭该动作），
使 PLC 接受 ROS 手部目标。该开关在实验结束后保持 true，不自动恢复。

## 2. 构建

在 catkin 工作空间根目录执行：

```bash
catkin_make
source devel/setup.bash
```

运行测试：

```bash
catkin_make run_tests
catkin_test_results
```

## 3. 离线生成

```bash
roslaunch twincat_talker single_experiment.launch \
  waveform:=sine \
  frequency_hz:=0.5 \
  amplitude:=1.0 \
  offset:=10.0 \
  duration_s:=10.0 \
  sample_rate_hz:=200.0 \
  experiment_name:=offline_sine
```

该模式：

- 生成 `generated_trajectory.csv`；
- 写入 `experiment.yaml`；
- 写入 `runtime.log` 和 `result.yaml`；
- 不启动 VRPN/TwinCAT 数据管线；
- 不连接 ADS；
- 不写 PLC。

## 4. 轨迹参数

支持波形：

```text
sine
triangle
square
```

主要参数：

| 参数 | 含义 |
|---|---|
| `frequency_hz` | 波形频率 |
| `amplitude` | 振幅，单位为 `MAIN.ROS_A` 手部目标值 |
| `offset` | 偏置，单位为 `MAIN.ROS_A` 手部目标值 |
| `phase_rad` | 初始相位（弧度） |
| `duration_s` | 单次轨迹时长 |
| `sample_rate_hz` | 轨迹采样率 |
| `target_joint` | 激励的手部通道，范围 0～15 |

点数为：

```text
round(duration_s × sample_rate_hz) + 1
```

点数不能超过 60001。只有 `target_joint` 指定的通道按轨迹运动，其余 15 个通道在整场
实验中保持连接时从 PLC 读到的 `MAIN.ROS_A` 当前值。

## 5. 安全校验

流式写入要求：

```text
limits_enabled=true
```

并明确提供：

```text
minimum_value
maximum_value
maximum_step
```

限位单位为 `MAIN.ROS_A` 手部目标值。任何目标点越界、相邻变化过大、出现 NaN/Inf 或
点数超限时，节点都会在 ADS 连接前拒绝执行。连接后还会回读 `MAIN.ROS_A` 校验全部
通道值有限。

## 6. 实时流式采集

先确认 ADS 地址，然后执行：

```bash
roslaunch twincat_talker single_experiment.launch \
  start_pipeline:=true \
  waveform:=sine frequency_hz:=0.2 amplitude:=0.5 offset:=10.0 \
  duration_s:=5.0 sample_rate_hz:=200.0 target_joint:=3 \
  limits_enabled:=true minimum_value:=9.0 maximum_value:=11.0 maximum_step:=0.05 \
  stream_to_plc:=true write_rate_hz:=50.0 \
  remote_ip:=<PLC_IP> remote_ams_net_id:=<PLC_NETID> local_ams_net_id:=<LOCAL_NETID> \
  experiment_name:=hand_stream_test
```

执行顺序：

```text
生成并校验
  → 连接 ADS
  → 写 ROSControl=true
  → 读取当前 MAIN.ROS_A 作为非激励通道保持值
  → 等待 /optitrack/pose1
  → 等待 /twincat/joint_states
  → 记录 PRE_ROLL
  → 按 write_rate_hz 逐点写 MAIN.ROS_A + 置 CurrentJob=210
  → 最后一点写入完成
  → 记录 POST_ROLL
  → 关闭文件
```

写入采用**采样保持**方式：每拍根据流逝时间换算轨迹索引
（`index = elapsed × sample_rate_hz`），写入频率与轨迹采样率解耦，写入节拍抖动不会
累积相位误差。轨迹播完后 `MAIN.ROS_A` 保持在最后一点，**不自动回零**。

## 7. 输出文件

默认目录：

```text
~/.ros/hand_datalogger/<时间戳>_<实验名>/
```

文件：

```text
experiment.yaml
generated_trajectory.csv
recorded_data.csv
runtime.log
result.yaml
```

`recorded_data.csv` 包含：

```text
记录时间
Pose 时间
JointState 时间
同步误差
实验状态（PRE_ROLL / STREAMING / POST_ROLL）
CurrentJob
OptiTrack XYZ 和四元数
两个目标关节位置（mcp、pip）
当拍写入的指令值（cmd_target；进入流式前为 NaN）
```

## 8. CurrentJob 的角色

`CurrentJob=210` 在每拍写入时随 `MAIN.ROS_A` 一起写给 PLC（与 teleoperation 一致），
节点运行期间只读取并记录它用于诊断，**不再依赖 CurrentJob 判断运动开始或结束**——
流式写入的起止由 ROS 侧完全掌握。

## 9. 已知限制

- 尚未接入 PLC 主动停止变量；节点超时或退出不能停止已开始的 PLC 运动；
- PLC 对 50 Hz 连续"写 ROS_A + 置 210"的响应行为尚未实机验证，首次使用必须短时、
  低幅验证；
- 尚未支持多通道独立波形和循环/分段激励；
- 轨迹播完后手部保持在最后一点，回零需另行显式操作；
- `robot.launch` 中仍使用现有硬编码 TwinCAT/VRPN 参数，实机前必须核对；
- 本功能不能替代 PLC 侧限位、速度限制和急停逻辑。
