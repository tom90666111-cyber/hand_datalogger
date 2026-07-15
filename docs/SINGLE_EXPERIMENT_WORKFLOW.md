# 单次轨迹自动采集使用说明

## 1. 安全说明

新节点默认只生成轨迹：

```yaml
upload_to_plc: false
trigger_motion: false
```

默认模式不会建立 ADS 连接。不要跳过“离线生成 → 只上传 → 短轨迹触发”的验证顺序。

当前没有配置 PLC 停止变量。ROS 节点超时或退出不能保证停止已经开始的 PLC 运动。第一次触发必须满足：

- 急停可用；
- 操作者在现场；
- 工作空间清空；
- 使用短时、低频、小幅值轨迹；
- 明确轨迹单位和限位；
- 已验证非目标轴保持值与 PLC 轨迹数组单位一致。

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
| `amplitude` | 振幅，单位与 PLC 轨迹数组一致 |
| `offset` | 偏置，单位与 PLC 轨迹数组一致 |
| `phase_rad` | 初始相位（弧度） |
| `duration_s` | 单次轨迹时长 |
| `sample_rate_hz` | 轨迹采样率 |
| `target_axis` | PLC 轨迹数组目标轴，范围 0～15 |

点数为：

```text
round(duration_s × sample_rate_hz) + 1
```

点数不能超过 60001。

## 5. 安全校验

PLC 上传要求：

```text
limits_enabled=true
```

并明确提供：

```text
minimum_value
maximum_value
maximum_step
```

任何目标点越界、相邻变化过大、出现 NaN/Inf 或点数超限时，节点都会在 ADS 连接前拒绝执行。

## 6. 只上传，不触发

先确认 `config/single_experiment.yaml` 中的 ADS 地址和 PLC 变量名与实际工程一致，然后执行：

```bash
roslaunch twincat_talker single_experiment.launch \
  waveform:=sine frequency_hz:=0.2 amplitude:=0.5 offset:=10.0 \
  duration_s:=5.0 sample_rate_hz:=200.0 target_axis:=12 \
  limits_enabled:=true minimum_value:=9.0 maximum_value:=11.0 maximum_step:=0.05 \
  upload_to_plc:=true trigger_motion:=false \
  experiment_name:=upload_only_test
```

该模式会：

1. 读取 `VariableMAIN.CurrentPositionReal`；
2. 目标轴使用生成轨迹；
3. 其他 15 个轴使用上传前当前位置；
4. 写入 `MyTRAJ.database_read_LREAL2`；
5. 写入并回读 `MyTRAJ.SIZE_POS_TRAJ`；
6. 不写 `VariableMAIN.CurrentJob`。

## 7. 完整自动采集

在只上传验证通过后运行：

```bash
roslaunch twincat_talker single_experiment.launch \
  start_pipeline:=true \
  waveform:=sine frequency_hz:=0.2 amplitude:=0.5 offset:=10.0 \
  duration_s:=5.0 sample_rate_hz:=200.0 target_axis:=12 \
  limits_enabled:=true minimum_value:=9.0 maximum_value:=11.0 maximum_step:=0.05 \
  upload_to_plc:=true trigger_motion:=true \
  experiment_name:=short_motion_test
```

执行顺序：

```text
生成并校验
  → 上传并回读点数
  → 等待 /optitrack/pose1
  → 等待 /twincat/joint_states
  → 记录 PRE_ROLL
  → 写 CurrentJob=114
  → 确认读取到 CurrentJob==114
  → 记录 RUNNING
  → 连续检测 CurrentJob!=114
  → 记录 POST_ROLL
  → 关闭文件
```

## 8. 输出文件

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
实验状态
CurrentJob
OptiTrack XYZ 和四元数
两个目标关节位置
```

## 9. CurrentJob 判断

已采用以下规则：

```text
CurrentJob==114：运行中
CurrentJob!=114：结束
```

为避免触发前的非 114 状态被误判为完成，必须先确认至少一次 `CurrentJob==114`。完成状态默认连续确认 3 次。

如果触发后一直未读到 114，节点按启动超时失败；如果长时间保持 114，则按最大运行时间超时失败。

## 10. 已知限制

- 尚未接入 PLC 主动停止变量；
- 尚未支持循环和分段上传；
- 尚未支持多轴独立波形；
- 轨迹执行结束只依据 `CurrentJob`；
- `robot.launch` 中仍使用现有硬编码 TwinCAT/VRPN 参数，实机前必须核对；
- 本功能不能替代 PLC 侧限位、速度限制和急停逻辑。
