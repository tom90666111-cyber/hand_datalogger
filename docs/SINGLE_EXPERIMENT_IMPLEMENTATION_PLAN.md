# 单次轨迹自动采集实施计划

## 1. 目标

实现单次有限长度轨迹的自动生成、校验、上传、执行和数据采集闭环：

```text
读取实验参数
  → 自动生成单段轨迹
  → 保存轨迹文件
  → 安全检查
  → 等待 OptiTrack 和关节角就绪
  → 写入 TwinCAT
  → 显式触发 CurrentJob=114
  → 同步记录实验数据
  → CurrentJob!=114 时自动结束
  → 保存结果和实验元数据
```

第一版暂不处理轨迹循环、分段上传和一小时连续运行。先可靠完成一次轨迹实验，为后续长时间自动采集建立基础。

## 2. 预期使用方式

```bash
roslaunch twincat_talker single_experiment.launch \
  waveform:=sine \
  frequency_hz:=0.5 \
  amplitude:=10.0 \
  offset:=20.0 \
  duration_s:=20.0 \
  sample_rate_hz:=200.0 \
  target_axis:=12 \
  upload_to_plc:=true \
  trigger_motion:=true
```

自动执行：

```text
生成轨迹
  → 校验轨迹
  → 保存轨迹文件
  → 等待 OptiTrack 和关节角就绪
  → 读取其他轴当前位置
  → 上传轨迹
  → 启动同步记录
  → 记录 pre-roll
  → 写 CurrentJob=114
  → 监控 CurrentJob
  → CurrentJob!=114
  → 记录 post-roll
  → 关闭文件并保存结果
```

## 3. 第一版功能边界

### 支持

- 单次轨迹；
- 单个目标轴；
- 正弦波、三角波和方波；
- 振幅、频率、偏置、相位、时长和采样率参数；
- 自动生成轨迹文件；
- ADS 上传；
- `CurrentJob=114` 触发；
- 根据 `CurrentJob!=114` 自动停止记录；
- 同步记录 OptiTrack 与关节角；
- 运动前后短时间记录；
- 自动生成实验元数据。

### 暂不支持

- 轨迹循环；
- 超过 60001 点的轨迹；
- 分段上传；
- 多轴不同波形组合；
- 断点续传；
- PLC 自动急停；
- 一小时连续运行。

## 4. 程序结构

新增三个可复用模块。

### 4.1 轨迹生成器

```text
trajectory_generator
```

职责：

- 根据参数生成目标轴轨迹；
- 检查参数和数值；
- 保存生成的轨迹文件；
- 不连接 ROS 硬件和 ADS；
- 支持离线单元测试。

计划文件：

```text
include/twincat_talker/trajectory_generator.hpp
src/lib/trajectory_generator.cpp
```

### 4.2 ADS 轨迹上传器

```text
trajectory_uploader
```

职责：

- 解析 AMS NetId；
- 建立 ADS 路由；
- 读取当前 16 轴位置；
- 填充 PLC 轨迹数组；
- 上传数据和点数；
- 回读点数验证；
- 显式触发 `CurrentJob=114`；
- 读取并监控 `CurrentJob`。

计划文件：

```text
include/twincat_talker/trajectory_uploader.hpp
src/lib/trajectory_uploader.cpp
```

### 4.3 单次实验管理器

```text
single_experiment_manager
```

职责：

- 编排生成、上传、记录和结束；
- 同步订阅 OptiTrack 和关节状态；
- 管理实验状态；
- 根据 `CurrentJob` 停止；
- 保存 CSV、日志和结果。

计划文件：

```text
src/single_experiment_manager.cpp
```

现有 `trajectory_file_writer.cpp` 暂时保留，不破坏原有手工使用方式。

## 5. 自动实验状态机

```text
IDLE
  → GENERATING
  → VALIDATING
  → WAITING_FOR_DATA
  → LOGGER_READY
  → UPLOADING
  → PRE_ROLL
  → TRIGGERING
  → RUNNING
  → POST_ROLL
  → COMPLETED
```

任一步失败进入：

```text
FAILED
  → 停止 ROS 侧记录
  → 刷新并关闭文件
  → 保存失败原因
  → 不继续执行后续动作
```

由于当前没有确认 PLC 停止变量，第一版发生超时或 ROS/ADS 异常时，只能停止 ROS 侧流程，不能保证主动停止已经开始的 PLC 运动。首次实机运行必须采用短时、低频和低幅值轨迹。

## 6. 轨迹参数与生成规则

参数示例：

```yaml
trajectory:
  waveform: "sine"
  frequency_hz: 0.5
  amplitude: 10.0
  offset: 20.0
  phase_rad: 0.0
  duration_s: 20.0
  sample_rate_hz: 200.0
  target_axis: 12
```

点数计算：

```text
point_count = round(duration_s × sample_rate_hz) + 1
```

轨迹时间：

```text
t[i] = i / sample_rate_hz
```

正弦波：

```text
offset + amplitude × sin(2π × frequency × t + phase)
```

三角波：

```text
offset + amplitude × 2/π × asin(sin(2πft + phase))
```

方波：

```text
offset + amplitude × sign(sin(2πft + phase))
```

生成文件格式：

```csv
index,time,target
0,0.000000,20.000000
1,0.005000,20.157054
```

## 7. 轨迹安全检查

上传前必须通过：

- `duration_s > 0`；
- `sample_rate_hz > 0`；
- `frequency_hz > 0`；
- `amplitude >= 0`；
- `target_axis` 位于 `[0,15]`；
- 点数不超过 `60001`；
- 所有数值均为有限数；
- 不包含 NaN 或 Inf；
- 所有目标值处于指定限位；
- 相邻点变化量不超过允许值；
- ADS 变量名不能为空；
- 生成的轨迹文件成功写入。

安全配置：

```yaml
safety:
  limits_enabled: true
  minimum_value: 0.0
  maximum_value: 30.0
  maximum_step: 0.5
```

当启用 PLC 上传时，`limits_enabled` 必须为 `true`，否则拒绝上传。

第一版不自动推断单位。振幅、偏置、限位和最大步长均使用 PLC 当前轨迹数组的实际单位。

## 8. 非目标轴处理

不沿用其他 15 个轴全部写 0 的高风险方式。

默认模式：

```text
non_target_axis_mode = hold_current
```

上传前读取：

```text
VariableMAIN.CurrentPositionReal
```

填充规则：

```text
目标轴：写入生成轨迹
其他轴：每一个轨迹点均写入上传前读取的当前位置
```

PLC 变量名保持可配置：

```yaml
plc_variables:
  current_position: "VariableMAIN.CurrentPositionReal"
  trajectory_data: "MyTRAJ.database_read_LREAL2"
  trajectory_size: "MyTRAJ.SIZE_POS_TRAJ"
  current_job: "VariableMAIN.CurrentJob"
```

如果当前位置与轨迹数组单位不一致，实机上传阶段必须停止并重新确认。

## 9. 数据记录规则

同步订阅：

```text
/optitrack/pose1
/twincat/joint_states
```

使用：

```cpp
message_filters::ApproximateTime
```

只有两路消息均有效且成功配对时才写入一行。

CSV 字段：

```csv
record_index,
record_time,
pose_time,
joint_time,
sync_error,
experiment_elapsed,
experiment_state,
current_job,
x,y,z,
qx,qy,qz,qw,
mcp,pip
```

记录阶段：

```text
PRE_ROLL
RUNNING
POST_ROLL
```

不会在节点刚启动、消息尚未就绪时写入未初始化值。

### 9.1 pre-roll

`pre-roll` 是正式运动开始前的基线记录。例如 `pre_roll_s=2.0` 表示先记录 2 秒静止状态，再写入 `CurrentJob=114`。

用途：

- 保存初始关节角；
- 保存 OptiTrack 初始位姿；
- 检查传感器稳定性；
- 为后续分析提供运动前基线。

### 9.2 post-roll

`post-roll` 是检测到 `CurrentJob!=114` 后继续记录的一小段时间，用于保存运动停止后的稳定状态。

如不需要，可配置：

```yaml
pre_roll_s: 0.0
post_roll_s: 0.0
```

## 10. CurrentJob 结束判断

已确认：

```text
CurrentJob == 114：轨迹运行中
CurrentJob != 114：轨迹已经结束
```

只有确认至少读取到一次 `114` 后，才允许判断完成：

```cpp
if (current_job == 114) {
    running_confirmed = true;
    completion_count = 0;
} else if (running_confirmed) {
    ++completion_count;
}

if (completion_count >= completion_debounce_count) {
    finishExperiment();
}
```

默认参数：

```yaml
execution:
  trigger_job: 114
  current_job_poll_rate_hz: 50.0
  completion_debounce_count: 3
  start_timeout_s: 5.0
  maximum_runtime_s: 600.0
  pre_roll_s: 2.0
  post_roll_s: 2.0
```

异常规则：

- 写入 114 后一直未读到 114：启动失败；
- 已确认 114，随后连续读到非 114：正常完成；
- 一直保持 114 并超过最大运行时间：超时失败；
- ADS 读取异常：停止 ROS 记录并保存错误；
- 最终非 114 的具体数值写入结果文件；
- 预计轨迹时长只用于参数检查和超时参考，不作为正常结束条件。

## 11. 双重实机开关

默认：

```yaml
execution:
  upload_to_plc: false
  trigger_motion: false
```

### 模式 A：离线生成

```text
upload_to_plc=false
trigger_motion=false
```

- 生成轨迹；
- 执行安全检查；
- 保存文件；
- 不连接 ADS。

### 模式 B：上传但不运动

```text
upload_to_plc=true
trigger_motion=false
```

- 连接 ADS；
- 读取当前轴位置；
- 上传轨迹；
- 回读轨迹点数；
- 不写 `CurrentJob=114`。

### 模式 C：完整自动实验

```text
upload_to_plc=true
trigger_motion=true
```

完整执行上传、记录、触发和自动结束。

如果 `trigger_motion=true` 且 `upload_to_plc=false`，节点必须拒绝启动。

## 12. 实验输出结构

```text
data/output/
└── 20260716_153000_pip_sine_test/
    ├── experiment.yaml
    ├── generated_trajectory.csv
    ├── recorded_data.csv
    ├── runtime.log
    └── result.yaml
```

结果示例：

```yaml
status: completed
generated_points: 4001
uploaded_points: 4001
expected_duration_s: 20.0
recorded_rows: 4750
running_confirmed: true
trigger_job: 114
final_current_job: 0
maximum_sync_error_s: 0.008
failure_reason: ""
```

`data/output/` 加入 `.gitignore`，避免采集结果被意外提交。

## 13. 配置与启动文件

新增：

```text
config/single_experiment.yaml
launch/single_experiment.launch
```

常用 launch 参数：

```xml
<arg name="waveform" default="sine"/>
<arg name="frequency_hz" default="0.5"/>
<arg name="amplitude" default="1.0"/>
<arg name="offset" default="0.0"/>
<arg name="duration_s" default="10.0"/>
<arg name="sample_rate_hz" default="200.0"/>
<arg name="target_axis" default="12"/>
<arg name="upload_to_plc" default="false"/>
<arg name="trigger_motion" default="false"/>
```

默认启动不得连接、上传或触发实机。

## 14. 具体文件计划

### 新增

```text
include/twincat_talker/trajectory_generator.hpp
include/twincat_talker/trajectory_uploader.hpp
src/lib/trajectory_generator.cpp
src/lib/trajectory_uploader.cpp
src/single_experiment_manager.cpp
config/single_experiment.yaml
launch/single_experiment.launch
test/test_trajectory_generator.cpp
test/test_current_job_monitor.cpp
docs/SINGLE_EXPERIMENT_WORKFLOW.md
```

### 修改

```text
CMakeLists.txt
package.xml
.gitignore
README.md
```

### 第一版尽量不修改

```text
src/trajectory_file_writer.cpp
src/data_logger.cpp
src/twincat_node.cpp
src/optitrack_node.cpp
```

原有手工流程保持不变，新自动流程作为独立入口存在。自动实验节点直接订阅现有 `/optitrack/pose1` 和 `/twincat/joint_states`。

## 15. 实施与提交顺序

### 提交 1：纯轨迹生成

```text
feat: add validated one-shot trajectory generation
```

- 轨迹生成库；
- 波形参数；
- 安全校验；
- CSV 输出；
- 离线单元测试；
- 不涉及 ADS 和 ROS 实机。

### 提交 2：同步采集模块

```text
feat: add synchronized one-shot experiment logging
```

- OptiTrack/关节角同步；
- CSV 写入；
- pre-roll/post-roll；
- 数据就绪判断；
- 只读 ROS 话题，不写 PLC。

### 提交 3：受保护的 ADS 上传

```text
feat: add guarded TwinCAT trajectory uploading
```

- 读取当前轴位置；
- 非目标轴保持当前位置；
- 上传轨迹；
- 回读点数；
- 双重实机开关；
- CurrentJob 监控；
- 默认不上传、不触发。

### 提交 4：实验状态机

```text
feat: automate one-shot trajectory data collection
```

- 完整执行顺序；
- CurrentJob 自动结束；
- 超时和错误处理；
- 结果文件。

### 提交 5：启动和文档

```text
docs: add the one-shot automated experiment workflow
```

- launch；
- YAML；
- README；
- 实机分级操作说明。

## 16. 验证与实机准入

### S1：离线验证

- 编译；
- 三种波形生成测试；
- 参数非法测试；
- 点数上限测试；
- 限位测试；
- 最大步长测试；
- CurrentJob 状态机测试；
- 确认默认模式不建立 ADS 连接。

### S2：模拟 ROS 话题

发布模拟 `/optitrack/pose1` 和 `/twincat/joint_states`，检查：

- 两路数据就绪；
- 消息同步写入；
- CSV 字段；
- pre-roll、running、post-roll 状态；
- 自动结束。

### S3：PLC 只上传

```text
upload_to_plc=true
trigger_motion=false
```

检查：

- ADS 连接；
- 当前轴读取；
- 非目标轴填充值；
- 轨迹点数；
- 未写 `CurrentJob=114`。

### S4：短轨迹实机

必须使用：

- 5 秒以内；
- 低频；
- 小幅值；
- 明确限位；
- 急停可用；
- 人员现场监控。

短轨迹验证成功后，才逐步增加时长和幅值。

## 17. 硬约束

实施过程中必须保证：

- 不修改已有手工节点行为；
- 新自动流程默认不连接、不上传、不触发；
- 不以预计时长作为正常结束条件；
- 以确认运行后的 `CurrentJob!=114` 结束；
- 非目标轴不默认写零；
- 未提供限位时禁止 PLC 上传；
- 未确认两路采集数据就绪时禁止触发；
- 本开发环境不执行真实 PLC 写入和运动测试；
- 每个阶段独立提交并审查，上一阶段未通过不进入下一阶段。
