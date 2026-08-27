# ROS 与 Beckhoff TwinCAT PLC 通信接口 :robot: :rotating_light:

## :warning: 仍在开发中 :warning:

## 用途

本 ROS 功能包可通过 ROS 话题读写运行在 Beckhoff TwinCAT PLC 上的变量。

参考链接：
- [Beckhoff ADS 库](https://infosys.beckhoff.com/content/1033/tc3_ads_intro/index.html)
- [Beckhoff ADS GitHub](https://github.com/Beckhoff/ADS)

## 安装

将工程克隆至 `catkin_ws/src` 目录下：

```bash
git clone https://github.com/tom90666111-cyber/hand_datalogger.git
```

构建工程：

```bash
catkin_make
```

## 常用使用流程

典型的数据采集流程如下：

### 1. 启动 VRPN 客户端（动作捕捉）

```bash
roslaunch twincat_talker vrpn.launch
```

启动 `vrpn_client_node`，连接 OptiTrack 动作捕捉系统，发布 `/vrpn_client/RigidBody1/pose` 话题。

### 2. 启动 TwinCAT ADS 通信节点

```bash
rosrun twincat_talker twincat_node
```

通过 ADS 协议从 TwinCAT PLC 读取关节位置数据，发布 `/twincat/joint_states` 话题。

> **注意**：运行前请确认 `twincat_node.cpp` 中的 ADS 连接参数（IP、NetId）与目标 PLC 一致。

### 3. 启动数据记录节点

```bash
rosrun twincat_talker data_logger data/recordings/ori_data1.txt
```

同时订阅 VRPN 位姿（`/vrpn_client/RigidBody1/pose`）和关节角（`/twincat/joint_states`），将同步数据以 CSV 格式写入指定文件。

### 4.（可选）启动 OptiTrack 转发节点

```bash
rosrun twincat_talker optitrack_node
```

将 VRPN 原始位姿话题重新封装并发布为 `/optitrack/pose1` 和 `/optitrack/pose2`。仅在后续需要消费 `/optitrack/pose*` 话题时启动，普通数据采集无需此步骤。

### 流程依赖关系

```
vrpn.launch ──▶ /vrpn_client/RigidBody1/pose ──▶ optitrack_node ──▶ /optitrack/pose*
                    │
                    └──▶ data_logger ──▶ CSV 文件
                            ▲
twincat_node ──▶ /twincat/joint_states
```

## 使用说明

在 `main.cc` 中修改 AMS NetId 为 PLC 的 NetId：

```cpp
static const AmsNetId remoteNetId{x, x, x, x, x, x};
```

修改 `remoteIpV4` 为 PLC 的 IP 地址：

```cpp
static const remoteIpV4[] = "x.x.x.x";
```

使用上述变量及正确端口号初始化 ADS 设备：

```cpp
AdsDevice route{remoteIpV4, remoteNetId, AMSPORT_R0_PLC_TC3};
```

添加同名变量 `NAME` 用于操作：

```cpp
AdsVariable<double> variable_to_write{route, "NAME"};
```

即可读写变量：

```cpp
variable_to_write = 12.0;
```

## 数据类型对照

| 数据类型  | Beckhoff TC3 | C++       |
| --------- | ------------ | --------- |
| 布尔      | BOOL         | `bool`    |
| 字节      | BYTE         | `byte`    |
| 短整型    | INT          | `int32_t` |
| 长整型    | LINT         | `int64_t` |
| 浮点      | REAL         | `float`   |
| 双精度    | LREAL        | `double`  |

---

## src 目录文件说明

### 已在 CMakeLists.txt 中注册的可执行节点

#### `twincat_node.cpp`
- **用途**：通过 ADS 从 PLC 读取关节位置并发布为 ROS 话题
- **订阅**：无
- **发布**：`/twincat/joint_states`（`sensor_msgs/JointState`）
- **依赖 ADS**：是
- **使用方式**：`rosrun twincat_talker twincat_node`
- **注意**：当前默认连接 `169.254.207.231`，发布 2 个关节（axis[11], axis[12]），需根据实际需求修改 IP 和关节索引

#### `main.cc`
- **用途**：ADS 连接诊断与 PLC 状态读取节点
- **订阅**：无
- **发布**：无（仅 ROS_INFO 日志输出）
- **依赖 ADS**：是
- **使用方式**：`rosrun twincat_talker twincat_talker`
- **功能**：初始化 ADS 路由并打印本地 NetId、测试 PLC 连接状态、周期性读取 `CurrentPositionReal` 变量并输出日志

#### `TCwrite.cpp`
- **用途**：交互式关节目标值写入工具
- **订阅**：无
- **发布**：无
- **依赖 ADS**：是
- **使用方式**：`rosrun twincat_talker TCwrite`
- **功能**：终端中手动输入关节索引（0-15）和目标值，写入 `MAIN.ROS_A` 数组，并可选择触发运动（`CurrentJob=210`）

#### `trajectory_file_writer.cpp`
- **用途**：从文本文件加载轨迹数据并写入 PLC 的轨迹数组
- **订阅**：无
- **发布**：无
- **依赖 ADS**：是
- **使用方式**：
  ```bash
  rosrun twincat_talker trajectory_file_writer _file:=motor_trajectory_real.txt
  ```
- **功能**：解析 CSV/空格分隔的轨迹文件，支持按标志列过滤区间，提取指定列写入 PLC `MyTRAJ.database_read_LREAL2` 数组，可触发运动（`CurrentJob=114`）

#### `optitrack_node.cpp`
- **用途**：VRPN 位姿数据转发节点
- **订阅**：`/vrpn_client/RigidBody1/pose`、`/vrpn_client/RigidBody2/pose`（`geometry_msgs/PoseStamped`）
- **发布**：`/optitrack/pose1`、`/optitrack/pose2`（`geometry_msgs/PoseStamped`）
- **依赖 ADS**：否
- **使用方式**：`rosrun twincat_talker optitrack_node`
- **注意**：内部队列长度为 1，始终处理最新帧，覆盖时间戳并统一 frame_id 为 `world`

#### `data_logger.cpp`
- **用途**：同步记录位姿和关节角数据
- **订阅**：`/vrpn_client/RigidBody1/pose`（`geometry_msgs/PoseStamped`）、`/twincat/joint_states`（`sensor_msgs/JointState`）
- **发布**：无
- **依赖 ADS**：否
- **使用方式**：
  ```bash
  rosrun twincat_talker data_logger data/recordings/ori_data1.txt
  ```
- **功能**：以 100Hz 循环，将时间戳、XYZ 位置、四元数姿态、MCP/PIP 关节角写入指定 CSV 文件，每行立即刷盘

#### `sensor_read_five_tips.cpp`
- **用途**：五指触觉传感器数据采集节点
- **订阅**：无
- **发布**：`sensor_read_cpp`（`std_msgs/Int32MultiArray`，1095 个 int 值 = 5 指 × 219 个传感器读数）
- **依赖 ADS**：否
- **使用方式**：`rosrun twincat_talker sensor_read_five_tips`
- **注意**：通过串口（默认 `/dev/ttyS3`，备选 `/dev/ttyACM2`）以 460800 波特率与触觉传感器通信，端口号和设备路径可能需要根据实际硬件调整

### 未在 CMakeLists.txt 中注册的节点（需手动编译或添加构建规则）

#### `teleoperation.cpp`
- **用途**：单臂（7-DOF）+ 灵巧手（16 腱绳）遥操作主控节点
- **订阅**：
  - `/move_group/fake_controller_joint_states` — MoveIt 规划的关节状态
  - `/move_group/real_time_mode` — 实时模式开关
  - `left_hand_data` — 遥操作手套手部数据（16 个弧度值）
- **发布**：`/joint_states`（`sensor_msgs/JointState`
- **依赖 ADS**：是
- **核心功能**：
  - 定时器回调（50Hz）从 PLC 读取 `VariableMAIN.Joint_Position` 并发布关节状态
  - 提供 MoveIt actionlib 服务器（`gen3_rightarm_controller/follow_joint_trajectory`），接收并执行运动轨迹
  - 手部遥操作：将手套弧度值经腱绳映射转换为电机指令，通过 `MAIN.ROS_A` 写入 PLC
  - 包含非线性肘关节映射（motor2theta / theta2motor）和多手指标定常数
- **依赖**：需 `../include/math.hpp`（包含 `real2rad`、`rad2real`、`motor2theta`、`theta2motor`、`real2encoder`、`convertTrajectoryToVector` 等转换函数，该文件不在本仓库中）

#### `policy_bridge.cpp`
- **用途**：双臂遥操作节点（`teleoperation.cpp` 的双臂升级版）
- **订阅**：
  - `/move_group/fake_controller_joint_states` — MoveIt 规划
  - `/move_group/real_time_mode` — 实时模式
  - `/predicted_actions_hand` — ML 策略输出的手部动作（16 个弧度值）
- **发布**：
  - `/joint_states` — 双臂关节状态（14 个臂关节 + 40 个手部关节展开值）
  - `/all_joint_angle` — 完整关节角度
- **依赖 ADS**：是
- **核心功能**：
  - 支持左右双臂独立 MoveIt 轨迹执行（`gen3_leftarm_controller/follow_joint_trajectory`）
  - 手部数据 5 秒渐入插值，避免阶跃冲击
  - 含 9 窗口滑动平均滤波器，对编码器位置做平滑
  - 手部关节展开：16 根腱绳 → 40 个 URDF 关节状态输出（含指尖角度计算）
  - 关节限位保护（硬编码阈值）
- **依赖**：同 `teleoperation.cpp`，需 `../include/math.hpp`

## 单次轨迹自动采集（实时流式写入）

`single_experiment_manager` 根据少量参数生成一段单通道手部轨迹，并按顺序完成安全校验、实时流式写入 PLC 以及 OptiTrack/关节角同步记录。写入方式与 `teleoperation` 节点一致：按 `write_rate_hz`（默认 50 Hz）逐点写 `MAIN.ROS_A` 并置 `CurrentJob=210`，不使用 `MyTRAJ` 轨迹数组和 `CurrentJob=114`。

### 默认安全模式

以下命令只生成和校验轨迹，不建立 ADS 连接：

```bash
roslaunch twincat_talker single_experiment.launch \
  waveform:=sine \
  frequency_hz:=0.5 \
  amplitude:=1.0 \
  offset:=10.0 \
  duration_s:=10.0 \
  sample_rate_hz:=200.0
```

输出默认位于：

```text
~/.ros/hand_datalogger/<时间戳>_<实验名>/
```

包含生成轨迹、实际参数、运行日志和结果文件。

### 实时流式采集（真实运动）

`stream_to_plc:=true` 即开始流式写入，**没有"只上传不触发"的中间档位**。必须在命令中显式提供安全限位（单位为 `MAIN.ROS_A` 手部目标值）和 ADS 连接参数（默认留空，防止误连）：

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

执行流程：连接 ADS → 写 `ROSControl=true` → 读当前 `MAIN.ROS_A` 作为非激励通道保持值 → 等待同步数据 → PRE_ROLL → 按 50 Hz 逐点写入（每拍同时置 210）→ 最后一点 → POST_ROLL。轨迹播完后手部保持在最后一点，不自动回零；结束判断在 ROS 侧，不依赖 `CurrentJob`（仅记录用于诊断）。`recorded_data.csv` 中 `cmd_target` 列记录每拍写入的指令值。

详细参数、安全限制和已知限制见：

```text
docs/SINGLE_EXPERIMENT_WORKFLOW.md
docs/HARDWARE_SAFETY_POLICY.md
```
