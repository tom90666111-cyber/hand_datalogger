# 数据采集与轨迹写入流程规范化计划

## 1. 改进目标与范围

本轮改进只覆盖以下两条主流程：

1. 数据采集流程：VRPN → OptiTrack → TwinCAT → `data_logger`
2. 轨迹写入流程：轨迹文件 → 校验 → ADS 上传 → 可选触发 PLC 运动

本轮不重构遥操作、策略网络和触觉传感器控制逻辑。

主要目标：

- 提高仓库规范性；
- 提高跨机器可移植性；
- 提高源码与配置的可读性；
- 保证采集数据在时间和格式上的正确性；
- 降低轨迹写入和运动触发的误操作风险；
- 建立无需连接硬件即可运行的基础测试。

## 2. 基线运行环境

- 操作系统：Ubuntu 20.04 LTS
- ROS：ROS Noetic
- 构建系统：catkin / `catkin_make`
- C++ 标准：C++14
- PLC：Beckhoff TwinCAT 3
- ADS 端口：851（`AMSPORT_R0_PLC_TC3`）

在未确认硬件可用前，构建、解析、数据格式和 ROS 消息同步采用离线方式验证；ADS、VRPN 和机械运动在后续实机阶段单独验收。

## 3. 目标架构

### 3.1 数据采集

```text
OptiTrack/Motive
    │
    ▼
vrpn_client_ros
    │ /vrpn_client/<tracker>/pose
    ▼
optitrack_node
    │ /optitrack/pose
    ▼
data_logger ◀──────── /twincat/joint_states
                          ▲
                          │
                     twincat_node
                          ▲
                          │ ADS
                     TwinCAT PLC
```

### 3.2 轨迹写入

```text
轨迹文件
   │
   ▼
trajectory_file_writer
   │ 解析、校验、映射、预览
   ▼
ADS 写入 PLC
   │
   ▼
可选触发 CurrentJob
```

轨迹写入不加入默认数据采集启动文件，避免采集启动时意外写入 PLC 或触发机械运动。

## 4. 目标目录结构

```text
twincat_talker/
├── CMakeLists.txt
├── package.xml
├── README.md
├── LICENSE
├── config/
│   ├── twincat.yaml
│   ├── optitrack.yaml
│   ├── data_logger.yaml
│   └── trajectory_writer.yaml
├── launch/
│   ├── data_collection.launch
│   ├── trajectory_upload.launch
│   └── vrpn.launch
├── include/twincat_talker/
│   ├── ads_config.hpp
│   ├── trajectory_parser.hpp
│   └── csv_logger.hpp
├── src/
│   ├── twincat_node.cpp
│   ├── optitrack_node.cpp
│   ├── data_logger.cpp
│   ├── trajectory_file_writer.cpp
│   └── lib/
│       ├── ads_config.cpp
│       ├── trajectory_parser.cpp
│       └── csv_logger.cpp
├── data/
│   ├── recordings/
│   ├── trajectories/
│   └── output/
├── test/
│   ├── test_trajectory_parser.cpp
│   ├── test_csv_logger.cpp
│   └── data/
└── dep/
    └── ADS/
```

数据整理规则：

- 将现有 `src/JointAngleData/` 移到 `data/recordings/`；
- 将 `motor_trajectory*.txt` 和 `trajectory.txt` 移到 `data/trajectories/`；
- 将测试专用小文件放到 `test/data/`；
- 将新采集结果默认写入 `data/output/`；
- 在 `.gitignore` 中忽略 `data/output/` 和重复 ZIP；
- 使用 `git mv` 移动已有数据，保留 Git 追踪关系。

## 5. 分阶段实施计划

### 阶段 0：建立安全分支

从 `main` 创建：

```bash
git switch -c refactor/data-pipeline-standardization
```

撤销 ADS Visual Studio 工程文件中仅涉及权限位的非实质改动，确保工作区干净。不得删除 ADS 源码或许可证。

验收：

```bash
git status
```

建议提交：本阶段不单独提交纯权限恢复。

### 阶段 1：规范构建系统和依赖

修改 `CMakeLists.txt` 和 `package.xml`：

- 提高最低 CMake 版本；
- 使用 C++14；
- 完整声明 `roscpp`、`std_msgs`、`sensor_msgs`、`geometry_msgs`、`message_filters` 和 `roslib`；
- 使 `catkin_package()` 与实际依赖一致；
- 安装 `twincat_node`、`optitrack_node`、`data_logger` 和 `trajectory_file_writer`；
- 安装 `launch/` 和 `config/`；
- 将本轮范围外节点标记为实验节点，默认不构建；
- 继续使用内嵌 ADS 库，保证可复现构建。

建议提交：

```text
build: standardize catkin package dependencies and install rules
```

### 阶段 2：统一 TwinCAT 配置

新增：

```text
config/twincat.yaml
include/twincat_talker/ads_config.hpp
src/lib/ads_config.cpp
```

将以下内容从源码移入 ROS 参数：

- PLC IP；
- Remote AMS NetId；
- Local AMS NetId；
- ADS 端口；
- PLC 变量名；
- 轴数和数组索引。

示例配置：

```yaml
plc:
  remote_ip: "169.254.207.231"
  remote_ams_net_id: "127.0.0.1.2.2"
  local_ams_net_id: "192.168.10.1.1.2"
  port: 851

variables:
  joint_position: "VariableMAIN.CurrentPositionReal"
  trajectory_size: "MyTRAJ.SIZE_POS_TRAJ"
  trajectory_data: "MyTRAJ.database_read_LREAL2"
  current_job: "VariableMAIN.CurrentJob"
```

启动时验证 NetId、IP、端口和变量名，并在 ADS 路由失败时输出具体错误码。

建议提交：

```text
refactor: load ADS connection settings from ROS parameters
```

### 阶段 3：规范 OptiTrack 数据桥接

- `optitrack_node` 只处理一个可配置刚体；
- 输入和输出话题通过参数或 remap 指定；
- 默认输入为 `/vrpn_client/RigidBody1/pose`；
- 默认输出为 `/optitrack/pose`；
- 保留 VRPN 原始时间戳；
- `frame_id` 通过参数配置；
- 多刚体场景通过启动多个节点实例实现；
- 统一 `RigidBody 1`、`RigidBody1` 等命名差异。

建议提交：

```text
refactor: standardize the OptiTrack pose bridge
```

### 阶段 4：规范 TwinCAT 关节状态发布

将发布频率、数组索引、关节名称、话题名和坐标系参数化：

```yaml
joint_state:
  topic: "/twincat/joint_states"
  publish_rate: 200.0
  frame_id: "base_link"
  source_indices: [11, 12]
  names: ["mcp_joint", "pip_joint"]
```

启动时检查：

- `source_indices` 与 `names` 数量一致；
- 所有索引处于 PLC 数组范围内；
- 发布频率大于 0；
- `JointState.name` 与 `position` 长度一致；
- 不发布未初始化数据。

建议提交：

```text
refactor: parameterize TwinCAT joint-state publishing
```

### 阶段 5：重构 data_logger

使用 `message_filters::ApproximateTime` 同步：

- `/optitrack/pose`；
- `/twincat/joint_states`。

每次同步回调只写入一行，CSV 建议字段：

```csv
record_time,pose_time,joint_time,sync_error,x,y,z,qx,qy,qz,qw,mcp,pip
```

增加以下行为：

- 收到两路有效消息后才开始记录；
- 使用消息时间戳，不再将 `header.seq` 当作时间；
- 读取关节位置前检查数组长度；
- 自动创建输出目录；
- 默认文件名包含日期和时间；
- 默认拒绝覆盖已有文件；
- 每若干行或退出时刷新文件；
- SIGINT 时正常关闭文件；
- 记录两路消息的时间差；
- 使用节流日志，避免逐帧刷屏。

参数示例：

```yaml
pose_topic: "/optitrack/pose"
joint_topic: "/twincat/joint_states"
output_directory: "~/.ros/hand_datalogger"
file_prefix: "joint_pose"
sync_queue_size: 20
sync_tolerance: 0.02
flush_interval: 100
joint_indices: [0, 1]
```

建议提交：

```text
refactor: synchronize and validate pose-joint data logging
```

### 阶段 6：规范轨迹文件写入

将轨迹节点明确拆分为：

```text
parse → validate → upload → optional trigger
```

主要改进：

- ADS 配置改为 ROS 参数；
- 明确文件格式，不再自动猜测首行是否为点数；
- 校验源列、目标轴、过滤列和 PLC 容量；
- 检查空数据、NaN 和 Inf；
- 将过滤功能设为可选；
- 删除交互式 `std::cin`；
- 默认 `upload:=false`，只解析和验证；
- 默认 `trigger_motion:=false`；
- 上传前打印文件、点数、范围、目标轴和 PLC 变量；
- 上传后回读轨迹点数进行基础确认；
- 将解析逻辑提取为纯 C++ 库，支持单元测试。

参数示例：

```yaml
trajectory:
  file: ""
  format: "plain"
  source_columns: 4
  source_column: 1
  target_axis: 12
  filter_enabled: false
  filter_column: 3
  filter_start: 0.0
  filter_end: 0.0
  max_rows: 60001
  axis_count: 16
  upload: false
  trigger_motion: false
  trigger_job: 114
```

只有显式启用 `upload` 才能写 PLC，只有再次显式启用 `trigger_motion` 才能触发运动。

建议提交：

```text
refactor: validate and safely upload PLC trajectory files
```

### 阶段 7：统一启动入口

新增：

```text
launch/data_collection.launch
launch/trajectory_upload.launch
```

数据采集：

```bash
roslaunch twincat_talker data_collection.launch
```

轨迹离线检查：

```bash
roslaunch twincat_talker trajectory_upload.launch \
  trajectory_file:=/absolute/path/trajectory.txt \
  upload:=false
```

只上传、不触发：

```bash
roslaunch twincat_talker trajectory_upload.launch \
  trajectory_file:=/absolute/path/trajectory.txt \
  upload:=true \
  trigger_motion:=false
```

触发运动必须额外显式指定：

```bash
trigger_motion:=true
```

建议提交：

```text
feat: add unified data collection and trajectory launch files
```

### 阶段 8：整理数据与文档

使用 `git mv` 将实验记录和轨迹移出源码目录。更新 README，加入：

- 系统架构；
- 环境与依赖；
- TwinCAT 路由配置；
- VRPN 刚体配置；
- 一键数据采集；
- CSV 字段说明；
- 轨迹检查、上传和触发流程；
- 常见 ADS 错误；
- 无硬件离线验证；
- PLC 运动安全警告。

建议提交：

```text
docs: document reproducible data collection and trajectory workflows
```

### 阶段 9：离线测试

增加轨迹解析和 CSV 记录测试，包括：

- 纯数字轨迹格式；
- 首行点数格式；
- 空文件；
- 非法列数；
- NaN/Inf；
- 索引越界；
- 过滤起止值不存在；
- PLC 容量超限；
- JointState 数组长度不足；
- CSV 表头和列数；
- 时间同步误差计算。

ROS 离线测试通过模拟发布 `/optitrack/pose` 和 `/twincat/joint_states`，验证 logger 只在同步后写入数据。

验收命令：

```bash
catkin_make
catkin_make run_tests
catkin_test_results
```

建议提交：

```text
test: add offline tests for logging and trajectory parsing
```

## 6. 实机验收

### 6.1 OptiTrack

```bash
rostopic hz /vrpn_client/RigidBody1/pose
rostopic hz /optitrack/pose
rostopic echo -n 1 /optitrack/pose
```

检查话题名称、时间戳、`frame_id` 和输入输出频率。

### 6.2 TwinCAT 读取

```bash
rostopic hz /twincat/joint_states
rostopic echo -n 1 /twincat/joint_states
```

检查 ADS 路由、关节名称与位置数量、MCP/PIP 映射和发布频率。

### 6.3 数据采集

检查：

- 启动初期不存在未初始化全零记录；
- 每行列数一致；
- 时间戳单调递增；
- 同步误差在设定容差内；
- 退出后文件完整；
- 输出目录正确。

### 6.4 轨迹写入

按以下顺序逐步验收：

1. 只解析；
2. 只上传，不触发；
3. 回读轨迹点数；
4. PLC 处于安全状态时触发；
5. 在限速、限位条件下进行实机运动。

## 7. Git 工作方式

总改进分支：

```text
refactor/data-pipeline-standardization
```

每个阶段使用独立、可审查的原子提交。离线测试和必要的实机验收通过后，再合并到 `main`。

实施期间不得：

- 强制推送；
- 改写已有历史；
- 删除原始实验数据；
- 把 ADS 权限噪声混入提交；
- 在未显式确认时触发 PLC 运动；
- 将本轮范围外的遥操作和策略代码混入重构。

## 8. 完成标准

- 新机器按照 README 能完成构建；
- IP、AMS NetId、话题、频率和轴索引无需修改源码；
- 一条命令可以启动数据采集；
- OptiTrack 位姿和关节角按时间同步记录；
- CSV 格式明确且可验证；
- 实验数据与源码分离；
- 轨迹先离线验证，再明确上传，最后显式触发；
- 所有正式节点均有安装规则；
- 核心解析和记录逻辑具有离线测试；
- `main` 分支保持可构建；
- 内嵌 ADS 库继续保证依赖可复现。
