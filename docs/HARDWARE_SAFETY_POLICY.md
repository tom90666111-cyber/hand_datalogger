# 实机安全变更策略

## 目的

本策略用于约束数据采集与轨迹写入流程的后续重构，避免未经验证的变更直接影响 TwinCAT PLC、机械执行机构或实验数据。

## 变更等级

### S0：零运行时影响

只允许修改：

- Markdown 文档；
- Git 配置文件；
- 编辑器和代码风格配置；
- 不参与构建、启动和运行的说明文件。

S0 禁止修改：

- `src/`、`include/` 和 `dep/ADS/` 中的源代码；
- `CMakeLists.txt` 和 `package.xml`；
- `launch/` 和 `config/`；
- PLC IP、AMS NetId、PLC 变量名、任务号和数组索引；
- ROS 节点名、话题名、消息类型和发布频率；
- 轨迹及实验数据内容。

### S1：离线可验证，不允许连接实机

包括纯解析库、CSV 格式处理、参数校验和单元测试。S1 变更必须满足：

- 默认路径不会创建 ADS 连接；
- 默认路径不会写 PLC；
- 默认路径不会触发运动；
- 可在无 TwinCAT、无 OptiTrack 环境中完成测试；
- 通过代码审查和离线测试后才能进入 S2。

### S2：只读实机验证

只允许：

- 建立 ADS 路由；
- 读取 PLC 状态和关节位置；
- 订阅 VRPN/OptiTrack；
- 发布 ROS 状态话题；
- 将接收数据写入本地日志。

S2 禁止写入任何 PLC 变量或触发任务号。

### S3：PLC 写入但不触发运动

允许上传轨迹数据和点数，但必须同时满足：

- `trigger_motion` 默认且实际为 `false`；
- PLC 处于约定的安全状态；
- 上传前完成范围、长度、NaN/Inf 和索引检查；
- 上传后完成必要回读；
- 操作者明确批准本次实机写入。

### S4：实机运动

只有在 S0～S3 验收完成后才能执行。必须由操作者明确批准，并确认：

- 急停可用；
- 机械臂工作空间清空；
- 关节限位和速度限制生效；
- PLC 状态正确；
- 轨迹已经离线检查并完成只写不触发测试；
- 操作者在现场监控。

## 每次提交的安全审计

提交前执行：

```bash
git status --short
git diff --check
git diff --name-only
git diff --cached --name-only
```

S0 提交中出现以下路径时必须停止：

```text
CMakeLists.txt
package.xml
src/
include/
launch/
config/
dep/ADS/
```

## Git 约束

- 所有规范化工作在 `refactor/data-pipeline-standardization` 分支完成；
- 不强制推送；
- 不改写 `main` 历史；
- 每个安全等级使用独立提交；
- 实机验证结果记录在提交说明或合并请求中；
- 未通过对应等级验收的变更不得合并到 `main`。

## 当前阶段

当前阶段为 **S0：零运行时影响**。

本阶段仅建立计划、代码风格规则和安全策略，不修改任何 ROS、ADS、PLC 或数据处理行为。
