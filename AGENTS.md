# 工程协作入口

开始处理本工程前，必须先阅读
[`docs/project_handoff.md`](docs/project_handoff.md)，再执行 `git status --short --branch`
和 `git log -5 --oneline --decorate`，以恢复当前进度并确认是否存在未提交工作。

## 持续记录

- 完成新的实验、架构变更或合并后，更新 `docs/project_handoff.md` 中的状态、验证结果、
  已知限制和下一步计划。
- 只把实际运行通过的能力写入“已完成”；计划或只有数据流的功能必须保留在“待完成”。
- 不得将 RViz 实时三维点云描述为已经完成三维建图。只有在 SLAM 产生持久地图、完成
  位姿估计和闭环验收后，才能标记三维建图完成。
- 所有回复使用简体中文，中英文与数字之间留空格；代码标识符和命令保持英文。
- 提交时只暂存本任务直接修改的文件，提交信息使用 `类型: 中文描述` 格式。

## 工程约束

- 遵循现有 ROS 2 Jazzy、Gazebo Harmonic、Python 和 launch 文件风格。
- TF 发布权必须唯一：里程计估计器发布 `odom -> base_footprint`；定位或 SLAM 节点发布
  `map -> odom`；`robot_state_publisher` 发布车体与传感器 TF。
- 保留 `/tf_ground_truth` 仅用于仿真评估，不得让导航依赖 Gazebo 真值。
- 控制链保持 `Nav2 -> velocity_smoother -> Collision Monitor -> /cmd_vel_safe ->`
  `twist_to_ackermann -> /drive -> 底盘驱动器`。
- 修改公共函数时补充正常、边界和错误条件测试；提交前运行相关构建和测试。
