# 工程状态与会话交接

更新时间：2026-09-10。

## 当前可视化会话

- M0 已开始：新增 `navigation_regression` 与固定五目标路线，支持逐目标超时、失败
  停止、恢复/倒车/传感器健康记录；14 项工具测试通过，工具覆盖率 94%。
- 首轮从办公区出发失败，已保留 `/tmp/ackermann_m0_20260910_PPxNOy` 的结果与
  321 MiB rosbag；发现进展检查与安全低速不匹配，以及 20 ms 动作确认超时。
  已调整为 0.2 m / 15 s 与 500 ms，未改变停车保护，20 次回归尚未完成。
- 为加载上述配置，已完整重启仿真与导航并重新提供初始位姿；新主日志
  `/tmp/indoor_m0_launch_v2.log`，旧 RViz/Gazebo GUI 保留。以下旧会话参数覆盖描述
  仅为历史记录，当前转换器由新主 launch 管理。

- 已在桌面 `:1` 启动 Gazebo GUI、室内语义导航与 RViz，供用户直接观察。
- AMCL 已接收粗略初始位姿；“办公区”目标实际成功到达。
- 本次已观察到 Gazebo 三维场景及 RViz 车辆、点云与代价地图。临时展示配置
  `/tmp/indoor_visual_20260910.rviz` 使用更远俯视视角并隐藏未启用的 RGB-D 面板。
- 演示保持运行；主日志 `/tmp/indoor_visual_demo.log`，Gazebo GUI 日志
  `/tmp/indoor_visual_gazebo.log`，独立 RViz 日志 `/tmp/indoor_visual_rviz.log`。
- 用户点击目标无反应的原因已定位：`nav2_rviz_plugins/GoalTool` 只通知 Nav2 面板，
  原配置缺少 `Navigation 2` 面板。已补齐正式与当前临时配置并增加配对检查测试；
  仅重启 RViz，仿真与 AMCL 保持运行。
- 已修复低速转向死区；室内两级启动入口默认 `target_speed:=0.35`，可以显式回退。
  当前仿真已动态更新速度，转换器独立重启并运行在新阈值 `1e-6 m/s`；主 launch
  参数临时文件仍是旧值，后续完整重启将读取已更新的源配置。`/drive` 发布者只有一个。
- 六个连续目标成功（含前进、倒车与转弯），五项隔离安全测试通过；安全区未缩小。
  结果见 `docs/experiments/data/indoor_20260910/indoor_speed_safety.json`。
  尚未完成 20 次可靠性、动态障碍、实测制动距离与实车验收。
  两个受影响包构建通过，108 项测试通过；修复核心 `control.py` 测试覆盖率 86%。
  详细验证见 [`experiments/indoor_speed_safety.md`](experiments/indoor_speed_safety.md)。

## 当前开发：100 平方米语义导航

- 后续多传感器路线已整理为
  [`multisensor_fusion_plan.md`](multisensor_fusion_plan.md)，并在主 TODO 中增加 M0–M10。
  推荐先 RGB-D 障碍/语义观测，再 RTAB-Map 旁路三维闭环；融合定位需独立消融验收。
  官方开源选型已核查；本机 RTX 4060 Ti 约 8 GB；当前激光点云缺逐点时间，
  不能直接标记 FAST-LIO/FAST-LIVO2 时序与去畸变已适配。尚未引入这些新依赖。

- 功能分支：`feature/semantic-navigation-100sqm`；完整清单见
  [`navigation_semantic_todo.md`](navigation_semantic_todo.md)。
- 新增 `indoor_navigation.launch.py`：10 × 10 m 场景、EKF、Slam Toolbox、独立
  `/scan_slam` 和 Nav2；真值只发 `/tf_ground_truth`，无静态 `map -> odom`。
- 导航控制已贯通 `/cmd_vel_safe -> /drive -> Gazebo`，Gazebo 命令桥改为单向。
- 构建 3 包通过；在线 SLAM 首两个导航目标实际成功，23.42 s 和 15.35 s。
  三圈闭环精度、20 次重复导航与自主探索仍待验收。
- 初次在线导航因 SLAM TF 时间滞后失败；启用 `restamp_tf` 后上述两目标通过。
- 后续 6 个倒车模式目标连续成功并保存地图与 pose graph；重启 AMCL 后 2 个目标
  成功（19.52 s、28.96 s）。RGB-D 10 组同步与采集时刻 TF 检查通过。
  详情见 [`experiments/indoor_navigation_baseline.md`](experiments/indoor_navigation_baseline.md)。
- 命名地标已记录入口、中央通道、办公桌旁，明确标记 `manual_pose` 来源；
  三个别名目标实际连续成功，49.52 s、43.12 s、30.71 s。未知、歧义、不可达与取消
  路径已运行验证；不能描述为自动视觉识别或三维语义建图。
- 一体启动：`ros2 launch ackermann_line_following_bringup indoor_semantic.launch.py`；
  用户仍需通过 `/initialpose` 或 RViz 给 AMCL 粗略初始位姿。
- 使用说明见 [`indoor_semantic_navigation.md`](indoor_semantic_navigation.md)。
  语义模块 35 项测试，覆盖率 95.15%；最终全量 88 项测试通过，0 errors、0 failures、
  0 skipped，3 包构建通过。
- 一体启动入口运行验证：提供初始位姿后 Nav2 激活，语义服务报告三个地标 ready。
  本轮仿真与导航测试进程已退出；RViz 桌面交互尚未实际验收。

下文为此前静态导航基线，运行状态应以最新实验记录为准。

本文是后续会话的首要上下文。详细 Sim2Real 方案见
[`sim2real_plan.md`](sim2real_plan.md)，静态导航数据见
[`experiments/static_navigation_baseline.md`](experiments/static_navigation_baseline.md)。

## 仓库状态

- 公共仓库：`JLU-MCNS-MEC/ros2-ackermann-simulation`
- 默认分支：`main`
- 当前功能基线：`81a828f`，由 PR #10 合并
- PR #10：Sim2Real 控制边界、仿真 IMU、EKF 定位路径与诊断修复
- 保存本文前仿真、RViz、Nav2 和 EKF 进程均已退出

每次开始工作都应重新检查 Git 状态，不得假设工作区仍与本记录完全一致。

## 已完成并实际验证

1. Ackermann 小车 Gazebo 模型支持前进、倒车、制动和转向。
2. MID360 样式三维点云接入 Nav2 VoxelLayer，同时投影 `/scan_nav` 给
   Collision Monitor；RGB-D 点云默认关闭。
3. 静态地图 Nav2 使用 Smac Hybrid-A*、Regulated Pure Pursuit、
   Velocity Smoother 和 Collision Monitor，已有直线、偏置目标、已知障碍、未知障碍
   与大型复杂静态场景。
4. `/cmd_vel_safe` 可通过 `twist_to_ackermann` 转为 `/drive` 的
   `AckermannDriveStamped`。接口支持正反向、速度/转角限制、不可实现的原地旋转抑制，
   并在 0.25 秒命令超时后停车。
5. 仿真具有 100 Hz IMU。`robot_localization` 可融合轮式里程计和 IMU，输出
   `/odometry/filtered` 及 `odom -> base_footprint`；Gazebo 真值 TF 可隔离到
   `/tf_ground_truth`。
6. 导航诊断会通过 TF 将里程计位姿转换到规划路径坐标系后计算横向误差和目标距离。

最近一次完整回归结果：

- 构建：3 个包全部成功。
- 测试：46 项，0 errors、0 failures、0 skipped。
- 独立运行：IMU 约 90 Hz，EKF 约 29.4 Hz。
- 控制适配：输入 0.4 m/s、0.2 rad/s，输出 0.4 m/s、0.273 rad；断流后按时停车。
- Nav2 + EKF 短直线：成功，8.539 秒，1.253 米，最终目标距离 0.259 米，速度 RMSE
  0.01394 m/s，平均/最大横向误差 0.000020/0.000075 米，安全区未触发。

## 当前能力边界

- 当前只有实时三维点云，尚未完成三维 SLAM、闭环、持久地图或三维重建精度验收。
- EKF 使用的轮式里程计仍来自 Gazebo，接近理想值；尚未注入轮径误差、侧滑、编码器
  量化、IMU bias、时延和掉帧。
- EKF 导航测试中的 `map -> odom` 根据已知出生位姿静态设置。实车必须改由 AMCL 或
  Slam Toolbox localization 估计。
- `/drive` 是已经验证的数据接口，尚未接真实 CAN/串口驱动或 `ros2_control` hardware
  interface，也没有真实编码器和舵机反馈。
- ROS Collision Monitor 已存在，但不能代替独立硬件急停、驱动心跳和人工遥控抢占。
- 传感器全套频率测试曾在多个 Gazebo/RViz 实例并行时因资源竞争降频；不能将那次结果
  记为完整传感器性能通过。独立 IMU/EKF 链路已经通过。

## 下一阶段执行顺序

### 1. 二维 SLAM 与重定位

接入 Slam Toolbox，先完成在线建图，再保存地图和 pose graph，重启后使用
localization 模式发布 `map -> odom`。移除测试用静态 `map_to_odom` 后运行 Nav2。

验收条件：同一路线至少 3 圈有闭环；重启后能重定位；20 次固定目标统计成功率、ATE、
终点误差和恢复次数；正式 `/tf` 中不存在重复发布者。

### 2. 退化与故障注入

为轮式里程计增加左右轮比例误差、滑移、量化和延迟，为 IMU 增加 bias random walk，
为 MID360 增加随机丢点、整帧断流和时间偏移。用 `/tf_ground_truth` 计算 ATE/RPE，验证
传感器失联停车及恢复策略。

### 3. 实车底盘闭环

优先采用 ROS 2 Control Ackermann Steering Controller；若厂商只提供 SDK，则实现
订阅 `/drive`、发布轮式里程计和诊断的 CAN/串口节点。先做车轮离地台架，再进行
0.1 m/s 直线、定圆、八字和倒车标定。

### 4. MID360 三维建图

真实 MID360 使用 `livox_ros_driver2`，保留逐点时间戳并完成 LiDAR-IMU 外参、时间同步。
接 FAST-LIO2 做激光惯性里程计和点云地图；导航主 TF 稳定前，三维 SLAM 只在评估
命名空间运行。RGB-D 彩色地图后续接 RTAB-Map，不默认参与导航。

## 快速恢复命令

```bash
cd /home/xqiao/Workspace/ackermann_ros2_ws
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install --packages-select \
  ackermann_line_following_description \
  ackermann_line_following_controller \
  ackermann_line_following_bringup
source install/setup.zsh
```

运行带 Gazebo 和 RViz 的 EKF 静态导航：

```bash
ros2 launch ackermann_line_following_bringup \
  static_map_scenarios.launch.py \
  scenario:=straight use_ekf_localization:=true use_rviz:=true
```

单独检查 EKF：

```bash
ros2 launch ackermann_line_following_bringup ekf_localization_test.launch.py
ros2 topic hz /imu/data_raw
ros2 topic hz /odometry/filtered
ros2 run tf2_ros tf2_echo odom base_footprint
```

运行实车控制接口：

```bash
ros2 launch ackermann_line_following_bringup sim2real_control.launch.py \
  input_topic:=/cmd_vel_safe output_topic:=/drive \
  wheelbase:=0.56 max_speed:=0.30 max_steering:=0.55
```

回归测试：

```bash
colcon test --packages-select \
  ackermann_line_following_description \
  ackermann_line_following_controller \
  ackermann_line_following_bringup
colcon test-result --verbose
```
