# Sim2Real 架构与验收路线

更新日期：2026-09-07。本文件区分已经跑通的仿真能力、已实现但尚未接硬件的接口，
以及必须在实车上测量的项目。不能用 Gazebo 真值或理想传感器数据代替实车验收。

## 1. 当前差距

| 子系统 | 当前状态 | 实车前必须补齐 | 优先级 |
| --- | --- | --- | --- |
| 底盘 | Gazebo Ackermann 插件，已测前进、倒车、制动、转向 | 电机与舵机驱动、编码器读取、急停、上电零位、反馈故障码 | P0 |
| 命令接口 | 已新增 `/cmd_vel_safe` → `/drive` 适配器，仿真执行端仍使用 Gazebo `Twist` | 接真实驱动或 `ros2_control` | P0 |
| 局部里程计 | Nav2 可选用轮式里程计和仿真 IMU 的 EKF 输出 | 轮速/IMU 标定、真实协方差与故障注入 | P0 |
| 定位 | `map → odom` 为恒等静态 TF | 已知地图使用 AMCL 或 Slam Toolbox localization；定位器独占该 TF | P0 |
| 雷达 | MID360 样式点云，无真实 Livox 时序/扫描模式 | `livox_ros_driver2`、PTP/硬同步、逐点时间、盲区和反射率测试 | P0 |
| 安全 | Nav2 Collision Monitor 有减速/停车区 | 独立硬件急停、驱动心跳、传感器失联停车、人工遥控抢占 | P0 |
| 二维建图 | 静态地图由场景直接生成 | Slam Toolbox 建图、闭环、保存 pose graph、重复定位 | P1 |
| 三维建图 | 只有车体坐标系中的实时三维点云 | FAST-LIO2 激光惯性里程计或 RTAB-Map，闭环后重融合并导出地图 | P1 |
| 视觉 | RGB-D 仿真链路已建立但导航时关闭 | 内参/深度尺度/外参/时间偏移标定，暗光、逆光、运动模糊测试 | P1 |
| 工程化 | 单机 launch 与实验 JSON | rosbag2 记录、开机启动、参数版本、诊断聚合、温度与算力监控 | P1 |

## 2. 推荐运行架构

```text
MID360 ───────────────┬─> pointcloud_to_laserscan ─> /scan_nav ─┐
                      ├─> Nav2 VoxelLayer                         │
                      └─> FAST-LIO2/RTAB-Map (三维实验)           │
编码器 ─┐                                                        │
        ├─> robot_localization EKF ─> /odometry/filtered ─> TF    ├─> Nav2
IMU ────┘                                                        │
Slam Toolbox/AMCL ─────────────────────────────> map → odom ─────┘

Nav2 controller → velocity_smoother → Collision Monitor
     → /cmd_vel_safe → twist_to_ackermann → /drive → 底盘驱动器
                                              └────> 硬件急停
```

TF 的发布权必须唯一：EKF 发布 `odom → base_footprint`；AMCL、Slam Toolbox 或
三维 SLAM 中选一个发布 `map → odom`；`robot_state_publisher` 只发布车体到传感器
的静态或关节 TF。实车配置中不得保留仿真的 `map_to_odom` 静态发布器和 Gazebo
真值 TF。

ROS 2 Control 的
[Ackermann Steering Controller](https://control.ros.org/jazzy/doc/ros2_controllers/ackermann_steering_controller/doc/userdoc.html)
已经实现双转向轮、双驱动轮运动学，并能发布里程计和处理命令超时，应优先用它
承接标准硬件接口。若底盘厂商只提供 CAN/串口 SDK，则驱动节点订阅 `/drive`，
并必须发布带时间戳、坐标系和协方差的轮式里程计。

## 3. 本次新增的实车命令边界

```bash
ros2 launch ackermann_line_following_bringup sim2real_control.launch.py \
  input_topic:=/cmd_vel_safe output_topic:=/drive \
  wheelbase:=0.56 max_speed:=0.30 max_steering:=0.55
```

转换遵循 `steering = atan(wheelbase * yaw_rate / speed)`，支持前进和倒车，限制
车速与转角。2026-09-10 修正：仅在数值静止阈值 `1e-6 m/s` 内输出停车，
非零低速仍保留曲率，避免安全减速后丢失转向；输入超过
`0.25 s` 未更新时持续输出停车命令。真实驱动器还必须有独立、更底层的心跳和
急停，不能只依赖 ROS 节点。

## 4. 定位与建图选择

第一条可用路线是二维导航。按 Nav2 的
[robot_localization 指南](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/odom/setup_robot_localization/)
融合编码器与 IMU；用 Slam Toolbox 完成二维建图、闭环和 pose graph 保存，
再切到 AMCL 或 Slam Toolbox localization。先做到二维稳定，三维重建故障才不会
同时影响车辆导航。

三维路线分两条：

- MID360 + IMU 使用官方 `livox_ros_driver2` 保留每点 timestamp，再接 FAST-LIO2。
  FAST-LIO 明确依赖逐点时间做运动去畸变，必须优先解决硬同步和外参。
- RGB-D 使用 [RTAB-Map ROS 2](https://github.com/introlab/rtabmap_ros) 做同步、回环和
  彩色点云；可接已有可靠 odom。闭环优化后重新融合 TSDF，避免墙面重影。

导航地图和三维展示地图可以同时生成，但只允许一个节点发布 `map → odom`。
建议先让 EKF + Slam Toolbox/AMCL 服务导航，把 FAST-LIO/RTAB-Map 置于评估命名
空间，只比较轨迹，不接管主 TF。

## 5. 分阶段验收

1. **台架**：车轮离地，确认急停、方向、转角零位、编码器符号、命令超时；
   任何 ROS 断连在 0.3 s 内进入制动状态。
2. **低速几何**：0.1 m/s 直行 5 m；左右最大转角各画三圈；前进与倒车各测
   实际转弯半径，以测量值更新轴距等效参数和限制。
3. **里程计**：矩形和八字各 10 圈，对比地面真值；报告终点误差、ATE、航向
   漂移和协方差一致性，不仅看 RViz 轨迹。
4. **感知**：MID360 在 0.5/1/3/5/10 m、不同材质和车速下测有效点比例、距离
   偏差、时间延迟和掉帧；RGB-D 另测光照与纹理退化。
5. **导航**：先封闭场地 0.1 m/s，20 次固定目标；再测窄道、遮挡、未知静态
   障碍、定位跳变和雷达断流。记录成功率、最小净空、人工接管次数与停车距离。
6. **三维建图**：同一路线至少三圈，报告 ATE/RPE、闭环前后漂移、墙厚、覆盖率、
   地图体积和实时因子，并使用独立测量作为参考。

本阶段已经增加仿真 IMU、轮式里程计 EKF，以及 Nav2 的可选 EKF 输入。下一轮
应接 Slam Toolbox 进行首个二维闭环，让定位器估计 `map → odom`，并加入轮径
误差、打滑和偏置噪声。这一步继续暴露时间戳、协方差和定位恢复问题。

仿真 IMU 和第一版 EKF 配置现已加入，可独立回归：

```bash
ros2 launch ackermann_line_following_bringup ekf_localization_test.launch.py
ros2 topic hz /imu/data_raw
ros2 topic hz /odometry/filtered
ros2 run tf2_ros tf2_echo odom base_footprint
```

该配置有意把 Gazebo 真值 TF 改发到 `/tf_ground_truth`，由 EKF 独占正式 `/tf`。
当前仅验证融合链路；轮式里程计仍是 Gazebo 插件输出，噪声、打滑和协方差模型
尚未达到实车真实性，不能把这项测试记作定位精度验收。

也可以让静态地图导航直接使用 EKF 里程计进行回归：

```bash
ros2 launch ackermann_line_following_bringup static_map_scenarios.launch.py \
  scenario:=straight use_ekf_localization:=true record_experiment:=true
```

此模式仍用已知出生位姿设置 `map → odom`。实车应由 AMCL 或 Slam Toolbox
估计该变换，不能依赖启动参数中的出生坐标。

本机短直线回归成功：8.539 s 行驶 1.253 m，速度 RMSE 0.01394 m/s，平均/最大
横向误差 0.000020/0.000075 m，最终目标距离 0.259 m，未触发安全区。诊断节点
会通过 TF 把 `odom` 位姿变换到路径的 `map` 坐标系后计算误差。误差很小主要
来自仿真轮式里程计仍接近理想值，只证明接口和 TF 链路闭合，不代表实车精度。
