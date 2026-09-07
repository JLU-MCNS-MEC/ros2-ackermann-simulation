# Nav2 轨迹点跟踪与激光避障实例

这个实例使用 Nav2 的开源组件完成连续轨迹点导航。应用代码只负责读取 CSV、生成带切向航向的 `PoseStamped`，以及发布 RViz 标记；规划、控制、代价地图和安全停障由 Nav2 提供。

## 启动与可视化

先构建并加载工作空间：

```zsh
source /opt/ros/jazzy/setup.zsh
cd /home/xqiao/Workspace/ackermann_ros2_ws
colcon build --symlink-install
source install/local_setup.zsh
export ROS_DOMAIN_ID=47
```

有桌面显示时启动 Gazebo 和 RViz：

```zsh
ros2 launch ackermann_line_following_bringup nav2_waypoint_nav.launch.py \
  send_waypoints:=true use_rviz:=true
```

无桌面显示时使用完整 world 路径启动服务器，另开终端查看话题：

```zsh
ros2 launch ackermann_line_following_bringup nav2_waypoint_nav.launch.py \
  send_waypoints:=true use_rviz:=false \
  gz_args:='-r -s /home/xqiao/Workspace/ackermann_ros2_ws/install/ackermann_line_following_description/share/ackermann_line_following_description/worlds/waypoint_obstacle.sdf'

ros2 topic echo /model/ackermann_car/odometry
ros2 topic echo /plan
ros2 topic echo /cmd_vel_nav
ros2 topic echo /model/ackermann_car/cmd_vel
```

默认路线在 [nav2_trajectory.csv](../src/ackermann_line_following_controller/config/nav2_trajectory.csv)，包含起点 `(-3.5, 0)`、障碍前下侧检查点 `(-1.5, -0.9)` 和终点 `(3.5, 0)`。Smac Hybrid-A* 会根据实时雷达代价地图为后一个路段选择绕障侧向路径；检查点向下偏置是为了让重复测试保持在地图安全走廊内，同时没有把最终绕行轨迹写死。发送器也可以单独运行；现在不传 `waypoint_file` 时会自动使用安装包中的默认路线：

```zsh
ros2 run ackermann_line_following_controller nav2_waypoint_sender
```

## 当前导航架构

```text
/scan (MID-360 horizontal LaserScan) ─────┐
 /scan/points (MID-360 PointCloud2) ──────┤
                            ├─ global/local costmap + inflation
waypoint CSV → NavigateThroughPoses
                            ├─ Smac Hybrid-A* (DUBIN, search radius 0.90 m)
                            └─ Regulated Pure Pursuit
                                      ↓
                            velocity_smoother
                                      ↓
                            Collision Monitor (/scan)
                                      ↓
                 /model/ackermann_car/cmd_vel (Twist)
                                      ↓
                         Gazebo AckermannSteering

map ── static map→odom (仿真真值；实车替换为 SLAM/AMCL + EKF)
odom ── base_footprint ── base_link ── lidar_link / rgbd_optical_frame
```

仿真中的 `/model/ackermann_car/odometry` 来自 Gazebo `OdometryPublisher` 的模型位姿，目的是让地图、激光和车体保持同一坐标系；原生 Ackermann 插件的轮积分里程计仍保留在 `/model/ackermann_car/wheel_odometry`，用于打滑对照。实车不能使用仿真真值，应由编码器和 IMU 经 `robot_localization` 输出 `odom → base_footprint`，再由 SLAM Toolbox 或 RTAB-Map 提供 `map → odom`。

全局规划使用 [Smac Hybrid-A*](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/smac/smac_hybrid/configuring_smac_hybrid/)，运动模型为 DUBIN，只允许前进并显式设置最小转弯半径。仿真搜索半径取 0.90 m，以便在稀疏检查点之间生成可行路径；Gazebo Ackermann 插件仍把实际转角限制在 0.55 rad（约 1.08 m 几何半径）。局部跟踪使用 [Regulated Pure Pursuit](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/)，最后由 [Collision Monitor](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/) 根据雷达近场区域限速或停车。`NavigateToPose` 和 `NavigateThroughPoses` 各有一棵自定义行为树，均移除了原地 `Spin` 和倒车 `BackUp` 恢复动作，因为它们与当前前进式 Ackermann 约束不匹配。

## 为什么之前车辆看起来不前进

逐层观察 `/cmd_vel_nav → /cmd_vel_smoothed → /model/ackermann_car/cmd_vel` 可以区分“没有控制命令”和“车辆没有执行”。本工程之前同时存在以下可复现原因：

1. 直接运行发送器时 `waypoint_file` 默认是空字符串，CSV 解析只得到零个点，节点立即报 `At least two waypoints are required`，因此不会发布任何导航速度。现在默认指向安装包路线，并增加了单元测试。
2. 旧导航把 Gazebo Ackermann 插件的轮积分里程计当作唯一 `odom`。车辆顶住中心箱体时轮子仍会转，轮里程计继续积分；地图中的车体、TF、激光障碍和真实模型随后发生偏移，规划器会认为车已经越过障碍，控制器却无法得到有效前进轨迹。现在导航使用模型位姿作仿真基准，同时保留轮里程计用于比较。
3. 启动时路由发送器原来只等待 `bt_navigator`，没有等待 `map_server` 的瞬态 `/map`。在 Gazebo 渲染负载较高时，代价地图会暂时退化为默认边界 `0..4.98 m`，起点 `x=-3.5 m` 位于地图外，规划器自然不会发出有效速度。发送器现在在发目标前确认收到 `/map`。
4. 旧的 9 点侧向路线为每个点生成切向航向，几个相邻点之间的曲率和姿态约束会让 Hybrid-A* 生成长回环；车虽然在执行正向速度，却越出路线走廊，随后重新规划报 `exceeded maximum iterations`。当前默认路线保留一个下侧检查点，让规划器对障碍段自由选边；这样既保留实时雷达避障，又避免上侧参考箱体把车逼到地图边界，实测更稳定。
5. Nav2 的默认恢复行为包含原地旋转和倒车，当前车模只接收前进式 Ackermann 速度，规划失败时这些恢复动作不能正确执行。当前行为树只保留清图、等待和重新规划。
6. 视觉循迹、旧 `waypoint_tracker` 和 Nav2 不能同时向底盘发布命令。Nav2 启动文件明确关闭视觉循迹和旧跟踪器，避免多个控制源互相覆盖。

## 雷达为什么看起来只有一侧

`/scan` 仍保留为 Nav2 兼容的水平 LaserScan，配置是 `-π…π`、1024 点、10 Hz；同一 MID-360 样式传感器增加了 20 层垂直扫描（-7°…+52°），并通过 `/scan/points` 发布三维 PointCloud2。空旷方向没有实体碰撞体，LaserScan 返回 `inf`，RViz 不绘制无回波点，所以原 `line_track` 场景看起来像只有一侧。`waypoint_obstacle.sdf` 增加了中心、左侧和右侧实体箱体；在 RViz 中打开 `LaserScan` 和 `MID360 PointCloud` 可分别检查平面兼容接口和三维回波。

## 已完成的实测

- 短直线路径 `(-3.5,0) → (-2.0,0)`：Nav2 到达并返回 `Goal succeeded`。
- 三点轨迹（起点、障碍前检查点、终点）：发送器报告 `tracking route point 1/3`、`2/3`，车辆绕过中心箱体后返回 `Nav2 route completed successfully`；`/cmd_vel_nav`、`/cmd_vel_smoothed` 和最终 `/model/ackermann_car/cmd_vel` 均出现正向速度。
- 传感器基线：MID-360 样式雷达约 10 Hz，三维点云频率受 Gazebo GPU 渲染和 bridge 影响；RGB-D 接收频率同样受渲染影响，不能直接作为实机性能承诺。

## 动力学验收与场景矩阵

底盘的公开控制边界是 `AckermannDriveStamped`。`ackermann_to_twist` 以 50 Hz 输出 Gazebo 原生 Ackermann 插件所需的 `Twist`，并执行对称的前进/倒车速度限幅、加减速度限幅、前轮转角速率限幅和 0.5 s 命令看门狗。Nav2 场景关闭这个适配器，由 Nav2 的 `cmd_vel` 直接进入同一个 Gazebo 插件；两条控制链不会同时写底盘。

动力学实例使用独立的开放场地和真实模型里程计，按顺序验收直行、左右转、刹车、倒车直行、倒车左右转和最终停车。运行命令如下：

```zsh
ros2 launch ackermann_line_following_bringup dynamics_test.launch.py \
  use_rviz:=true
```

无桌面环境可使用 `use_rviz:=false` 和 `gz_args:='-r -s <install>/.../dynamics_test.sdf'`；报告默认写到 `/tmp/ackermann_dynamics_result.json`，`/dynamics_trajectory` 与 `/dynamics_phase` 可直接在 RViz 观察。

静态地图提供三种可重复场景，统一使用 `/scan`、全局/局部 costmap、Smac Hybrid-A*、Regulated Pure Pursuit 和 Collision Monitor：

| 场景 | 路线 | 验收用途 |
| --- | --- | --- |
| `straight` | `(-3.5,0) → (-2.0,0)` | 空载短直线和基本前进 |
| `obstacle` | `(-3.5,0) → (-1.5,-0.9) → (3.5,0)` | 中心障碍实时雷达绕行 |
| `offset` | `(-3.5,0) → (-1.5,0) → (3.0,1.2)` | 非零横向终点和姿态跟踪 |

```zsh
ros2 launch ackermann_line_following_bringup static_map_scenarios.launch.py \
  scenario:=straight use_rviz:=true
ros2 launch ackermann_line_following_bringup static_map_scenarios.launch.py \
  scenario:=obstacle use_rviz:=true
ros2 launch ackermann_line_following_bringup static_map_scenarios.launch.py \
  scenario:=offset use_rviz:=true
```

`perception.rviz` 已预置静态地图、全局/局部 costmap、`/scan`、`/scan/points`、RGB-D 点云和图像、TF、发送路线、`/plan`、`/local_plan` 以及 Collision Monitor 的停止/减速多边形；动力学场景使用 `dynamics.rviz` 额外显示实测路径和当前阶段标记。

## 开源路线与下一步

当前导航尽量复用了 [Nav2](https://github.com/ros-navigation/navigation2)、[ros2_controllers Ackermann Steering Controller](https://github.com/ros-controls/ros2_controllers) 的接口和 Gazebo 官方插件。二维建图/定位优先使用 [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)；带 RGB-D 和三维点云的重建优先评估 [RTAB-Map ROS 2](https://github.com/introlab/rtabmap_ros/tree/ros2)。三维点云接入 Nav2 前应先做时间同步、地面滤除、体素降采样和高度裁剪；三维重建地图与二维导航占据地图分开验收，避免用漂亮的网格掩盖定位漂移。

下一轮按以下顺序推进：先用 rosbag 回放验证雷达掉线和 TF 延迟时的 Collision Monitor 停车，再在有纹理的非对称场景运行 SLAM Toolbox/RTAB-Map，最后将仿真真值替换成编码器+IMU EKF，并用 ATE/RPE、到点误差、最小障碍距离和停车延迟做实车验收。

## 版本

- 视觉循迹基线：`visual-line-following-baseline` 分支和 tag。
- 当前 Nav2 实例：`waypoint-obstacle-avoidance` 分支。
