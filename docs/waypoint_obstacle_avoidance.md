# 轨迹点跟踪与激光避障实例

该实例从 `visual-line-following-baseline` 创建，视觉循迹和轨迹导航互斥：启动轨迹导航时关闭 `/camera/image_raw` 的 `line_follower`，只有一个节点向 `/cmd_ackermann` 发布控制指令。

## 激光雷达为什么看起来只有一侧

当前模型的 `/scan` 是 360°：`angle_min=-π`、`angle_max=π`、720 个采样点，角分辨率约 0.5°。在原 `line_track` 世界中，黑线是 visual-only，没有碰撞体，地面又是无限平面；水平激光向没有实体障碍的方向返回 `inf`，RViz 的 `LaserScan` 显示不会绘制这些无回波点，因此画面看起来像只有一侧。前向 RGB-D 相机本身只有约 69° 视场，不能用它的点云范围判断雷达视场。

本实例加入了 1 个挡住路径的红色实体箱和左右侧的蓝、绿色实体箱，运行 RViz 后可以同时看到 360° 的有效回波。用下面命令可直接核对传感器头：

```zsh
source /opt/ros/jazzy/setup.zsh
source /home/xqiao/Workspace/ackermann_ros2_ws/install/local_setup.zsh
export ROS_DOMAIN_ID=47
ros2 topic echo /scan --once
```

## 启动

```zsh
source /opt/ros/jazzy/setup.zsh
cd /home/xqiao/Workspace/ackermann_ros2_ws
colcon build --symlink-install
source install/local_setup.zsh
export ROS_DOMAIN_ID=47
export GZ_PARTITION=ackermann_waypoint
ros2 launch ackermann_line_following_bringup waypoint_nav.launch.py target_speed:=0.18
```

另开终端查看感知和轨迹：

```zsh
source /opt/ros/jazzy/setup.zsh
source /home/xqiao/Workspace/ackermann_ros2_ws/install/local_setup.zsh
export DISPLAY=:1
export XAUTHORITY=/home/xqiao/.Xauthority
export ROS_DOMAIN_ID=47
rviz2 -d /home/xqiao/Workspace/ackermann_ros2_ws/install/ackermann_line_following_bringup/share/ackermann_line_following_bringup/rviz/perception.rviz
```

RViz 预置了 `odom` 固定坐标系、车体、`/scan`、`/rgbd/points`、RGB 图像、黄色 `/trajectory` 和黄色 `/waypoint_target`。把 `Depth Image` 打开可以查看深度；在 RGB 与深度之间切换时，先关闭另一个 Image display，避免两个图层重叠。

## 控制逻辑

`waypoint_tracker` 从 [straight_trajectory.csv](../src/ackermann_line_following_controller/config/straight_trajectory.csv) 读取 `odom` 坐标系的路径点，用 Pure Pursuit 计算阿克曼前轮转角。轴距为 0.56 m，转角限制为 0.48 rad，默认速度为 0.18 m/s。

激光只承担局部安全覆盖层：前方 ±28° 小于 1.20 m 时减速，小于 0.72 m 时选择左右较宽的一侧，以 0.12 m/s 和最大转角保持 4 秒；前方及两侧同时受阻时发布零速度。它是可解释的演示控制器，不是完整的全局规划器；真实车辆应把障碍物送入 Nav2 costmap，并使用带最小转弯半径的 Hybrid-A* 与 Ackermann 控制器。

本次实际运行日志确认：车辆在红色障碍物前约 0.71 m 触发 `avoiding left`，随后抵达最后一个轨迹点并停车。测试场景只验证静态箱体和 ROS 数据链路，尚未覆盖动态障碍、盲区、急转、倒车和实车时延。

## 版本保存与远程分支

原视觉循迹实例已保存为：

- 分支：`visual-line-following-baseline`
- tag：`visual-line-following-baseline`
- 远程仓库：`https://github.com/JLU-MCNS-MEC/ros2-ackermann-simulation`

当前实例分支为 `waypoint-obstacle-avoidance`。推送后可直接在 GitHub 切换该分支查看代码；仓库可见性由远程仓库设置决定，本次推送不会修改仓库权限。
