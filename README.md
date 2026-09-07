# Ackermann 小车循迹仿真

新增雷达、前向 RGB-D 与静态测距场景，详见 [感知测试与导航路线](docs/perception_navigation.md)。

轨迹点跟踪和激光避障实例见 [Nav2 轨迹点与避障说明](docs/waypoint_obstacle_avoidance.md)。

这是一个面向 Ubuntu 24.04 的 ROS 2 Jazzy + Gazebo Harmonic 示例。车辆模型、赛道、桥接和循迹控制分开，后续替换车型时不需要重写循迹节点。

## 启动

```bash
source /opt/ros/jazzy/setup.zsh
cd /home/xqiao/Workspace/ackermann_ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/local_setup.zsh
ros2 launch ackermann_line_following_bringup sim.launch.py
```

默认会启动 Gazebo GUI、`ackermann_car`、圆角闭环赛道、相机、ROS-Gazebo bridge 和视觉 PID 循迹节点。屏幕演示可以把速度降到 `0.10 m/s`：

```bash
ros2 launch ackermann_line_following_bringup sim.launch.py line_speed:=0.10
```

感知导航实例使用 Nav2 的 Smac Hybrid-A*、Regulated Pure Pursuit、costmap 和 Collision Monitor：

```bash
ros2 launch ackermann_line_following_bringup nav2_waypoint_nav.launch.py \
  send_waypoints:=true use_rviz:=true
```

导航启动会关闭视觉循迹和未使用的 RGB-D 渲染，确保只有 Nav2 向底盘发布速度。MID-360 PointCloud2 直接进入全局和局部 VoxelLayer，并投影为 `/scan_nav` 供 Collision Monitor 使用。默认场景在不含内部障碍的地图上感知 Gazebo 中的未知箱体，绕行后在终点停车；传感器、TF、故障定位和开源方案比较见 [感知测试与导航路线](docs/perception_navigation.md)。

完整的前进、倒车、刹车和转向验收，以及短直线、已知障碍、未知障碍和偏置终点四种静态地图场景，见 [Nav2 轨迹点与激光避障说明](docs/waypoint_obstacle_avoidance.md)。动力学测试会把实际模型里程计写入 `/tmp/ackermann_dynamics_result.json`，并在 RViz 显示实测轨迹。

查看相机画面可以另开终端执行：

```bash
source /opt/ros/jazzy/setup.zsh
source /home/xqiao/Workspace/ackermann_ros2_ws/install/local_setup.zsh
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

如果当前终端激活了 Anaconda，ROS 依赖和 `colcon` 应使用系统 Python；新终端会自动优先使用 `/usr/bin/python3`，必要时可先执行 `conda deactivate` 再构建。

## 检查话题

```bash
ros2 topic list
ros2 topic hz /camera/image_raw
ros2 topic echo /cmd_ackermann
ros2 topic echo /model/ackermann_car/odometry
ros2 topic echo /model/ackermann_car/wheel_odometry
ros2 topic echo /scan
ros2 topic echo /scan_nav
```

也可以手动给控制器发送阿克曼指令：

```bash
ros2 topic pub --once /cmd_ackermann ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {speed: 0.2, steering_angle: 0.15}}"
```

## 替换车型

将新的 URDF/Xacro 文件传给同一个 launch：

```bash
ros2 launch ackermann_line_following_bringup sim.launch.py \
  robot_description_file:=/absolute/path/to/your_car.urdf.xacro
```

新模型需要保留或在模型内部更新 Gazebo 的 `AckermannSteering` 插件映射：

- 前轮转向关节：`left_steering_joint`、`right_steering_joint`
- 左右侧驱动轮关节：重复填写 `left_joint`、`right_joint`
- 根据实际车身修改 `wheel_base`、`wheel_separation`、`kingpin_width`、`wheel_radius`

循迹节点只依赖 `/camera/image_raw` 和 `/cmd_ackermann`，因此车体尺寸、外观和关节名称可以独立变化。若换车后相机话题不同，只需在 launch 中调整相机 bridge 和 `image_topic`。

## 控制接口

视觉节点发布标准的 `ackermann_msgs/msg/AckermannDriveStamped`。适配节点使用

`angular.z = speed * tan(steering_angle) / wheelbase`

转换为 Gazebo 原生 Ackermann 插件需要的 `geometry_msgs/msg/Twist`，这样以后接真实车底盘时可以替换适配层而保留上层循迹逻辑。
