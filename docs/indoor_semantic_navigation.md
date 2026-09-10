# 百平方米建图与命名语义目标导航

场景内部为 10 × 10 m，包含隔断、桌子、椅子、货架和箱子。定位采用轮式里程计、
IMU EKF 与激光扫描；导航不读取 Gazebo 真值。建图使用 Slam Toolbox，保存后使用
AMCL。几何场景名字仅用于评估，不向语义导航提供位置。

## 使用已经保存的演示地图

```bash
ros2 launch ackermann_line_following_bringup indoor_semantic.launch.py use_rviz:=true
```

在 RViz 的 `2D Pose Estimate` 中给出地图左下侧建图起点附近的粗略位姿，再用
下文 `/semantic_goal` 命令发送 `入口`、`中央通道`、`办公桌旁` 或对应别名。
演示地图来自实际 SLAM；演示目标来自定位后人工记录的停靠位姿。
无桌面环境时省略 `use_rviz:=true`，通过 `/initialpose` 提供粗略位姿。

### RViz 中手动设置目标

- 普通点到点导航使用工具栏的 `2D Goal Pose`。它发布到 `/goal_pose_auto`，系统会根据
  车辆当前位置自动选择前进或倒车的到达朝向，减少空旷区域内仅由终点朝向造成的大弧线。
- 需要精确控制最终车头朝向时使用 `Nav2 Goal`。这是完整位姿目标；受 Ackermann
  最小转弯半径限制，即使没有障碍也可能先绕出一段调整车身。

位置优先适配器只修改终点朝向，不修改目标位置、障碍代价、安全区或车辆物理转弯半径。

## 1. 在线建图

```bash
ros2 launch ackermann_line_following_bringup indoor_navigation.launch.py
```

默认无界面运行 Gazebo；需要桌面显示时加 `use_rviz:=true`。在 RViz 的 `map`
坐标系中设置可达目标，车辆行驶时持续更新地图。初始地图坐标原点接近建图起点，
与 Gazebo 世界坐标不同。`/scan_slam` 使用雷达附近高度带，`/scan_nav` 使用车身避障
高度带。未观测区域与遮挡后方必须通过继续行驶补齐。

运行指定的地图坐标目标并记录结果：

```bash
python3 scripts/indoor_route_check.py --goals '[[5,0,0],[7,2,1.5708]]'
```

室内模式启用 Reeds-Shepp 与 RPP 倒车跟踪；普通静态导航基线默认仍只允许前进。
控制链为 `Nav2 -> velocity_smoother -> Collision Monitor -> /cmd_vel_safe ->`
`twist_to_ackermann -> /drive -> navigation_chassis_adapter -> Gazebo`。

## 2. 保存与重启定位

先创建自己的地图输出目录。下面的路径是示例，保存时避免覆盖已有实验。

```bash
ros2 run nav2_map_server map_saver_cli -f /tmp/indoor_map \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/tmp/indoor_posegraph'}"
```

栅格地图供 AMCL 导航，pose graph 用于后续继续建图。退出建图 launch 后再启动：

```bash
ros2 launch ackermann_line_following_bringup indoor_navigation.launch.py \
  mode:=amcl map_file:=/tmp/indoor_map.yaml
```

在 RViz 中使用 `2D Pose Estimate` 提供粗略初始位置及朝向，再发送导航目标。
AMCL 不会自动知道车辆在旧地图中的出生位置。不应同时运行建图与 AMCL。

## 3. 设置命名目标

第一版支持人工命名的停靠位姿和别名；这不是自动视觉语义建图。先在已保存地图中
完成定位并把车辆导航到希望停靠的位置，然后记录实际 `map -> base_footprint`：

```bash
ros2 run ackermann_line_following_controller semantic_navigation \
  --map /tmp/indoor_map.yaml --database /tmp/indoor_landmarks.json \
  --record '办公桌旁' --alias '办公区' --ros-args -p use_sim_time:=true
```

可在其他位置重复记录，例如 `货架旁`、`入口`。记录的是可停靠车位，不是物体中心。
地标含名称、别名、来源、位姿和观测时间，并绑定栅格地图内容与坐标定义的指纹。
地图发生改变时必须重新校验或标注，程序拒绝静默复用不同地图的地标。

启动命名导航服务：

```bash
ros2 run ackermann_line_following_controller semantic_navigation \
  --map /tmp/indoor_map.yaml --database /tmp/indoor_landmarks.json \
  --ros-args -p use_sim_time:=true
ros2 topic pub --once /semantic_goal std_msgs/msg/String "{data: '办公桌旁'}"
```

查看状态或取消：

```bash
ros2 topic echo /semantic_navigation/status --qos-durability transient_local \
  --qos-reliability reliable
ros2 topic pub --once /semantic_goal std_msgs/msg/String "{data: '__cancel__'}"
```

`/semantic_navigation/landmarks` 为 RViz MarkerArray。命令精确匹配名称或别名；
未知、歧义、忙碌时的新目标和不可用的导航服务会被拒绝。不可达目标由 Nav2 报告
失败；默认任务超时 180 秒。需要重载新增地标时重启语义导航节点。

## 后续语义建图

`enable_rgbd:=true` 可启用 RGB-D。自动视觉识别、深度投影、跨帧融合、关键帧关联、
闭环后语义地图更新和开放词汇查询仍是独立待验收任务。不要把命名车位或 RViz
点云展示称为已经完成这些功能。
