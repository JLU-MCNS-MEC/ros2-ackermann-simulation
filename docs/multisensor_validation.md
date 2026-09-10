# 多传感器阶段验证操作说明

工具与实测结果分开验收。导航测试会移动车辆，只在封闭仿真/安全测试场地运行，
先定位、确认没有其他目标发送者；不要在回归中点击 RViz 目标或改变控制参数。

## M0：固定路线与运行快照

先使用保存地图启动，提供约 `(0.2, 0.2, 0)` 的初始位姿并等待 AMCL/Nav2 就绪：

```bash
ros2 launch ackermann_line_following_bringup indoor_semantic.launch.py \
  enable_rgbd:=false target_speed:=0.35 use_rviz:=true
```

创建一个新的实验目录。记录启动参数、`git rev-parse HEAD`、地图与路线文件的
SHA-256；若工作区有修改，保存 diff，不把基线提交号误当成实际运行的完整版本。
用 `ros2 param dump <node>` 保存控制器、规划器、BT、速度平滑器、安全层、转换器、
EKF 与 AMCL 的有效参数，避免只保存未被 launch 重写的 YAML。

录制到一个尚不存在的 bag 目录，按下 Ctrl+C 正常结束并检查 `ros2 bag info`：

```bash
ros2 bag record --storage mcap --use-sim-time --disable-keyboard-controls \
  -o /tmp/experiment_unique/bag --topics \
  /clock /tf /tf_static /scan/points /scan_nav /scan_slam /imu/data_raw \
  /model/ackermann_car/wheel_odometry /odometry/filtered /amcl_pose /map \
  /cmd_vel_nav /cmd_vel_smoothed /cmd_vel_safe /drive /collision_monitor_state \
  /plan /diagnostics

ros2 run ackermann_line_following_controller navigation_regression \
  --route src/ackermann_line_following_controller/config/indoor_regression.json \
  --output /tmp/experiment_unique/results.json --count 20 --timeout 150 --use-sim-time
```

结果文件必须不存在；工具拒绝覆盖。每个目标写一次原子快照。失败或超时立即停止
后续目标；`complete=false` 不能作为 20 次完成。超时取消目标，异常也请求取消。
这不是紧急停车系统，底盘心跳与 Collision Monitor 保持独立运行。

指标包括：成功数/请求数、耗时中位数/P95、恢复次数、倒车采样比例、路径跟踪误差、
各传感器帧率/最大接收间隔/时间戳单调性。传感器帧率分别报告墙钟与消息时间。
人工接管无法仅从这些话题可靠判断，需要实验操作者另行记录。
净空字段是从扫描原点测得的距离，不是车体边缘净空；不用于宣称零接触或制动通过。

## M1：RGB-D 质量验收

完成不带视觉的基线后，正常停止 launch 和其 Gazebo server，确认没有遗留服务端或
重复命令发布者，再使用 `enable_rgbd:=true` 完整启动。保留相同地图、导航参数与目标。
新会话必须重新定位；Gazebo GPU 相机不是在已有实体上仅改 ROS 参数就能启用。

```bash
ros2 launch ackermann_line_following_bringup indoor_semantic.launch.py \
  enable_rgbd:=true target_speed:=0.35 use_rviz:=true

ros2 run ackermann_line_following_controller rgbd_audit \
  --output /tmp/rgbd_audit_unique.json --seconds 120 --minimum-samples 300 --use-sim-time
```

工具只订阅，不输出车辆命令。同步 `/rgbd/image`、`/rgbd/depth_image`、
`/rgbd/camera_info`，要求图像已经配准；不替用户把未对齐的深度假定为已对齐。
检查 `32FC1` 米制与 `16UC1` 毫米制、内参/尺寸/坐标系、RGB 解码、有效深度，
并在采集时刻查询 `map -> camera`。TF 等待限 0.3 s，不用最新位姿代替缺失历史位姿。
仿真相机固有配准/理想内参通过，不代表实车外参和深度尺度已标定。

默认验收门槛：至少 300 组，匹配比例 ≥ 90%，有效数据对 ≥ 95%，采集时刻 TF
成功率 ≥ 99%，同步偏差 P95 ≤ 20 ms，有效深度比例中位数 ≥ 10%，无非单调时间戳，
各流最大接收间隔与最终接收年龄 ≤ 0.5 s。失败返回非零退出码，结果仍保存。
场景正常空洞会降低深度比例；无效/缺失深度不会被解释为自由空间。

分别做静止与运动检查，并将视觉负载下导航表现与 M0 比较；首次没有达到门槛时
保存失败结果，不能通过放宽阈值静默变成成功。真实标定、长时稳定性和退化注入
还需在对应 TODO 中单独确认。

## 数据保存

提交小型摘要与配置版本到 `docs/experiments/data/`。大体积 bag 和完整图像采样留在
本地实验目录，并记录路径、大小、哈希和消息数；不自动把数 GB 的数据推到源码仓库。
回放时使用独立 ROS_DOMAIN_ID 或全部重映射测试命令话题，避免 rosbag 的 `/drive`
和 `/cmd_vel_safe` 等命令进入正在运行的车辆。
