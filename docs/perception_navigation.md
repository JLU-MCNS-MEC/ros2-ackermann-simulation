# 感知传感器、测试与导航路线

调研与本机测试日期：2026-09-07。平台：ROS 2 Jazzy / Gazebo Harmonic。

## 1. 已确定的传感器基线

按当前室内低速仿真选择 **360° 二维雷达 + 前向 RGB-D + 轮式里程计**，保留向下的循迹 RGB 相机。实车增加 IMU，与编码器融合。预算、户外使用和算力尚未给出，因此下面型号为候选，仿真参数并非厂家参数复刻。

| 部件 | 本次仿真配置 | 用途与实车候选 |
| --- | --- | --- |
| 二维雷达 | 720 点，10 Hz，0.12–12 m，水平 360°，高 0.52 m，1 cm 高斯噪声 | 平面 SLAM、障碍边界；候选 [RPLIDAR S2](https://www.slamtec.com/en/s2/)，官方典型转速 10 Hz |
| RGB-D | 640×480，15 Hz，水平视场约 69.4°，0.2–8 m；前移 0.36 m，高 0.41 m | 彩色点云、低矮/悬空障碍、三维重建；候选 [RealSense D435i](https://www.realsenseai.com/cn/products/d435i/)，带 IMU |
| 原 RGB 相机 | 640×360，30 Hz，向下倾斜 | 循迹回归测试，不作为前向建图相机 |
| 里程计/IMU | 仿真导航使用 Gazebo `OdometryPublisher` 的模型位姿；原生轮积分输出另存为 `/model/ackermann_car/wheel_odometry`；本次未新增 IMU 仿真 | 实车编码器 + IMU，经 robot_localization 输出连续 odom |

若主要目标改为户外三维 SLAM，优先评估 [Livox MID-360](https://www.livoxtech.com/cn/mid-360/specs) + 同步相机 + IMU。其扫描模式不能用本次单线雷达仿真代表。二维雷达无法单独恢复完整三维场景；RGB-D 的 8 m 裁剪上限也不意味着实机在 8 m 仍有可靠深度。

## 2. 开源方案比较

| 方案 | 输入和产物 | 本工程采用方式 |
| --- | --- | --- |
| [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) | LaserScan、TF/里程计；二维栅格与位姿图 | 优先跑通二维建图、闭环、保存地图和定位 |
| [RTAB-Map ROS 2](https://github.com/introlab/rtabmap_ros/tree/ros2) | RGB-D/双目/激光，支持视觉和激光 SLAM；三维地图 | 首选三维路线；官方有 Jazzy 二进制及 Nav2 示例，先用已有 odom + 同步 RGB-D，再评估激光约束 |
| [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) | 三维激光 + IMU；激光惯性里程计与点云地图 | 户外升级候选；必须有逐点时间、外参和 IMU 时间同步，不能直接接本次 LaserScan；移植版本需单独验证 Jazzy |
| [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2) | 激光 + IMU + 图像；融合里程计和彩色地图 | 后续研究对照，需要更严格的标定与同步；不当作已跑通的 ROS 2 插件 |
| [Open3D](https://github.com/isl-org/Open3D) | 已配准 RGB-D/点云；配准、融合及网格处理 | 离线三维重建与误差评估，不替代底盘导航栈 |

三维路线先得到可信位姿与彩色点云，再进行 TSDF 融合/网格导出。完整闭环优化后需按优化轨迹重新融合，避免墙体重影。视觉质量与导航占据地图分别评价；漂亮的模型不能证明定位与避障可靠。

## 3. 可复现测试

终端一（使用小数格式的出生坐标；上游 spawn 会把 `0` 识别为整数并报参数类型错误）：

```zsh
source /opt/ros/jazzy/setup.zsh
cd /home/xqiao/Workspace/ackermann_ros2_ws
colcon build --symlink-install
source install/local_setup.zsh
export ROS_DOMAIN_ID=47
export GZ_PARTITION=ackermann_sensor_test
ros2 launch ackermann_line_following_bringup sim.launch.py \
  enable_line_follower:=false start_x:=0.0 start_y:=0.0 \
  gz_args:="-r -s --headless-rendering $PWD/src/ackermann_line_following_description/worlds/sensor_lab.sdf"
```

终端二：

```zsh
source /opt/ros/jazzy/setup.zsh
cd /home/xqiao/Workspace/ackermann_ros2_ws
source install/local_setup.zsh
export ROS_DOMAIN_ID=47
/usr/bin/python3 scripts/check_sensors.py > docs/results/sensor_lab.json
```

脚本等待发现后采样 20 秒墙钟时间；检查五条话题、时间戳递增、中心测距误差 < 5 cm。频率门槛：雷达 ≥ 9 Hz，RGB-D 各流 ≥ 12 Hz。失败返回非零；JSON 区分 data_passed 与 rate_passed。仅用于静止、原点出生的 sensor_lab。单调时间戳不等于跨传感器已经同步。

本机首轮结果：雷达 9.76 Hz，均值 3.000893 m，标准差 0.009658 m；RGB 8.15 Hz，深度 11.14 Hz，点云 9.38 Hz。深度中心为 2.640000 m，符合 3−0.36 m 的几何真值。深度未模拟实机噪声，因此极小误差不能用于硬件选购。按正式门槛复测的原始结果保存在 [sensor_lab.json](results/sensor_lab.json)。这些是传感器链路基线，不代表完成了实机同步验收。

TF 实测可查询 `odom -> rgbd_optical_frame`，平移约 (0.36, 0, 0.41)，光学坐标方向正确。当前 RGB-D 接收频率不足，不能宣布性能验收通过。接收频率同时受渲染、bridge、DDS 和 Python 采集器影响，尚未定位瓶颈；先分别订阅图像、深度和点云，与同时订阅作对照，再比较 320×240 / 640×480，以及关闭原循迹相机渲染后的差异。

下一轮测试矩阵：

- 静态：0.5/1/2/3/5 m 靶板，每档 60 秒，测偏差、标准差、P95、有效深度占比；覆盖玻璃、黑色、反光表面（这些须实机）。
- 动态：0.1/0.3/0.6 m/s 和连续转弯，记录跨流时间差、消息间隔 P95、丢帧、CPU/GPU、实时因子；时间差目标 ≤ 20 ms。
- 建图：有纹理且非对称的室内场景，闭环至少三圈；用独立 Gazebo 真值评估 ATE/RPE、闭环前后漂移、墙面厚度和地图完整度。当前单靶板场景仅用于测距，不足以评估 SLAM。
- 实车：标定相机内参、雷达/相机/IMU 外参及时间偏移，再按相同测试矩阵记录 rosbag。

## 4. 导航实现思路

1. 仿真已用已知静态地图验证 Nav2 到点和绕障；实车使用 SLAM Toolbox 建二维地图，定位阶段可用 AMCL。TF 为 `map -> odom -> base_footprint -> base_link -> sensors`。每条动态 TF 只允许一个发布源：启用 EKF 后应关闭 Gazebo 对同一 odom TF 的桥接。
2. RGB-D 经同步、地面过滤、高度裁剪与体素降采样生成障碍点云，加入 Nav2 的局部 voxel/obstacle layer；雷达提供平面障碍。三维重建由 RTAB-Map 独立验证，同一时刻不能和另一个 SLAM 同时争用 `map -> odom`。
3. 当前仿真已经采用 [Smac Hybrid-A*](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/smac/smac_hybrid/configuring_smac_hybrid/)，局部采用 [Regulated Pure Pursuit](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/)，并由 [Collision Monitor](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/) 对激光近场限速/停车。先只向前行驶，禁用原地旋转及 spin 恢复；需要倒车时再配置 Reeds-Shepp 和倒车跟踪。
4. 轴距 0.56 m、转角上限 0.55 rad，由单轨模型计算转弯半径约 0.91 m；仿真搜索半径取 0.90 m，执行层仍受 Gazebo 的 0.55 rad 转角上限约束，实车应以实测转弯半径校正。车体参考点目前在车身中心，实车以后轴为控制参考时要统一 TF 与模型定义。碰撞 footprint 覆盖车身和轮胎，不能只用车身宽 0.48 m。
5. 控制接入新增 Twist 到 Ackermann 适配：`delta = atan(L * omega / v)`，限转角、转角速度与加速度；低速除零处理、超时停车和控制源仲裁必需。现有适配方向相反，不能直接作为 Nav2 到实车的适配器。导航时关闭循迹，防止两个节点同时发命令。
6. 已验证已知地图上的短直线到点、中心障碍绕行和终点停车，再做在线建图导航。初始限速 0.1–0.2 m/s，验收至少包含 20 次目标点任务、动态障碍和感知掉线停车。

本次交付范围：传感器基线、仿真接入、Nav2 静态地图导航、静态绕障测试与开源三维/SLAM 路线设计。SLAM Toolbox 和 RTAB-Map 已安装但尚未在本车模型上完成闭环建图、重建网格和在线建图导航；这些仍是下一阶段测试。
