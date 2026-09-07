# 静态场景导航基线

本基线使用 Gazebo Harmonic、Nav2 Smac Hybrid-A*、Regulated Pure Pursuit、
MID-360 点云 VoxelLayer 和 Collision Monitor。记录器只在
`NavigateThroughPoses` 接受任务后开始采样，并在任务成功、失败或取消时写出
JSON 汇总和同名 CSV 原始数据。

## 2026-09-07 基线结果

| 场景 | 结果 | 时间 (s) | 路程 (m) | 平均/最大横向误差 (m) | 速度 RMSE (m/s) | 前方/全向最小净空 (m) |
|---|---:|---:|---:|---:|---:|---:|
| straight | 成功 | 7.08 | 1.26 | 0.000 / 0.000 | 0.0074 | 2.536 / 2.536 |
| offset | 成功 | 31.75 | 6.52 | 0.010 / 0.039 | 0.0177 | 0.924 / 0.504 |
| obstacle | 成功 | 34.52 | 7.19 | 0.012 / 0.065 | 0.0126 | 1.450 / 0.528 |
| unknown_obstacle | 成功 | 35.84 | 7.24 | 0.013 / 0.060 | 0.0115 | 1.105 / 0.513 |
| complex_static | 成功 | 230.46 | 49.52 | 0.015 / 0.091 | 0.0100 | 1.046 / 0.608 |

四组有效结果保存在 [data](data/) 中。`unknown_obstacle` 最初复用了包含三个
物体的点云展示世界，但静态地图为空，导致三个物体都成为未知障碍。车辆绕过
中央障碍后在左上物体与地图边界之间失去满足 0.9 m 最小转弯半径的前进路径。
该失败样本保存在
[unknown_obstacle_confounded_failure.json](data/unknown_obstacle_confounded_failure.json)。
场景现已改用只包含中央箱体的 `unknown_obstacle.sdf`，复跑成功。

大型 `complex_static` 场景为 30×20 m、600×400 栅格，包含边界墙、5 组
交错货架和 3 个立柱。首次基线以 0.22 m/s 完成 9 个路线点，首条控制指令
延迟 0.201 s，车辆起步延迟 0.301 s，有效采样频率 10.002 Hz。完整结果见
[complex_static.json](data/complex_static.json)。

## 运行方法

```bash
ros2 launch ackermann_line_following_bringup \
  static_map_scenarios.launch.py \
  scenario:=unknown_obstacle \
  record_experiment:=true \
  experiment_result_file:=/tmp/unknown_obstacle.json
```

运行完成后，`/tmp/unknown_obstacle.json` 保存汇总指标，
`/tmp/unknown_obstacle.csv` 保存 10 Hz 原始样本。可将 `scenario` 替换为
`straight`、`offset` 或 `obstacle`。

## 下一轮验收

- 每个场景至少重复 10 次，统计成功率、均值和 95% 置信区间。
- 增加不同车速、障碍尺寸、障碍位置以及低反射点云丢失组合。
- 单独测量点云时间戳延迟。本轮在高负载启动阶段偶发约 0.83 s 延迟，超过
  Collision Monitor 的 0.8 s `source_timeout`，产生了 1–2 个安全停车样本。
- 已加入带噪 IMU 和轮速里程计 EKF 链路；下一步把 Nav2 的默认里程计切换为
  `/odometry/filtered`，再比较真值定位与 EKF/AMCL 的横向误差、重规划次数和成功率。

## EKF 接入回归

`use_ekf_localization:=true` 会把 Gazebo 真值 TF 移到 `/tf_ground_truth`，由
`robot_localization` 融合轮式里程计和带噪 IMU，并独占 `odom → base_footprint`。
短直线场景于 2026-09-07 完成：耗时 8.539 s、路程 1.253 m、速度 RMSE
0.01394 m/s、平均/最大横向误差 0.000020/0.000075 m，最终目标距离 0.259 m。
诊断数据已在 `map` 坐标系计算。该结果验证了完整导航链路，仍需通过轮径误差、
打滑和 IMU 偏置注入评估定位退化。
