# small_dlio

`small_dlio` 是一个基于 ROS 2 Jazzy 的 Livox MID-360 LiDAR-IMU SLAM 实验工程。当前工程包含两条主要使用路径：

- 自带轻量级 LIO 前端：IMU 传播、Livox 点云累计、去畸变、scan-to-map GICP、几何观测器融合、keyframe 发布和在线地图重建。
- 可选后端：LiDAR-Iris + Cart Context 回环检测、GICP 几何验证、g2o SE(3) PGO、优化轨迹发布和基于优化 keyframe 的地图重建。

工程仍然是实验性质。PGO 当前会修正 `/optimized_path`、`/optimized_keyframes` 和 `/global_map` 重建结果，但不会把优化结果反向写回前端 `small_dlio_odom` 的内部状态。

## 当前能力

### 前端 LIO

节点：`small_dlio_odom`，由 `dlio_node` 启动。

- 订阅 Livox `CustomMsg` 点云和 IMU。
- 按 `cloud_accumulate_interval_sec` 累计高频 Livox 包，形成完整扫描。
- 根据 IMU 轨迹做点云去畸变。
- 支持车体附近 3D exclusion box 裁剪，去掉车身/传感器支架附近点。
- 支持两种 scan-to-map submap 选择：
  - `recent_window`：只取最近 N 个 keyframe，默认模式，用于避免前端被旧地图错误吸附。
  - `spatial_knn`：从全历史 keyframe KDTree 中按当前位置召回，兼容旧模式。
- 使用 small_gicp 做 scan-to-map 配准。
- 使用固定结构的几何观测器融合 IMU 预测和 GICP pose。
- 发布 `/odom`、`/pose`、`/path`、`/deskewed`、`/keyframe` 和 `/keyframe_msg`。

### 地图重建

节点：`map_node`。在 `dlio_node` 中和 `small_dlio_odom` 同进程运行，也可以单独启动。

- 订阅 `/keyframe_msg` 保存原始 keyframe 点云。
- 订阅 `/optimized_keyframes` 后使用优化 pose 全量重建 `/global_map`。
- 对尚未被 PGO 覆盖的尾部 keyframe，可按最新 PGO correction 做近似修正。
- 提供 `/save_map` service 保存 PCD。

### 回环与 PGO 后端

节点：`small_dlio_loop_detector`，由 `loop_detector_node` 启动。

- 对收到的 keyframe 再做一层后端稀疏化，避免 LCD/PGO 负载过大。
- LiDAR-Iris 做回环候选匹配。
- Cart Context 做粗召回和二次过滤：
  - retrieval key 支持正向和反向距离。
  - Cart filtering 支持 normal 和 180 度翻转描述子二选一。
- Loop GICP 对候选回环做几何验证，支持单帧或局部 submap。
- g2o PGO：
  - 节点是 SE(3) pose。
  - odom edge 来自连续 keyframe 的相对位姿。
  - loop edge 来自 GICP 验证后的 `history <- current` 测量。
  - 只给 loop edge 挂 Huber robust kernel。
  - loop information matrix 可按 GICP score 动态缩放。
- 发布 `/loop_candidates_marker`、`/optimized_path`、`/optimized_keyframes` 和 `/lcd_pgo_timing`。

### FAST-LIO 后端接入

`fastlio_keyframe_bridge` 可把外部 FAST-LIO 的 `/Odometry` 和 `/cloud_registered_body` 转成 `dlio/msg/KeyFrame`，然后复用本工程的 `map_node`、LCD 和 PGO 后端。

`fastlio_backend.launch.py` 不启动 FAST-LIO 本体，它只启动：

- 可选 Livox 累计脚本 `tools/livox_scan_accumulator.py`
- `fastlio_keyframe_bridge`
- `map_node`
- `loop_detector_node`

## 目录结构

```text
small_dlio/
├── quick_source.sh
├── README.md
├── docs/images/global_map.png
├── tools/livox_scan_accumulator.py
└── src/dlio/
    ├── CMakeLists.txt
    ├── package.xml
    ├── config/
    │   ├── backend.yaml
    │   ├── config.yaml
    │   ├── dlio.rviz
    │   └── fastlio_garage_mid360.yaml
    ├── launch/
    │   ├── full.launch.py
    │   └── fastlio_backend.launch.py
    ├── msg/
    │   ├── KeyFrame.msg
    │   └── OptimizedKeyFrames.msg
    ├── include/small_dlio/
    ├── src/
    │   ├── odom_node.cpp
    │   ├── map_node.cpp
    │   ├── loop_detector_node.cpp
    │   ├── pose_graph.cpp
    │   ├── cart_context.cpp
    │   ├── fastlio_keyframe_bridge.cpp
    │   └── odom_path_bridge.cpp
    └── third_party/lidar_iris/
```

## 依赖

当前工作区是 ROS 2 colcon / ament_cmake 单包工程。

已按本机环境配置的主要依赖：

- Ubuntu 24.04
- ROS 2 Jazzy
- `livox_ros_driver2`
- PCL
- Eigen3
- OpenCV
- OpenMP
- `small_gicp`
- g2o

`quick_source.sh` 默认会 source：

```bash
/opt/ros/jazzy/setup.bash
/home/goose/fastlio/livox_ws/install/setup.bash
/home/goose/small_dlio/install/setup.bash
```

如果你的 Livox/FAST-LIO 工作空间不在 `/home/goose/fastlio/livox_ws`，需要先改 `quick_source.sh` 或手动 source 对应工作区。

## 构建

第一次构建前：

```bash
cd /home/goose/small_dlio
source /opt/ros/jazzy/setup.bash
source /home/goose/fastlio/livox_ws/install/setup.bash
colcon build --packages-select dlio --event-handlers console_direct+
```

之后常用：

```bash
cd /home/goose/small_dlio
source quick_source.sh
colcon build --packages-select dlio
```

## 运行 small_dlio 自带前端

### 启动前端和地图

```bash
cd /home/goose/small_dlio
source quick_source.sh
ros2 launch dlio full.launch.py rviz:=true backend:=false
```

### 启动前端、地图和后端 PGO

```bash
cd /home/goose/small_dlio
source quick_source.sh
ros2 launch dlio full.launch.py rviz:=true backend:=true
```

`full.launch.py` 中：

- `dlio_node` 同时包含 `small_dlio_odom` 和 `map_node`。
- `backend:=true` 会额外启动 `small_dlio_loop_detector`。
- 默认输入话题是 `/livox/lidar` 和 `/livox/imu`。

### 播放第一个测试 rosbag

这个 bag 的原始话题带设备 IP 后缀，需要 remap 到配置文件使用的标准话题：

```bash
cd /home/goose/small_dlio
source quick_source.sh
ros2 bag play /home/goose/下载/rosbag2_2026_04_27-18_26_11 --clock \
  --remap /livox/lidar_192_168_1_185:=/livox/lidar \
          /livox/imu_192_168_1_185:=/livox/imu
```

这个 bag 中也有 `192_168_1_3` 设备话题。当前默认命令使用 `192_168_1_185` 这一组。

## 运行 FAST-LIO 前端 + 本项目后端

前提：外部 FAST-LIO 正在发布：

- `/Odometry`
- `/cloud_registered_body`

启动本项目后端：

```bash
cd /home/goose/small_dlio
source quick_source.sh
ros2 launch dlio fastlio_backend.launch.py backend:=true accumulator:=true
```

常用 launch 参数：

```bash
ros2 launch dlio fastlio_backend.launch.py \
  map_frame:=camera_init \
  body_frame:=body \
  backend:=true \
  accumulator:=true \
  accumulator_input_topic:=/livox/lidar_192_168_1_185 \
  accumulator_output_topic:=/livox/lidar \
  odom_topic:=/Odometry \
  cloud_topic:=/cloud_registered_body \
  candidate_keyframe_topic:=/keyframe_candidates \
  keyframe_topic:=/keyframe_msg
```

`fastlio_backend.launch.py` 会把 bridge 产生的 `/keyframe_candidates` 送进 loop detector，loop detector 通过 `filtered_keyframe_topic` 再发布筛选后的 `/keyframe_msg` 给 map_node。

## 话题和服务

### `dlio_node`

订阅：

- `/livox/imu` (`sensor_msgs/msg/Imu`)
- `/livox/lidar` (`livox_ros_driver2/msg/CustomMsg`)

发布：

- `/odom` (`nav_msgs/msg/Odometry`)
- `/pose` (`geometry_msgs/msg/PoseStamped`)
- `/path` (`nav_msgs/msg/Path`)
- `/deskewed` (`sensor_msgs/msg/PointCloud2`)
- `/keyframe` (`sensor_msgs/msg/PointCloud2`)
- `/keyframe_msg` (`dlio/msg/KeyFrame`)
- `/global_map` (`sensor_msgs/msg/PointCloud2`)
- `/tf`
- `/tf_static`

服务：

- `/save_map` (`std_srvs/srv/Trigger`)

### `small_dlio_loop_detector`

订阅：

- `/keyframe_msg` 或 launch remap 后的 candidate topic

发布：

- `/loop_candidates_marker` (`visualization_msgs/msg/MarkerArray`)
- `/optimized_path` (`nav_msgs/msg/Path`)
- `/optimized_keyframes` (`dlio/msg/OptimizedKeyFrames`)
- `/lcd_pgo_timing` (`geometry_msgs/msg/Vector3Stamped`)
- 可选 `filtered_keyframe_topic`

### `fastlio_keyframe_bridge`

订阅：

- `/Odometry`
- `/cloud_registered_body`

发布：

- `/keyframe_candidates` 或 `/keyframe_msg`

## Keyframe 机制

当前项目有两层 keyframe 阈值。

前端生成 keyframe，配置在 `src/dlio/config/config.yaml` 的 `small_dlio_odom`：

```yaml
kf_trans_thresh: 0.1
kf_rot_thresh: 0.05
max_alignment_score: 3.0
```

后端 LCD/PGO 接收 keyframe，配置在 `src/dlio/config/backend.yaml` 的 `small_dlio_loop_detector`：

```yaml
kf_trans_thresh: 1.0
kf_rot_thresh: 0.3
min_kf_interval_sec: 0.0
min_cloud_points: 10
```

前端 `/keyframe_msg` 可以很密，后端会再筛一遍。日志中类似：

```text
Skipped keyframe candidate: candidate_id=748 stored=161
```

表示前端已经发布到 id 748，但后端 LCD/PGO 只接受了 161 个 keyframe。

注意：loop detector 现在保留输入 keyframe 的原始 id。`/optimized_keyframes` 的 id 必须和 map_node 存储的 raw keyframe id 保持一致，否则优化 pose 会套到错误点云上。

## 关键参数

### 前端 `config.yaml`

文件：`src/dlio/config/config.yaml`

常用参数：

- `cloud_accumulate_interval_sec`：Livox 点云累计时间，默认 `0.1s`。
- `keyframe_exclusion_box_enable`：是否删除车体附近 3D box 内点。
- `keyframe_exclusion_min_* / max_*`：body frame 下 exclusion box 范围。
- `submap_selection_mode`：`recent_window` 或 `spatial_knn`。
- `submap_window_keyframes`：recent window 模式下 scan-to-map 使用的最近 keyframe 数。
- `knn_limit`、`max_distance`：spatial KNN 模式下的历史 keyframe 召回参数。
- `gicp_leaf_size`：GICP 输入降采样 leaf size。
- `gicp_max_correction_trans`、`gicp_max_correction_rot_deg`：单次 GICP 修正幅度门限。
- `gicp_max_imu_to_gicp_trans`、`gicp_max_imu_to_gicp_rot_deg`：IMU prior 和 GICP pose 差异门限。
- `geo_Kp/Kv/Kq/Ka/Kg`：几何观测器固定增益。
- `map_leaf_size`：全局地图重建后的 voxel leaf size。

默认 submap 模式：

```yaml
submap_selection_mode: "recent_window"
submap_window_keyframes: 20
submap_window_min_keyframes: 5
submap_window_max_travel_distance: -1.0
```

如果要临时恢复旧的全历史空间 KNN：

```yaml
submap_selection_mode: "spatial_knn"
```

### 后端 `backend.yaml`

文件：`src/dlio/config/backend.yaml`

常用参数：

- `loop_iris_distance_thresh`：LiDAR-Iris 候选阈值。
- `lcd_retrieval_enable`：是否用 Cart retrieval key 做粗召回。
- `lcd_retrieval_top_k`：粗召回送入 Iris 精排的历史帧数量。
- `cart_enable`：是否启用 Cart Context 过滤。
- `cart_distance_thresh`：Cart Context 接受阈值。
- `loop_gicp_enable`：是否用 GICP 验证回环。
- `loop_gicp_score_thresh`：loop GICP score 门限。
- `loop_gicp_max_correction_trans`、`loop_gicp_max_correction_rot_deg`：loop GICP 相对初值的最大修正。
- `loop_gicp_use_submap`：loop GICP 是否使用 keyframe 子图。
- `pgo_enable`：是否启用 g2o PGO。
- `pgo_optimize_on_loop`：是否只在新增 loop edge 后优化。
- `pgo_odom_info_diag`：odom edge 信息矩阵对角线 `[x,y,z,roll,pitch,yaw]`。
- `pgo_loop_info_diag`：loop edge 基础信息矩阵对角线。
- `pgo_loop_info_dynamic_enable`：是否按 GICP score 缩放 loop information。
- `pgo_loop_robust_kernel_enable`：是否给 loop edge 启用 Huber robust kernel。
- `loop_edge_min_current_gap`、`loop_edge_min_travel_gap`：loop edge 稀疏化。

当前 loop information 动态缩放公式：

```text
scale = clamp(score_ref / max(gicp_score, score_floor), min_scale, max_scale)
loop_information = base_loop_information * scale
```

## 保存地图

运行中调用：

```bash
ros2 service call /save_map std_srvs/srv/Trigger {}
```

默认保存路径：

```text
/home/goose/small_dlio/global_map.pcd
```

保存路径由 `config.yaml` 中 `map_node.ros__parameters.map_save_path` 控制。

## 常用检查命令

确认前端是否有输出：

```bash
ros2 topic hz /odom
ros2 topic hz /keyframe_msg
ros2 topic hz /global_map
```

确认后端是否发布优化轨迹：

```bash
ros2 topic echo /optimized_path --once
ros2 topic echo /optimized_keyframes --once
```

确认 loop marker：

```bash
ros2 topic echo /loop_candidates_marker --once
```

查看 bag 话题：

```bash
ros2 bag info /home/goose/下载/rosbag2_2026_04_27-18_26_11
```

## 重要日志

前端：

```text
Odom input exclusion box removed ...
Submap selection: mode=...
Submap selected: mode=recent_window selected=...
gicp accepted ...
Published keyframe_msg ...
Map rebuild stats ...
```

后端：

```text
Loop descriptor top-k ...
Loop init debug ...
GICP candidate accepted ...
Loop candidate selected ...
Loop edge debug ...
PGO optimized ...
Published optimized_path ...
```

调试 PGO 地图跳变时，重点看：

- `Loop edge debug` 中 `odom_vs_direct_dp/dr` 和 `odom_vs_inverse_dp/dr`。
- `loop_info_scale` 是否被放大到 `2.0`。
- `Map rebuild stats` 中 `optimized`、`corrected_unoptimized` 和 `correction_t/rot`。
- `/optimized_keyframes` 的 id 是否能和 `/keyframe_msg` 原始 id 对齐。

## 常见问题

### RViz 没有地图

先确认输入话题是否匹配。第一个 bag 的点云和 IMU 话题不是 `/livox/lidar`、`/livox/imu`，必须 remap：

```bash
ros2 bag play /home/goose/下载/rosbag2_2026_04_27-18_26_11 --clock \
  --remap /livox/lidar_192_168_1_185:=/livox/lidar \
          /livox/imu_192_168_1_185:=/livox/imu
```

再检查：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /odom
```

### PGO 后地图突然乱飞

优先排查错误 loop edge：

- 降低 `loop_iris_distance_thresh`。
- 降低或关闭 `pgo_loop_info_dynamic_enable`。
- 临时关闭 `lcd_retrieval_enable` 对比。
- 查看 `Loop edge debug` 的 direct/inverse delta。

如果 `Map rebuild stats` 中 `correction_t` 很大，需要确认 `/optimized_keyframes` id 和 map_node 存的 raw keyframe id 是否一致。

### 前端跑一半漂移或飞走

`spatial_knn` 会从全历史地图中按当前位置召回 keyframe，可能在重复结构中错误吸附。默认使用 `recent_window` 是为了让前端只做局部里程计，把全局一致性交给后端 PGO。

如果 GICP 单次修正过大，可以收紧：

```yaml
gicp_max_correction_trans: 1.0
gicp_max_correction_rot_deg: 8.0
gicp_max_imu_to_gicp_trans: 1.0
gicp_max_imu_to_gicp_rot_deg: 8.0
```

### 后端 stored keyframe 比前端 id 小很多

这是正常现象。前端负责生成较密的 `/keyframe_msg`，后端 `small_dlio_loop_detector` 还会按 `backend.yaml` 的 `kf_trans_thresh/kf_rot_thresh` 再做稀疏化。LCD/PGO 只处理后端接受的 keyframe。

## 设计限制

- PGO 不反向更新 `small_dlio_odom` 内部状态，前端仍在自己的 odom frame 中继续运行。
- `recent_window` 前端不会主动吸回历史地图，全局 drift 需要后端回环修正。
- `spatial_knn` 前端可能在重复结构中错误吸附。
- LiDAR-Iris/Cart/GICP 在车库、长走廊等重复场景中仍可能产生假回环，需要依赖门限、GICP gate 和 PGO robust kernel 降低风险。
- 当前 `quick_source.sh` 包含本机固定路径，迁移机器时需要改。

## 可视化截图

历史全局地图示例：

![Global Map](docs/images/global_map.png)
