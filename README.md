# small_dlio

一个基于 ROS 2 Jazzy 的轻量级 LiDAR-IMU 里程计/建图实验工程，当前面向 Livox Mid-360 数据流开发。

当前仓库包含：

- `OdomNode`：IMU 预积分、点云去畸变、submap 构建、GICP 配准、状态发布
- `MapNode`：订阅 keyframe 点云、体素滤波、在线累积并发布全局地图，支持手动保存 PCD

## 当前状态

目前工程重点仍是里程计主链路，建图部分已经具备基础在线建图能力，当前支持：

- 订阅 `/keyframe`
- 对每帧 keyframe 做 voxel filter
- 累积 keyframe 点云
- 发布 `/global_map`
- 通过 `/save_map` service 保存当前全局地图为 PCD
- 使用保存下来的 `global_map.pcd` 离线渲染地图图片

但还没有做完整的地图管理能力，例如：

- 全局地图的二次重采样或分层管理
- 地图加载与重发布
- 分块地图或局部地图裁剪
- 后台优化/回环

## 目录结构

```text
small_dlio/
├── src/dlio/
│   ├── config/config.yaml
│   ├── include/small_dlio/
│   ├── launch/full.launch.py
│   └── src/
│       ├── main.cpp
│       ├── odom_node.cpp
│       └── map_node.cpp
├── quick_source.sh
└── README.md
```

## 依赖环境

- Ubuntu 24.04
- ROS 2 Jazzy
- `livox_ros_driver2`
- PCL
- Eigen3
- OpenMP
- `small_gicp`

构建前需要 source：

```bash
source /opt/ros/jazzy/setup.bash
source /home/goose/fastlio/livox_ws/install/setup.bash
```

或者使用一键 source 脚本：

```bash
source quick_source.sh
```

## 构建

```bash
colcon build --packages-select dlio --event-handlers console_direct+
```

## 运行

启动节点：

```bash
source install/setup.bash
ros2 launch dlio full.launch.py rviz:=false
```

回放当前测试 bag：

```bash
ros2 bag play /home/goose/fastlio/rosbag2_mid360_10hz --clock
```

如果需要 RViz：

```bash
ros2 launch dlio full.launch.py rviz:=true
```

## 默认订阅与发布

### 订阅

- `/livox/imu`
- `/livox/lidar`
- `/keyframe`

### 发布

- `/odom`
- `/pose`
- `/path`
- `/tf`
- `/tf_static`
- `/deskewed`
- `/global_map`

### Service

- `/save_map`：保存当前 `global_map_` 到 PCD

## 关键参数

配置文件位置：

[`src/dlio/config/config.yaml`](/home/goose/small_dlio/src/dlio/config/config.yaml)

当前较关键的参数包括：

- `kf_trans_thresh`：关键帧平移阈值
- `kf_rot_thresh`：关键帧旋转阈值
- `max_alignment_score`：GICP 接受阈值
- `gicp_leaf_size`：配准前点云降采样体素大小
- `gicp_num_threads`：GICP 线程数
- `map_leaf_size`：`MapNode` 对 keyframe 做体素滤波时使用的 leaf size
- `save_map_service_name`：地图保存 service 名称
- `map_save_path`：PCD 默认保存路径

## 保存地图

运行过程中可以手动调用：

```bash
ros2 service call /save_map std_srvs/srv/Trigger {}
```

默认会把地图保存到：

```text
/home/goose/small_dlio/global_map.pcd
```

## 可视化检查

回放 bag 后，至少应能看到这些话题：

- `/odom`
- `/pose`
- `/path`
- `/tf`
- `/deskewed`
- `/global_map`

建议固定坐标系使用 `odom`。

## 运行截图

下面这张图由保存下来的 `global_map.pcd` 离线渲染得到，左侧是俯视图，右侧是斜视图。

![Global Map](docs/images/global_map.png)

## 后续待做

- 对 `global_map_` 增加全局重滤波或分块管理，控制地图规模
- 增加地图加载与重发布能力
- 将 odom / map 节点进一步解耦
- 为保存地图增加可配置输出格式或保存前重采样选项
- 评估是否引入回环或离线建图流程
