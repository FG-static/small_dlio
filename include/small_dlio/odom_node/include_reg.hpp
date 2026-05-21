#ifndef SMALL_DLIO__INCLUDE_REG
#define SMALL_DLIO__INCLUDE_REG

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <mutex>
#include <deque>
#include <vector>
#include <cmath>
#include <algorithm>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <pcl-1.14/pcl/point_cloud.h> // 点云基础类型
#include <pcl-1.14/pcl/point_types.h> // 点类型定义
#include <pcl_conversions/pcl_conversions.h> // ros2和pcl消息互转
#include <pcl-1.14/pcl/filters/voxel_grid.h> // 滤波器
#include <pcl-1.14/pcl/common/transforms.h> // 点云坐标变换

#include <unordered_map>

// FLANN (used by small_gicp) expects a serialization implementation for
// std::unordered_map when saving/loading LSH indices. The system FLANN
// version shipped with Ubuntu does not provide this specialization, so we
// provide it here to avoid compilation errors.
namespace flann {
namespace serialization {

// Forward-declare the primary template so we can provide a specialization
// before the full FLANN serialization headers are included.
template<typename T>
struct Serializer;

template<typename K, typename V>
struct Serializer<std::unordered_map<K, V>>
{
  template<typename InputArchive>
  static inline void load(InputArchive &ar, std::unordered_map<K, V> &map_val)
  {
    size_t size;
    ar & size;
    for (size_t i = 0; i < size; ++i) {
      K key;
      V value;
      ar & key;
      ar & value;
      map_val.emplace(std::move(key), std::move(value));
    }
  }

  template<typename OutputArchive>
  static inline void save(OutputArchive &ar, const std::unordered_map<K, V> &map_val)
  {
    ar & map_val.size();
    for (const auto &kv : map_val) {
      ar & kv.first;
      ar & kv.second;
    }
  }
};

}  // namespace serialization
}  // namespace flann

#include <small_gicp/pcl/pcl_registration.hpp> // pcl结合使用
#include <small_gicp/registration/registration.hpp> // 核心注册算法
#include <small_gicp/factors/gicp_factor.hpp>

#endif // SMALL_DLIO__INCLUDE_REG
