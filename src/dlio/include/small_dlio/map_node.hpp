#ifndef SMALL_DLIO__MAP_NODE
#define SMALL_DLIO__MAP_NODE

#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/publisher.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace small_dlio {

    class MapNode : public rclcpp::Node {

    public:

        MapNode();
    private:

        void loadParams();

        void callbackKeyframeCloud(
            sensor_msgs::msg::PointCloud2::ConstSharedPtr msg
        );

        pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_keyframe_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;

        double map_leaf_size_ = 0.1;
        std::string map_topic_ = "/global_map";
        std::string keyframe_topic_ = "/keyframe";
    };
} // small_dlio

#endif // SMALL_DLIO__MAP_NODE
