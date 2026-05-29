#ifndef SMALL_DLIO__MAP_NODE
#define SMALL_DLIO__MAP_NODE

#include <mutex>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace small_dlio {

    class MapNode : public rclcpp::Node {

    public:

        MapNode();
    private:

        void loadParams();

        void callbackKeyframeCloud(
            sensor_msgs::msg::PointCloud2::ConstSharedPtr msg
        );
        void callbackSaveMap(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response
        );

        pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
        std::mutex map_mutex_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_keyframe_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_save_map_;

        double map_leaf_size_ = 0.1;
        std::string map_topic_ = "/global_map";
        std::string keyframe_topic_ = "/keyframe";
        std::string save_map_service_name_ = "/save_map";
        std::string map_save_path_ = "/home/goose/small_dlio/global_map.pcd";
    };
} // small_dlio

#endif // SMALL_DLIO__MAP_NODE
