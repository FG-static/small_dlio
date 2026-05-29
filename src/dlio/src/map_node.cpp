#include "map_node.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>

namespace small_dlio {

    MapNode::MapNode() : Node("map_node") {

        loadParams();

        global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        pub_map_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            map_topic_, rclcpp::QoS(1).reliable().transient_local()
        );

        sub_keyframe_ =
            create_subscription<sensor_msgs::msg::PointCloud2>(
                keyframe_topic_, rclcpp::SensorDataQoS(),
                [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
                    callbackKeyframeCloud(msg);
                }
            );
    }

    void MapNode::loadParams() {

        this->declare_parameter("map_leaf_size", 0.1);
        this->declare_parameter("map_topic", "/global_map");
        this->declare_parameter("keyframe_topic", "/keyframe");

        this->get_parameter("map_leaf_size", map_leaf_size_);
        this->get_parameter("map_topic", map_topic_);
        this->get_parameter("keyframe_topic", keyframe_topic_);
    }

    void MapNode::callbackKeyframeCloud(
        sensor_msgs::msg::PointCloud2::ConstSharedPtr msg
    ) {

        auto cloud =
            std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(*msg, *cloud);

        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setLeafSize(
            static_cast<float>(map_leaf_size_),
            static_cast<float>(map_leaf_size_),
            static_cast<float>(map_leaf_size_)
        );
        auto cloud_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        voxel_filter.setInputCloud(cloud);
        voxel_filter.filter(*cloud_filtered);

        *global_map_ += *cloud_filtered;

        sensor_msgs::msg::PointCloud2 map_msg;
        pcl::toROSMsg(*global_map_, map_msg);
        map_msg.header.stamp = msg->header.stamp;
        map_msg.header.frame_id = msg->header.frame_id;
        pub_map_->publish(map_msg);
    }

} // small_dlio
