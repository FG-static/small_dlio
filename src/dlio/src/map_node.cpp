#include "map_node.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

namespace small_dlio {

    MapNode::MapNode() : Node("map_node") {

        loadParams();

        global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        pub_map_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            map_topic_, rclcpp::QoS(1).reliable().transient_local()
        );

        srv_save_map_ = create_service<std_srvs::srv::Trigger>(
            save_map_service_name_,
            [this](
                const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response
            ) {

                callbackSaveMap(request, response);
            }
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
        this->declare_parameter("save_map_service_name", "/save_map");
        this->declare_parameter("map_save_path", "/home/goose/small_dlio/global_map.pcd");

        this->get_parameter("map_leaf_size", map_leaf_size_);
        this->get_parameter("map_topic", map_topic_);
        this->get_parameter("keyframe_topic", keyframe_topic_);
        this->get_parameter("save_map_service_name", save_map_service_name_);
        this->get_parameter("map_save_path", map_save_path_);
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

        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            *global_map_ += *cloud_filtered;

            sensor_msgs::msg::PointCloud2 map_msg;
            pcl::toROSMsg(*global_map_, map_msg);
            map_msg.header.stamp = msg->header.stamp;
            map_msg.header.frame_id = msg->header.frame_id;
            pub_map_->publish(map_msg);
        }
    }

    void MapNode::callbackSaveMap(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response
    ) {

        (void)request;

        std::lock_guard<std::mutex> lock(map_mutex_);
        if (global_map_->empty()) {

            response->success = false;
            response->message = "global_map is empty";
            return;
        }

        const int ret = pcl::io::savePCDFileBinary(map_save_path_, *global_map_);
        response->success = (ret == 0);
        response->message = response->success
            ? "saved map to " + map_save_path_
            : "failed to save map to " + map_save_path_;
    }

} // small_dlio
