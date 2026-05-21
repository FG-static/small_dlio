#ifndef SMALL_DLIO__ODOM_NODE
#define SMALL_DLIO__ODOM_NODE

#include "point_types.hpp"
#include "rclcpp/time.hpp"

namespace small_dlio {

    class OdomNode : public rclcpp::Node {

    public:

        OdomNode();
    private:

        // ROS 层面
        void callbackImu(
            const sensor_msgs::msg::Imu::SharedPtr &msg
        );
        void callbackLivoxCloud(
            const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg
        );

        void publishOdometry();
        void publishTf();

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr
            sub_imu_;
        rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr
            sub_cloud_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
            pub_odom_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
            pub_pose_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
            pub_path_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        std::mutex mtx_;
        rclcpp::CallbackGroup::SharedPtr imu_cb_group_;
        rclcpp::CallbackGroup::SharedPtr cloud_cb_group_;

        double prev_imu_stamp_ = 0.0;
        double prev_scan_stamp_ = 0.0;
        Eigen::Vector3d prev_acc_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d prev_gyro_ = Eigen::Vector3d::Zero();

        bool initialized_ = false;

        void loadParams();

        // 分块核心函数
        bool motionCorrection(
            const pcl::PointCloud<PointXYZIT>::Ptr &cloud_in,
            const State &prev_state,
            const double &scan_start_time,
            pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out,
            State &state_end
        );

        // TODO: maybe remove
        bool optimizationPrior(
            const State &state,
            Eigen::Matrix4d &trans_lidar_to_world
        ) const;

        bool submapGeneration(
            const State &cur_state,
            pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_submap
        ) const;

        bool scanToMap(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud,
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud,
            Eigen::Matrix4d &trans_gicp,
            double &alignment_score
        ) const;

        bool geometricFuser(
            const State &imu_state,
            const Pose &gicp_pose,
            const double &dt,
            State &fused_state
        ) const;

        bool keyframeDetection(
            const State &fused_state,
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_world,
            const double alignment_score
        );

        bool statePropagation(
            const ImuMeas &imu,
            const double &dt
        );

        bool integrateStep(
            State &state,
            const ImuMeas &imu,
            const double &dt
        ) const;

        bool integrateImu(
            State &state_end,
            const double &t_point
        );

        State state_;
        rclcpp::Time current_stamp_;

        std::deque<ImuMeas> imu_data_;
        std::vector<KeyFrame> keyframes_;

        int knn_limit_ = 5;
        double max_distance_ = 20.0;
        double gicp_leaf_size_ = 0.10;

        double Kp_ = 4.5;
        double Kq_ = 4.0;
        double Kv_ = 11.25;
        double Ka_ = 2.25;
        double Kg_ = 1.0;
        double b_max_ = 1.0; // TODO: 后期上下限单独分开

        double kf_trans_thresh_ = 0.5;
        double kf_rot_thresh_ = 10.0 * M_PI / 180.0;
        double max_alignment_score_ = 1.0;

        Eigen::Vector3d gravity_ = {0, 0, 9.8};
    };

}; // small_dlio

#endif // SMALL_DLIO__ODOM_NODE
