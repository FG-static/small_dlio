#ifndef SMALL_DLIO__ODOM_NODE
#define SMALL_DLIO__ODOM_NODE

#include "nav_msgs/msg/path.hpp"
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
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
            pub_cloud_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

        std::mutex mtx_;
        rclcpp::CallbackGroup::SharedPtr imu_cb_group_;
        rclcpp::CallbackGroup::SharedPtr cloud_cb_group_;

        double prev_imu_stamp_ = 0.0;
        double prev_scan_stamp_ = 0.0;
        Eigen::Vector3d prev_acc_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d prev_gyro_ = Eigen::Vector3d::Zero();

        bool initialized_ = false;

        template <typename T>
        void declare_param(const std::string &name, T &param, const T &default_value) {
            this->declare_parameter(name, default_value);
            this->get_parameter(name, param);
        }

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
        Extrinsics extrinsics_;
        rclcpp::Time current_stamp_;

        std::deque<ImuMeas> imu_data_;
        std::vector<KeyFrame> keyframes_;
        nav_msgs::msg::Path path_;

        // Topics & frames
        std::string imu_topic_ = "/livox/imu";
        std::string cloud_topic_ = "/livox/lidar";
        std::string odom_frame_ = "odom";
        std::string body_frame_ = "body";
        std::string lidar_frame_ = "livox_frame";

        // Submap
        int knn_limit_ = 5;
        double max_distance_ = 20.0;

        // GICP
        double gicp_leaf_size_ = 0.10;
        int gicp_num_threads_ = 4;
        int gicp_correspondence_randomness_ = 20;
        double gicp_max_correspondence_distance_ = 1.0;

        // Geometric observer
        double Kp_ = 1.0;
        double Kq_ = 1.0;
        double Kv_ = 1.0;
        double Ka_ = 1.0;
        double Kg_ = 1.0;
        double b_max_ = 1.0;

        // Keyframe detection
        double kf_trans_thresh_ = 0.5;
        double kf_rot_thresh_ = 10.0 * M_PI / 180.0;
        double max_alignment_score_ = 1.0;

        Eigen::Vector3d gravity_ = {0, 0, 9.8};
    };

}; // small_dlio

#endif // SMALL_DLIO__ODOM_NODE
