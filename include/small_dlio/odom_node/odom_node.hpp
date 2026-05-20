#ifndef SMALL_DLIO__ODOM_NODE
#define SMALL_DLIO__ODOM_NODE

#include "point_types.hpp"

namespace small_dlio {

    class OdomNode : public rclcpp::Node {

    public:

        OdomNode();
    private:

        void loadParams();

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
            const std::vector<KeyFrame> &keyframes,
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
