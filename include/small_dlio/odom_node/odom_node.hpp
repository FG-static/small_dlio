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

        bool keyframeDetection(
            const Eigen::Matrix4d &trans_gicp
        );

        bool scanToMap(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud,
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud,
            Eigen::Matrix4d &trans_gicp,
            double &alignment_score
        ) const;

        bool integrateImu(
            State &state_end,
            const double &t_point
        );

        std::deque<ImuMeas> imu_data_;

        int knn_limit_ = 5;
        double max_distance_ = 20.0;
        double gicp_leaf_size_ = 0.10;
    };

}; // small_dlio

#endif // SMALL_DLIO__ODOM_NODE
