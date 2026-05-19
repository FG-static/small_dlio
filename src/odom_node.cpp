#include "odom_node/odom_node.hpp"

namespace small_dlio {

    OdomNode::OdomNode() : Node("small_dlio_odom") {

        loadParams();
    }

    bool OdomNode::motionCorrection(
        const pcl::PointCloud<PointXYZIT>::Ptr &cloud_in,
        const State &prev_state,
        const double &scan_start_time,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out,
        State &state_end
    ) {
        
        pcl::PointCloud<PointXYZIT> cloud_sorted = *cloud_in;
        std::sort(cloud_sorted.points.begin(), cloud_sorted.points.end(),
                  [](const PointXYZIT &a, const PointXYZIT &b) {
                      return a.offset_time < b.offset_time;
                  });
        
        state_end = prev_state;
        cloud_out->clear();
        cloud_out->reserve(cloud_sorted.size());

        for (const auto &point : cloud_sorted) {

            double t_point = scan_start_time + point.offset_time;
            if (!integrateImu(state_end, t_point)) {
             
                RCLCPP_WARN(get_logger(), "Failed to integrate IMU data at time %f", t_point);
                continue;
            }

            Eigen::Matrix4d T = Eigen::Matrix4d::Zero();
            T.block<3, 3>(0, 0) = state_end.pose.q.toRotationMatrix();
            T.block<3, 1>(0, 3) = state_end.pose.p;
            T(3, 3) = 1.0;

            Eigen::Vector4d point_lidar(point.x, point.y, point.z, 1.0);
            Eigen::Vector4d point_world = T * point_lidar;

            pcl::PointXYZ p;
            p.x = static_cast<float>(point_world.x());
            p.y = static_cast<float>(point_world.y());
            p.z = static_cast<float>(point_world.z());
            cloud_out->push_back(p);
        }
        return true;
    }

    bool OdomNode::submapGeneration(
        const std::vector<KeyFrame> &keyframes,
        const State &cur_state,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_submap
    ) const {

        cloud_submap->clear();
        
        // TODO: 临时使用kNN暴力匹配，后续换kdtree加速
        std::vector<std::pair<double, size_t>> dists;
        for (size_t i = 0; i < keyframes.size(); i ++) {

            double d = (keyframes[i].pose.p - cur_state.pose.p).norm();
            dists.emplace_back(d, i);
        }
        std::sort(dists.begin(), dists.end());
        for (size_t i = 0; i < std::min(dists.size(), static_cast<size_t>(knn_limit_)); i ++) {

            if (dists[i].first > max_distance_) break;
            *cloud_submap += *keyframes[dists[i].second].cloud;
        }

        return true;
    }

    bool OdomNode::scanToMap(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud,
        Eigen::Matrix4d &trans_gicp,
        double &alignment_score
    ) const {

        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setLeafSize(
            static_cast<float>(gicp_leaf_size_),
            static_cast<float>(gicp_leaf_size_),
            static_cast<float>(gicp_leaf_size_)
        );

        pcl::PointCloud<pcl::PointXYZ>::Ptr 
            filtered_source(new pcl::PointCloud<pcl::PointXYZ>),
            filtered_target(new pcl::PointCloud<pcl::PointXYZ>);
        
        voxel_filter.setInputCloud(source_cloud);
        voxel_filter.filter(*filtered_source);
        voxel_filter.setInputCloud(target_cloud);
        voxel_filter.filter(*filtered_target);

        small_gicp::RegistrationPCL<pcl::PointXYZ, pcl::PointXYZ> reg;
        reg.setNumThreads(4);
        reg.setCorrespondenceRandomness(20);
        reg.setMaxCorrespondenceDistance(1.0);
        reg.setRegistrationType("GICP");

        reg.setInputSource(filtered_source);
        reg.setInputTarget(filtered_target);

        auto aligned = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity(); // TODO: 先单位阵，后期改为上一次猜测结果

        try {

            reg.align(*aligned, init_guess);
        } catch (const std::exception &e) {

            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "GICP 配准异常，跳过本帧: %s", e.what());
            trans_gicp = Eigen::Matrix4d::Identity();
            alignment_score = 1e9;
            return false;
        }

        alignment_score = reg.getFitnessScore();
        trans_gicp = reg.getFinalTransformation().cast<double>();
        if (!trans_gicp.allFinite()) {

            alignment_score = 1e9;
            trans_gicp = Eigen::Matrix4d::Identity();
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "GICP 配准存在非法，跳过本帧");
            return false;
        }
        return true;
    }
} // small_dlio