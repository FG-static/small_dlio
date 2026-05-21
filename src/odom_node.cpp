#include "odom_node/odom_node.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Geometry/Quaternion.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pcl/impl/point_types.hpp"
#include "pcl/point_cloud.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <memory>

namespace small_dlio {

    OdomNode::OdomNode() : Node("small_dlio_odom") {

        loadParams();

        imu_cb_group_ = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive
        );
        cloud_cb_group_ = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive
        );

        auto imu_opt = rclcpp::SubscriptionOptions();
        imu_opt.callback_group = imu_cb_group_;
        auto cloud_opt = rclcpp::SubscriptionOptions();
        cloud_opt.callback_group = cloud_cb_group_;

        sub_imu_ = 
            create_subscription<sensor_msgs::msg::Imu>(
                "imu", 10,
                [this](sensor_msgs::msg::Imu::SharedPtr &msg) {

                    callbackImu(msg);
                },
                imu_opt
            );
        sub_cloud_ =
            create_subscription<livox_ros_driver2::msg::CustomMsg>(
                "/livox/lidar", 10,
                [this](livox_ros_driver2::msg::CustomMsg::SharedPtr &msg) {
                 
                    callbackLivoxCloud(msg);
                },
                cloud_opt
            );
        
        pub_odom_ = 
            create_publisher<nav_msgs::msg::Odometry>(
                "odom", 10
            );
        pub_path_ = 
            create_publisher<nav_msgs::msg::Path>(
                "path", 10
            );
        pub_pose_ = 
            create_publisher<geometry_msgs::msg::PoseStamped>(
                "pose", 10
            );
    }

    /**
     * @brief 去除点云运动畸变 + 将点云转换到世界坐标系下
     * @param cloud_in 输入点云
     * @param prev_state 上一帧的积分后状态
     * @param scan_start_time 该帧点云的扫描开始时间
     * @param cloud_out 输出去畸变点云
     * @param state_end 输出积分后状态
     * @return bool 是否成功
     * 
     * 将点云按 offset_time 排序后对每个点按时间戳进行 imu 数据积分，同时同步推进
     * 当前小车状态，在完成所有点云处理后小车状态会被积分到点云扫描结束的位置，在处理
     * 点云后，还对每个点云做了转换到世界坐标系的变换
     */
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
        const State &cur_state,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_submap
    ) const {

        cloud_submap->clear();
        
        // TODO: 临时使用kNN暴力匹配，后续换kdtree加速
        std::vector<std::pair<double, size_t>> dists;
        for (size_t i = 0; i < keyframes_.size(); i ++) {

            double d = (keyframes_[i].pose.p - cur_state.pose.p).norm();
            dists.emplace_back(d, i);
        }
        std::sort(dists.begin(), dists.end());
        for (size_t i = 0; i < std::min(dists.size(), static_cast<size_t>(knn_limit_)); i ++) {

            if (dists[i].first > max_distance_) break;
            *cloud_submap += *keyframes_[dists[i].second].cloud;
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
                "GICP 配准存在非法结果，跳过本帧");
            return false;
        }
        return true;
    }

    bool OdomNode::geometricFuser(
        const State &imu_state,
        const Pose &gicp_pose,
        const double &dt,
        State &fused_state
    ) const {

        if (dt <= 0.0 || !std::isfinite(dt) ||
            !imu_state.pose.p.allFinite() || !gicp_pose.p.allFinite() ||
            !imu_state.pose.q.coeffs().allFinite() || !gicp_pose.q.coeffs().allFinite()) {
                
            fused_state = imu_state;
            return false;
        }

        const Eigen::Quaterniond q_hat = imu_state.pose.q.normalized();
        const Eigen::Quaterniond q_in = gicp_pose.q.normalized();

        const Eigen::Vector3d e_p = gicp_pose.p - imu_state.pose.p;
        const Eigen::Vector3d e_p_body = q_hat.conjugate() * e_p;

        const Eigen::Quaterniond q_e = (q_hat.conjugate() * q_in).normalized();
        const double q_e_sign = q_e.w() >= 0.0 ? 1.0 : -1.0;
        const Eigen::Quaterniond q_delta(
            1.0 - std::abs(q_e.w()),
            q_e_sign * q_e.x(),
            q_e_sign * q_e.y(),
            q_e_sign * q_e.z());
        const Eigen::Quaterniond q_corr = q_hat * q_delta;

        fused_state = imu_state;
        fused_state.b_a = imu_state.b_a - dt * Ka_ * e_p_body;
        fused_state.b_g = imu_state.b_g - dt * Kg_ * Eigen::Vector3d(q_e.x(), q_e.y(), q_e.z());
        fused_state.b_a = fused_state.b_a.cwiseMax(-b_max_).cwiseMin(b_max_);
        fused_state.b_g = fused_state.b_g.cwiseMax(-b_max_).cwiseMin(b_max_);
        fused_state.pose.p = imu_state.pose.p + dt * Kp_ * e_p;
        fused_state.v = imu_state.v + dt * Kv_ * e_p;
        fused_state.pose.q.coeffs() = q_hat.coeffs() + dt * Kq_ * q_corr.coeffs();
        fused_state.pose.q.normalize();

        return true;
    }

    bool OdomNode::keyframeDetection(
        const State &fused_state,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_world,
        const double alignment_score
    ) {

        if (!cloud_world || cloud_world->empty() ||
            !std::isfinite(alignment_score) ||
            alignment_score > max_alignment_score_ ||
            !fused_state.pose.p.allFinite() ||
            !fused_state.pose.q.coeffs().allFinite()) 
            return false;

        KeyFrame kf;
        kf.pose = fused_state.pose;
        kf.pose.q.normalize();
        kf.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>(*cloud_world));

        if (keyframes_.empty()) {

            keyframes_.push_back(kf);
            return true;
        }

        const KeyFrame &last_kf = keyframes_.back();
        const double trans = (fused_state.pose.p - last_kf.pose.p).norm();

        Eigen::Quaterniond dq =
            (last_kf.pose.q.conjugate() * kf.pose.q).normalized();
        if (dq.w() < 0.0) dq.coeffs() *= -1.0;
        const double rot = Eigen::AngleAxisd(dq).angle();

        if (trans < kf_trans_thresh_ && rot < kf_rot_thresh_)
            return false;

        keyframes_.push_back(kf);
        return true;
    }

    bool OdomNode::statePropagation(
        const ImuMeas &imu,
        const double &dt
    ) {

        return integrateStep(state_, imu, dt);
    }

    bool OdomNode::integrateStep(
        State &state,
        const ImuMeas &imu,
        const double &dt
    ) const {

        if (dt <= 0.0 || !std::isfinite(dt) ||
            !imu.gyro.allFinite() || !imu.acc.allFinite() ||
            !state.pose.p.allFinite() || !state.pose.q.coeffs().allFinite() ||
            !state.v.allFinite() || !state.b_a.allFinite() || !state.b_g.allFinite())
            return false;

        const Eigen::Quaterniond q = state.pose.q.normalized();
        const Eigen::Vector3d clean_gyro = imu.gyro - state.b_g;
        const Eigen::Vector3d clean_acc = imu.acc - state.b_a;

        const Eigen::Vector3d world_gyro = q * clean_gyro;
        const Eigen::Vector3d world_acc = q * clean_acc - gravity_;
        (void)world_gyro;  // TODO: 后期看是否删除

        state.pose.p += state.v * dt + 0.5 * world_acc * dt * dt;
        state.v += world_acc * dt;

        const Eigen::Quaterniond omega_body(
            0.0,
            clean_gyro.x(),
            clean_gyro.y(),
            clean_gyro.z());
        state.pose.q.coeffs() =
            q.coeffs() + 0.5 * dt * (q * omega_body).coeffs();
        state.pose.q.normalize();

        return state.pose.p.allFinite() &&
            state.v.allFinite() &&
            state.pose.q.coeffs().allFinite();
    }

    bool OdomNode::integrateImu(
        State &state_end,
        const double &t_point
    ) {

        if (!std::isfinite(t_point) || imu_data_.empty())
            return false;

        std::sort(
            imu_data_.begin(), imu_data_.end(),
            [](const ImuMeas &a, const ImuMeas &b) {
                return a.stamp < b.stamp;
            });

        while (imu_data_.size() >= 2 && imu_data_[1].stamp <= t_point) {

            const ImuMeas &imu = imu_data_[0];
            const double dt = imu_data_[1].stamp - imu_data_[0].stamp;
            if (!integrateStep(state_end, imu, dt))
                return false;

            imu_data_.pop_front();
        }

        if (imu_data_.empty())
            return true;

        // handle the special point which between the 2 imu frames
        const double dt = t_point - imu_data_.front().stamp;
        if (dt <= 0.0)
            return true;

        if (!integrateStep(state_end, imu_data_.front(), dt))
            return false;

        imu_data_.front().stamp = t_point;
        return true;
    }

    void OdomNode::callbackImu(
        const sensor_msgs::msg::Imu::SharedPtr &msg
    ) {

        std::lock_guard<std::mutex> lock(mtx_);

        double stamp = 
            msg->header.stamp.sec +
            msg->header.stamp.nanosec * 1e-9;
        
        double dt = stamp - prev_imu_stamp_;
        if (prev_imu_stamp_ == 0.0) {

            prev_imu_stamp_ = stamp;
            return;
        }
        prev_imu_stamp_ = stamp;
        
        ImuMeas imu;
        imu.dt = dt;
        imu.stamp = msg->header.stamp.sec +
            msg->header.stamp.nanosec * 1e-9;
        imu.acc = Eigen::Vector3d(
            msg->linear_acceleration.x, 
            msg->linear_acceleration.y, 
            msg->linear_acceleration.z
        );
        imu.gyro = Eigen::Vector3d(
            msg->angular_velocity.x, 
            msg->angular_velocity.y,
            msg->angular_velocity.z
        );
        imu_data_.push_back(imu);
        statePropagation(imu, dt);
    }

    void OdomNode::callbackLivoxCloud(
        const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg
    ) {

        std::lock_guard<std::mutex> lock(mtx_);
        auto cloud = std::make_shared<pcl::PointCloud<PointXYZIT>>();
        cloud->reserve(msg->points.size());

        for (const auto &src : msg->points) {

            if (!std::isfinite(src.x) ||
                !std::isfinite(src.y) ||
                !std::isfinite(src.z))
                continue;

            PointXYZIT dst;
            dst.x = src.x;
            dst.y = src.y;
            dst.z = src.z;
            dst.intensity = static_cast<float>(src.reflectivity);
            dst.offset_time = static_cast<float>(src.offset_time) * 1e-9f;
            cloud->push_back(dst);
        }

        if (cloud->empty())
            return;

        double scan_start = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        double dt = scan_start - prev_scan_stamp_;
        if (prev_scan_stamp_ == 0.0) {

            prev_scan_stamp_ = scan_start;
            return;
        }
        prev_scan_stamp_ = scan_start;

        // deskew
        auto cloud_deskewed = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        State imu_state;
        motionCorrection(cloud, state_, scan_start, cloud_deskewed, imu_state);
        
        // submapGeneration
        auto submap = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        submapGeneration(imu_state, submap);

        // gicp
         Eigen::Matrix4d T_gicp;
         double score;
         if (!scanToMap(cloud_deskewed, submap, T_gicp, score)) return;

         Pose gicp_pose;
         gicp_pose.p = T_gicp.block<3, 1>(0, 3);
         gicp_pose.q = Eigen::Quaterniond(T_gicp.block<3, 3>(0, 0));

         // geometricFuser
         State fused_state;
         geometricFuser(imu_state, gicp_pose, dt, fused_state);
         state_ = fused_state;

         // keyframeDetection
         keyframeDetection(fused_state, cloud_deskewed, score);

         // publish
         current_stamp_ = msg->header.stamp;
         publishOdometry();
         publishTf();
    }

    void OdomNode::publishOdometry() {

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_stamp_;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "body";
        odom.pose.pose.orientation.w = state_.pose.q.w();
        odom.pose.pose.orientation.x = state_.pose.q.x();
        odom.pose.pose.orientation.y = state_.pose.q.y();
        odom.pose.pose.orientation.z = state_.pose.q.z();
        odom.pose.pose.position.x = state_.pose.p.x();
        odom.pose.pose.position.y = state_.pose.p.y();
        odom.pose.pose.position.z = state_.pose.p.z();
        odom.twist.twist.linear.x = state_.v.x();
        odom.twist.twist.linear.y = state_.v.y();
        odom.twist.twist.linear.z = state_.v.z();
        pub_odom_->publish(odom);

        geometry_msgs::msg::PoseStamped pose;
        pose.header = odom.header;
        pose.pose = odom.pose.pose;
        pub_pose_->publish(pose);

        // TODO: Path
    }

    void OdomNode::publishTf() {

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = current_stamp_;
        tf.header.frame_id = "odom";
        tf.child_frame_id = "body";
        tf.transform.translation.x = state_.pose.p.x();
        tf.transform.translation.y = state_.pose.p.y();
        tf.transform.translation.z = state_.pose.p.z();
        tf.transform.rotation.w = state_.pose.q.w();
        tf.transform.rotation.x = state_.pose.q.x();
        tf.transform.rotation.y = state_.pose.q.y();
        tf.transform.rotation.z = state_.pose.q.z();
        tf_broadcaster_->sendTransform(tf);
    }
} // small_dlio
