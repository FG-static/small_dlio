#include "odom_node/odom_node.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Core/VectorBlock.h"
#include "Eigen/src/Geometry/Quaternion.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pcl/impl/point_types.hpp"
#include "pcl/point_cloud.h"
#include "point_types.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <vector>

namespace small_dlio {

    OdomNode::OdomNode() : Node("small_dlio_odom") {

        loadParams();
        state_.pose.p.setZero();
        state_.pose.q.setIdentity();
        state_.v.setZero();
        state_.b_a.setZero();
        state_.b_g.setZero();
        laser_scan_start_state_ = state_;
        keyframe_positions_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        keyframe_kdtree_ = std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>();
        tf_broadcaster_ = std:: make_unique<tf2_ros::TransformBroadcaster>(*this);
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

        // 静态 TF: body -> livox_frame (外参固定，只发一次)
        geometry_msgs::msg::TransformStamped static_tf;
        static_tf.header.stamp = now();
        static_tf.header.frame_id = body_frame_;
        static_tf.child_frame_id = lidar_frame_;
        static_tf.transform.translation.x = extrinsics_.t_body_lidar.x();
        static_tf.transform.translation.y = extrinsics_.t_body_lidar.y();
        static_tf.transform.translation.z = extrinsics_.t_body_lidar.z();
        static_tf.transform.rotation.w = extrinsics_.q_body_lidar.w();
        static_tf.transform.rotation.x = extrinsics_.q_body_lidar.x();
        static_tf.transform.rotation.y = extrinsics_.q_body_lidar.y();
        static_tf.transform.rotation.z = extrinsics_.q_body_lidar.z();
        static_tf_broadcaster_->sendTransform(static_tf);

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
        auto cloud_qos = rclcpp::QoS(rclcpp::KeepLast(1)); // ensure just 1 data
        cloud_qos.best_effort();
        cloud_qos.durability_volatile();

        sub_imu_ =
            create_subscription<sensor_msgs::msg::Imu>(
                imu_topic_, rclcpp::SensorDataQoS(),
                [this](sensor_msgs::msg::Imu::SharedPtr msg) {

                    callbackImu(msg);
                },
                imu_opt
            );
        sub_cloud_ =
            create_subscription<livox_ros_driver2::msg::CustomMsg>(
                cloud_topic_, cloud_qos,
                [this](livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {

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
        pub_cloud_ =
            create_publisher<sensor_msgs::msg::PointCloud2>(
                "deskewed", 10
            );
        pub_keyframe_cloud_ =
            create_publisher<sensor_msgs::msg::PointCloud2>(
                "keyframe", 10
            );
        publish_timer_ = create_wall_timer(
            std::chrono::milliseconds(10),
            [this]() {
                publishLiveState();
            }
        );

    }

    void OdomNode::loadParams() {

        // Frames
        declare_param("odom_frame", odom_frame_, std::string("odom"));
        declare_param("body_frame", body_frame_, std::string("body"));
        declare_param("lidar_frame", lidar_frame_, std::string("livox_frame"));

        // Topics
        declare_param("imu_topic", imu_topic_, std::string("/livox/imu"));
        declare_param("cloud_topic", cloud_topic_, std::string("/livox/lidar"));

        // Extrinsics: body -> IMU
        std::vector<double> t_body_imu_default = {0.0, 0.0, 0.0};
        std::vector<double> q_body_imu_default = {1.0, 0.0, 0.0, 0.0};
        std::vector<double> t_body_imu, q_body_imu;
        this->declare_parameter("t_body_imu", t_body_imu_default);
        this->get_parameter("t_body_imu", t_body_imu);
        this->declare_parameter("q_body_imu", q_body_imu_default);
        this->get_parameter("q_body_imu", q_body_imu);
        extrinsics_.t_body_imu = Eigen::Vector3d(t_body_imu[0], t_body_imu[1], t_body_imu[2]);
        extrinsics_.q_body_imu = Eigen::Quaterniond(q_body_imu[0], q_body_imu[1], q_body_imu[2], q_body_imu[3]);

        // Extrinsics: body -> LiDAR
        std::vector<double> t_body_lidar_default = {0.0, 0.0, 0.0};
        std::vector<double> q_body_lidar_default = {1.0, 0.0, 0.0, 0.0};
        std::vector<double> t_body_lidar, q_body_lidar;
        this->declare_parameter("t_body_lidar", t_body_lidar_default);
        this->get_parameter("t_body_lidar", t_body_lidar);
        this->declare_parameter("q_body_lidar", q_body_lidar_default);
        this->get_parameter("q_body_lidar", q_body_lidar);
        extrinsics_.t_body_lidar = Eigen::Vector3d(t_body_lidar[0], t_body_lidar[1], t_body_lidar[2]);
        extrinsics_.q_body_lidar = Eigen::Quaterniond(q_body_lidar[0], q_body_lidar[1], q_body_lidar[2], q_body_lidar[3]);

        // Gravity
        std::vector<double> gravity_default = {0.0, 0.0, 9.8};
        std::vector<double> gravity;
        this->declare_parameter("gravity", gravity_default);
        this->get_parameter("gravity", gravity);
        gravity_ = Eigen::Vector3d(gravity[0], gravity[1], gravity[2]);
        declare_param("imu_acc_scale", imu_acc_scale_, gravity_.norm());

        // Imu Calibration
        bool imu_calibrate_default = true;
        double imu_calib_time_default = 1.5;
        this->declare_parameter("imu_calibrate", imu_calibrate_default);
        this->get_parameter("imu_calibrate", imu_calibrate_);
        this->declare_parameter("imu_calib_time", imu_calib_time_default);
        this->get_parameter("imu_calib_time", imu_calib_time_);

        // Keyframe Detection
        declare_param("kf_trans_thresh", kf_trans_thresh_, 0.5);
        declare_param("kf_rot_thresh", kf_rot_thresh_, 0.1745);
        declare_param("max_alignment_score", max_alignment_score_, 1.0);

        // Submap
        declare_param("knn_limit", knn_limit_, 5);
        declare_param("max_distance", max_distance_, 20.0);
        declare_param("log_throttle_ms", log_throttle_ms_, 2000);

        // GICP
        declare_param("gicp_leaf_size", gicp_leaf_size_, 0.10);
        declare_param("gicp_num_threads", gicp_num_threads_, 4);
        declare_param("gicp_correspondence_randomness", gicp_correspondence_randomness_, 20);
        declare_param("gicp_max_correspondence_distance", gicp_max_correspondence_distance_, 1.0);
        declare_param("gicp_reject_reset_threshold", gicp_reject_reset_threshold_, 3);
        declare_param("gicp_max_correction_trans", gicp_max_correction_trans_, 0.8);
        declare_param("gicp_max_correction_rot_deg", gicp_max_correction_rot_deg_, 8.0);
        declare_param("gicp_max_imu_to_gicp_trans", gicp_max_imu_to_gicp_trans_, 0.8);
        declare_param("gicp_max_imu_to_gicp_rot_deg", gicp_max_imu_to_gicp_rot_deg_, 8.0);

        reg_.setRegistrationType("GICP");
        reg_.setNumThreads(gicp_num_threads_);
        reg_.setCorrespondenceRandomness(gicp_correspondence_randomness_);
        reg_.setMaxCorrespondenceDistance(gicp_max_correspondence_distance_);

        // Geometric Observer
        declare_param("geo_Kp", Kp_, 1.0);
        declare_param("geo_Kv", Kv_, 1.0);
        declare_param("geo_Kq", Kq_, 1.0);
        declare_param("geo_Ka", Ka_, 1.0);
        declare_param("geo_Kg", Kg_, 1.0);
        declare_param("geo_b_max", b_max_, 1.0);
        declare_param("geo_max_delta_v", geo_max_delta_v_, 0.5);
        declare_param("geo_max_delta_ba", geo_max_delta_ba_, 0.1);
        declare_param("geo_max_delta_bg", geo_max_delta_bg_, 0.05);
        declare_param("geo_max_velocity", geo_max_velocity_, 20.0);
    }

    /**
     * @brief 去除点云运动畸变 + 将点云转换到世界坐标系下
     * @param cloud_in 输入点云
     * @param trajectory 用于去除畸变用的参照轨迹
     * @param cloud_out 输出去畸变点云
     * @return bool 是否成功
     *
     * 本函数仅按照参照轨迹进行点云运动畸变处理，轨迹生成由 buildTrajectory 完成
     */
    bool OdomNode::motionCorrection(
        const pcl::PointCloud<PointXYZIT>::Ptr &cloud_in,
        const FrameTrajectory &trajectory,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out
    ) const {

        if (!cloud_in || cloud_in->empty() || trajectory.samples.empty())
            return false;
        if (trajectory.point_sample_indices.size() != trajectory.samples.size() + 1)
            return false;

        cloud_out->clear();
        cloud_out->points.resize(cloud_in->size());
        cloud_out->width = cloud_in->width;
        cloud_out->height = cloud_in->height;
        cloud_out->is_dense = false;

        Eigen::Matrix4d T_body_lidar = Eigen::Matrix4d::Identity();
        T_body_lidar.block<3, 3>(0, 0) =
            extrinsics_.q_body_lidar.normalized().toRotationMatrix();
        T_body_lidar.block<3, 1>(0, 3) = extrinsics_.t_body_lidar;

        // OpenMP
        #pragma omp parallel for num_threads(gicp_num_threads_) schedule(static)
        for (int i = 0; i < static_cast<int>(trajectory.samples.size()); ++ i) {

            const int begin = trajectory.point_sample_indices[i];
            const int end = trajectory.point_sample_indices[i + 1];
            if (begin < 0 || end < begin ||
                end > static_cast<int>(cloud_in->size()))
                continue;

            const State &point_state = trajectory.samples[i].state;
            Eigen::Matrix4d T = Eigen::Matrix4d::Zero();
            T.block<3, 3>(0, 0) = point_state.pose.q.toRotationMatrix();
            T.block<3, 1>(0, 3) = point_state.pose.p;
            T(3, 3) = 1.0;

            for (int k = begin; k < end; ++ k) {

                const auto &point = cloud_in->points[k];
                Eigen::Vector4d point_lidar(point.x, point.y, point.z, 1.0);
                Eigen::Vector4d point_world = T * T_body_lidar * point_lidar;

                pcl::PointXYZ p;
                p.x = static_cast<float>(point_world.x());
                p.y = static_cast<float>(point_world.y());
                p.z = static_cast<float>(point_world.z());
                cloud_out->points[k] = p;
            }
        }
        return true;
    }

    bool OdomNode::submapGeneration(
        const State &cur_state,
        const std::vector<KeyFrame> &keyframes_snapshot,
        const size_t source_point_count,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_submap
    ) const {

        cloud_submap->clear();

        (void)cur_state;
        for (const auto &kf : keyframes_snapshot) {

            const auto &kf_cloud = kf.cloud;
            if (kf_cloud && !kf_cloud->empty()) *cloud_submap += *kf_cloud;
        }

        if (cloud_submap->empty() || source_point_count == 0)
            return true;

        const size_t target_point_limit =
            std::max<size_t>(source_point_count * 2U, 1U);
        if (cloud_submap->size() <= target_point_limit)
            return true;

        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setLeafSize(
            static_cast<float>(gicp_leaf_size_),
            static_cast<float>(gicp_leaf_size_),
            static_cast<float>(gicp_leaf_size_)
        );
        auto filtered_submap = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        voxel_filter.setInputCloud(cloud_submap);
        voxel_filter.filter(*filtered_submap);

        if (!filtered_submap->empty())
            *cloud_submap = *filtered_submap;

        (void)source_point_count;

        return true;
    }

    bool OdomNode::scanToMap(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud,
        const Eigen::Matrix4d &init_guess,
        Eigen::Matrix4d &trans_gicp,
        double &alignment_score
    ) {

        if (!source_cloud || source_cloud->empty() ||
            !target_cloud || target_cloud->empty()) {

            trans_gicp = Eigen::Matrix4d::Identity();
            alignment_score = 1e9;
            return false;
        }

        reg_.setInputSource(source_cloud);
        reg_.setInputTarget(target_cloud);

        auto aligned = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        try {

            reg_.align(*aligned, init_guess.cast<float>());
        } catch (const std::exception &e) {

            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), log_throttle_ms_,
                "GICP 配准异常，跳过本帧: %s", e.what());
            trans_gicp = Eigen::Matrix4d::Identity();
            alignment_score = 1e9;
            return false;
        }

        alignment_score = reg_.getFitnessScore();
        trans_gicp = reg_.getFinalTransformation().cast<double>();
        if (!trans_gicp.allFinite()) {

            alignment_score = 1e9;
            trans_gicp = Eigen::Matrix4d::Identity();
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), log_throttle_ms_,
                "GICP 配准存在非法结果，跳过本帧");
            return false;
        }
        return true;
    }

    bool OdomNode::geometricFuser(
        const State &imu_state,
        const Pose &gicp_pose,
        const double &dt,
        const double &observation_weight,
        State &fused_state
    ) const {

        if (dt <= 0.0 || !std::isfinite(dt) ||
            !std::isfinite(observation_weight) ||
            observation_weight <= 0.0 || observation_weight > 1.0 ||
            !imu_state.pose.p.allFinite() || !gicp_pose.p.allFinite() ||
            !imu_state.pose.q.coeffs().allFinite() || !gicp_pose.q.coeffs().allFinite() ||
            !imu_state.v.allFinite() || !imu_state.b_a.allFinite() || !imu_state.b_g.allFinite()) {

            fused_state = imu_state;
            return false;
        }

        // Scale observer gains instead of clipping the state update, so the fused
        // state still follows the same model with lower LiDAR confidence.
        const double Kp_eff = observation_weight * Kp_;
        const double Kv_eff = observation_weight * Kv_;
        const double Kq_eff = observation_weight * Kq_;
        const double Ka_eff = observation_weight * Ka_;
        const double Kg_eff = observation_weight * Kg_;

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
        fused_state.b_a = imu_state.b_a - dt * Ka_eff * e_p_body;
        fused_state.b_g =
            imu_state.b_g - dt * Kg_eff * Eigen::Vector3d(q_e.x(), q_e.y(), q_e.z());
        fused_state.b_a = fused_state.b_a.cwiseMax(-b_max_).cwiseMin(b_max_);
        fused_state.b_g = fused_state.b_g.cwiseMax(-b_max_).cwiseMin(b_max_);
        fused_state.pose.p = imu_state.pose.p + dt * Kp_eff * e_p;
        fused_state.v = imu_state.v + dt * Kv_eff * e_p;
        fused_state.pose.q.coeffs() = q_hat.coeffs() + dt * Kq_eff * q_corr.coeffs();
        fused_state.pose.q.normalize();

        if (!fused_state.pose.p.allFinite() ||
            !fused_state.pose.q.coeffs().allFinite() ||
            !fused_state.v.allFinite() ||
            !fused_state.b_a.allFinite() ||
            !fused_state.b_g.allFinite() ||
            fused_state.v.norm() > geo_max_velocity_) {

            fused_state = imu_state;
            return false;
        }

        return true;
    }

    bool OdomNode::keyframeDetection(
        const Pose &gicp_pose,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_aligned,
        const double alignment_score
    ) {

        if (!cloud_aligned || cloud_aligned->empty() ||
            !std::isfinite(alignment_score) ||
            alignment_score > max_alignment_score_ ||
            !gicp_pose.p.allFinite() ||
            !gicp_pose.q.coeffs().allFinite())
            return false;

        KeyFrame kf;
        kf.pose = gicp_pose;
        kf.pose.q.normalize();
        kf.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>(*cloud_aligned));

        auto append_keyframe = [&]() {

            keyframes_.push_back(kf);

            pcl::PointXYZ p;
            p.x = static_cast<float>(kf.pose.p.x());
            p.y = static_cast<float>(kf.pose.p.y());
            p.z = static_cast<float>(kf.pose.p.z());
            keyframe_positions_->push_back(p);
            keyframe_kdtree_->setInputCloud(keyframe_positions_);
        };

        if (keyframes_.empty()) {

            append_keyframe();
            return true;
        }

        const KeyFrame &last_kf = keyframes_.back();
        const double trans = (gicp_pose.p - last_kf.pose.p).norm();

        Eigen::Quaterniond dq =
            (last_kf.pose.q.conjugate() * kf.pose.q).normalized();
        if (dq.w() < 0.0) dq.coeffs() *= -1.0;
        const double rot = Eigen::AngleAxisd(dq).angle();

        if (trans < kf_trans_thresh_ && rot < kf_rot_thresh_)
            return false;

        append_keyframe();
        return true;
    }

    /**
     * @brief 积分构建 SE(3) 参考轨迹
     * @param start_state 上一次帧的 fused_state
     * @param imu_buffer imu 历史数据，用于积分状态
     * @param timestamps 时间戳数组，对应 imu_buffer
     * @param trajectory 输出轨迹
     * @return bool 是否成功
     *
     * 本函数用于构建给 motionCorrection 使用的小车 SE(3) 参考轨迹
     */
    bool OdomNode::buildTrajectory(
        const State &start_state,
        const double start_time,
        const std::deque<ImuMeas> &imu_buffer,
        const std::vector<double> &timestamps,
        FrameTrajectory &trajectory
    ) const {

        trajectory.samples.clear();
        trajectory.point_sample_indices.clear();
        trajectory.median_index = 0;

        if (timestamps.empty() ||
            !std::isfinite(start_time) ||
            !start_state.pose.p.allFinite() ||
            !start_state.pose.q.coeffs().allFinite() ||
            !start_state.v.allFinite() ||
            !start_state.b_a.allFinite() ||
            !start_state.b_g.allFinite())
            return false;

        std::vector<double> sorted_timestamps = timestamps;
        std::sort(sorted_timestamps.begin(), sorted_timestamps.end());
        sorted_timestamps.erase(
            std::unique(
                sorted_timestamps.begin(),
                sorted_timestamps.end(),
                [](const double a, const double b) {
                    return std::abs(a - b) < 1e-9;
                }),
            sorted_timestamps.end()
        );

        // check
        for (const double stamp : sorted_timestamps) {

            if (!std::isfinite(stamp))
                return false;
        }

        if (sorted_timestamps.front() < start_time)
            return false;

        State cur_state = start_state;
        double cur_time = start_time;
        std::deque<ImuMeas> imu_work = imu_buffer;
        std::sort(
            imu_work.begin(), imu_work.end(),
            [](const ImuMeas &a, const ImuMeas &b) {
                return a.stamp < b.stamp;
            }
        );
        if (imu_work.empty() && sorted_timestamps.back() > start_time)
            return false;

        while (imu_work.size() >= 2 && imu_work[1].stamp <= cur_time)
            imu_work.pop_front();
        if (!imu_work.empty() && imu_work.front().stamp < cur_time)
            imu_work.front().stamp = cur_time;

        trajectory.samples.reserve(sorted_timestamps.size());

        for (size_t i = 0; i < sorted_timestamps.size(); ++ i) {

            const double target_time = sorted_timestamps[i];
            if (target_time < cur_time)
                return false;

            while (imu_work.size() >= 2 && imu_work[1].stamp <= target_time) {

                const double dt = imu_work[1].stamp - imu_work[0].stamp;
                if (!integrateStep(cur_state, imu_work[0], dt))
                    return false;

                cur_time = imu_work[1].stamp;
                imu_work.pop_front();
            }

            if (!imu_work.empty()) {

                const double interval_start = std::max(cur_time, imu_work.front().stamp);
                if (target_time > interval_start) {

                    const double dt = target_time - interval_start;
                    if (!integrateStep(cur_state, imu_work.front(), dt))
                        return false;

                    cur_time = target_time;
                    imu_work.front().stamp = cur_time;
                }
            }

            trajectory.samples.push_back({target_time, cur_state});
        }

        // select the median stamp
        trajectory.median_index = trajectory.samples.size() / 2;
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

        // Transform from IMU frame to body frame
        const Eigen::Vector3d body_gyro = extrinsics_.q_body_imu.conjugate() * clean_gyro;
        const Eigen::Vector3d body_acc  = extrinsics_.q_body_imu.conjugate() * clean_acc;

        const Eigen::Vector3d world_gyro = q * body_gyro;
        const Eigen::Vector3d world_acc = q * body_acc - gravity_;
        (void)world_gyro;

        state.pose.p += state.v * dt + 0.5 * world_acc * dt * dt;
        state.v += world_acc * dt;

        const Eigen::Quaterniond omega_body(
            0.0,
            body_gyro.x(),
            body_gyro.y(),
            body_gyro.z());
        state.pose.q.coeffs() =
            q.coeffs() + 0.5 * dt * (q * omega_body).coeffs();
        state.pose.q.normalize();

        return state.pose.p.allFinite() &&
            state.v.allFinite() &&
            state.pose.q.coeffs().allFinite();
    }

    bool OdomNode::integrateImu(
        State &state_end,
        const double &t_point,
        std::deque<ImuMeas> &imu_buffer
    ) {

        if (!std::isfinite(t_point) || imu_buffer.empty())
            return false;

        std::sort(
            imu_buffer.begin(), imu_buffer.end(),
            [](const ImuMeas &a, const ImuMeas &b) {
                return a.stamp < b.stamp;
            });

        while (imu_buffer.size() >= 2 && imu_buffer[1].stamp <= t_point) {

            const ImuMeas &imu = imu_buffer[0];
            const double dt = imu_buffer[1].stamp - imu_buffer[0].stamp;
            if (!integrateStep(state_end, imu, dt))
                return false;

            imu_buffer.pop_front();
        }

        if (imu_buffer.empty())
            return true;

        const double dt = t_point - imu_buffer.front().stamp;
        if (dt <= 0.0)
            return true;

        if (!integrateStep(state_end, imu_buffer.front(), dt))
            return false;

        imu_buffer.front().stamp = t_point;
        return true;
    }

    bool OdomNode::finalizeImuCalibration(
        const Eigen::Vector3d &calib_acc_avg,
        const Eigen::Vector3d &calib_gyro_avg
    ) {

        const double acc_norm = calib_acc_avg.norm();
        if (!std::isfinite(acc_norm) || acc_norm < 1e-6) {

            RCLCPP_WARN(
                get_logger(),
                "IMU calibration failed: invalid average acceleration norm %.6f",
                acc_norm
            );
            return false;
        }

        // pitch and roll
        // Rodrigues
        Eigen::Vector3d acc_body_nom =
            extrinsics_.q_body_imu.conjugate() * (calib_acc_avg / acc_norm);

        Eigen::Vector3d source = acc_body_nom.normalized(); // body
        Eigen::Vector3d target = (state_.pose.q.conjugate() * gravity_).normalized(); // world -> body
        Eigen::Vector3d vary = source.cross(target);

        double sin_v = vary.norm();
        double cos_v = source.dot(target);

        Eigen::Matrix3d K;
        K <<         0, -vary.z(),  vary.y(),
              vary.z(),         0, -vary.x(),
             -vary.y(),  vary.x(),         0;

        Eigen::Matrix3d R_corr;
        if (sin_v < 1e-6) R_corr = Eigen::Matrix3d::Identity();
        else R_corr =
                Eigen::Matrix3d::Identity() + K +
                ((1.0 - cos_v) / (sin_v * sin_v)) * K * K;

        Eigen::Matrix3d R_bi_nom = extrinsics_.q_body_imu.toRotationMatrix();
        Eigen::Matrix3d R_ib_nom = R_bi_nom.transpose();

        Eigen::Matrix3d R_ib_new = R_corr * R_ib_nom;
        Eigen::Matrix3d R_bi_new = R_ib_new.transpose();

        extrinsics_.q_body_imu = Eigen::Quaterniond(R_bi_new).normalized();

        const Eigen::Vector3d gravity_body =
            state_.pose.q.conjugate() * gravity_;
        const Eigen::Vector3d gravity_imu =
            extrinsics_.q_body_imu * gravity_body;

        state_.b_a = calib_acc_avg - gravity_imu;
        state_.b_g = calib_gyro_avg;
        laser_scan_start_state_ = state_;

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
        ) * imu_acc_scale_;
        imu.gyro = Eigen::Vector3d(
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z
        );

        // Imu Calibration
        if (imu_calibrate_ && !imu_calibrated_) {

            if (!initialized_) {

                first_imu_stamp_ = stamp - dt;
                initialized_ = true;
                calib_elapsed = 0.0;
                calib_samples = 0;
                calib_acc_sum_.setZero();
                calib_gyro_sum_.setZero();
                RCLCPP_INFO(
                    get_logger(),
                    "IMU calibration started: target duration %.2f s",
                    imu_calib_time_
                );
            }
            if (calib_elapsed >= imu_calib_time_) {

                Eigen::Vector3d acc_avg = Eigen::Vector3d::Zero();
                Eigen::Vector3d gyro_avg = Eigen::Vector3d::Zero();
                acc_avg = calib_acc_sum_ / calib_samples;
                gyro_avg = calib_gyro_sum_ / calib_samples;
                if (!finalizeImuCalibration(acc_avg, gyro_avg)) {

                    RCLCPP_WARN(
                        get_logger(),
                        "IMU calibration aborted: samples=%d, elapsed=%.3f s",
                        calib_samples, calib_elapsed
                    );
                    return;
                }
                imu_calibrated_ = true;
                RCLCPP_INFO(
                    get_logger(),
                    "IMU calibration completed: samples=%d, elapsed=%.3f s, "
                    "acc_avg=[%.6f, %.6f, %.6f], gyro_avg=[%.6f, %.6f, %.6f], "
                    "b_a=[%.6f, %.6f, %.6f], b_g=[%.6f, %.6f, %.6f], "
                    "q_body_imu=[%.6f, %.6f, %.6f, %.6f]",
                    calib_samples, calib_elapsed,
                    acc_avg.x(), acc_avg.y(), acc_avg.z(),
                    gyro_avg.x(), gyro_avg.y(), gyro_avg.z(),
                    state_.b_a.x(), state_.b_a.y(), state_.b_a.z(),
                    state_.b_g.x(), state_.b_g.y(), state_.b_g.z(),
                    extrinsics_.q_body_imu.w(), extrinsics_.q_body_imu.x(),
                    extrinsics_.q_body_imu.y(), extrinsics_.q_body_imu.z()
                );
                goto PROPAGATION;
            }
            calib_acc_sum_ += imu.acc;
            calib_gyro_sum_ += imu.gyro;
            calib_elapsed += dt;
            calib_samples ++;
        } else {

PROPAGATION:
            imu_data_.push_back(imu);
            imu_history_.push_back(imu);
            statePropagation(imu, dt);
            latest_imu_stamp_ = rclcpp::Time(msg->header.stamp);
        }
    }

    void OdomNode::callbackLivoxCloud(
        const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg
    ) {
        auto pose_delta = [](const Pose &from, const Pose &to) {
            Eigen::Quaterniond dq = (from.q.conjugate() * to.q).normalized();
            if (dq.w() < 0.0)
                dq.coeffs() *= -1.0;
            return std::make_pair(
                (to.p - from.p).norm(),
                Eigen::AngleAxisd(dq).angle() * 180.0 / M_PI);
        };

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

        // create a snapshot data
        FrameSnapshot snapshot;
        const rclcpp::Time stamp = msg->header.stamp;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (imu_calibrate_ && !imu_calibrated_)
                return;

            snapshot.scan_start =
                msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
            if (prev_scan_stamp_ == 0.0) {

                prev_scan_stamp_ = snapshot.scan_start;
                return;
            }
            snapshot.prev_ref_stamp = prev_scan_stamp_;

            snapshot.scan_start_state = laser_scan_start_state_;
            snapshot.submap_query_state = last_registered_state_;
            snapshot.init_gicp = prev_T_gicp_;
            //snapshot.init_gicp = Eigen::Matrix4d::Identity();
            snapshot.imu_buffer = imu_data_;
            snapshot.has_keyframes = !keyframes_.empty();
            snapshot.keyframes.clear();

            if (snapshot.has_keyframes) {

                if (keyframe_positions_ &&
                    keyframe_kdtree_ &&
                    !keyframe_positions_->empty() &&
                    keyframe_positions_->size() == keyframes_.size() &&
                    snapshot.submap_query_state.pose.p.allFinite() &&
                    knn_limit_ > 0 &&
                    max_distance_ > 0.0) {

                    pcl::PointXYZ query;
                    query.x = static_cast<float>(snapshot.submap_query_state.pose.p.x());
                    query.y = static_cast<float>(snapshot.submap_query_state.pose.p.y());
                    query.z = static_cast<float>(snapshot.submap_query_state.pose.p.z());

                    const int k =
                        std::min(knn_limit_, static_cast<int>(keyframes_.size()));
                    std::vector<int> indices;
                    std::vector<float> sq_dists;
                    indices.reserve(k);
                    sq_dists.reserve(k);

                    const int found =
                        keyframe_kdtree_->nearestKSearch(query, k, indices, sq_dists);
                    const double max_sq_dist = max_distance_ * max_distance_;

                    for (int i = 0; i < found; ++ i) {

                        if (sq_dists[i] > max_sq_dist) continue;

                        const int idx = indices[i];
                        if (idx >= 0 && static_cast<size_t>(idx) < keyframes_.size())
                            snapshot.keyframes.push_back(keyframes_[idx]);
                    }
                } else {

                    std::vector<std::pair<double, size_t>> dists;
                    dists.reserve(keyframes_.size());
                    for (size_t i = 0; i < keyframes_.size(); ++ i) {

                        const auto &kf = keyframes_[i];
                        if (!kf.pose.p.allFinite() || !kf.cloud || kf.cloud->empty())
                            continue;

                        const double d =
                            (kf.pose.p - snapshot.submap_query_state.pose.p).norm();
                        dists.emplace_back(d, i);
                    }

                    std::sort(dists.begin(), dists.end());
                    for (size_t i = 0;
                        i < std::min(dists.size(), static_cast<size_t>(knn_limit_));
                        ++ i) {

                        if (dists[i].first > max_distance_) break;
                        snapshot.keyframes.push_back(keyframes_[dists[i].second]);
                    }
                }
            }
        }

        std::sort(
            cloud->points.begin(), cloud->points.end(),
            [](const PointXYZIT &a, const PointXYZIT &b) {
                return a.offset_time < b.offset_time;
            }
        );
        cloud->width = static_cast<uint32_t>(cloud->points.size());
        cloud->height = 1;

        std::vector<double> timestamps;
        std::vector<int> point_sample_indices;
        timestamps.reserve(cloud->size() + 1);
        point_sample_indices.reserve(cloud->size() + 2); // label timestamps index
        // it can use two index to symbol some points at the sane offset_time

        constexpr double time_epsilon = 1e-9;
        if (std::abs(static_cast<double>(cloud->points.front().offset_time)) > time_epsilon) {

            timestamps.push_back(snapshot.scan_start);
            point_sample_indices.push_back(0);
        }

        for (size_t i = 0; i < cloud->points.size(); ++ i) {

            const double point_stamp =
                snapshot.scan_start + static_cast<double>(cloud->points[i].offset_time);
            if (timestamps.empty() ||
                std::abs(point_stamp - timestamps.back()) > time_epsilon) {

                timestamps.push_back(point_stamp);
                point_sample_indices.push_back(static_cast<int>(i));
            }
        }
        point_sample_indices.push_back(static_cast<int>(cloud->points.size()));

        // buildTrajectory
        FrameTrajectory trajectory;
        if (!buildTrajectory(
                snapshot.scan_start_state,
                snapshot.prev_ref_stamp,
                snapshot.imu_buffer,
                timestamps,
                trajectory) ||
            trajectory.samples.empty()) {

            RCLCPP_WARN(get_logger(), "buildTrajectory failed or empty result");
            return;
        }
        trajectory.point_sample_indices = std::move(point_sample_indices);
        const size_t frame_ref_index =
            std::min(trajectory.median_index, trajectory.samples.size() - 1);
        const double frame_ref_time = trajectory.samples[frame_ref_index].stamp;
        const double frame_dt = frame_ref_time - snapshot.prev_ref_stamp;
        snapshot.scan_dt = frame_dt;

        auto cloud_deskewed = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        State imu_state = trajectory.samples[frame_ref_index].state;

        // motionCorrection
        if (!motionCorrection(cloud, trajectory, cloud_deskewed) ||
            cloud_deskewed->empty()) {

            RCLCPP_WARN(get_logger(), "motionCorrection failed or empty result");
            return;
        }

        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setLeafSize(
            static_cast<float>(gicp_leaf_size_),
            static_cast<float>(gicp_leaf_size_),
            static_cast<float>(gicp_leaf_size_)
        );
        auto cloud_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        voxel_filter.setInputCloud(cloud_deskewed);
        voxel_filter.filter(*cloud_filtered);
        if (cloud_filtered->empty()) {

            RCLCPP_WARN(get_logger(), "voxel filter produced empty cloud");
            return;
        }

        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud_deskewed, cloud_msg);
        cloud_msg.header.stamp = stamp;
        cloud_msg.header.frame_id = odom_frame_;
        pub_cloud_->publish(cloud_msg);

        /**
         * @brief 帧结束收尾工作
         * @param frame_state 目前点云回调内积分到的最新state
         * @param update_registration 更新且注册状态
         * @param keyframe_pose 注册到keyframe的pose
         * @param cloud_aligned 对齐的点云
         * @param alignment_score 对齐分数
         * @param accepted_T_gicp 被接收的T_gicp，用于gicp初始位置猜测
         * @return 无
         */
        auto commit_frame = [&](
            const State &frame_state,
            const bool update_registration,
            const Pose *keyframe_pose,
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_aligned,
            const double alignment_score,
            const Eigen::Matrix4d *accepted_T_gicp,
            const bool reset_gicp_guess = false
        ) {

            bool replay_ok = true;
            bool keyframe_added = false;
            // Snapshot live/registered states around commit for divergence logging.
            State live_before;
            State live_after;
            State reg_before;
            State reg_after;
            {
                std::lock_guard<std::mutex> lock(mtx_);
                live_before = state_;
                reg_before = last_registered_state_;
                while (imu_data_.size() >= 2 && imu_data_[1].stamp <= frame_ref_time)
                    imu_data_.pop_front();
                if (!imu_data_.empty() && imu_data_.front().stamp < frame_ref_time)
                    imu_data_.front().stamp = frame_ref_time;

                state_ = frame_state;
                if (!imu_data_.empty()) {

                    std::deque<ImuMeas> live_imu = imu_data_;
                    const double live_end_time = live_imu.back().stamp;
                    if (live_end_time > frame_ref_time)
                        replay_ok = integrateImu(state_, live_end_time, live_imu);
                }

                laser_scan_start_state_ = frame_state;
                prev_scan_stamp_ = frame_ref_time;
                if (reset_gicp_guess)
                    prev_T_gicp_ = Eigen::Matrix4d::Identity();
                if (update_registration) {

                    last_registered_state_ = frame_state;
                    last_registered_stamp_ = frame_ref_time;
                    consecutive_gicp_rejects_ = 0;
                    if (accepted_T_gicp) prev_T_gicp_ = *accepted_T_gicp;
                    if (keyframe_pose && cloud_aligned)
                        keyframe_added =
                            keyframeDetection(*keyframe_pose, cloud_aligned, alignment_score);

                    while (imu_history_.size() >= 2 &&
                        imu_history_[1].stamp <= last_registered_stamp_)
                        imu_history_.pop_front();
                    if (!imu_history_.empty() &&
                        imu_history_.front().stamp < last_registered_stamp_)
                        imu_history_.front().stamp = last_registered_stamp_;
                }
                current_stamp_ = stamp;
                live_after = state_;
                reg_after = last_registered_state_;
            }

            if (keyframe_added && cloud_aligned)
                publishKeyframeCloud(cloud_aligned, stamp);

            if (!replay_ok) {

                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                    "Failed to replay post-frame IMU data from %.6f", frame_ref_time);
            }

            const auto live_before_to_frame = pose_delta(live_before.pose, frame_state.pose);
            const auto frame_to_live_after = pose_delta(frame_state.pose, live_after.pose);
            const auto reg_before_to_frame = pose_delta(reg_before.pose, frame_state.pose);
            const auto reg_after_to_live_after = pose_delta(reg_after.pose, live_after.pose);
            RCLCPP_INFO(
                get_logger(),
                "commit state: update_reg=%d score=%.4f frame_t=%.6f "
                "live_before_to_frame_dp=%.3f dr=%.2f "
                "frame_to_live_after_dp=%.3f dr=%.2f "
                "reg_before_to_frame_dp=%.3f dr=%.2f "
                "reg_after_to_live_after_dp=%.3f dr=%.2f "
                "live_before_p=[%.3f %.3f %.3f] frame_p=[%.3f %.3f %.3f] "
                "live_after_p=[%.3f %.3f %.3f] reg_before_p=[%.3f %.3f %.3f] reg_after_p=[%.3f %.3f %.3f]",
                update_registration ? 1 : 0, alignment_score, frame_ref_time,
                live_before_to_frame.first, live_before_to_frame.second,
                frame_to_live_after.first, frame_to_live_after.second,
                reg_before_to_frame.first, reg_before_to_frame.second,
                reg_after_to_live_after.first, reg_after_to_live_after.second,
                live_before.pose.p.x(), live_before.pose.p.y(), live_before.pose.p.z(),
                frame_state.pose.p.x(), frame_state.pose.p.y(), frame_state.pose.p.z(),
                live_after.pose.p.x(), live_after.pose.p.y(), live_after.pose.p.z(),
                reg_before.pose.p.x(), reg_before.pose.p.y(), reg_before.pose.p.z(),
                reg_after.pose.p.x(), reg_after.pose.p.y(), reg_after.pose.p.z());
            publishPath(frame_state, stamp);
        };

        auto replay_from_last_registration = [&](
            const double target_time,
            State &replayed_state
        ) {

            State start_state;
            double start_time = 0.0;
            std::deque<ImuMeas> replay_imu;
            int reject_count = 0;
            {
                std::lock_guard<std::mutex> lock(mtx_);
                ++consecutive_gicp_rejects_;
                reject_count = consecutive_gicp_rejects_;
                start_state = last_registered_state_;
                start_time = last_registered_stamp_;
                replay_imu = imu_history_;
            }

            if (reject_count <= gicp_reject_reset_threshold_)
                return false;

            if (start_time <= 0.0 || !std::isfinite(start_time) ||
                target_time <= start_time || replay_imu.empty()) {

                RCLCPP_WARN(
                    get_logger(),
                    "tracking recovery skipped: rejects=%d start_t=%.6f target_t=%.6f imu_history=%zu",
                    reject_count, start_time, target_time, replay_imu.size());
                return false;
            }

            std::sort(
                replay_imu.begin(), replay_imu.end(),
                [](const ImuMeas &a, const ImuMeas &b) {
                    return a.stamp < b.stamp;
                });

            while (replay_imu.size() >= 2 && replay_imu[1].stamp <= start_time)
                replay_imu.pop_front();

            if (replay_imu.empty() || replay_imu.back().stamp < target_time) {

                RCLCPP_WARN(
                    get_logger(),
                    "tracking recovery skipped: rejects=%d insufficient imu history start_t=%.6f "
                    "target_t=%.6f imu_front=%.6f imu_back=%.6f size=%zu",
                    reject_count, start_time, target_time,
                    replay_imu.empty() ? 0.0 : replay_imu.front().stamp,
                    replay_imu.empty() ? 0.0 : replay_imu.back().stamp,
                    replay_imu.size());
                return false;
            }

            if (replay_imu.front().stamp < start_time)
                replay_imu.front().stamp = start_time;

            if (replay_imu.front().stamp > start_time + 0.02) {

                RCLCPP_WARN(
                    get_logger(),
                    "tracking recovery skipped: rejects=%d imu history starts too late "
                    "start_t=%.6f imu_front=%.6f target_t=%.6f",
                    reject_count, start_time, replay_imu.front().stamp, target_time);
                return false;
            }

            replayed_state = start_state;
            if (!integrateImu(replayed_state, target_time, replay_imu)) {

                RCLCPP_WARN(
                    get_logger(),
                    "tracking recovery failed during imu replay: rejects=%d start_t=%.6f target_t=%.6f",
                    reject_count, start_time, target_time);
                return false;
            }

            const auto recovery_delta =
                pose_delta(start_state.pose, replayed_state.pose);
            RCLCPP_WARN(
                get_logger(),
                "tracking recovery: rejects=%d reset prev_T_gicp and replay from last accepted "
                "gicp state start_t=%.6f target_t=%.6f dp=%.3f dr=%.2f",
                reject_count, start_time, target_time,
                recovery_delta.first, recovery_delta.second);
            return true;
        };

        if (!snapshot.has_keyframes) {

            auto cloud_aligned =
                std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*cloud_filtered);
            const Pose first_pose = imu_state.pose;
            commit_frame(imu_state, true, &first_pose, cloud_aligned, 0.0, nullptr);
            RCLCPP_INFO(get_logger(), "First keyframe published");
            return;
        }

        // submapGeneration
        auto submap = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        submapGeneration(
            snapshot.submap_query_state,
            snapshot.keyframes,
            cloud_filtered->size(),
            submap
        );

        if (submap->empty()) {

            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                "submapGeneration produced an empty submap, committed IMU-only state");
            commit_frame(imu_state, false, nullptr, pcl::PointCloud<pcl::PointXYZ>::Ptr(), 0.0, nullptr);
            return;
        }

        Eigen::Matrix4d T_corr;
        double score;

        // scanToMap
        if (!scanToMap(cloud_filtered, submap, snapshot.init_gicp, T_corr, score)) {

            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                "scanToMap failed, committed IMU-only state");
            commit_frame(imu_state, false, nullptr, pcl::PointCloud<pcl::PointXYZ>::Ptr(), 0.0, nullptr);
            return;
        }

        Eigen::Matrix4d T_prior = Eigen::Matrix4d::Identity();
        T_prior.block<3, 3>(0, 0) = imu_state.pose.q.toRotationMatrix();
        T_prior.block<3, 1>(0, 3) = imu_state.pose.p;

        Eigen::Matrix4d T_global = T_corr * T_prior;

        Pose gicp_pose;
        gicp_pose.p = T_global.block<3, 1>(0, 3);
        gicp_pose.q = Eigen::Quaterniond(T_global.block<3, 3>(0, 0));
        gicp_pose.q.normalize();

        const auto imu_to_gicp = pose_delta(imu_state.pose, gicp_pose);
        const double corr_trans = T_corr.block<3, 1>(0, 3).norm();
        const double corr_rot =
            Eigen::AngleAxisd(T_corr.block<3, 3>(0, 0)).angle() * 180.0 / M_PI;

        // Hard gate: reject the LiDAR observation before it can perturb the state.
        const bool score_rejected =
            !std::isfinite(score) || score >= max_alignment_score_;
        const bool corr_rejected =
            !std::isfinite(corr_trans) || !std::isfinite(corr_rot) ||
            corr_trans > gicp_max_correction_trans_ ||
            corr_rot > gicp_max_correction_rot_deg_;
        const bool prior_rejected =
            !std::isfinite(imu_to_gicp.first) || !std::isfinite(imu_to_gicp.second) ||
            imu_to_gicp.first > gicp_max_imu_to_gicp_trans_ ||
            imu_to_gicp.second > gicp_max_imu_to_gicp_rot_deg_;

        if (score_rejected || corr_rejected || prior_rejected) {

            RCLCPP_WARN(
                get_logger(),
                "gicp rejected: reason=[score:%d corr:%d prior:%d] source=%zu target=%zu "
                "score=%.4f/%.4f corr_t=%.3f/%.3f corr_r=%.2f/%.2f "
                "imu_to_gicp_dp=%.3f/%.3f dr=%.2f/%.2f "
                "imu_p=[%.3f %.3f %.3f] gicp_p=[%.3f %.3f %.3f]",
                score_rejected ? 1 : 0,
                corr_rejected ? 1 : 0,
                prior_rejected ? 1 : 0,
                cloud_filtered->size(), submap->size(),
                score, max_alignment_score_,
                corr_trans, gicp_max_correction_trans_,
                corr_rot, gicp_max_correction_rot_deg_,
                imu_to_gicp.first, gicp_max_imu_to_gicp_trans_,
                imu_to_gicp.second, gicp_max_imu_to_gicp_rot_deg_,
                imu_state.pose.p.x(), imu_state.pose.p.y(), imu_state.pose.p.z(),
                gicp_pose.p.x(), gicp_pose.p.y(), gicp_pose.p.z());
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                "GICP rejected by gate, committed IMU-only state");

            State recovery_state;
            const bool recovered =
                replay_from_last_registration(frame_ref_time, recovery_state);
            commit_frame(
                recovered ? recovery_state : imu_state,
                false,
                nullptr,
                pcl::PointCloud<pcl::PointXYZ>::Ptr(),
                0.0,
                nullptr,
                recovered);
            return;
        }

        // Soft weighting keeps the same observer equations but trusts near-threshold
        // observations less than clean matches.
        auto normalized_ratio = [](const double value, const double limit) {
            if (!std::isfinite(value))
                return 1.0;
            if (limit <= 1e-9)
                return value <= 0.0 ? 0.0 : 1.0;
            return std::clamp(value / limit, 0.0, 1.0);
        };

        const double score_ratio = normalized_ratio(score, max_alignment_score_);
        const double corr_trans_ratio = normalized_ratio(corr_trans, gicp_max_correction_trans_);
        const double corr_rot_ratio = normalized_ratio(corr_rot, gicp_max_correction_rot_deg_);
        const double prior_trans_ratio = normalized_ratio(imu_to_gicp.first, gicp_max_imu_to_gicp_trans_);
        const double prior_rot_ratio = normalized_ratio(imu_to_gicp.second, gicp_max_imu_to_gicp_rot_deg_);
        const double worst_ratio = std::max({
            score_ratio, corr_trans_ratio, corr_rot_ratio,
            prior_trans_ratio, prior_rot_ratio});
        const double observation_weight = std::clamp(1.0 - worst_ratio, 0.0, 1.0);

        // geometricFuser
        State fused_state;
        const double fuser_dt = snapshot.scan_dt;
        const bool fuser_ok =
            geometricFuser(imu_state, gicp_pose, fuser_dt, observation_weight, fused_state);

        if (!fuser_ok) {

            RCLCPP_WARN(
                get_logger(),
                "fuser rejected: weight=%.3f fuser_dt=%.6f imu_v=%.3f/%.3f "
                "imu_to_gicp_dp=%.3f dr=%.2f corr_t=%.3f corr_r=%.2f",
                observation_weight, fuser_dt, imu_state.v.norm(), geo_max_velocity_,
                imu_to_gicp.first, imu_to_gicp.second, corr_trans, corr_rot);

            State recovery_state;
            const bool recovered =
                replay_from_last_registration(frame_ref_time, recovery_state);
            commit_frame(
                recovered ? recovery_state : imu_state,
                false,
                nullptr,
                pcl::PointCloud<pcl::PointXYZ>::Ptr(),
                0.0,
                nullptr,
                recovered);
            return;
        }

        const auto imu_to_fused = pose_delta(imu_state.pose, fused_state.pose);
        const auto gicp_to_fused = pose_delta(gicp_pose, fused_state.pose);
        RCLCPP_INFO(
            get_logger(),
            "gicp accepted: score=%.4f source=%zu target=%zu "
            "corr_t=%.3f corr_r=%.2f fuser_dt=%.6f weight=%.3f "
            "imu_to_gicp_dp=%.3f dr=%.2f imu_to_fused_dp=%.3f dr=%.2f gicp_to_fused_dp=%.3f dr=%.2f "
            "imu_p=[%.3f %.3f %.3f] gicp_p=[%.3f %.3f %.3f] fused_p=[%.3f %.3f %.3f]",
            score, cloud_filtered->size(), submap->size(),
            corr_trans, corr_rot, fuser_dt, observation_weight,
            imu_to_gicp.first, imu_to_gicp.second,
            imu_to_fused.first, imu_to_fused.second,
            gicp_to_fused.first, gicp_to_fused.second,
            imu_state.pose.p.x(), imu_state.pose.p.y(), imu_state.pose.p.z(),
            gicp_pose.p.x(), gicp_pose.p.y(), gicp_pose.p.z(),
            fused_state.pose.p.x(), fused_state.pose.p.y(), fused_state.pose.p.z());

        auto cloud_aligned = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::transformPointCloud(*cloud_filtered, *cloud_aligned, T_corr.cast<float>());

        // publish normal end
        commit_frame(fused_state, true, &gicp_pose, cloud_aligned, score, &T_corr);

        // RCLCPP_INFO(
        //     get_logger(),
        //     "pose compare final: score=%.4f gicp_p=[%.3f %.3f %.3f] "
        //     "fused_p=[%.3f %.3f %.3f] dp=%.3f dr=%.2f "
        //     "gicp_q=[%.5f %.5f %.5f %.5f] "
        //     "fused_q=[%.5f %.5f %.5f %.5f]",
        //     score,
        //     gicp_pose.p.x(), gicp_pose.p.y(), gicp_pose.p.z(),
        //     fused_state.pose.p.x(), fused_state.pose.p.y(), fused_state.pose.p.z(),
        //     gicp_to_fused.first, gicp_to_fused.second,
        //     gicp_pose.q.w(), gicp_pose.q.x(), gicp_pose.q.y(), gicp_pose.q.z(),
        //     fused_state.pose.q.w(), fused_state.pose.q.x(),
        //     fused_state.pose.q.y(), fused_state.pose.q.z());
    }

    void OdomNode::publishOdometry(
        const State &state,
        const rclcpp::Time &stamp
    ) {

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
            "publishOdometry called, pos=[%.2f, %.2f, %.2f]",
            state.pose.p.x(), state.pose.p.y(), state.pose.p.z());

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = body_frame_;
        odom.pose.pose.orientation.w = state.pose.q.w();
        odom.pose.pose.orientation.x = state.pose.q.x();
        odom.pose.pose.orientation.y = state.pose.q.y();
        odom.pose.pose.orientation.z = state.pose.q.z();
        odom.pose.pose.position.x = state.pose.p.x();
        odom.pose.pose.position.y = state.pose.p.y();
        odom.pose.pose.position.z = state.pose.p.z();
        odom.twist.twist.linear.x = state.v.x();
        odom.twist.twist.linear.y = state.v.y();
        odom.twist.twist.linear.z = state.v.z();
        pub_odom_->publish(odom);

        geometry_msgs::msg::PoseStamped pose;
        pose.header = odom.header;
        pose.pose = odom.pose.pose;
        pub_pose_->publish(pose);
    }

    void OdomNode::publishTf(
        const State &state,
        const rclcpp::Time &stamp
    ) {

        // odom -> body (动态)
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = odom_frame_;
        tf.child_frame_id = body_frame_;
        tf.transform.translation.x = state.pose.p.x();
        tf.transform.translation.y = state.pose.p.y();
        tf.transform.translation.z = state.pose.p.z();
        tf.transform.rotation.w = state.pose.q.w();
        tf.transform.rotation.x = state.pose.q.x();
        tf.transform.rotation.y = state.pose.q.y();
        tf.transform.rotation.z = state.pose.q.z();
        tf_broadcaster_->sendTransform(tf);
    }

    void OdomNode::publishPath(
        const State &state,
        const rclcpp::Time &stamp
    ) {

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = odom_frame_;
        pose.pose.orientation.w = state.pose.q.w();
        pose.pose.orientation.x = state.pose.q.x();
        pose.pose.orientation.y = state.pose.q.y();
        pose.pose.orientation.z = state.pose.q.z();
        pose.pose.position.x = state.pose.p.x();
        pose.pose.position.y = state.pose.p.y();
        pose.pose.position.z = state.pose.p.z();

        path_.header = pose.header;
        path_.poses.push_back(pose);
        pub_path_->publish(path_);
    }

    void OdomNode::publishKeyframeCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        const rclcpp::Time &stamp
    ) {

        if (!cloud || cloud->empty())
            return;

        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.stamp = stamp;
        cloud_msg.header.frame_id = odom_frame_;
        pub_keyframe_cloud_->publish(cloud_msg);
    }

    void OdomNode::publishLiveState() {

        State state_snapshot;
        rclcpp::Time stamp_snapshot;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (latest_imu_stamp_.nanoseconds() == 0)
                return;
            if (imu_calibrate_ && !imu_calibrated_)
                return;

            state_snapshot = state_;
            stamp_snapshot = latest_imu_stamp_;
        }

        publishOdometry(state_snapshot, stamp_snapshot);
        publishTf(state_snapshot, stamp_snapshot);
    }
} // small_dlio
