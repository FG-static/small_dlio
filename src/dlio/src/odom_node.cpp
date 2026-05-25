#include "odom_node/odom_node.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Core/VectorBlock.h"
#include "Eigen/src/Geometry/Quaternion.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pcl/impl/point_types.hpp"
#include "pcl/point_cloud.h"
#include "rclcpp/qos.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <memory>

namespace small_dlio {

    OdomNode::OdomNode() : Node("small_dlio_odom") {

        loadParams();
        state_.pose.p.setZero();
        state_.pose.q.setIdentity();
        state_.v.setZero();
        state_.b_a.setZero();
        state_.b_g.setZero();
        laser_scan_start_state_ = state_;
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

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
                imu_topic_, rclcpp::SensorDataQoS(),
                [this](sensor_msgs::msg::Imu::SharedPtr msg) {

                    callbackImu(msg);
                },
                imu_opt
            );
        sub_cloud_ =
            create_subscription<livox_ros_driver2::msg::CustomMsg>(
                cloud_topic_, rclcpp::SensorDataQoS(),
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

        // 静态 TF: body -> lidar
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

        // GICP
        declare_param("gicp_leaf_size", gicp_leaf_size_, 0.10);
        declare_param("gicp_num_threads", gicp_num_threads_, 4);
        declare_param("gicp_correspondence_randomness", gicp_correspondence_randomness_, 20);
        declare_param("gicp_max_correspondence_distance", gicp_max_correspondence_distance_, 1.0);

        // Geometric Observer
        declare_param("geo_Kp", Kp_, 1.0);
        declare_param("geo_Kv", Kv_, 1.0);
        declare_param("geo_Kq", Kq_, 1.0);
        declare_param("geo_Ka", Ka_, 1.0);
        declare_param("geo_Kg", Kg_, 1.0);
        declare_param("geo_b_max", b_max_, 1.0);
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

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                "submap kf[%zu]: dist=%.2f, cloud_size=%zu",
                dists[i].second, dists[i].first,
                keyframes_[dists[i].second].cloud ? keyframes_[dists[i].second].cloud->size() : 0);
            if (dists[i].first > max_distance_) break;
            const auto &kf_cloud = keyframes_[dists[i].second].cloud;
            if (kf_cloud && !kf_cloud->empty()) {
                *cloud_submap += *kf_cloud;
            }
        }

        return true;
    }

    bool OdomNode::scanToMap(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud,
        Eigen::Matrix4d &trans_gicp,
        double &alignment_score
    ) const {

        if (!source_cloud || source_cloud->empty() ||
            !target_cloud || target_cloud->empty()) {

            trans_gicp = Eigen::Matrix4d::Identity();
            alignment_score = 1e9;
            return false;
        }

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
        reg.setNumThreads(gicp_num_threads_);
        reg.setCorrespondenceRandomness(gicp_correspondence_randomness_);
        reg.setMaxCorrespondenceDistance(gicp_max_correspondence_distance_);
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
            statePropagation(imu, dt);
        }
    }

    void OdomNode::callbackLivoxCloud(
        const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg
    ) {

        if (imu_calibrate_ && !imu_calibrated_) return;
        std::lock_guard<std::mutex> lock(mtx_);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "callbackLivoxCloud triggered, imu_queue=%zu, keyframes=%zu",
            imu_data_.size(), keyframes_.size());

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

        if (!motionCorrection(cloud, laser_scan_start_state_, scan_start, cloud_deskewed, imu_state) ||
            cloud_deskewed->empty()) {
            RCLCPP_WARN(get_logger(), "motionCorrection failed or empty result");
            return;
        }
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "motionCorrection OK, deskewed=%zu points", cloud_deskewed->size());

        // 发布去畸变点云（世界坐标系）
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud_deskewed, cloud_msg);
        cloud_msg.header.stamp = msg->header.stamp;
        cloud_msg.header.frame_id = odom_frame_;
        pub_cloud_->publish(cloud_msg);

        if (keyframes_.empty()) {

            state_ = imu_state;
            laser_scan_start_state_ = state_;
            keyframeDetection(state_, cloud_deskewed, 0.0);
            current_stamp_ = msg->header.stamp;
            publishOdometry();
            publishTf();
            RCLCPP_INFO(get_logger(), "First keyframe published");
            return;
        }

        // submapGeneration: use state_ (fused_state from previous frame)
        auto submap = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        submapGeneration(state_, submap);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "submapGeneration: %zu points from %zu keyframes",
            submap->size(), keyframes_.size());

        if (submap->empty())
            return;

        // gicp
        Eigen::Matrix4d T_gicp; // TODO: tomorrow remain to learn the reason about T_global
        double score;

        if (!scanToMap(cloud_deskewed, submap, T_gicp, score)) {
            RCLCPP_WARN(get_logger(), "scanToMap failed");
            return;
        }
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "scanToMap OK, score=%.4f", score);

        Eigen::Matrix4d T_prior = Eigen::Matrix4d::Identity();
        T_prior.block<3, 3>(0, 0) = imu_state.pose.q.toRotationMatrix();
        T_prior.block<3, 1>(0, 3) = imu_state.pose.p;

        Eigen::Matrix4d T_global = T_gicp * T_prior;

        Pose gicp_pose;
        gicp_pose.p = T_global.block<3, 1>(0, 3);
        gicp_pose.q = Eigen::Quaterniond(T_global.block<3, 3>(0, 0));

        // geometricFuser
        State fused_state;
        geometricFuser(imu_state, gicp_pose, dt, fused_state);
        state_ = fused_state;
        laser_scan_start_state_ = state_;

        // keyframeDetection
        keyframeDetection(fused_state, cloud_deskewed, score);

        // publish
        current_stamp_ = msg->header.stamp;
        publishOdometry();
        publishTf();

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "Published odom, pos=[%.2f, %.2f, %.2f]",
            state_.pose.p.x(), state_.pose.p.y(), state_.pose.p.z());
    }

    void OdomNode::publishOdometry() {

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "publishOdometry called, pos=[%.2f, %.2f, %.2f]",
            state_.pose.p.x(), state_.pose.p.y(), state_.pose.p.z());

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_stamp_;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = body_frame_;
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

        path_.header = odom.header;
        path_.poses.push_back(pose);
        pub_path_->publish(path_);
    }

    void OdomNode::publishTf() {

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = current_stamp_;
        tf.header.frame_id = odom_frame_;
        tf.child_frame_id = body_frame_;
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
