#include "loop_detector_node.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "pcl_conversions/pcl_conversions.h"

#include <Eigen/Geometry>

#include <cmath>
#include <limits>
#include <utility>

namespace small_dlio {

    LoopDetectorNode::LoopDetectorNode() : Node("small_dlio_loop_detector") {

        loadParams();

        lidar_iris_ = std::make_unique<LidarIris>(
            iris_nscale_,
            iris_min_wave_length_,
            iris_mult_,
            iris_sigma_onf_,
            iris_match_num_
        );

        gicp_matcher_.configure(GicpParams{
            loop_gicp_num_threads_,
            loop_gicp_correspondence_randomness_,
            loop_gicp_max_correspondence_distance_
        });

        sub_keyframe_ =
            create_subscription<dlio::msg::KeyFrame>(
                keyframe_topic_,
                rclcpp::SensorDataQoS(),
                [this](dlio::msg::KeyFrame::SharedPtr msg) {
                    callbackKeyFrame(msg);
                }
            );

        pub_loop_markers_ =
            create_publisher<visualization_msgs::msg::MarkerArray>(
                marker_topic_,
                10
            );
    }

    void LoopDetectorNode::loadParams() {

        declare_parameter("keyframe_topic", keyframe_topic_);
        declare_parameter("marker_topic", marker_topic_);
        declare_parameter("marker_frame", marker_frame_);
        declare_parameter("loop_enable", loop_enable_);
        declare_parameter("loop_min_keyframe_gap", loop_min_keyframe_gap_);
        declare_parameter("loop_iris_distance_thresh", loop_iris_distance_thresh_);
        declare_parameter("loop_gicp_enable", loop_gicp_enable_);
        declare_parameter("loop_gicp_score_thresh", loop_gicp_score_thresh_);
        declare_parameter("loop_gicp_max_correction_trans", loop_gicp_max_correction_trans_);
        declare_parameter("loop_gicp_max_correction_rot_deg", loop_gicp_max_correction_rot_deg_);
        declare_parameter("loop_gicp_num_threads", loop_gicp_num_threads_);
        declare_parameter(
            "loop_gicp_correspondence_randomness",
            loop_gicp_correspondence_randomness_);
        declare_parameter(
            "loop_gicp_max_correspondence_distance",
            loop_gicp_max_correspondence_distance_);
        declare_parameter("iris_nscale", iris_nscale_);
        declare_parameter("iris_min_wave_length", iris_min_wave_length_);
        declare_parameter("iris_mult", iris_mult_);
        declare_parameter("iris_sigma_onf", iris_sigma_onf_);
        declare_parameter("iris_match_num", iris_match_num_);

        get_parameter("keyframe_topic", keyframe_topic_);
        get_parameter("marker_topic", marker_topic_);
        get_parameter("marker_frame", marker_frame_);
        get_parameter("loop_enable", loop_enable_);
        get_parameter("loop_min_keyframe_gap", loop_min_keyframe_gap_);
        get_parameter("loop_iris_distance_thresh", loop_iris_distance_thresh_);
        get_parameter("loop_gicp_enable", loop_gicp_enable_);
        get_parameter("loop_gicp_score_thresh", loop_gicp_score_thresh_);
        get_parameter("loop_gicp_max_correction_trans", loop_gicp_max_correction_trans_);
        get_parameter("loop_gicp_max_correction_rot_deg", loop_gicp_max_correction_rot_deg_);
        get_parameter("loop_gicp_num_threads", loop_gicp_num_threads_);
        get_parameter(
            "loop_gicp_correspondence_randomness",
            loop_gicp_correspondence_randomness_);
        get_parameter(
            "loop_gicp_max_correspondence_distance",
            loop_gicp_max_correspondence_distance_);
        get_parameter("iris_nscale", iris_nscale_);
        get_parameter("iris_min_wave_length", iris_min_wave_length_);
        get_parameter("iris_mult", iris_mult_);
        get_parameter("iris_sigma_onf", iris_sigma_onf_);
        get_parameter("iris_match_num", iris_match_num_);
    }

    void LoopDetectorNode::callbackKeyFrame(
        const dlio::msg::KeyFrame::SharedPtr msg
    ) {

        if (!msg) return;

        LoopKeyFrame keyframe;
        if (!convertKeyFrameMsg(*msg, keyframe)) {

            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Rejected invalid keyframe message: id=%u",
                msg->id
            );
            return;
        }

        if (!computeIrisDescriptor(keyframe)) {

            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Failed to compute LiDAR-Iris descriptor: id=%u cloud_size=%zu",
                keyframe.id,
                keyframe.cloud ? keyframe.cloud->size() : 0U
            );
            return;
        }

        if (loop_enable_) {

            LoopCandidate candidate;
            if (detectLoopCandidate(keyframe, candidate)) {

                if (!loop_gicp_enable_ ||
                    verifyLoopCandidateByGicp(keyframe, candidate)) {

                    loop_candidates_.push_back(candidate);
                    RCLCPP_INFO(
                        get_logger(),
                        "Loop candidate: current=%u history=%u iris=%.4f "
                        "yaw_bias=%d gicp_enable=%d gicp_score=%.4f "
                        "corr_t=%.3f corr_r=%.2f",
                        candidate.current_id,
                        candidate.history_id,
                        candidate.iris_distance,
                        candidate.yaw_bias,
                        loop_gicp_enable_ ? 1 : 0,
                        candidate.gicp_score,
                        candidate.gicp_correction_trans,
                        candidate.gicp_correction_rot_deg
                    );
                }
            }
        }

        storeKeyFrame(std::move(keyframe));
        publishLoopMarkers();
    }

    bool LoopDetectorNode::convertKeyFrameMsg(
        const dlio::msg::KeyFrame &msg,
        LoopKeyFrame &keyframe
    ) const {

        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(msg.cloud, *cloud);

        if (cloud->empty()) return false;

        keyframe.id = msg.id;
        keyframe.stamp = rclcpp::Time(msg.header.stamp);
        keyframe.pose = msg.pose;
        keyframe.cloud = cloud;
        return true;
    }

    bool LoopDetectorNode::computeIrisDescriptor(
        LoopKeyFrame &keyframe
    ) {

        if (!lidar_iris_ || !keyframe.cloud || keyframe.cloud->empty())
            return false;

        keyframe.iris_image = LidarIris::GetIris(*keyframe.cloud);
        if (keyframe.iris_image.empty())
            return false;

        keyframe.iris_descriptor = lidar_iris_->GetFeature(keyframe.iris_image);
        return !keyframe.iris_descriptor.img.empty() &&
            !keyframe.iris_descriptor.T.empty() &&
            !keyframe.iris_descriptor.M.empty();
    }

    bool LoopDetectorNode::detectLoopCandidate(
        const LoopKeyFrame &current,
        LoopCandidate &candidate
    ) const {

        float best_distance = std::numeric_limits<float>::infinity();
        int best_yaw_bias = 0;
        uint32_t best_history_id = 0;
        bool found = false;

        // O(N * C) kNN search, it will be exchange in the future
        for (const auto &history : keyframes_) {

            if (!isCandidateAllowed(current, history))
                continue;

            int yaw_bias = 0;
            float dis = lidar_iris_->Compare(
                current.iris_descriptor,
                history.iris_descriptor,
                &yaw_bias
            );

            if (!std::isfinite(dis))
                continue;

            if (dis < best_distance) {

                best_distance = dis;
                best_yaw_bias = yaw_bias;
                best_history_id = history.id;
                found = true;
            }
        }

        if (!found || best_distance >= loop_iris_distance_thresh_)
            return false;

        candidate.current_id = current.id;
        candidate.history_id = best_history_id;
        candidate.iris_distance = best_distance;
        candidate.yaw_bias = best_yaw_bias;
        return true;
    }

    /**
     * @brief 使用 GICP 匹配潜在回环帧筛选真正回环帧
     * @param current 目前的点云帧
     * @param candidate 潜在回环点云帧
     * @return bool 是否成功
     */
    bool LoopDetectorNode::verifyLoopCandidateByGicp(
        const LoopKeyFrame &current,
        LoopCandidate &candidate
    ) {

        const auto *history = findKeyFrame(candidate.history_id);
        if (!history || !current.cloud || current.cloud->empty() ||
            !history->cloud || history->cloud->empty())
            return false;

        const Eigen::Matrix4d T_world_current = poseToMatrix(current.pose);
        const Eigen::Matrix4d T_world_history = poseToMatrix(history->pose);
        if (!T_world_current.allFinite() || !T_world_history.allFinite())
            return false;

        const Eigen::Matrix4d init_guess =
            T_world_history.inverse() * T_world_current;

        const GicpResult result =
            gicp_matcher_.align(current.cloud, history->cloud, init_guess);

        if (!result.success) {

            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Loop GICP failed: current=%u history=%u iris=%.4f reason=%s",
                candidate.current_id,
                candidate.history_id,
                candidate.iris_distance,
                result.error_message.c_str()
            );
            return false;
        }

        const Eigen::Matrix4d correction = init_guess.inverse() * result.transform;
        const double corr_t = correction.block<3, 1>(0, 3).norm();
        Eigen::Matrix3d corr_R = correction.block<3, 3>(0, 0);
        Eigen::Quaterniond corr_q(corr_R);
        corr_q.normalize();
        double corr_r_deg = Eigen::AngleAxisd(corr_q).angle() * 180.0 / M_PI;
        if (corr_r_deg > 180.0)
            corr_r_deg = 360.0 - corr_r_deg;

        candidate.gicp_score = result.score;
        candidate.gicp_correction_trans = corr_t;
        candidate.gicp_correction_rot_deg = corr_r_deg;
        candidate.source_to_target = result.transform;

        const bool accepted =
            std::isfinite(result.score) &&
            result.score < loop_gicp_score_thresh_ &&
            std::isfinite(corr_t) &&
            corr_t <= loop_gicp_max_correction_trans_ &&
            std::isfinite(corr_r_deg) &&
            corr_r_deg <= loop_gicp_max_correction_rot_deg_;

        candidate.gicp_verified = accepted;

        if (!accepted) {

            RCLCPP_INFO(
                get_logger(),
                "Rejected loop by GICP: current=%u history=%u iris=%.4f "
                "score=%.4f corr_t=%.3f corr_r=%.2f",
                candidate.current_id,
                candidate.history_id,
                candidate.iris_distance,
                result.score,
                corr_t,
                corr_r_deg
            );
            return false;
        }

        return true;
    }

    bool LoopDetectorNode::isCandidateAllowed(
        const LoopKeyFrame &current,
        const LoopKeyFrame &history
    ) const {

        const int id_gap =
            std::abs(
                static_cast<int>(current.id) -
                static_cast<int>(history.id)
            );
        if (id_gap < loop_min_keyframe_gap_)
            return false;

        return !current.iris_descriptor.T.empty() &&
            !current.iris_descriptor.M.empty() &&
            !history.iris_descriptor.T.empty() &&
            !history.iris_descriptor.M.empty();
    }

    const LoopDetectorNode::LoopKeyFrame *LoopDetectorNode::findKeyFrame(
        const uint32_t id
    ) const {

        for (const auto &keyframe : keyframes_)
            if (keyframe.id == id)
                return &keyframe;
        return nullptr;
    }

    Eigen::Matrix4d LoopDetectorNode::poseToMatrix(
        const geometry_msgs::msg::Pose &pose
    ) const {

        Eigen::Quaterniond q(
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z
        );
        if (!q.coeffs().allFinite() || q.norm() < 1e-6)
            return Eigen::Matrix4d::Constant(
                std::numeric_limits<double>::quiet_NaN());
        q.normalize();

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = q.toRotationMatrix();
        T(0, 3) = pose.position.x;
        T(1, 3) = pose.position.y;
        T(2, 3) = pose.position.z;
        return T;
    }

    void LoopDetectorNode::storeKeyFrame(
        LoopKeyFrame &&keyframe
    ) {

        keyframes_.push_back(std::move(keyframe));
    }

    void LoopDetectorNode::publishLoopMarkers() const {

        visualization_msgs::msg::MarkerArray markers;
        const rclcpp::Time stamp = now();

        auto make_base_marker = [&](const int id, const int type, const std::string &ns) {
            visualization_msgs::msg::Marker marker;
            marker.header.stamp = stamp;
            marker.header.frame_id = marker_frame_;
            marker.ns = ns;
            marker.id = id;
            marker.type = type;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            return marker;
        };

        auto to_point = [](const geometry_msgs::msg::Pose &pose) {
            geometry_msgs::msg::Point point;
            point.x = pose.position.x;
            point.y = pose.position.y;
            point.z = pose.position.z;
            return point;
        };

        auto nodes = make_base_marker(
            0,
            visualization_msgs::msg::Marker::SPHERE_LIST,
            "loop_detector_keyframe_nodes"
        );
        nodes.scale.x = 0.15;
        nodes.scale.y = 0.15;
        nodes.scale.z = 0.15;
        nodes.color.r = 0.10F;
        nodes.color.g = 0.85F;
        nodes.color.b = 1.00F;
        nodes.color.a = 1.00F;
        nodes.points.reserve(keyframes_.size());
        for (const auto &keyframe : keyframes_)
            nodes.points.push_back(to_point(keyframe.pose));

        auto odom_edges = make_base_marker(
            1,
            visualization_msgs::msg::Marker::LINE_LIST,
            "loop_detector_odom_edges"
        );
        odom_edges.scale.x = 0.030;
        odom_edges.color.r = 0.25F;
        odom_edges.color.g = 0.45F;
        odom_edges.color.b = 1.00F;
        odom_edges.color.a = 0.80F;
        if (keyframes_.size() >= 2)
            odom_edges.points.reserve((keyframes_.size() - 1U) * 2U);
        for (size_t i = 1; i < keyframes_.size(); ++ i) {

            odom_edges.points.push_back(to_point(keyframes_[i - 1].pose));
            odom_edges.points.push_back(to_point(keyframes_[i].pose));
        }

        auto loop_edges = make_base_marker(
            2,
            visualization_msgs::msg::Marker::LINE_LIST,
            "loop_detector_candidate_edges"
        );
        loop_edges.scale.x = 0.08;
        loop_edges.color.r = 1.00F;
        loop_edges.color.g = 0.80F;
        loop_edges.color.b = 0.00F;
        loop_edges.color.a = 1.00F;
        loop_edges.points.reserve(loop_candidates_.size() * 2U);
        for (const auto &candidate : loop_candidates_) {

            const auto *current = findKeyFrame(candidate.current_id);
            const auto *history = findKeyFrame(candidate.history_id);
            if (!current || !history)
                continue;

            loop_edges.points.push_back(to_point(history->pose));
            loop_edges.points.push_back(to_point(current->pose));
        }

        markers.markers.push_back(std::move(nodes));
        markers.markers.push_back(std::move(odom_edges));
        markers.markers.push_back(std::move(loop_edges));
        pub_loop_markers_->publish(markers);
    }

} // namespace small_dlio
