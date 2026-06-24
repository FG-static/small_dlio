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
        pose_graph_.configure(PoseGraphOptions{
            pgo_max_iterations_,
            true,
            pgo_odom_edge_weight_,
            pgo_loop_edge_weight_
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

        pub_optimized_path_ =
            create_publisher<nav_msgs::msg::Path>(
                optimized_path_topic_,
                rclcpp::QoS(1).transient_local().reliable()
            );

        RCLCPP_INFO(
            get_logger(),
            "Loop detector backend: loop_gicp_enable=%d pgo_enable=%d "
            "optimized_path_topic=%s min_keyframe_gap=%d min_travel=%.2f",
            loop_gicp_enable_ ? 1 : 0,
            pgo_enable_ ? 1 : 0,
            optimized_path_topic_.c_str(),
            loop_min_keyframe_gap_,
            loop_min_travel_distance_
        );
    }

    void LoopDetectorNode::loadParams() {

        declare_parameter("keyframe_topic", keyframe_topic_);
        declare_parameter("marker_topic", marker_topic_);
        declare_parameter("optimized_path_topic", optimized_path_topic_);
        declare_parameter("marker_frame", marker_frame_);
        declare_parameter("loop_enable", loop_enable_);
        declare_parameter("loop_min_keyframe_gap", loop_min_keyframe_gap_);
        declare_parameter("loop_min_travel_distance", loop_min_travel_distance_);
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
        declare_parameter("pgo_enable", pgo_enable_);
        declare_parameter("pgo_optimize_on_loop", pgo_optimize_on_loop_);
        declare_parameter("pgo_max_iterations", pgo_max_iterations_);
        declare_parameter("pgo_odom_edge_weight", pgo_odom_edge_weight_);
        declare_parameter("pgo_loop_edge_weight", pgo_loop_edge_weight_);
        declare_parameter("iris_nscale", iris_nscale_);
        declare_parameter("iris_min_wave_length", iris_min_wave_length_);
        declare_parameter("iris_mult", iris_mult_);
        declare_parameter("iris_sigma_onf", iris_sigma_onf_);
        declare_parameter("iris_match_num", iris_match_num_);

        get_parameter("keyframe_topic", keyframe_topic_);
        get_parameter("marker_topic", marker_topic_);
        get_parameter("optimized_path_topic", optimized_path_topic_);
        get_parameter("marker_frame", marker_frame_);
        get_parameter("loop_enable", loop_enable_);
        get_parameter("loop_min_keyframe_gap", loop_min_keyframe_gap_);
        get_parameter("loop_min_travel_distance", loop_min_travel_distance_);
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
        get_parameter("pgo_enable", pgo_enable_);
        get_parameter("pgo_optimize_on_loop", pgo_optimize_on_loop_);
        get_parameter("pgo_max_iterations", pgo_max_iterations_);
        get_parameter("pgo_odom_edge_weight", pgo_odom_edge_weight_);
        get_parameter("pgo_loop_edge_weight", pgo_loop_edge_weight_);
        get_parameter("iris_nscale", iris_nscale_);
        get_parameter("iris_min_wave_length", iris_min_wave_length_);
        get_parameter("iris_mult", iris_mult_);
        get_parameter("iris_sigma_onf", iris_sigma_onf_);
        get_parameter("iris_match_num", iris_match_num_);
    }

    void LoopDetectorNode::callbackKeyFrame(
        const dlio::msg::KeyFrame::SharedPtr msg
    ) {

        try {

        if (!msg) return;
        ++ received_keyframes_;

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

            ++ iris_failures_;
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Failed to compute LiDAR-Iris descriptor: id=%u cloud_size=%zu",
                keyframe.id,
                keyframe.cloud ? keyframe.cloud->size() : 0U
            );
            return;
        }

        keyframe.travel_distance = accumulated_travel_distance_;
        if (!keyframes_.empty()) {

            const Eigen::Vector3d previous(
                keyframes_.back().pose.position.x,
                keyframes_.back().pose.position.y,
                keyframes_.back().pose.position.z
            );
            const Eigen::Vector3d current(
                keyframe.pose.position.x,
                keyframe.pose.position.y,
                keyframe.pose.position.z
            );
            const double step = (current - previous).norm();
            if (std::isfinite(step))
                keyframe.travel_distance += step;
        }

        const bool pgo_node_added =
            pgo_enable_ && addKeyFrameToPoseGraph(keyframe);

        bool added_loop_edge = false;
        LoopCandidate accepted_candidate;
        bool has_accepted_candidate = false;
        if (loop_enable_) {

            LoopCandidate candidate;
            float best_distance = std::numeric_limits<float>::infinity();
            int eligible_count = 0;
            if (detectLoopCandidate(
                    keyframe, candidate, best_distance, eligible_count)) {

                if (!loop_gicp_enable_ ||
                    verifyLoopCandidateByGicp(keyframe, candidate)) {

                    ++ candidate_hits_;
                    accepted_candidate = candidate;
                    has_accepted_candidate = true;
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
            } else {

                ++ candidate_misses_;
                RCLCPP_INFO_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Loop candidate miss: current=%u eligible=%d best_iris=%.4f "
                    "thresh=%.4f stored=%zu received=%lu iris_fail=%lu "
                    "hits=%lu misses=%lu",
                    keyframe.id,
                    eligible_count,
                    best_distance,
                    loop_iris_distance_thresh_,
                    keyframes_.size(),
                    received_keyframes_,
                    iris_failures_,
                    candidate_hits_,
                    candidate_misses_
                );
            }
        }

        storeKeyFrame(std::move(keyframe));
        ++ stored_keyframes_;

        if (has_accepted_candidate) {

            loop_candidates_.push_back(accepted_candidate);
            if (pgo_node_added && pgo_enable_)
                added_loop_edge =
                    addLoopCandidateToPoseGraph(accepted_candidate);
        }

        if (pgo_enable_)
            optimizePoseGraphIfNeeded(added_loop_edge);

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Loop detector status: received=%lu stored=%lu graph_nodes=%zu "
            "graph_edges=%zu candidates=%zu iris_fail=%lu",
            received_keyframes_,
            stored_keyframes_,
            pose_graph_.nodeCount(),
            pose_graph_.edgeCount(),
            loop_candidates_.size(),
            iris_failures_
        );
        publishLoopMarkers();
        if (pgo_enable_)
            publishOptimizedPath();
        } catch (const cv::Exception &e) {

            ++ iris_failures_;
            RCLCPP_ERROR(
                get_logger(),
                "OpenCV exception in loop detector keyframe callback: id=%u what=%s",
                msg ? msg->id : 0U,
                e.what()
            );
        } catch (const std::exception &e) {

            RCLCPP_ERROR(
                get_logger(),
                "Exception in loop detector keyframe callback: id=%u what=%s",
                msg ? msg->id : 0U,
                e.what()
            );
        }
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
        int eligible_count = 0;
        return detectLoopCandidate(
            current, candidate, best_distance, eligible_count);
    }

    bool LoopDetectorNode::detectLoopCandidate(
        const LoopKeyFrame &current,
        LoopCandidate &candidate,
        float &best_distance,
        int &eligible_count
    ) const {

        best_distance = std::numeric_limits<float>::infinity();
        eligible_count = 0;
        int best_yaw_bias = 0;
        uint32_t best_history_id = 0;
        bool found = false;

        // O(N * C) kNN search, it will be exchange in the future
        for (const auto &history : keyframes_) {

            if (!isCandidateAllowed(current, history))
                continue;
            ++ eligible_count;

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

        const double travel_gap =
            current.travel_distance - history.travel_distance;
        if (!std::isfinite(travel_gap) ||
            travel_gap < loop_min_travel_distance_)
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

    Eigen::Isometry3d LoopDetectorNode::poseToIsometry(
        const geometry_msgs::msg::Pose &pose
    ) const {

        return matrixToIsometry(poseToMatrix(pose));
    }

    /**
     * @brief 等距变换矩阵
     * @param 要变换的矩阵
     * @return Eigen::Isometry3d 刚体变换矩阵
     *
     * 具体来说，就是[R t]，方式矩阵不合法
     *             [0 1]
     */
    Eigen::Isometry3d LoopDetectorNode::matrixToIsometry(
        const Eigen::Matrix4d &matrix
    ) const {

        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        if (!matrix.allFinite())
            return Eigen::Isometry3d(
                Eigen::Matrix4d::Constant(
                    std::numeric_limits<double>::quiet_NaN()));

        Eigen::Quaterniond q(matrix.block<3, 3>(0, 0));
        if (!q.coeffs().allFinite() || q.norm() < 1e-6)
            return Eigen::Isometry3d(
                Eigen::Matrix4d::Constant(
                    std::numeric_limits<double>::quiet_NaN()));
        q.normalize();

        T.linear() = q.toRotationMatrix();
        T.translation() = matrix.block<3, 1>(0, 3);
        return T;
    }

    geometry_msgs::msg::Pose LoopDetectorNode::isometryToPose(
        const Eigen::Isometry3d &pose
    ) const {

        geometry_msgs::msg::Pose msg;
        msg.position.x = pose.translation().x();
        msg.position.y = pose.translation().y();
        msg.position.z = pose.translation().z();

        Eigen::Quaterniond q(pose.rotation());
        q.normalize();
        msg.orientation.w = q.w();
        msg.orientation.x = q.x();
        msg.orientation.y = q.y();
        msg.orientation.z = q.z();
        return msg;
    }

    Eigen::Matrix<double, 6, 6> LoopDetectorNode::makeInformationMatrix(
        const double weight
    ) const {

        const double safe_weight =
            std::isfinite(weight) && weight > 0.0 ? weight : 1.0;
        return safe_weight * Eigen::Matrix<double, 6, 6>::Identity();
    }

    std::pair<double, double> LoopDetectorNode::relativeDelta(
        const Eigen::Isometry3d &a,
        const Eigen::Isometry3d &b
    ) const {

        const Eigen::Isometry3d delta = a.inverse() * b;
        const double trans = delta.translation().norm();
        Eigen::Quaterniond q(delta.rotation());
        q.normalize();
        double rot_deg = Eigen::AngleAxisd(q).angle() * 180.0 / M_PI;
        if (rot_deg > 180.0)
            rot_deg = 360.0 - rot_deg;
        return {trans, rot_deg};
    }

    bool LoopDetectorNode::addKeyFrameToPoseGraph(
        const LoopKeyFrame &keyframe
    ) {

        const Eigen::Isometry3d current_pose = poseToIsometry(keyframe.pose);
        if (!current_pose.matrix().allFinite())
            return false;

        PoseGraphNode node;
        node.id = keyframe.id;
        node.stamp = keyframe.stamp.seconds();
        node.pose = current_pose;

        const bool added = pose_graph_.addNode(node);
        if (!added)
            return false;

        if (keyframes_.empty())
            return true;

        const LoopKeyFrame &previous = keyframes_.back();
        const Eigen::Isometry3d previous_pose = poseToIsometry(previous.pose);
        if (!previous_pose.matrix().allFinite())
            return false;

        const Eigen::Isometry3d relative_pose =
            previous_pose.inverse() * current_pose;
        return pose_graph_.addOdomEdge(
            previous.id,
            keyframe.id,
            relative_pose,
            makeInformationMatrix(pgo_odom_edge_weight_)
        );
    }

    bool LoopDetectorNode::addLoopCandidateToPoseGraph(
        const LoopCandidate &candidate
    ) {

        if (!candidate.gicp_verified ||
            !candidate.source_to_target.allFinite())
            return false;

        const Eigen::Isometry3d history_from_current =
            matrixToIsometry(candidate.source_to_target);
        if (!history_from_current.matrix().allFinite())
            return false;

        const auto *history = findKeyFrame(candidate.history_id);
        const auto *current = findKeyFrame(candidate.current_id);
        if (!history || !current)
            return false;

        const Eigen::Isometry3d odom_history_from_current =
            poseToIsometry(history->pose).inverse() * poseToIsometry(current->pose);
        const auto direct_delta =
            relativeDelta(odom_history_from_current, history_from_current);
        const auto inverse_delta =
            relativeDelta(odom_history_from_current, history_from_current.inverse());

        RCLCPP_INFO(
            get_logger(),
            "Loop edge debug: history=%u current=%u score=%.4f iris=%.4f "
            "odom_vs_direct_dp=%.3f dr=%.2f odom_vs_inverse_dp=%.3f dr=%.2f "
            "direct_t=[%.3f %.3f %.3f] inv_t=[%.3f %.3f %.3f]",
            candidate.history_id,
            candidate.current_id,
            candidate.gicp_score,
            candidate.iris_distance,
            direct_delta.first,
            direct_delta.second,
            inverse_delta.first,
            inverse_delta.second,
            history_from_current.translation().x(),
            history_from_current.translation().y(),
            history_from_current.translation().z(),
            history_from_current.inverse().translation().x(),
            history_from_current.inverse().translation().y(),
            history_from_current.inverse().translation().z()
        );

        return pose_graph_.addLoopEdge(
            candidate.history_id,
            candidate.current_id,
            history_from_current,
            makeInformationMatrix(pgo_loop_edge_weight_),
            candidate.gicp_score
        );
    }

    void LoopDetectorNode::optimizePoseGraphIfNeeded(
        const bool has_new_loop
    ) {

        if (!pgo_enable_)
            return;
        if (pgo_optimize_on_loop_ && !has_new_loop)
            return;

        const PoseGraphOptimizationSummary summary = pose_graph_.optimize();
        if (!summary.success) {

            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "PGO skipped/failed: %s nodes=%d edges=%d",
                summary.message.c_str(),
                summary.node_count,
                summary.edge_count
            );
            return;
        }

        RCLCPP_INFO(
            get_logger(),
            "PGO optimized: iter=%d nodes=%d edges=%d chi2 %.3f -> %.3f",
            summary.iterations,
            summary.node_count,
            summary.edge_count,
            summary.initial_chi2,
            summary.final_chi2
        );
    }

    void LoopDetectorNode::storeKeyFrame(
        LoopKeyFrame &&keyframe
    ) {

        accumulated_travel_distance_ = keyframe.travel_distance;
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

    void LoopDetectorNode::publishOptimizedPath() const {

        if (!pub_optimized_path_ || pose_graph_.empty())
            return;

        nav_msgs::msg::Path path;
        path.header.stamp = now();
        path.header.frame_id = marker_frame_;
        path.poses.reserve(pose_graph_.nodeCount());

        for (const auto &node : pose_graph_.nodes()) {

            if (!node.pose.matrix().allFinite())
                continue;

            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose = isometryToPose(node.pose);
            path.poses.push_back(std::move(pose));
        }

        pub_optimized_path_->publish(path);
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Published optimized_path: poses=%zu nodes=%zu edges=%zu",
            path.poses.size(),
            pose_graph_.nodeCount(),
            pose_graph_.edgeCount()
        );
    }

} // namespace small_dlio
