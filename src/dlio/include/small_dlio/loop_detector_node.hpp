#ifndef SMALL_DLIO__LOOP_DETECTOR_NODE_HPP
#define SMALL_DLIO__LOOP_DETECTOR_NODE_HPP

#include "LidarIris.h"
#include "dlio/msg/key_frame.hpp"
#include "gicp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <string>
#include <vector>

namespace small_dlio {

    class LoopDetectorNode : public rclcpp::Node {

    public:

        LoopDetectorNode();

    private:

        struct LoopKeyFrame {

            uint32_t id = 0;
            rclcpp::Time stamp;
            geometry_msgs::msg::Pose pose;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
            cv::Mat1b iris_image;
            LidarIris::FeatureDesc iris_descriptor;
        };

        // Save potential loop-detection frame
        struct LoopCandidate {

            uint32_t current_id = 0;
            uint32_t history_id = 0;
            float iris_distance = 0.0F;
            int yaw_bias = 0;
            bool gicp_verified = false;
            double gicp_score = 1e9;
            double gicp_correction_trans = 0.0;
            double gicp_correction_rot_deg = 0.0;
            Eigen::Matrix4d source_to_target = Eigen::Matrix4d::Identity();
        };

        void loadParams();

        void callbackKeyFrame(
            const dlio::msg::KeyFrame::SharedPtr msg
        );

        bool convertKeyFrameMsg(
            const dlio::msg::KeyFrame &msg,
            LoopKeyFrame &keyframe
        ) const;

        bool computeIrisDescriptor(
            LoopKeyFrame &keyframe
        );

        bool detectLoopCandidate(
            const LoopKeyFrame &current,
            LoopCandidate &candidate
        ) const;

        bool verifyLoopCandidateByGicp(
            const LoopKeyFrame &current,
            LoopCandidate &candidate
        );

        bool isCandidateAllowed(
            const LoopKeyFrame &current,
            const LoopKeyFrame &history
        ) const;

        const LoopKeyFrame *findKeyFrame(
            uint32_t id
        ) const;

        Eigen::Matrix4d poseToMatrix(
            const geometry_msgs::msg::Pose &pose
        ) const;

        void storeKeyFrame(
            LoopKeyFrame &&keyframe
        );

        void publishLoopMarkers() const;

        rclcpp::Subscription<dlio::msg::KeyFrame>::SharedPtr sub_keyframe_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
            pub_loop_markers_;

        std::unique_ptr<LidarIris> lidar_iris_;
        GicpMatcher gicp_matcher_;
        std::vector<LoopKeyFrame> keyframes_;
        std::vector<LoopCandidate> loop_candidates_;

        std::string keyframe_topic_ = "keyframe_msg";
        std::string marker_topic_ = "loop_candidates_marker";
        std::string marker_frame_ = "odom";

        bool loop_enable_ = true;
        int loop_min_keyframe_gap_ = 30;
        float loop_iris_distance_thresh_ = 0.30F;
        bool loop_gicp_enable_ = true;
        double loop_gicp_score_thresh_ = 1.0;
        double loop_gicp_max_correction_trans_ = 3.0;
        double loop_gicp_max_correction_rot_deg_ = 20.0;
        int loop_gicp_num_threads_ = 4;
        int loop_gicp_correspondence_randomness_ = 20;
        double loop_gicp_max_correspondence_distance_ = 1.0;

        int iris_nscale_ = 4;
        int iris_min_wave_length_ = 18;
        float iris_mult_ = 1.6F;
        float iris_sigma_onf_ = 0.75F;
        int iris_match_num_ = 2;
    };

} // namespace small_dlio

#endif // SMALL_DLIO__LOOP_DETECTOR_NODE_HPP
