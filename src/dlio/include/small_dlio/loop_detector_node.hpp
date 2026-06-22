#ifndef SMALL_DLIO__LOOP_DETECTOR_NODE_HPP
#define SMALL_DLIO__LOOP_DETECTOR_NODE_HPP

#include "LidarIris.h"
#include "dlio/msg/key_frame.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

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

        bool isCandidateAllowed(
            const LoopKeyFrame &current,
            const LoopKeyFrame &history
        ) const;

        void storeKeyFrame(
            LoopKeyFrame &&keyframe
        );

        void publishLoopMarkers() const;

        rclcpp::Subscription<dlio::msg::KeyFrame>::SharedPtr sub_keyframe_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
            pub_loop_markers_;

        std::unique_ptr<LidarIris> lidar_iris_;
        std::vector<LoopKeyFrame> keyframes_;
        std::vector<LoopCandidate> loop_candidates_;

        std::string keyframe_topic_ = "keyframe_msg";
        std::string marker_topic_ = "loop_candidates_marker";
        std::string marker_frame_ = "odom";

        bool loop_enable_ = true;
        int loop_min_keyframe_gap_ = 30;
        float loop_iris_distance_thresh_ = 0.30F;

        int iris_nscale_ = 4;
        int iris_min_wave_length_ = 18;
        float iris_mult_ = 1.6F;
        float iris_sigma_onf_ = 0.75F;
        int iris_match_num_ = 2;
    };

} // namespace small_dlio

#endif // SMALL_DLIO__LOOP_DETECTOR_NODE_HPP
