#include "gicp.hpp"

#include <exception>

namespace small_dlio {

    GicpMatcher::GicpMatcher() {

        configure(GicpParams{});
    }

    void GicpMatcher::configure(const GicpParams &params) {

        reg_.setRegistrationType("GICP");
        reg_.setNumThreads(params.num_threads);
        reg_.setCorrespondenceRandomness(params.correspondence_randomness);
        reg_.setMaxCorrespondenceDistance(params.max_correspondence_distance);
    }

    GicpResult GicpMatcher::align(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud,
        const Eigen::Matrix4d &init_guess
    ) {

        GicpResult result;

        if (!source_cloud || source_cloud->empty() ||
            !target_cloud || target_cloud->empty()) {

            result.error_message = "empty source or target cloud";
            return result;
        }

        reg_.setInputSource(source_cloud);
        reg_.setInputTarget(target_cloud);

        auto aligned = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        try {

            reg_.align(*aligned, init_guess.cast<float>());
        } catch (const std::exception &e) {

            result.error_message = e.what();
            return result;
        }

        result.score = reg_.getFitnessScore();
        result.transform = reg_.getFinalTransformation().cast<double>();
        if (!result.transform.allFinite()) {

            result.score = 1e9;
            result.transform = Eigen::Matrix4d::Identity();
            result.error_message = "non-finite transformation";
            return result;
        }

        result.success = true;
        return result;
    }

} // namespace small_dlio
