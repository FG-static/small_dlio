#ifndef SMALL_DLIO__GICP_HPP
#define SMALL_DLIO__GICP_HPP

#include "small_gicp_compat.hpp"

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>

namespace small_dlio {

    struct GicpParams {

        int num_threads = 4;
        int correspondence_randomness = 20;
        double max_correspondence_distance = 1.0;
    };

    struct GicpResult {

        bool success = false;
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        double score = 1e9;
        std::string error_message;
    };

    class GicpMatcher {

    public:

        GicpMatcher();

        void configure(const GicpParams &params);

        GicpResult align(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud,
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud,
            const Eigen::Matrix4d &init_guess
        );

    private:

        small_gicp::RegistrationPCL<pcl::PointXYZ, pcl::PointXYZ> reg_;
    };

} // namespace small_dlio

#endif // SMALL_DLIO__GICP_HPP
