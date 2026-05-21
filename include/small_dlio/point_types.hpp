#ifndef SMALL_DLIO__POINT_TYPES
#define SMALL_DLIO__POINT_TYPES

#define PCL_NO_PRECOMPILE
#include <pcl-1.14/pcl/point_types.h>
#include <Eigen/Core>

namespace small_dlio {

    struct PointXYZIT {

        PCL_ADD_POINT4D;
        float intensity;
        float offset_time;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

POINT_CLOUD_REGISTER_POINT_STRUCT(small_dlio::PointXYZIT,                                                                          
    (float, x, x)                                                                                                                    
    (float, y, y)                                                                                                                    
    (float, z, z)                                                                                                                    
    (float, intensity, intensity)                                                                                                    
    (float, offset_time, offset_time)                                                                                                
)      

#include "odom_node/include_reg.hpp"

namespace small_dlio {

    struct Pose {

        Eigen::Vector3d p;
        Eigen::Quaterniond q;
    };

    struct State {

        Pose pose;
        Eigen::Vector3d v;
        Eigen::Vector3d b_a;
        Eigen::Vector3d b_g;
    };

    struct ImuMeas {

        double stamp;
        double dt;
        Eigen::Vector3d gyro;
        Eigen::Vector3d acc;
    };

    struct Extrinsics {

        Eigen::Vector3d t_body_imu; // body -> IMU translation
        Eigen::Quaterniond q_body_imu; // body -> IMU quaternion
        Eigen::Vector3d t_body_lidar;
        Eigen::Quaterniond q_body_lidar;
    };

    struct KeyFrame {

        Pose pose;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud; // pc after deskew
    };
} // small_dlio

#endif
