cmake_minimum_required(VERSION 3.8)
project(dlio)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters kdtree)
find_package(small_gicp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_library(G2O_CORE_LIBRARY g2o_core REQUIRED)
find_library(G2O_STUFF_LIBRARY g2o_stuff REQUIRED)
find_library(G2O_TYPES_SLAM3D_LIBRARY g2o_types_slam3d REQUIRED)
find_library(G2O_SOLVER_EIGEN_LIBRARY g2o_solver_eigen REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/KeyFrame.msg"
  "msg/OptimizedKeyFrames.msg"
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
)

add_library(lidar_iris_vendor STATIC
  third_party/lidar_iris/LidarIris.cpp
  third_party/lidar_iris/fftm/fftm.cpp
)

target_include_directories(lidar_iris_vendor
  PUBLIC
    third_party/lidar_iris
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(lidar_iris_vendor
  PUBLIC
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)

add_library(small_dlio_gicp STATIC
  src/gicp.cpp
)

target_include_directories(small_dlio_gicp
  PUBLIC
    include
    include/small_dlio
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(small_dlio_gicp
  PUBLIC
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    small_gicp::small_gicp
)

add_library(small_dlio_pose_graph STATIC
  src/pose_graph.cpp
)

target_include_directories(small_dlio_pose_graph
  PUBLIC
    include
    include/small_dlio
)

target_link_libraries(small_dlio_pose_graph
  PUBLIC
    Eigen3::Eigen
    ${G2O_CORE_LIBRARY}
    ${G2O_STUFF_LIBRARY}
    ${G2O_TYPES_SLAM3D_LIBRARY}
    ${G2O_SOLVER_EIGEN_LIBRARY}
)

add_executable(dlio_node
  src/main.cpp
  src/map_node.cpp
  src/odom_node.cpp
)

add_executable(loop_detector_node
  src/loop_detector_main.cpp
  src/loop_detector_node.cpp
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp
)

target_include_directories(dlio_node
  PRIVATE
    include
    include/small_dlio
    ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(dlio_node
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_eigen
  pcl_conversions
  livox_ros_driver2
)

target_include_directories(loop_detector_node
  PRIVATE
    include
    include/small_dlio
    third_party/lidar_iris
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(loop_detector_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  pcl_conversions
)

target_link_libraries(dlio_node
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  small_dlio_gicp
  lidar_iris_vendor
  OpenMP::OpenMP_CXX
  "${cpp_typesupport_target}"
)

target_link_libraries(loop_detector_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  small_dlio_gicp
  small_dlio_pose_graph
  lidar_iris_vendor
  "${cpp_typesupport_target}"
)

install(TARGETS dlio_node loop_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY third_party/lidar_iris/
  DESTINATION share/${PROJECT_NAME}/third_party/lidar_iris
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
