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
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(small_gicp REQUIRED)
find_package(OpenMP REQUIRED)

add_executable(dlio_node
  src/main.cpp
  src/map_node.cpp
  src/odom_node.cpp
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
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_eigen
  pcl_conversions
  livox_ros_driver2
)

target_link_libraries(dlio_node
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  small_gicp::small_gicp
  OpenMP::OpenMP_CXX
)

install(TARGETS dlio_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
