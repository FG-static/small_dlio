#include "rclcpp/rclcpp.hpp"
#include "odom_node/odom_node.hpp"
#include "map_node.hpp"

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    auto odom_node = std::make_shared<small_dlio::OdomNode>();
    auto map_node = std::make_shared<small_dlio::MapNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(odom_node);
    executor.add_node(map_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
