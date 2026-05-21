#include "rclcpp/rclcpp.hpp"
#include "odom_node/odom_node.hpp"

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<small_dlio::OdomNode>());
    rclcpp::shutdown();
    return 0;
}
