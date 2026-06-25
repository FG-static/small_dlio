#include "map_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<small_dlio::MapNode>());
    rclcpp::shutdown();
    return 0;
}
