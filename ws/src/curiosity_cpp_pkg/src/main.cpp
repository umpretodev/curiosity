#include "rclcpp/rclcpp.hpp"
#include "curiosity_cpp_pkg/navigate_node_cpp.hpp"

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}