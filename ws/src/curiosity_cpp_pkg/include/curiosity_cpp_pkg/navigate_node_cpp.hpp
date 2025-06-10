#ifndef CURIOSITY_CPP_PKG_NAVIGATE_NODE_CPP_HPP_
#define CURIOSITY_CPP_PKG_NAVIGATE_NODE_CPP_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <vector>
#include <utility>

class NavigatorNode : public rclcpp::Node
{
public:
    NavigatorNode();

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void control_loop();

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double x_, y_, theta_;
    bool pose_received_;
    bool obstacle_detected_;

    std::vector<std::pair<double, double>> goals_;
    size_t current_goal_index_;
};

#endif // CURIOSITY_CPP_PKG_NAVIGATE_NODE_CPP_HPP_
