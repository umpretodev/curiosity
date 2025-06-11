#ifndef CURIOSITY_CPP_PKG_NAVIGATE_NODE_CPP_HPP_
#define CURIOSITY_CPP_PKG_NAVIGATE_NODE_CPP_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <vector>
#include <utility>

using namespace std;

class NavigatorNode : public rclcpp::Node {
public:
    NavigatorNode();

private:
    double x, y, theta;
    bool pose_received;
    bool obstacle_detected;

    bool goal1_checked;
    bool goal2_checked;

    vector<pair<double, double>> goals;
    size_t current_goal_index;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
    rclcpp::TimerBase::SharedPtr timer_;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void control_loop();

    void handle_goal_by_go_to_goal_strategy(size_t goal_index);
    void handle_goal_by_wall_following_strategy(size_t goal_index);
};

#endif
