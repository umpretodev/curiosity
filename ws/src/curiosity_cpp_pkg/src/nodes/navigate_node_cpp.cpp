#include "curiosity_cpp_pkg/navigate_node_cpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include "curiosity_cpp_pkg/math_util.hpp"

using namespace std;

NavigatorNode::NavigatorNode() : Node("navigator_node") {
    this->x, this->y, this->theta = 0;

    this->pose_received = false;
    this->obstacle_detected = false;
    
    // Brute Force (7, -3), (7, 7)
    this->goals = {{14, -7.5}, {14, 14}};
    this->current_goal_index = 0;

    this-> goal1_checked = false;
    this-> goal2_checked = false;

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&NavigatorNode::odom_callback, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/base_scan", 10, std::bind(&NavigatorNode::scan_callback, this, std::placeholders::_1));
    cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&NavigatorNode::control_loop, this));
}

void NavigatorNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    this->x = msg->pose.pose.position.x;
    this->y = msg->pose.pose.position.y;

    // Handle theta
    tf2::Quaternion quaternion(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );


    tf2::Matrix3x3 matrix(quaternion);
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);
    this->theta = yaw;

    pose_received = true;
}

void NavigatorNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    obstacle_detected = false;

    for (float range : msg->ranges) {
        if (range < 0.3 && range > msg->range_min) {
            obstacle_detected = true;
            break;
        }
    }
}

void NavigatorNode::handle_goal_by_wall_following_strategy(size_t goal_index) {
    RCLCPP_INFO(this->get_logger(), "Letssssss gooooooooooooooooo (7, 3)! 🚩\a");
}

void NavigatorNode::handle_goal_by_go_to_goal_strategy(size_t goal_index) {
    auto [goal_x, goal_y] = this->goals[goal_index];

    double dx = goal_x - this->x;
    double dy = goal_y - this->y;
    double goal_distance = std::hypot(dx, dy);

    double angle_to_goal = std::atan2(dy, dx);
    double angle_diff = angle_to_goal - this->theta;

    angle_diff = MathUtil::normalize_angle(angle_diff);

    geometry_msgs::msg::Twist cmd;

    if (goal_distance < 0.3) {
        RCLCPP_INFO(this->get_logger(), "First completed Goal (7, 3)! 🚩\a");

        cmd.linear.x = 0;
        cmd.angular.z = 0;
        cmd_pub->publish(cmd);

        this->current_goal_index ++;
        this->goal1_checked = true;

        return;
    }

    if (obstacle_detected) {
        RCLCPP_INFO(this->get_logger(), "\a");

        cmd.angular.z = 0.0; 
        cmd.linear.x = 0.0;

        cmd_pub->publish(cmd);
        return;
    }

    cmd.angular.z = angle_diff;
    cmd.linear.x = .5;
    cmd_pub->publish(cmd);
}

void NavigatorNode::control_loop() {
    if (!pose_received) return;

    if (this->goal1_checked && this->goal2_checked) {
        RCLCPP_INFO(this->get_logger(), "Mission Complete! 🚩");

        geometry_msgs::msg::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        cmd_pub->publish(stop_cmd);
        return;
    }

    this->goal1_checked ? this->handle_goal_by_go_to_goal_strategy(current_goal_index) : this->handle_goal_by_wall_following_strategy(current_goal_index);
}