#include "curiosity_cpp_pkg/navigate_node_cpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

using namespace std;

NavigatorNode::NavigatorNode() : Node("navigator_node") {
    this->x, this->y, this->theta = 0;

    this->pose_received = false;
    this->obstacle_detected = false;
    
    // Brute Force (7, -3), (7, 7)
    this->goals = {{14, -7.5}};
    this->current_goal_index = 0;

    this-> goal1_flag = false;
    this-> goal2_flag = false;

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
        if (range < 1 && range > msg->range_min) {
            obstacle_detected = true;
            break;
        }
    }
}


void NavigatorNode::control_loop() {
    if (!pose_received) return;

    if (current_goal_index >= this->goals.size()) {
        RCLCPP_INFO(this->get_logger(), "Mission Complete! 🚩");

        geometry_msgs::msg::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        cmd_pub->publish(stop_cmd);
        return;
    }

    auto [goal_x, goal_y] = this->goals[current_goal_index];

    double dx = goal_x - this->x;
    double dy = goal_y - this->y;
    double goal_distance = std::hypot(dx, dy);

    double angle_to_goal = std::atan2(dy, dx);
    double angle_diff = angle_to_goal - this->theta;

    // Normalize angle_diff para ficar dentro de [-pi, pi]
    while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

    RCLCPP_INFO(this->get_logger(), "Pos (%.2f, %.2f), Theta: %.2f, Goal %ld: (%.2f, %.2f), Dist: %.2f, Obs: %s",
                this->x, this->y, this->theta, current_goal_index + 1, goal_x, goal_y, goal_distance,
                obstacle_detected ? "SIM" : "NÃO");

    geometry_msgs::msg::Twist cmd;


    if (goal_distance < 0.5) {
        RCLCPP_INFO(this->get_logger(), "Goal %ld alcançado! \a", current_goal_index + 1);

        cmd.linear.x = 0;
        cmd.angular.z = 0;

        cmd_pub->publish(cmd);

        current_goal_index++;
        return;
    }

    if (obstacle_detected) {
        RCLCPP_INFO(this->get_logger(),"aaaaaaaaaaaaaaaaaaaa \a");
        cmd.angular.z = 0.0; // gira para desviar do obstáculo
        cmd.linear.x = 0.0;

        cmd.linear.x = 0;
        cmd.angular.z = 0;

        cmd_pub->publish(cmd);
    }


    cmd.angular.z = angle_diff;
    cmd.linear.x = .5;

    cmd_pub->publish(cmd);
}

