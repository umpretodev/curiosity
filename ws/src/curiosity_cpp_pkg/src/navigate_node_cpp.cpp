#include "curiosity_cpp_pkg/navigate_node_cpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

NavigatorNode::NavigatorNode()
    : Node("navigator_node"),
      x_(0.0),
      y_(0.0),
      theta_(0.0),
      pose_received_(false),
      obstacle_detected_(false),
      current_goal_index_(0)
{
    // Defina seus goals (alvo) aqui
    goals_ = {{7.0, 7.0}, {7.0, -3.0}};

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&NavigatorNode::odom_callback, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/base_scan", 10, std::bind(&NavigatorNode::scan_callback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&NavigatorNode::control_loop, this));
}

void NavigatorNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    theta_ = yaw;
    pose_received_ = true;
}

void NavigatorNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    obstacle_detected_ = false;
    for (float r : msg->ranges)
    {
        if (r < 0.6 && r > msg->range_min)
        {
            obstacle_detected_ = true;
            break;
        }
    }
}

void NavigatorNode::control_loop()
{
    if (!pose_received_)
    {
        RCLCPP_WARN(this->get_logger(), "Pose ainda não recebida");
        return;
    }

    if (current_goal_index_ >= goals_.size())
    {
        RCLCPP_INFO(this->get_logger(), "Todos os goals alcançados!");
        geometry_msgs::msg::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        cmd_pub_->publish(stop_cmd);
        return;
    }

    auto [goal_x, goal_y] = goals_[current_goal_index_];
    double dx = goal_x - x_;
    double dy = goal_y - y_;
    double distance = std::hypot(dx, dy);
    double angle_to_goal = std::atan2(dy, dx);
    double angle_diff = angle_to_goal - theta_;

    // Normalize angle_diff para ficar dentro de [-pi, pi]
    while (angle_diff > M_PI)
        angle_diff -= 2 * M_PI;
    while (angle_diff < -M_PI)
        angle_diff += 2 * M_PI;

    RCLCPP_INFO(this->get_logger(), "Pos (%.2f, %.2f), Theta: %.2f, Goal %ld: (%.2f, %.2f), Dist: %.2f, Obs: %s",
                x_, y_, theta_, current_goal_index_ + 1, goal_x, goal_y, distance,
                obstacle_detected_ ? "SIM" : "NÃO");

    geometry_msgs::msg::Twist cmd;

    if (distance < 0.3)
    {
        RCLCPP_INFO(this->get_logger(), "Goal %ld alcançado!", current_goal_index_ + 1);
        current_goal_index_++;
        return;
    }

    if (obstacle_detected_)
    {
        cmd.angular.z = 0.5; // gira para desviar do obstáculo
        cmd.linear.x = 0.0;
    }
    else
    {
        if (std::abs(angle_diff) > 0.2)
        {
            cmd.angular.z = (angle_diff > 0) ? 0.5 : -0.5;
            cmd.linear.x = 0.0;
        }
        else
        {
            cmd.angular.z = angle_diff;
            cmd.linear.x = 0.5;
        }
    }

    cmd_pub_->publish(cmd);
}

