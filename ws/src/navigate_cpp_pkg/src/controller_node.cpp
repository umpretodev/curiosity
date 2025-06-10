#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class SimpleController : public rclcpp::Node {
public:
    SimpleController()
        : Node("simple_controller")
    {
        // Inicializa destino
        target_x_ = 7.0;
        target_y_ = -3.0;
        tolerance_ = 0.3;

        // Publisher para cmd_vel
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Subscriber para odom
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&SimpleController::odom_callback, this, std::placeholders::_1));

        // Timer para publicar comandos
        timer_ = this->create_wall_timer(
            100ms, std::bind(&SimpleController::publish_cmd, this));
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
    }

    void publish_cmd()
    {
        geometry_msgs::msg::Twist cmd;

        double dx = target_x_ - current_x_;
        double dy = target_y_ - current_y_;
        double distance = std::sqrt(dx * dx + dy * dy);

        if (distance > tolerance_)
        {
            // Vai reto com velocidade constante
            cmd.linear.x = 0.2;
            cmd.angular.z = 0.0;
        }
        else
        {
            // Parar
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Chegou no destino!");
        }

        publisher_->publish(cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    double current_x_{0.0};
    double current_y_{0.0};
    double target_x_;
    double target_y_;
    double tolerance_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleController>());
    rclcpp::shutdown();
    return 0;
}
