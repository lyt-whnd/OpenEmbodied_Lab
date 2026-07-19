#include <chrono>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class VelocityPublisher : public rclcpp::Node
{
public:
    VelocityPublisher()
        : Node("velocity_publisher")  //gei node ming ming
    {
        // 创建 Twist 消息发布者，发布到 /velocity。
        publisher_ =
            this->create_publisher<geometry_msgs::msg::Twist>(
                "/velocity",
                10
            );

        // 每 100 ms 调用一次 timer_callback。
        timer_ =
            this->create_wall_timer(
                100ms,
                [this]() {
                    timer_callback();
                }
            );

        RCLCPP_INFO(
            this->get_logger(),
            "Velocity publisher started."
        );
    }

private:
    void timer_callback()
    {
        geometry_msgs::msg::Twist message;

        message.linear.x = 1.0;
        message.angular.z = 0.5;

        publisher_->publish(message);

        RCLCPP_INFO(
            this->get_logger(),
            "Publish: linear.x=%.2f, angular.z=%.2f",
            message.linear.x,
            message.angular.z
        );
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<VelocityPublisher>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}