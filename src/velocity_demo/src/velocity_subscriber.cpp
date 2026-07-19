#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

class VelocitySubscriber : public rclcpp::Node
{
public:
    VelocitySubscriber()
        : Node("velocity_subscriber")
    {
        // 订阅 /velocity 话题。
        subscription_ =
            this->create_subscription<geometry_msgs::msg::Twist>(
                "/velocity",
                10,
                [this](
                    const geometry_msgs::msg::Twist::SharedPtr message
                ) {
                    topic_callback(message);
                }
            );

        RCLCPP_INFO(
            this->get_logger(),
            "Velocity subscriber started."
        );
    }

private:
    void topic_callback(
        const geometry_msgs::msg::Twist::SharedPtr message
    )
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Receive: linear.x=%.2f, angular.z=%.2f",
            message->linear.x,
            message->angular.z
        );
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
        subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<VelocitySubscriber>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}