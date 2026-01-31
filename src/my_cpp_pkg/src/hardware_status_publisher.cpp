#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/hardware_status.hpp"

using namespace std::chrono_literals;

class HardwareStatusPublisherNode : public rclcpp::Node
{
public:
    HardwareStatusPublisherNode() : Node("hardware_status_publisher")
    {
      pub_ = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>("hardware_status", 10);
      timer_ = this->create_wall_timer(
        1s, std::bind(&HardwareStatusPublisherNode::publish_status, this));
      RCLCPP_INFO(this->get_logger(), "Hardware Status Publisher Node has been started.");
    }
private:
    void publish_status()
    {
      auto message = my_robot_interfaces::msg::HardwareStatus();
      message.temperature = 36.5;  // Example temperature
      message.are_motors_ready = true;  // Example motor status
      message.debug_message = "All systems operational.";  // Example debug message
      pub_->publish(message);
    }

    rclcpp::Publisher<my_robot_interfaces::msg::HardwareStatus>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareStatusPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}