#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using namespace std::placeholders;

class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter")
    {
        number_counter_publisher_ = this->create_publisher<example_interfaces::msg::Int64>(
          "number_count", 10);

        number_counter_ = this->create_subscription<example_interfaces::msg::Int64>(
          "number", 10,
          std::bind(&NumberCounterNode::callbackNumber, this, _1));
        RCLCPP_INFO(this->get_logger(), "Number Counter has been started.");
        reset_counter_service_ = this->create_service<example_interfaces::srv::SetBool>(
          "reset_counter",
          std::bind(&NumberCounterNode::callbackResetCounter, this, _1, _2));
    }
private:
    void callbackNumber(const example_interfaces::msg::Int64::SharedPtr msg)
    {
      counter_ += msg->data;

      auto new_msg_ = example_interfaces::msg::Int64();
      new_msg_.data = counter_;
      number_counter_publisher_->publish(new_msg_);
      RCLCPP_INFO(this->get_logger(), "%ld", new_msg_.data);
    }

    void callbackResetCounter(
      const example_interfaces::srv::SetBool::Request::SharedPtr request,
      const example_interfaces::srv::SetBool::Response::SharedPtr response)
    {
      if (request->data) {
        counter_ = 0;
        response->success = true;
        response->message = "Counter has been reset.";
      } else {
        response->success = false;
        response->message = "Counter has not been reset.";
      }
    }


    int64_t counter_ = 0;

    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr number_counter_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr number_counter_publisher_;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr reset_counter_service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}