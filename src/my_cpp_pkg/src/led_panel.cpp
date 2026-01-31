#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/led_state_array.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

using namespace std::placeholders;

class LedPanelNode : public rclcpp::Node
{
public:
    LedPanelNode() : Node("led_panel")
    {
        led_state_pub_ = this->create_publisher<my_robot_interfaces::msg::LedStateArray>(
          "led_states", 10);
        // led_state_timer_= this->create_wall_timer(
        //   std::chrono::seconds(1),
        //   std::bind(&LedPanelNode::publishLedStates, this));
        set_led_service_ = this->create_service<my_robot_interfaces::srv::SetLed>(
          "set_led",
          std::bind(&LedPanelNode::callbackSetLed, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "LED Panel Node has been started.");
    }
private:
    void publishLedStates(){
      auto msg = my_robot_interfaces::msg::LedStateArray();
      msg.led_states = led_states_;
      led_state_pub_->publish(msg);
    }

    void callbackSetLed(
      const my_robot_interfaces::srv::SetLed::Request::SharedPtr request,
      const my_robot_interfaces::srv::SetLed::Response::SharedPtr response)
    {
      led_number_ = request->led_number;
      state_ = request->state;

      if (led_number_ >= static_cast<int64_t>(led_states_.size()) || led_number_ < 0) {
        response->success = false;
        return;
      }
      if (state_ != 0 && state_ != 1) {
        response->success = false;
        return;
      }

      led_states_[led_number_] = state_;
      publishLedStates();
      response->success = true;
      return;
    }

    int64_t led_number_ = 0;
    int64_t state_ = 0;

    std::vector<int64_t> led_states_ = {0, 0, 0};
    rclcpp::Publisher<my_robot_interfaces::msg::LedStateArray>::SharedPtr led_state_pub_;
    // rclcpp::TimerBase::SharedPtr led_state_timer_;
    rclcpp::Service<my_robot_interfaces::srv::SetLed>::SharedPtr set_led_service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedPanelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}