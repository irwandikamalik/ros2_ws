#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

class BatteryNode : public rclcpp::Node
{
public:
    BatteryNode() : Node("battery")
    {
      battery_state_ = "full";
      last_time_battery_state_changed_ = getCurrentTimeSeconds();
      battery_timer_ = this->create_wall_timer(
        1s, std::bind(&BatteryNode::callbackCheckBatteryState, this));
      set_led_client_ = this->create_client<my_robot_interfaces::srv::SetLed>("set_led");
      RCLCPP_INFO(this->get_logger(), "Battery node has been started.");
    }
private:
    double getCurrentTimeSeconds(){
      rclcpp::Time now = this->get_clock()->now();
      return now.seconds();
    }


    void callbackCheckBatteryState(){
      double time_now = getCurrentTimeSeconds();
      if (battery_state_ == "full"){
        if (time_now - last_time_battery_state_changed_ >= 4.0){
          battery_state_ = "empty";
          RCLCPP_INFO(this->get_logger(), "Battery is now empty. Charging...");
          callSetLed(2, 1);
          last_time_battery_state_changed_ = time_now;
        }
      } else if (battery_state_ == "empty"){
        if (time_now - last_time_battery_state_changed_ >= 6.0){
          battery_state_ = "full";
          RCLCPP_INFO(this->get_logger(), "Battery is now full. Stopping Charge.");
          callSetLed(2, 0);
          last_time_battery_state_changed_ = time_now;
        }
      }

    }

    void callSetLed(int led_number, int state){
      while (!set_led_client_->wait_for_service(std::chrono::seconds(1)))
      {
        RCLCPP_WARN(this->get_logger(), "Waiting for the set_led service to be available...");
      }

      auto request = std::make_shared<my_robot_interfaces::srv::SetLed::Request>();
      request->led_number = led_number;
      request->state = state;

      auto future = set_led_client_->async_send_request(
        request,
        std::bind(&BatteryNode::callbackCallSetLed, this, _1));
    }

    void callbackCallSetLed(rclcpp::Client<my_robot_interfaces::srv::SetLed>::SharedFuture future)
    {
      auto response = future.get();
      if (response->success){
        RCLCPP_INFO(this->get_logger(), "LED state changed successfully.");
      }else{
        RCLCPP_ERROR(this->get_logger(), "Failed to change LED state.");
      }
    }

    std::string battery_state_;
    double last_time_battery_state_changed_;

    rclcpp::TimerBase::SharedPtr battery_timer_;
    rclcpp::Client<my_robot_interfaces::srv::SetLed>::SharedPtr set_led_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}