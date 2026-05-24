#ifndef CONTROLLER_NODE_HPP
#define CONTROLLER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode();

private:
  // Topic Callbacks
  void joyCallback(
    const sensor_msgs::msg::Joy::SharedPtr msg
  );

  // Service Calls
  void setAuton();
  void resetNav();
  void setEstop();

  void setCourse();

  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

  // Service Client
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr set_auton_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_nav_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr set_estop_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr set_course_client_;

  // Track auton/estop state
  bool auton_enabled_ = false;
  bool estop_enabled_ = false;
  bool practice_course_enabled_ = false;

  // Track button state
  bool set_auton_pressed_ = false;
  bool reset_nav_pressed_ = false;
  bool set_estop_pressed_ = false;
  bool set_course_pressed_ = false;

  // Constants
  static constexpr uint8_t k_reset_nav_button_ = 0;
  static constexpr uint8_t k_set_estop_button_ = 1;
  static constexpr uint8_t k_set_course_button_ = 4;
  static constexpr uint8_t k_set_auton_button_ = 3;
};

#endif