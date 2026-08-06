#ifndef CONTROLLER_NODE_HPP
#define CONTROLLER_NODE_HPP

#include <deque>
#include <chrono>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joy_feedback.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode();

private:
  struct TimedJoyFeedback
  {
    float intensity;
    std::chrono::milliseconds duration;
  };

  // Publisher Callbacks
  void setJoyFeedbackCallback();

  // Subscriber Callbacks
  void joyCallback(
    const sensor_msgs::msg::Joy::SharedPtr msg
  );

  // Service Calls
  void setAuton();
  void resetNav();
  void setEstop();

  void setCourse();

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::JoyFeedback>::SharedPtr joy_feedback_publisher_;

  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

  // Service Client
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr set_auton_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_nav_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr set_estop_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr set_course_client_;

  // Service Client Callbacks
  void setCourseClientCallback(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future);

  // Vector to store feedback queue
  std::deque<TimedJoyFeedback> pending_feedback_queue;

  // Vector for each feedback type
  inline static const std::deque<TimedJoyFeedback> north_course_feedback_ = 
  {
    {1.0F, 500ms}
  };
  
  inline static const std::deque<TimedJoyFeedback> south_course_feedback_ = 
  {
    {1.0F, 500ms},
    {0.0F, 250ms},
    {1.0F, 500ms}
  };

  // Timer for feedback callbacks
  rclcpp::TimerBase::SharedPtr timer_;

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
  static constexpr uint8_t k_drive_forward_button_ = 11;
};

#endif