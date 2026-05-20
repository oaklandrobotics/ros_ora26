#include "../include/controller_node.hpp"

ControllerNode::ControllerNode() : Node("controller_node")
{
  // Initialize Subscriber and assign Callback
  joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy",
    10,
    [this](sensor_msgs::msg::Joy::SharedPtr msg)
    {
      joyCallback(msg);
    }
  );

  // Initialize Service Clients
  set_auton_client_ = this->create_client<std_srvs::srv::SetBool>("navigation/set_auton");
  reset_nav_client_ = this->create_client<std_srvs::srv::Trigger>("navigation/reset_auton");
  set_estop_client_ = this->create_client<std_srvs::srv::SetBool>("estop");
  set_course_client_ = this->create_client<std_srvs::srv::SetBool>("navigation/set_course");

  RCLCPP_INFO(
    this->get_logger(),
    "ControllerNode initialized"
  );
}

void ControllerNode::joyCallback(sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (!msg->buttons.empty())
  {
    // Get the current state of the buttons for rising edge logic
    const bool set_auton_now = msg->buttons[k_set_auton_button_];
    const bool reset_nav_now = msg->buttons[k_reset_nav_button_];
    const bool set_estop_now = msg->buttons[k_set_estop_button_];
    const bool set_course_now = msg->buttons[k_set_course_button_];

    // Set autonomous state
    if (set_auton_now && !set_auton_pressed_)
    {
      setAuton();
    }

    // Reset autonomous state
    if (reset_nav_now && !reset_nav_pressed_)
    {
      resetNav();
    }

    // Set estop state
    if (set_estop_now && !set_estop_pressed_)
    {
      setEstop();
    }

    // Set active course
    if (set_course_now && !set_course_pressed_)
    {
      setCourse();
    }

    // Store the state of the button for rising edge logic
    set_auton_pressed_ = set_auton_now;
    reset_nav_pressed_ = reset_nav_now;
    set_estop_pressed_ = set_estop_now;
    set_course_pressed_ = set_course_now;
  }
}

void ControllerNode::setAuton()
{
  RCLCPP_INFO(
    this->get_logger(),
    "Auton button pressed"
  );

  if (!set_auton_client_->service_is_ready())
  {
    RCLCPP_WARN(
      this->get_logger(),
      "set_auton service is not ready yet"
    );

    return;
  }

  // auton_enabled_ is flipped
  auton_enabled_ = !auton_enabled_;

  // Send a SetBool request to the set_auton_client
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = auton_enabled_;

  set_auton_client_->async_send_request(request);
}

void ControllerNode::resetNav()
{
  RCLCPP_INFO(
    this->get_logger(),
    "Reset button pressed"
  );

  if (!reset_nav_client_->service_is_ready())
  {
    RCLCPP_WARN(
      this->get_logger(),
      "reset_nav service is not ready yet"
    );

    return;
  }

  // Send a Trigger request to the reset_nav_client_
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  reset_nav_client_->async_send_request(request);
}

void ControllerNode::setEstop()
{
  RCLCPP_INFO(
    this->get_logger(),
    "E-Stop button pressed"
  );

  if (!set_estop_client_->service_is_ready())
  {
    RCLCPP_WARN(
      this->get_logger(),
      "estop service is not ready yet"
    );

    return;
  }

  // estop_enabled_ is flipped
  estop_enabled_ = !estop_enabled_;

  // Send a message to the set_estop_client_
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = estop_enabled_;

  set_estop_client_->async_send_request(request);
}

void ControllerNode::setCourse()
{
  RCLCPP_INFO(
    this->get_logger(),
    "Set course button pressed"
  );

  if (!set_course_client_->service_is_ready())
  {
    RCLCPP_WARN(
      this->get_logger(),
      "set_course service is not ready yet"
    );

    return;
  }

  // practice_course_enabled_ is flipped
  practice_course_enabled_ = !practice_course_enabled_;

  // Send a message to the set_auton_client
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = practice_course_enabled_;

  set_course_client_->async_send_request(request);
}

/**
 * Create and spin the ControllerNode
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ControllerNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}