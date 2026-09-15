#ifndef VELOCITY_NAVIGATION_HPP_
#define VELOCITY_NAVIGATION_HPP_

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"

#include "navigation.hpp"

/**
 * VelocityNavigation derived from the Navigation base class.
 * Used for basic velocity navigation.
 */
class VelocityNavigation : public Navigation
{
public:
  VelocityNavigation(
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock,
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher
  );

  // Navigation Control Interfaces
  void startNavigation() override;
  void stopNavigation() override;
  void resetNavigation() override;

  void update();
  void setVelocity(double velocity);

private:
  bool enable_velocity_ = false;
  double active_velocity_ = 0.0;
  double velocity_setpoint_ = 1.0;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_;
};

#endif