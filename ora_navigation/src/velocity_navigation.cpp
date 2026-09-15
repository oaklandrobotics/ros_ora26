#include "../include/velocity_navigation.hpp"

VelocityNavigation::VelocityNavigation(
  rclcpp::Logger logger,
  rclcpp::Clock::SharedPtr clock,
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher
) : Navigation(logger, clock), twist_publisher_(twist_publisher)
{}

void VelocityNavigation::startNavigation()
{
  enable_velocity_ = true;
  active_velocity_ = velocity_setpoint_;
}

void VelocityNavigation::stopNavigation()
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = clock_->now();

  // Publish 0 velocity and rotation
  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.angular.z = 0.0;
  twist_publisher_->publish(cmd_vel);

  // Disable velocity control and set velocity to 0
  enable_velocity_ = false;
  active_velocity_ = 0.0;
}

void VelocityNavigation::resetNavigation()
{
  stopNavigation();

  velocity_setpoint_ = 1.0;
}

void VelocityNavigation::setVelocity(double velocity)
{
  velocity_setpoint_ = velocity;
  active_velocity_ = (enable_velocity_ ? velocity_setpoint_ : 0.0);
}

void VelocityNavigation::update()
{
  if (enable_velocity_)
  {
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = clock_->now();

    // Publish velocity command with no rotation
    cmd_vel.twist.linear.x = active_velocity_;
    cmd_vel.twist.angular.z = 0.0;
    twist_publisher_->publish(cmd_vel);
  }
}