#ifndef ORA_NAVIGATION_HPP_
#define ORA_NAVIGATION_HPP_

#include "rclcpp/rclcpp.hpp"

/**
 * Base class for all navigation implementations. Defines common navigation control interfaces
 * and provides shared resources used by derived navigation classes.
 */
class Navigation
{
public:
  Navigation(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock) :
    logger_(logger), clock_(clock) {}

  virtual ~Navigation() = default;

  // Virtual navigation control interfaces
  virtual void startNavigation() = 0;
  virtual void stopNavigation() = 0;
  virtual void resetNavigation() = 0;

protected:
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
};

#endif