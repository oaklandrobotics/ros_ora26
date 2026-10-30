#ifndef NAVIGATION_MANAGER_HPP_
#define NAVIGATION_MANAGER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "fusioncore_ros/srv/from_ll.hpp"
#include "geographic_msgs/msg/geo_point.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"

#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
#include "ora_interfaces/srv/navigation_info.hpp"

#include "../include/navigation.hpp"
#include "../include/gps_navigation.hpp"
#include "../include/velocity_navigation.hpp"

enum NavigationMode
{
  GPS,
  Velocity,
  Position
};

class NavigationManager : public rclcpp::Node
{
public:
  NavigationManager();

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  // Track navigation mode
  NavigationMode navigation_mode_ = NavigationMode::GPS;

  // Navigation Classes
  Navigation* active_navigation_ = nullptr;
  std::unique_ptr<GpsNavigation> gps_navigation_;
  std::unique_ptr<VelocityNavigation> velocity_navigation_;

  // Track auton state
  bool enable_navigation_ = false;

  // Navigation Control
  void startNavigation();
  void stopNavigation();
  void resetNavigation();

  void setNavigation(NavigationMode navigation_mode);

  // Update Callback
  void updateTimerCallback();

  // Subscriber Callbacks
  void poseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped msg
  );

  // Service Callbacks
  void setAutonCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response
  );

  void resetAutonCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response
  );

  void setCourseCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response
  );
  
  void getNavInfoCallback(
    const std::shared_ptr<ora_interfaces::srv::NavigationInfo::Request> request,
    std::shared_ptr<ora_interfaces::srv::NavigationInfo::Response> response
  );

  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_;

  // Timer
  rclcpp::TimerBase::SharedPtr update_timer_;

  // Subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

  // Service Client
  rclcpp::Client<fusioncore_ros::srv::FromLL>::SharedPtr from_ll_client_;

  // Services
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_auton_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_auton_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_course_srv_;

  rclcpp::Service<ora_interfaces::srv::NavigationInfo>::SharedPtr get_navigation_info_srv_;

  // Action Client
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
};

#endif