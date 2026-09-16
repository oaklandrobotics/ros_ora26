#ifndef GPS_WAYPOINT_FOLLOWER_HPP_
#define GPS_WAYPOINT_FOLLOWER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "fusioncore_ros/srv/from_ll.hpp"
#include "geographic_msgs/msg/geo_point.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "action_msgs/srv/cancel_goal.hpp"

#include "../include/navigation.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;

/**
 * GpsNavigation derived from the Navigation base class.
 * Used for GPS navigation between defined lat/lon waypoints.
 */
class GpsNavigation : public Navigation
{
public:
  GpsNavigation(
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock,
    rclcpp::Client<fusioncore_ros::srv::FromLL>::SharedPtr from_ll_client,
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client,
    std::string waypoint_file_path
  );

  struct NavigationState
  {
    std::vector<geometry_msgs::msg::Point> localized_waypoints;
    size_t active_index;
    std::string starting_direction;
  };

  // Navigation Control Interfaces
  void startNavigation() override;
  void stopNavigation() override;
  void resetNavigation() override;

  // Navigation Information Interfaces
  void updatePose(const geometry_msgs::msg::PoseWithCovarianceStamped pose);
  void setCourse(const bool is_practice_course);
  const NavigationState getNavigationState();

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  // Setup / Loading
  void initialize(const YAML::Node& config);
  void loadWaypoints(
    const YAML::Node& waypoint_group,
    std::vector<geographic_msgs::msg::GeoPoint>& destination_vector
  );
  void transformNextWaypoint();
  void addStartingWaypoint();

  NavigateToPose::Goal buildNavigateToPoseGoal(
    const geometry_msgs::msg::Point& localized_waypoint
  );
  void navigateToWaypoint(const geometry_msgs::msg::Point& localized_waypoint);

  // Waypoints
  bool practice_course_ = false;
  std::vector<geographic_msgs::msg::GeoPoint> practice_course_waypoints_;
  std::vector<geographic_msgs::msg::GeoPoint> main_course_waypoints_;
  std::vector<geographic_msgs::msg::GeoPoint> selected_waypoints_;

  std::vector<geometry_msgs::msg::Point> localized_waypoints_;

  // Track last known pose
  geometry_msgs::msg::PoseWithCovarianceStamped last_known_pose_;

  // Track auton state
  bool enable_follower_ = false;
  size_t waypoint_transform_index_ = 0;

  // Track which waypoint is currently being navigated to
  bool waypoints_configured_ = false;
  size_t current_waypoint_index_ = 0;
  size_t retry_events_ = 0;

  void fromLLCallback(
    geographic_msgs::msg::GeoPoint waypoint,
    rclcpp::Client<fusioncore_ros::srv::FromLL>::SharedFuture future_response
  );

  // Action Callbacks
  void navGoalResponseCallback(
    const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr& goal_handle
  );

  void navGoalFeedbackCallback(
    rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback
  );

  void navGoalResultCallback(
    const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult& result
  );

  void navCancelGoalCallback(
    const std::shared_ptr<action_msgs::srv::CancelGoal_Response>& cancel_response
  );

  // Service Client
  rclcpp::Client<fusioncore_ros::srv::FromLL>::SharedPtr from_ll_client_;

  // Action Client
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
  rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr current_goal_handle_;
};

#endif