#ifndef GPS_WAYPOINT_FOLLOWER
#define GPS_WAYPOINT_FOLLOWER

#include <chrono>
#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "fusioncore_ros/srv/from_ll.hpp"
#include "geographic_msgs/msg/geo_point.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

class GpsWaypointFollower : public rclcpp::Node
{
  public:
    GpsWaypointFollower();

  private:
    void initialize(const YAML::Node& config);
    void loadWaypoints(const YAML::Node& waypoint_group, std::vector<geographic_msgs::msg::GeoPoint>& destination_vector);
    void transformNextWaypoint();

    void startNavigation();
    void stopNavigation();
    void resetNavigation();
    void navigateToWaypoint(const geometry_msgs::msg::Point& localized_waypoint);

    void setAutonCallback(std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    void resetAutonCallback(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void setCourseCallback(std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    // Waypoints
    bool practiceCourse_ = false;
    std::vector<geographic_msgs::msg::GeoPoint> practice_course_waypoints_;
    std::vector<geographic_msgs::msg::GeoPoint> main_course_waypoints_;
    std::vector<geographic_msgs::msg::GeoPoint> selected_waypoints_;

    std::vector<geometry_msgs::msg::Point> localized_waypoints_;

    // Track auton state
    bool enableFollower_ = false;
    size_t waypoint_transform_index_ = 0;

    // Track which waypoint is currently being navigated to
    bool waypointsConfigured_ = false;
    size_t current_waypoint_index_ = 0;

    // Service Client
    rclcpp::Client<fusioncore_ros::srv::FromLL>::SharedPtr from_ll_client_;

    // Services
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_auton_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_auton_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_course_srv_;

    // Action Client
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client_;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr current_goal_handle_;
};

#endif