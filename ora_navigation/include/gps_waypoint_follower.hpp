#ifndef GPS_WAYPOINT_FOLLOWER
#define GPS_WAYPOINT_FOLLOWER

#include <chrono>
#include <memory>
#include <string>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "fusioncore_ros/srv/from_ll.hpp"
#include "geographic_msgs/msg/geo_point.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "std_srvs/srv/set_bool.hpp"

#include "std_msgs/msg/string.hpp"

class GpsWaypointFollower : public rclcpp::Node
{
  public:
    GpsWaypointFollower();

  private:
    void initialize(const YAML::Node& config);
    void loadWaypoints(const YAML::Node& waypoint_group, std::vector<geographic_msgs::msg::GeoPoint>& destination_vector);
    bool localizeWaypoints(const std::vector<geographic_msgs::msg::GeoPoint>& known_gps_waypoints);
    void transformNextWaypoint();

    void setAutonCallback(std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    void setCourseCallback(std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    rclcpp::TimerBase::SharedPtr waypoint_init_timer_;

    rclcpp::Client<fusioncore_ros::srv::FromLL>::SharedPtr from_ll_client_;

    bool practiceCourse_ = false;
    std::vector<geographic_msgs::msg::GeoPoint> practice_course_waypoints_;
    std::vector<geographic_msgs::msg::GeoPoint> main_course_waypoints_;
    std::vector<geographic_msgs::msg::GeoPoint> selected_waypoints_;

    std::vector<geometry_msgs::msg::Point> localized_waypoints_;

    bool enableFollower_ = false;
    size_t waypoint_transform_index_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr temp_publisher_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_auton_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_course_srv_;
};

#endif