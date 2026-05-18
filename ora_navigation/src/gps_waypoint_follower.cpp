#include "../include/gps_waypoint_follower.hpp"

GpsWaypointFollower::GpsWaypointFollower() : Node("gps_waypoint_follower")
{
  const auto package_share_dir = ament_index_cpp::get_package_share_directory("ora_navigation");

  this->declare_parameter("waypoints_file", package_share_dir + "/config/waypoints.yaml");

  const auto waypoints_file = this->get_parameter("waypoints_file").as_string();

  from_ll_client_ = this->create_client<fusioncore_ros::srv::FromLL>("/fromLL");

  set_auton_srv_ = this->create_service<std_srvs::srv::SetBool>("navigation/set_auton",
                        std::bind(&GpsWaypointFollower::setAutonCallback, this, std::placeholders::_1, std::placeholders::_2));
  set_course_srv_ = this->create_service<std_srvs::srv::SetBool>("navigation/set_course",
                        std::bind(&GpsWaypointFollower::setCourseCallback, this, std::placeholders::_1, std::placeholders::_2));

  const YAML::Node config_file = YAML::LoadFile(waypoints_file);
  initialize(config_file);
}

void GpsWaypointFollower::initialize(const YAML::Node& config)
{
  practice_course_waypoints_.clear();
  main_course_waypoints_.clear();

  loadWaypoints(config["practice_course"], practice_course_waypoints_);
  loadWaypoints(config["main_course"], main_course_waypoints_);
}

/**
 * Load waypoints from the `waypoint_group` file into the `destination_vector`
 */
void GpsWaypointFollower::loadWaypoints(const YAML::Node& waypoint_group, std::vector<geographic_msgs::msg::GeoPoint>& destination_vector)
{
  if (!waypoint_group || !waypoint_group.IsSequence())
  {
    RCLCPP_WARN(
      this->get_logger(),
      "Waypoint group is missing or is not a list."
    );
    return;
  }

  for (const auto& waypoint : waypoint_group)
  {
    const auto name = waypoint["name"] ? waypoint["name"].as<std::string>() : "Unnamed";

    if (!waypoint["lat"] || !waypoint["lon"])
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Waypoint %s is missing lat or lon, skipping...",
        name.c_str()
      );
      continue;
    }

    geographic_msgs::msg::GeoPoint navWaypoint;
    navWaypoint.latitude = waypoint["lat"].as<double>();
    navWaypoint.longitude = waypoint["lon"].as<double>();

    destination_vector.push_back(navWaypoint);
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Waypoint group loaded."
  );
}

/**
 * Use the `/fromLL` service from `fusioncore_ros` to transform a waypoint from GPS to the local coordinate system
 */
void GpsWaypointFollower::transformNextWaypoint()
{
  if (!from_ll_client_->service_is_ready())
  {
    RCLCPP_WARN(
      this->get_logger(),
      "/fromLL service is not available yet."
    );
    return;
  }

  if (waypoint_transform_index_ >= selected_waypoints_.size())
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Successfully transformed %zu GPS waypoints.",
      localized_waypoints_.size()
    );

    // Start navigation after transforming all points
    // startNavigation();

    return;
  }

  const auto waypoint = selected_waypoints_[waypoint_transform_index_];

  auto request = std::make_shared<fusioncore_ros::srv::FromLL::Request>();
  request->ll_point = waypoint;

  from_ll_client_->async_send_request(
    request,
    [this, waypoint](
      rclcpp::Client<fusioncore_ros::srv::FromLL>::SharedFuture future_response)
    {
      const auto response = future_response.get();
      const auto& point = response->map_point;

      if ((point.x == 0.0) && (point.y == 0.0) && (point.z == 0.0))
      {
        RCLCPP_WARN(
          this->get_logger(),
          "FusionCore returned (0, 0, 0). GPS reference may not be set. Waypoint lat=%f, lon=%f was not added.",
          waypoint.latitude,
          waypoint.longitude
        );

        enableFollower_ = false;
        return;
      }

      localized_waypoints_.push_back(point);

      RCLCPP_INFO(
        this->get_logger(),
        "Converted GPS waypoint lat=%f, lon=%f -> x=%.3f, y=%.3f",
        waypoint.latitude,
        waypoint.longitude,
        point.x,
        point.y
      );

      ++waypoint_transform_index_;
      transformNextWaypoint();
    }
  );
}


/**
 * `request->data: false` - Autonomous control disabled
 * `request->data: true` - Autonomous control enabled
 */
void GpsWaypointFollower::setAutonCallback(std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                           std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  enableFollower_ = request->data;
  response->success = true;

  if (!enableFollower_)
  {
    response->message = "Waypoint follower disabled";
    return;
  }

  selected_waypoints_ = practiceCourse_ ? practice_course_waypoints_ : main_course_waypoints_;
  waypoint_transform_index_ = 0;
  localized_waypoints_.clear();

  response->message = "Waypoint follower enabled";

  transformNextWaypoint();
}

/**
 * `request->data: false` - Main Course in use
 * `request->data: true` - Practice Course in use
 */
void GpsWaypointFollower::setCourseCallback(std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                           std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  practiceCourse_ = request->data;
  response->success = true;

  if (practiceCourse_)
  {
    response->message = "Practice Course Waypoint Navigation";

    RCLCPP_INFO(
      this->get_logger(),
      "Practice course navigation enabled"
    );
  }
  else
  {
    response->message = "Main Course Waypoint Navigation";

    RCLCPP_INFO(
      this->get_logger(),
      "Main course navigation enabled"
    );
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<GpsWaypointFollower>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}