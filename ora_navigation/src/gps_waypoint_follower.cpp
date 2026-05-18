#include "../include/gps_waypoint_follower.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;

GpsWaypointFollower::GpsWaypointFollower() : Node("gps_waypoint_follower")
{
  const auto package_share_dir = ament_index_cpp::get_package_share_directory("ora_navigation");

  this->declare_parameter("waypoints_file", package_share_dir + "/config/waypoints.yaml");

  const auto waypoints_file = this->get_parameter("waypoints_file").as_string();

  // Service
  set_auton_srv_ = this->create_service<std_srvs::srv::SetBool>("navigation/set_auton",
                        std::bind(&GpsWaypointFollower::setAutonCallback, this, std::placeholders::_1, std::placeholders::_2));
  reset_auton_srv_ = this->create_service<std_srvs::srv::Trigger>("navigation/reset_auton",
                        std::bind(&GpsWaypointFollower::resetAutonCallback, this, std::placeholders::_1, std::placeholders::_2));
  set_course_srv_ = this->create_service<std_srvs::srv::SetBool>("navigation/set_course",
                        std::bind(&GpsWaypointFollower::setCourseCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Service Client
  from_ll_client_ = this->create_client<fusioncore_ros::srv::FromLL>("/fromLL");

  // Action Client
  nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

  const YAML::Node config_file = YAML::LoadFile(waypoints_file);
  initialize(config_file);
}

void GpsWaypointFollower::initialize(const YAML::Node& config)
{
  practice_course_waypoints_.clear();
  main_course_waypoints_.clear();

  current_waypoint_index_ = 0;

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

    startNavigation();

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

void GpsWaypointFollower::startNavigation()
{
  if (localized_waypoints_.empty())
  {
    RCLCPP_WARN(
      this->get_logger(),
      "No localized waypoints available"
    );

    return;
  }

  // Waypoint navigation complete
  if (current_waypoint_index_ >= localized_waypoints_.size())
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Successfully navigated to all waypoints. Resetting navigation to first waypoint."
    );

    resetNavigation();
    enableFollower_ = false;
    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Starting navigation to waypoint %zu",
    current_waypoint_index_
  );

  navigateToWaypoint(localized_waypoints_[current_waypoint_index_]);
}

/**
 * Cancel the current Nav2 goal
 * Does not restart navigation from the beginning unless `resetNavigation()` is called in tandem
 */
void GpsWaypointFollower::stopNavigation()
{
  enableFollower_ = false;

  if (!current_goal_handle_)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Waypoint follower stopped. No active goal to cancel."
    );
    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Canceling active Nav2 goal."
  );

  nav_to_pose_client_->async_cancel_goal(
    current_goal_handle_,
    [this](const auto& future)
    {
      const auto cancel_response = future.get();

      if (cancel_response->return_code == action_msgs::srv::CancelGoal::Response::ERROR_NONE)
      {
        RCLCPP_INFO(
          this->get_logger(),
          "Nav2 goal cancelled successfully"
        );
      }
      else
      {
        RCLCPP_WARN(
          this->get_logger(),
          "Nav2 goal cancelled with an error. Return code: %d",
          cancel_response->return_code
        );
      }

      current_goal_handle_.reset();
    }
  );
}

/**
 * Allow waypoints to be reconfigured
 * Reset the current goal to first waypoint
 */
void GpsWaypointFollower::resetNavigation()
{
  RCLCPP_INFO(
    this->get_logger(),
    "Waypoint navigation reset"
  );

  waypointsConfigured_ = false;
  waypoint_transform_index_ = 0;

  current_waypoint_index_ = 0;
}

void GpsWaypointFollower::navigateToWaypoint(const geometry_msgs::msg::Point& localizedWaypoint)
{
  if (!this->nav_to_pose_client_->wait_for_action_server())
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Action server not available"
    );
    return;
  }

  auto goal_msg = NavigateToPose::Goal();

  // Set goal position to localized waypoint
  goal_msg.pose.header.frame_id = "odom";
  goal_msg.pose.header.stamp = this->get_clock()->now();
  goal_msg.pose.pose.position.x = localizedWaypoint.x;
  goal_msg.pose.pose.position.y = localizedWaypoint.y;
  goal_msg.pose.pose.position.z = localizedWaypoint.z;

  // Set goal orientation to an arbitrary value, our parameters shouldn't require orientation
  goal_msg.pose.pose.orientation.x = 0;
  goal_msg.pose.pose.orientation.y = 0;
  goal_msg.pose.pose.orientation.z = 0;
  goal_msg.pose.pose.orientation.w = 1;

  RCLCPP_INFO(
    this->get_logger(),
    "Sending goal to navigate to {x: %.2f, y: %.2f}",
    goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y
  );

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback = [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr& goal_handle)
  {
    if (!goal_handle)
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Goal was rejected by server"
      );

      enableFollower_ = false;
      return;
    }

    current_goal_handle_ = goal_handle;

    RCLCPP_INFO(
      this->get_logger(),
      "Goal was accepted by server, waiting for result"
    );
  };

  // Feedback Callback
  // We don't particularly need the feedback, but we will log the current pose and distance remaining
  send_goal_options.feedback_callback = [this](rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr,
                                               const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "current_pose: x:%.2f, y:%.2f, distance_remaining:%.2f",
      feedback->current_pose.pose.position.x, feedback->current_pose.pose.position.y, feedback->distance_remaining
    );
  };

  // Result Callback
  // Nav2 only provides error codes for results. If there is no error break and continue, otherwise log the error
  send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult& result)
  {
    current_goal_handle_.reset();

    if (!enableFollower_)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Waypoint follower is disabled, ignoring Nav2 result"
      );
      return;
    }

    switch (result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
      {
        RCLCPP_INFO(
          this->get_logger(),
          "Reached waypoint %zu",
          current_waypoint_index_ + 1
        );

        ++current_waypoint_index_;

        // Successfully navigated to all waypoints
        if (current_waypoint_index_ >= localized_waypoints_.size())
        {
          RCLCPP_INFO(
            this->get_logger(),
            "Navigation complete"
          );

          enableFollower_ = false;
          return;
        }

        // Navigate to next point
        navigateToWaypoint(localized_waypoints_[current_waypoint_index_]);
        return;
      }
      case rclcpp_action::ResultCode::CANCELED:
      {
        RCLCPP_INFO(
          this->get_logger(),
          "Waypoint navigation goal canceled"
        );
        return;
      }
      case rclcpp_action::ResultCode::ABORTED:
      {
        RCLCPP_ERROR(
          this->get_logger(),
          "Navigation goal was aborted by Nav2"
        );

        enableFollower_ = false;
        return;
      }
      default:
      {
        RCLCPP_ERROR(
            this->get_logger(),
            "Finished with unknown result code"
          );

          enableFollower_ = false;
          return;
      }
    }
  };

  // Send the goal
  this->nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
  return;
}

/**
 * `request->data: false` - Autonomous control disabled
 * `request->data: true` - Autonomous control enabled
 */
void GpsWaypointFollower::setAutonCallback(std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                           std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  enableFollower_ = request->data;

  if (!enableFollower_)
  {
    stopNavigation();

    response->success = true;
    response->message = "Waypoint follower disabled";
    return;
  }

  if (!waypointsConfigured_)
  {
    // Get vector of waypoints
    selected_waypoints_ = practiceCourse_ ? practice_course_waypoints_ : main_course_waypoints_;

    // Reset waypoint index and vector
    waypoint_transform_index_ = 0;
    localized_waypoints_.clear();
    waypointsConfigured_ = true;

    response->success = true;
    response->message = "Waypoint follower enabled, transforming waypoints";

    transformNextWaypoint();
    return;
  }

  response->success = true;
  response->message = "Waypoint follower enabled, resuming navigation";

  startNavigation();
}

/**
 * Sending a `Trigger` request sets navigation back to initial values
 */
void GpsWaypointFollower::resetAutonCallback(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                           std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void) request;

  stopNavigation();
  resetNavigation();

  response->success = true;
  response->message = "Reset navigation values.";
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

  waypointsConfigured_ = false;
  current_waypoint_index_ = 0;
  waypoint_transform_index_ = 0;
  localized_waypoints_.clear();

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