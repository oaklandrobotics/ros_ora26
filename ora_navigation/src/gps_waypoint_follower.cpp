#include "../include/gps_waypoint_follower.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;

GpsWaypointFollower::GpsWaypointFollower() : Node("gps_waypoint_follower")
{
  const auto package_share_dir = ament_index_cpp::get_package_share_directory("ora_navigation");
  const std::string default_waypoint_file = package_share_dir + "/config/waypoints.yaml";

  this->declare_parameter("waypoints_file", default_waypoint_file);

  const auto waypoints_file = this->get_parameter("waypoints_file").as_string();

  // Create services and assign callbacks
  set_auton_srv_ = this->create_service<std_srvs::srv::SetBool>(
    "navigation/set_auton",
    [this](
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response
    )
    {
      setAutonCallback(request, response);
    }
  );

  reset_auton_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "navigation/reset_auton",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response
    )
    {
      resetAutonCallback(request, response);
    }
  );

  set_course_srv_ = this->create_service<std_srvs::srv::SetBool>(
    "navigation/set_course",
    [this](
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response
    )
    {
      setCourseCallback(request, response);
    }
  );

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
void GpsWaypointFollower::loadWaypoints(
  const YAML::Node& waypoint_group,
  std::vector<geographic_msgs::msg::GeoPoint>& destination_vector
)
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

    geographic_msgs::msg::GeoPoint nav_waypoint;
    nav_waypoint.latitude = waypoint["lat"].as<double>();
    nav_waypoint.longitude = waypoint["lon"].as<double>();

    destination_vector.push_back(nav_waypoint);
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Waypoint group loaded."
  );
}

/**
 * Use the `/fromLL` service from `fusioncore_ros` to
 * transform a waypoint from GPS to the local coordinate system
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
      rclcpp::Client<fusioncore_ros::srv::FromLL>::SharedFuture future_response
    )
    {
      fromLLCallback(waypoint, future_response);
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
    enable_follower_ = false;
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
  enable_follower_ = false;

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
    [this](
      const rclcpp_action::Client<NavigateToPose>::CancelResponse::SharedPtr cancel_response
    )
    {
      navCancelGoalCallback(cancel_response);
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

  waypoints_configured_ = false;
  waypoint_transform_index_ = 0;

  current_waypoint_index_ = 0;
}

NavigateToPose::Goal GpsWaypointFollower::buildNavigateToPoseGoal(
  const geometry_msgs::msg::Point& localized_waypoint
)
{
  // Create a goal message
  auto goal_msg = NavigateToPose::Goal();

  // Set goal position to localized waypoint
  goal_msg.pose.header.frame_id = "odom";
  goal_msg.pose.header.stamp = this->get_clock()->now();
  goal_msg.pose.pose.position.x = localized_waypoint.x;
  goal_msg.pose.pose.position.y = localized_waypoint.y;
  goal_msg.pose.pose.position.z = localized_waypoint.z;

  // Set goal orientation to an arbitrary value, our parameters shouldn't require orientation
  goal_msg.pose.pose.orientation.x = 0;
  goal_msg.pose.pose.orientation.y = 0;
  goal_msg.pose.pose.orientation.z = 0;
  goal_msg.pose.pose.orientation.w = 1;

  return goal_msg;
}

void GpsWaypointFollower::navigateToWaypoint(const geometry_msgs::msg::Point& localized_waypoint)
{
  if (!this->nav_to_pose_client_->wait_for_action_server())
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Action server not available"
    );
    return;
  }

  auto goal_msg = buildNavigateToPoseGoal(localized_waypoint);

  RCLCPP_INFO(
    this->get_logger(),
    "Sending goal to navigate to {x: %.2f, y: %.2f}",
    goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y
  );

  // Goal Options
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  send_goal_options.goal_response_callback =
  [this](
    const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal_handle
  )
  {
    navGoalResponseCallback(goal_handle);
  };

  send_goal_options.feedback_callback =
  [this](
    rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal_handle,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback
  )
  {
    navGoalFeedbackCallback(goal_handle, feedback);
  };

  send_goal_options.result_callback = 
  [this](
    const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &result
  )
  {
    navGoalResultCallback(result);
  };

  // Send the goal
  this->nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
  return;
}

/**
 * `request->data: false` - Autonomous control disabled
 * `request->data: true` - Autonomous control enabled
 */
void GpsWaypointFollower::setAutonCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response
)
{
  enable_follower_ = request->data;

  if (!enable_follower_)
  {
    stopNavigation();

    response->success = true;
    response->message = "Waypoint follower disabled";
    return;
  }

  if (!waypoints_configured_)
  {
    // Get vector of waypoints
    selected_waypoints_ = practice_course_ ? practice_course_waypoints_ : main_course_waypoints_;

    // Reset waypoint index and vector
    waypoint_transform_index_ = 0;
    localized_waypoints_.clear();
    waypoints_configured_ = true;

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
void GpsWaypointFollower::resetAutonCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response
)
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
void GpsWaypointFollower::setCourseCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response
)
{
  practice_course_ = request->data;
  response->success = true;

  waypoints_configured_ = false;
  current_waypoint_index_ = 0;
  waypoint_transform_index_ = 0;
  localized_waypoints_.clear();

  if (practice_course_)
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

/**
 * Called when a message is received from the `/from_ll` service
 */
void GpsWaypointFollower::fromLLCallback(
  geographic_msgs::msg::GeoPoint waypoint,
  rclcpp::Client<fusioncore_ros::srv::FromLL>::SharedFuture future_response
)
{
  const auto response = future_response.get();
  const auto& point = response->map_point;

  if ((point.x == 0.0) && (point.y == 0.0) && (point.z == 0.0))
  {
    RCLCPP_WARN(
      this->get_logger(),
      "FusionCore returned (0, 0, 0). GPS reference may not be set. " \
      "Waypoint lat=%f, lon=%f was not added.",
      waypoint.latitude,
      waypoint.longitude
    );

    enable_follower_ = false;
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

/**
 * Called whenever a response is received by the `/navigate_to_pose` client
 */
void GpsWaypointFollower::navGoalResponseCallback(
  const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr& goal_handle
)
{
  if (!goal_handle)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Goal was rejected by server"
    );

    enable_follower_ = false;
    return;
  }

  current_goal_handle_ = goal_handle;

  RCLCPP_INFO(
    this->get_logger(),
    "Goal was accepted by server, waiting for result"
  );
}

void GpsWaypointFollower::navGoalFeedbackCallback(
  rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback
)
{
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    1000,
    "current_pose: x:%.2f, y:%.2f, distance_remaining:%.2f",
    feedback->current_pose.pose.position.x,
    feedback->current_pose.pose.position.y,
    feedback->distance_remaining
  );
}

/**
 * Called whenever a goal is received by the `/navigate_to_pose` client
 */
void GpsWaypointFollower::navGoalResultCallback(
  const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult& result
)
{
  current_goal_handle_.reset();

  if (!enable_follower_)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Waypoint follower is disabled, ignoring Nav2 result"
    );
    return;
  }

  switch (result.code)
  {
    // Goal successfully reached
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

        enable_follower_ = false;
        return;
      }

      // Navigate to next point
      navigateToWaypoint(localized_waypoints_[current_waypoint_index_]);
      return;
    }

    // Waypoint Cancelled
    case rclcpp_action::ResultCode::CANCELED:
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Waypoint navigation goal canceled."
      );

      enable_follower_ = false;
      return;
    }

    // Nav2 aborted 
    case rclcpp_action::ResultCode::ABORTED:
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Navigation goal was aborted by Nav2"
      );

      if (!enable_follower_)
      {
        RCLCPP_INFO(
          this->get_logger(),
          "Follower not enabled, ending navigation"
        );

        return;
      }

      RCLCPP_INFO(
        this->get_logger(),
        "Follower still enabled, restarting navigation"
      );

      if (retry_events_ < 10)
      {
        // Restart navigation to current waypoint
        retry_events_++;
        navigateToWaypoint(localized_waypoints_[current_waypoint_index_]);
      }
      else
      {
        RCLCPP_ERROR(
          this->get_logger(),
          "Too many retries, aborting navigation"
        );

        retry_events_ = 0;
        enable_follower_ = false;
      }

      return;
    }

    default:
    {
      RCLCPP_ERROR(
          this->get_logger(),
          "Finished with unknown result code"
        );

        enable_follower_ = false;
        return;
    }
  } // switch (result.code)
}

/**
 * Called whenever a cancel response is received by the `/navigate_to_pose` client
 */
void GpsWaypointFollower::navCancelGoalCallback(
  const std::shared_ptr<action_msgs::srv::CancelGoal_Response>& cancel_response
)
{
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

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<GpsWaypointFollower>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}