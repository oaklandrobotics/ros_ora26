#include "../include/gps_navigation.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using namespace std::chrono_literals;

GpsNavigation::GpsNavigation(
  rclcpp::Logger logger,
  rclcpp::Clock::SharedPtr clock,
  rclcpp::Client<fusioncore_ros::srv::FromLL>::SharedPtr from_ll_client,
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client,
  std::string waypoint_file_path
) : Navigation(logger, clock), 
    from_ll_client_(from_ll_client), nav_to_pose_client_(nav_to_pose_client)
{
  const YAML::Node config_file = YAML::LoadFile(waypoint_file_path);
  initialize(config_file);
}

/**
 * Reset waypoints and navigation index and load waypoints from configuration file
 */
void GpsNavigation::initialize(const YAML::Node& config)
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
void GpsNavigation::loadWaypoints(
  const YAML::Node& waypoint_group,
  std::vector<geographic_msgs::msg::GeoPoint>& destination_vector
)
{
  if (!waypoint_group || !waypoint_group.IsSequence())
  {
    RCLCPP_WARN(
      logger_,
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
        logger_,
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
    logger_,
    "Waypoint group loaded."
  );
}

/**
 * Use the `/fromLL` service from `fusioncore_ros` to
 * transform a waypoint from GPS to the local coordinate system
 */
void GpsNavigation::transformNextWaypoint()
{
  if (!from_ll_client_->service_is_ready())
  {
    RCLCPP_WARN(
      logger_,
      "/fromLL service is not available yet."
    );
    return;
  }

  if (waypoint_transform_index_ >= selected_waypoints_.size())
  {
    RCLCPP_INFO(
      logger_,
      "Successfully transformed %zu GPS waypoints.",
      localized_waypoints_.size()
    );

    // Add current location at the end of the localized_waypoints_ for navigation back to start
    addStartingWaypoint();

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

/**
 * 
 */
void GpsNavigation::updatePose(const geometry_msgs::msg::PoseWithCovarianceStamped pose)
{

  last_known_pose_ = pose;
}

/**
 * 
 */
void GpsNavigation::setCourse(const bool is_practice_course)
{

  practice_course_ = is_practice_course;

  waypoints_configured_ = false;
  current_waypoint_index_ = 0;
  waypoint_transform_index_ = 0;
  localized_waypoints_.clear();
}

const GpsNavigation::NavigationState GpsNavigation::getNavigationState()
{
  NavigationState navigation_state;

  navigation_state.localized_waypoints = localized_waypoints_;
  navigation_state.active_index = current_waypoint_index_;
  navigation_state.starting_direction = practice_course_ ? "Practice Course" : "North Course";

  return navigation_state;
}

/**
 * 
 */
void GpsNavigation::addStartingWaypoint()
{
  // Last known pose is good for starting pose
  auto starting_pose = this->last_known_pose_.pose.pose;

  RCLCPP_INFO(
    logger_,
    "Adding the point x=%.3f, y=%.3f as the starting waypoint",
    starting_pose.position.x, starting_pose.position.y
  );

  localized_waypoints_.push_back(starting_pose.position);
}

/**
 * 
 */
void GpsNavigation::startNavigation()
{
  RCLCPP_INFO(
    logger_,
    "Starting GPS based navigation."
  );

  enable_follower_ = true;

  // Configure waypoints if there is no configuration
  if (!waypoints_configured_)
  {
    // Get vector of waypoints
    selected_waypoints_ = practice_course_ ? practice_course_waypoints_ : main_course_waypoints_;

    // Reset waypoint index and vector
    waypoint_transform_index_ = 0;
    localized_waypoints_.clear();
    waypoints_configured_ = true;

    transformNextWaypoint();

    return;
  }

  if (localized_waypoints_.empty())
  {
    RCLCPP_WARN(
      logger_,
      "No localized waypoints available"
    );

    return;
  }

  // Waypoint navigation complete
  if (current_waypoint_index_ >= localized_waypoints_.size())
  {
    RCLCPP_INFO(
      logger_,
      "Successfully navigated to all waypoints. Resetting navigation to first waypoint."
    );

    resetNavigation();
    enable_follower_ = false;
    return;
  }

  RCLCPP_INFO(
    logger_,
    "Starting navigation to waypoint %zu",
    current_waypoint_index_
  );

  navigateToWaypoint(localized_waypoints_[current_waypoint_index_]);
}

/**
 * Cancel the current Nav2 goal
 * Does not restart navigation from the beginning unless `resetNavigation()` is called in tandem
 */
void GpsNavigation::stopNavigation()
{
  RCLCPP_INFO(
    logger_,
    "Stopping GPS based navigation."
  );

  enable_follower_ = false;

  if (!current_goal_handle_)
  {
    RCLCPP_INFO(
      logger_,
      "Waypoint follower stopped. No active goal to cancel."
    );
    return;
  }

  RCLCPP_INFO(
    logger_,
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
void GpsNavigation::resetNavigation()
{
  RCLCPP_INFO(
    logger_,
    "Resetting GPS based navigation."
  );

  enable_follower_ = false;

  waypoints_configured_ = false;
  waypoint_transform_index_ = 0;

  current_waypoint_index_ = 0;
}

NavigateToPose::Goal GpsNavigation::buildNavigateToPoseGoal(
  const geometry_msgs::msg::Point& localized_waypoint
)
{
  // Create a goal message
  auto goal_msg = NavigateToPose::Goal();

  // Set goal position to localized waypoint
  goal_msg.pose.header.frame_id = "odom";
  goal_msg.pose.header.stamp = clock_->now();
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

void GpsNavigation::navigateToWaypoint(const geometry_msgs::msg::Point& localized_waypoint)
{
  if (!this->nav_to_pose_client_->wait_for_action_server())
  {
    RCLCPP_ERROR(
      logger_,
      "Action server not available"
    );
    return;
  }

  auto goal_msg = buildNavigateToPoseGoal(localized_waypoint);

  RCLCPP_INFO(
    logger_,
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
 * Called when a message is received from the `/from_ll` service
 */
void GpsNavigation::fromLLCallback(
  geographic_msgs::msg::GeoPoint waypoint,
  rclcpp::Client<fusioncore_ros::srv::FromLL>::SharedFuture future_response
)
{
  const auto response = future_response.get();
  const auto& point = response->map_point;

  if ((point.x == 0.0) && (point.y == 0.0) && (point.z == 0.0))
  {
    RCLCPP_WARN(
      logger_,
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
    logger_,
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
void GpsNavigation::navGoalResponseCallback(
  const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr& goal_handle
)
{
  if (!goal_handle)
  {
    RCLCPP_ERROR(
      logger_,
      "Goal was rejected by server"
    );

    enable_follower_ = false;
    return;
  }

  current_goal_handle_ = goal_handle;

  RCLCPP_INFO(
    logger_,
    "Goal was accepted by server, waiting for result"
  );
}

void GpsNavigation::navGoalFeedbackCallback(
  rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback
)
{
  RCLCPP_INFO_THROTTLE(
    logger_,
    *clock_,
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
void GpsNavigation::navGoalResultCallback(
  const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult& result
)
{
  current_goal_handle_.reset();

  if (!enable_follower_)
  {
    RCLCPP_INFO(
      logger_,
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
        logger_,
        "Reached waypoint %zu",
        current_waypoint_index_ + 1
      );

      ++current_waypoint_index_;

      // Successfully navigated to all waypoints
      if (current_waypoint_index_ >= localized_waypoints_.size())
      {
        RCLCPP_INFO(
          logger_,
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
        logger_,
        "Waypoint navigation goal canceled."
      );

      enable_follower_ = false;
      return;
    }

    // Nav2 aborted 
    case rclcpp_action::ResultCode::ABORTED:
    {
      RCLCPP_ERROR(
        logger_,
        "Navigation goal was aborted by Nav2"
      );

      if (!enable_follower_)
      {
        RCLCPP_INFO(
          logger_,
          "Follower not enabled, ending navigation"
        );

        return;
      }

      RCLCPP_INFO(
        logger_,
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
          logger_,
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
          logger_,
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
void GpsNavigation::navCancelGoalCallback(
  const std::shared_ptr<action_msgs::srv::CancelGoal_Response>& cancel_response
)
{
  if (cancel_response->return_code == action_msgs::srv::CancelGoal::Response::ERROR_NONE)
  {
    RCLCPP_INFO(
      logger_,
      "Nav2 goal cancelled successfully"
    );
  }
  else
  {
    RCLCPP_WARN(
      logger_,
      "Nav2 goal cancelled with an error. Return code: %d",
      cancel_response->return_code
    );
  }

  current_goal_handle_.reset();
}