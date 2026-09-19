#include "../include/navigation_manager.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using namespace std::chrono_literals;

NavigationManager::NavigationManager() : Node("navigation_manager")
{
  // Set up defaults for waypoint configuration file
  const auto package_share_dir = ament_index_cpp::get_package_share_directory("ora_navigation");
  const std::string default_waypoint_file = package_share_dir + "/config/waypoints.yaml";

  // Add a paremeter for non-standard waypoint path
  this->declare_parameter("waypoints_file", default_waypoint_file);

  const auto waypoints_file = this->get_parameter("waypoints_file").as_string();

  // Create publisher
  twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/cmd_vel_nav_out",
    10
  );

  // Create publisher timer for callback function
  update_timer_ = this->create_wall_timer(
    100ms,
    [this]()
    {
      updateTimerCallback();
    }
  );

  // Subscriber and callback
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/fusion/pose", 20,
    [this](
      const geometry_msgs::msg::PoseWithCovarianceStamped msg
    )
    {
      poseCallback(msg);
    }
  );

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

  get_navigation_info_srv_ = this->create_service<ora_interfaces::srv::NavigationInfo>(
    "navigation/get_info",
    [this](
      const std::shared_ptr<ora_interfaces::srv::NavigationInfo::Request> request,
      std::shared_ptr<ora_interfaces::srv::NavigationInfo::Response> response
    )
    {
      getNavInfoCallback(request, response);
    }
  );

  reload_waypoint_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "navigation/reload_waypoint",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response
    )
    {
      reloadWaypointCallback(request, response);
    }
  );

  // Service Client
  from_ll_client_ = this->create_client<fusioncore_ros::srv::FromLL>("/fromLL");

  // Action Client
  nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

  // Create Navigation classes
  gps_navigation_ = std::make_unique<GpsNavigation>(
    this->get_logger(),
    this->get_clock(),
    from_ll_client_,
    nav_to_pose_client_,
    waypoints_file
  );

  velocity_navigation_ = std::make_unique<VelocityNavigation>(
    this->get_logger(),
    this->get_clock(),
    twist_publisher_
  );

  setNavigation(NavigationMode::GPS);
}

/**
 * Start navigation based on current `navigation_mode_`
 */
void NavigationManager::startNavigation()
{
  enable_navigation_ = true;

  active_navigation_->startNavigation();
}

/**
 * Stop navigation based on current `navigation_mode_`
 */
void NavigationManager::stopNavigation()
{
  enable_navigation_ = false;

  active_navigation_->stopNavigation();
}

/**
 * Reset navigation based on current `navigation_mode_`
 */
void NavigationManager::resetNavigation()
{
  enable_navigation_ = false;

  active_navigation_->resetNavigation();
}

void NavigationManager::setNavigation(NavigationMode navigation_mode)
{
  stopNavigation();

  switch (navigation_mode)
  {
    case NavigationMode::GPS:
      active_navigation_ = gps_navigation_.get();
      navigation_mode_ = NavigationMode::GPS;
      break;

    case NavigationMode::Velocity:
      active_navigation_ = velocity_navigation_.get();
      navigation_mode_ = NavigationMode::Velocity;
      break;

    default:
      break;
  }
}

void NavigationManager::updateTimerCallback()
{
  switch (navigation_mode_)
  {
    case NavigationMode::Velocity:
      velocity_navigation_->update();
      break;
    default:
      break;
  }
}

void NavigationManager::poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped msg)
{
  gps_navigation_->updatePose(msg);
}

/**
 * `request->data: false` - Autonomous control disabled
 * `request->data: true` - Autonomous control enabled
 */
void NavigationManager::setAutonCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response
)
{
  enable_navigation_ = request->data;

  if (!enable_navigation_)
  {
    stopNavigation();

    response->success = true;
    response->message = "Navigation Disabled.";
    return;
  }

  response->success = true;
  response->message = "Navigation Enabled.";

  startNavigation();
}

/**
 * Sending a `Trigger` request sets navigation back to initial values
 */
void NavigationManager::resetAutonCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response
)
{
  (void) request;

  stopNavigation();
  resetNavigation();

  response->success = true;
  response->message = "Navigation Reset.";
}

/**
 * `request->data: false` - Main Course in use
 * `request->data: true` - Practice Course in use
 */
void NavigationManager::setCourseCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response
)
{
  auto practice_course = request->data;

  gps_navigation_->setCourse(practice_course);

  response->success = true;
  
  if (practice_course)
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
 * 
 */
void NavigationManager::getNavInfoCallback(
  const std::shared_ptr<ora_interfaces::srv::NavigationInfo::Request> request,
  std::shared_ptr<ora_interfaces::srv::NavigationInfo::Response> response
)
{
  // Ignore request as it is blank
  (void) request;

  // Retrieve the current navigation state from the waypoint follower
  auto navigation_state = gps_navigation_->getNavigationState();

  // Populate response with navigation state information
  response->localized_waypoints = navigation_state.localized_waypoints;
  response->active_index = static_cast<uint8_t>(navigation_state.active_index);
  response->starting_direction = navigation_state.starting_direction;
  response->success = true;
  response->message = "Navigation goals returned";
}

void NavigationManager::reloadWaypointCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response
)
{
  (void) request;
  gps_navigation_->initialize();

  response->success = true;
  response->message = "Navigation Reload Waypoints.";
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<NavigationManager>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}

