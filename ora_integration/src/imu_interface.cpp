#include "../include/imu_interface.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using State = rclcpp_lifecycle::State;

ImuInterface::ImuInterface(const rclcpp::NodeOptions& options) : LifecycleNode("imu_interface", options)
{
  // Add a parameter for non-standard can interface
  this->declare_parameter("can_interface", can_intf_name_);
}

CallbackReturn ImuInterface::on_configure(const State &previous_state)
{
  // Read interface parameter
  can_intf_name_ = this->get_parameter("can_interface").as_string();
  RCLCPP_INFO(
    this->get_logger(),
    "Configuring using the %s interface...",
    can_intf_name_.c_str()
  );

  // Create publisher for IMU
  imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
    "/esp/imu",
    10
  );

  RCLCPP_INFO(
    this->get_logger(),
    "Created IMU publisher on %s",
    "/esp/imu"
  );

  // Initialize can interface
  if (!can_intf_.init(can_intf_name_,
                      &event_loop_,
                      std::bind(&ImuInterface::on_can_msg, this, _1)))
  {
    RCLCPP_WARN(
      this->get_logger(),
      "Failed to initialize SocketCAN on %s",
      can_intf_name_.c_str()
    );
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Initialized SocketCAN on %s",
    can_intf_name_.c_str()
  );

  RCLCPP_INFO(
    this->get_logger(),
    "Successfully configured"
  );

  return CallbackReturn::SUCCESS;
}

CallbackReturn ImuInterface::on_activate(const State& previous_state)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Activating..."
  );

  RCLCPP_INFO(
    this->get_logger(),
    "Spinning IO event loop thread..."
  );

  event_loop_worker_ = std::thread(&EpollEventLoop::run, &this->event_loop_);

  // Nothing needed to activate the controller or node at the moment
  // Could possibly send a message to move controller out of low power mode here?

  RCLCPP_INFO(
    this->get_logger(),
    "Successfully activated"
  );

  return CallbackReturn::SUCCESS;
}

CallbackReturn ImuInterface::on_deactivate(const State& previous_state)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Deactivating..."
  );

  event_loop_.stop_loop();

  if (event_loop_worker_.joinable())
  {
    event_loop_worker_.join();
  }

  // Nothing needed to deactivate the controller or node at the moment
  // Could possibly send a message to move controller into low power mode here?

  RCLCPP_INFO(
    this->get_logger(),
    "Successfully deactivated"
  );

  return CallbackReturn::SUCCESS;
}

CallbackReturn ImuInterface::on_cleanup(const State& previous_state)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Cleaning up..."
  );

  RCLCPP_INFO(
    this->get_logger(),
    "De-initializing %s CAN interface",
    can_intf_name_.c_str()
  );

  can_intf_.deinit();

  RCLCPP_INFO(
    this->get_logger(),
    "Stopping publisher"
  );

  imu_publisher_.reset();

  RCLCPP_INFO(
    this->get_logger(),
    "Successful cleanup"
  );

  return CallbackReturn::SUCCESS;
}

CallbackReturn ImuInterface::on_shutdown(const State& previous_state)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn ImuInterface::on_error(const State& previous_state)
{
  RCLCPP_WARN(
    this->get_logger(),
    "An error occurred during operation, resetting interface..."
  );

  event_loop_.stop_loop();

  if (event_loop_worker_.joinable())
  {
    event_loop_worker_.join();
  }

  try
  {
    can_intf_.deinit();
  }
  catch(const std::exception& e)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "%s",
      e.what()
    );

    return CallbackReturn::FAILURE;
  }

  try
  {
    imu_publisher_.reset();
  }
  catch(const std::exception& e)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "%s",
      e.what()
    );

    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Successful recovery, returning to 'Unconfigured' State"
  );

  return CallbackReturn::SUCCESS;
}

void ImuInterface::on_can_msg(const can_frame& frame)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Got a can msg :D"
  );
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ImuInterface>(rclcpp::NodeOptions{});
  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}

