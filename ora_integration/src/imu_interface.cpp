#include "../include/imu_interface.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using State = rclcpp_lifecycle::State;

ImuInterface::ImuInterface(const rclcpp::NodeOptions& options) : LifecycleNode("imu_interface", options)
{
  // Add a parameter for non-standard can interface
  this->declare_parameter("can_interface", can_intf_name_);
  this->declare_parameter("imu_topic", imu_topic_);
  this->declare_parameter("imu_frame", imu_frame_);

  initialize_imu_cov();
}

CallbackReturn ImuInterface::on_configure(const State &previous_state)
{
  // Read parameters
  can_intf_name_ = this->get_parameter("can_interface").as_string();
  imu_topic_ = this->get_parameter("imu_topic").as_string();
  imu_frame_ = this->get_parameter("imu_frame").as_string();

  RCLCPP_INFO(
    this->get_logger(),
    "Configuring using the %s interface...",
    can_intf_name_.c_str()
  );

  RCLCPP_INFO(
    this->get_logger(),
    "Publishing to the %s topic using the %s frame",
    imu_topic_.c_str(), imu_frame_.c_str()
  );

  // Create publisher for IMU
  imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
    imu_topic_,
    10
  );

  imu_msg_.header.frame_id = imu_frame_;

  RCLCPP_INFO(
    this->get_logger(),
    "Created IMU publisher on %s",
    imu_topic_.c_str()
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

void ImuInterface::initialize_imu_cov()
{
  // Initialize to 0
  for (int i = 0; i < k_cov_1d_array_size; i++)
  {
    imu_msg_.linear_acceleration_covariance[i] = 0.0;
    imu_msg_.angular_velocity_covariance[i] = 0.0;
    imu_msg_.orientation_covariance[i] = 0.0;
  }

  // Setting the variance indices to the proper weight
  imu_msg_.linear_acceleration_covariance[k_x_variance_index] = k_accel_cov_diag.x;
  imu_msg_.linear_acceleration_covariance[k_y_variance_index] = k_accel_cov_diag.y;
  imu_msg_.linear_acceleration_covariance[k_z_variance_index] = k_accel_cov_diag.z;

  imu_msg_.angular_velocity_covariance[k_x_variance_index] = k_gyro_cov_diag.x;
  imu_msg_.angular_velocity_covariance[k_y_variance_index] = k_gyro_cov_diag.y;
  imu_msg_.angular_velocity_covariance[k_z_variance_index] = k_gyro_cov_diag.z;

  imu_msg_.orientation_covariance[k_x_variance_index] = k_orient_cov_diag.x;
  imu_msg_.orientation_covariance[k_y_variance_index] = k_orient_cov_diag.y;
  imu_msg_.orientation_covariance[k_z_variance_index] = k_orient_cov_diag.z;
}

void ImuInterface::on_can_msg(const can_frame& frame)
{
  // Check for invalid frames
  if (frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG))
  {
    return;
  }

  // Isolate information from the CAN frame
  const uint16_t can_id = static_cast<uint16_t>(frame.can_id) & CAN_SFF_MASK;
  const uint8_t dlc = static_cast<uint8_t>(frame.can_dlc);

  // Check that received message is valid for IMU
  if (
    (can_id < esp::can::IMU_ACCELERATION_DATA_XY_CAN_ID)
    || (can_id > esp::can::IMU_QUATERNION_DATA_YZ_CAN_ID)
    || (dlc != 8)
  )
  {
    return;
  }

  // Reconstruct data frames
  const uint32_t data_1 = (
    (static_cast<uint32_t>(frame.data[0]) << 24) |
    (static_cast<uint32_t>(frame.data[1]) << 16) |
    (static_cast<uint32_t>(frame.data[2]) << 8) |
    (static_cast<uint32_t>(frame.data[3]) << 0)
  );
  const uint32_t data_2 = (
    (static_cast<uint32_t>(frame.data[4]) << 24) |
    (static_cast<uint32_t>(frame.data[5]) << 16) |
    (static_cast<uint32_t>(frame.data[6]) << 8) |
    (static_cast<uint32_t>(frame.data[7]) << 0)
  );

  float float_1;
  float float_2;

  std::memcpy(&float_1, &data_1, sizeof(float_1));
  std::memcpy(&float_2, &data_2, sizeof(float_2));

  switch (can_id)
  {
    case esp::can::IMU_ACCELERATION_DATA_XY_CAN_ID:
      imu_msg_.linear_acceleration.x = float_1;
      imu_msg_.linear_acceleration.y = float_2;

      imu_data_flags_ |= 0b000001;
      break;

    case esp::can::IMU_ACCELERATION_DATA_Z_CAN_ID:
      imu_msg_.linear_acceleration.z = float_1;

      imu_data_flags_ |= 0b000010;
      break;

    case esp::can::IMU_GYROSCOPIC_DATA_XY_CAN_ID:
      imu_msg_.angular_velocity.x = float_1;
      imu_msg_.angular_velocity.y = float_2;

      imu_data_flags_ |= 0b000100;
      break;

    case esp::can::IMU_GYROSCOPIC_DATA_Z_CAN_ID:
      imu_msg_.angular_velocity.z = float_1;

      imu_data_flags_ |= 0b001000;
      break;

    case esp::can::IMU_QUATERNION_DATA_WX_CAN_ID:
      imu_msg_.orientation.w = float_1;
      imu_msg_.orientation.x = float_2;

      imu_data_flags_ |= 0b010000;
      break;

    case esp::can::IMU_QUATERNION_DATA_YZ_CAN_ID:
      imu_msg_.orientation.y = float_1;
      imu_msg_.orientation.z = float_2;

      imu_data_flags_ |= 0b100000;
      break;
  }

  // If the IMU data flags are all set, send off the IMU message
  if (imu_data_flags_ == 0b111111)
  {
    send_imu_message();
    imu_data_flags_ = 0;
  }
}

void ImuInterface::send_imu_message()
{
  // Update timestamp on message header
  imu_msg_.header.stamp = this->get_clock()->now();

  // Publish message
  imu_publisher_->publish(imu_msg_);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ImuInterface>(rclcpp::NodeOptions{});
  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}

