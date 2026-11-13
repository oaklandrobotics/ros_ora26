#ifndef IMU_INTERFACE_HPP_
#define IMU_INTERFACE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cstdint>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "std_srvs/srv/set_bool.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "can/epoll_event_loop.hpp"
#include "can/can_helpers.hpp"
#include "can/socket_can.hpp"

namespace esp::can
{
  // RX CAN IDs
  inline constexpr std::uint16_t IMU_ACCELERATION_DATA_XY_CAN_ID = 0x010;
  inline constexpr std::uint16_t IMU_ACCELERATION_DATA_Z_CAN_ID = 0x011;
  inline constexpr std::uint16_t IMU_GYROSCOPIC_DATA_XY_CAN_ID = 0x012;
  inline constexpr std::uint16_t IMU_GYROSCOPIC_DATA_Z_CAN_ID = 0x013;
  inline constexpr std::uint16_t IMU_QUATERNION_DATA_WX_CAN_ID = 0x014;
  inline constexpr std::uint16_t IMU_QUATERNION_DATA_YZ_CAN_ID = 0x015;
} // esp::imu

struct Vector_t
{
  float x;
  float y;
  float z;
};

class ImuInterface : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using State = rclcpp_lifecycle::State;

  ImuInterface(const rclcpp::NodeOptions& options);

  // Lifecycle Node
  CallbackReturn on_configure(const State& previous_state) override;
  CallbackReturn on_activate(const State& previous_state) override;
  CallbackReturn on_deactivate(const State& previous_state) override;
  CallbackReturn on_cleanup(const State& previous_state) override;
  CallbackReturn on_shutdown(const State& previous_state) override;
  CallbackReturn on_error(const State& previous_state) override;

private:
  static inline constexpr std::uint8_t k_cov_1d_array_size = 9;
  static inline constexpr std::uint8_t k_x_variance_index = 0;
  static inline constexpr std::uint8_t k_y_variance_index = 4;
  static inline constexpr std::uint8_t k_z_variance_index = 8;

  static inline constexpr Vector_t k_accel_cov_diag = {0.02f, 0.02f, 0.02f};
  static inline constexpr Vector_t k_gyro_cov_diag = {0.0001f, 0.000144f, 0.000081f};
  static inline constexpr Vector_t k_orient_cov_diag = {0.01f, 0.01f, 0.01f};

  void initialize_imu_cov();

  void on_can_msg(const can_frame& frame);
  void send_imu_message();

  SocketCanIntf can_intf_;
  EpollEventLoop event_loop_;
  std::string can_intf_name_ = "can0";

  std::thread event_loop_worker_;

  std::uint8_t imu_data_flags_ = 0x00;

  std::string imu_topic_ = "/esp/imu";
  std::string imu_frame_ = "imu_frame";

  // Service Callbacks
  void autonStateCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request
  );

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  sensor_msgs::msg::Imu imu_msg_;

  // Service Client
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr set_auton_client_;
};

#endif // IMU_INTERFACE_HPP_