#ifndef IMU_INTERFACE_HPP_
#define IMU_INTERFACE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

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

  // CAN message Structures
  typedef struct
  {
    float accelerationX;
    float accelerationY;
  } IMUAccelerationDataXYMsg_t;

  typedef struct
  {
    float accelerationZ;
    uint8_t padding[4]; 
  } IMUAccelerationDataZMsg_t;

  typedef struct
  {
    float gyroX;
    float gyroY;
  } IMUGyroscopicDataXYMsg_t;

  typedef struct
  {
    float gyroZ;
    uint8_t padding[4]; 
  } IMUGyroscopicDataZMsg_t;

  typedef struct
  {
    float quaternionW;
    float quaternionX;
  } IMUQuaternionDataWXMsg_t;

  typedef struct
  {
    float quaternionY;
    float quaternionZ;
  } IMUQuaternionDataYZMsg_t;
} // esp::imu

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
  void on_can_msg(const can_frame& frame);

  SocketCanIntf can_intf_;
  EpollEventLoop event_loop_;
  std::string can_intf_name_ = "can0";
  rclcpp::Time timestamp_;

  std::thread event_loop_worker_;

  // Commands

  // Feedback
  // IMU
  float accelerationX = 0.0f;
  float accelerationY = 0.0f;
  float accelerationZ = 0.0f;
  float gyroX = 0.0f;
  float gyroY = 0.0f;
  float gyroZ = 0.0f;
  float quaternionW = 0.0f;
  float quaternionX = 0.0f;
  float quaternionY = 0.0f;
  float quaternionZ = 0.0f;

  // Service Callbacks
  void autonStateCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request
  );

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;

  // Service Client
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr set_auton_client_;
};

#endif // IMU_INTERFACE_HPP_