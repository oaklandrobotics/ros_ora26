#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from lifecycle_msgs.msg import (
  State
)

from controller_manager_msgs.msg import (
  ControllerManagerActivity
)

from controller_manager_msgs.srv import (
    SetHardwareComponentState,
    SwitchController
)

class Ros2ControlRecovery(Node):
  def __init__(self):
    super().__init__('ros2_control_recovery_node')

    self.declare_parameter('hardware_components', ['RoboteqControllers'])
    self.declare_parameter('hardware_components_default_state', 'unconfigured')
    self.declare_parameter('controllers', ['diff_cont', 'joint_broad'])
    self.declare_parameter('controllers_default_state', 'inactive')

    self.declare_parameter('check_period_sec', 1.0)

    # Get parameters
    self.hardware_components = list(
      self.get_parameter('hardware_components').value
    )

    self.hardware_components_default_state = self.get_parameter('hardware_components_default_state').value

    self.controllers = list(
      self.get_parameter('controllers').value
    )

    self.controllers_default_state = self.get_parameter('controllers_default_state').value

    self.check_period_sec = float(
      self.get_parameter('check_period_sec').value
    )

    self.components_active = False
    self.controllers_active = False
    self.hw_future = None
    self.controller_future = None

    self.current_component_state = dict()
    self.current_controller_state = dict()

    for component in self.hardware_components:
      self.current_component_state[component] = self.hardware_components_default_state

    for controller in self.controllers:
      self.current_controller_state[controller] = self.controllers_default_state

    # Service Clients to update components/controllers
    self.set_hw_comp_state_cli = self.create_client(SetHardwareComponentState, '/controller_manager/set_hardware_component_state')
    self.switch_controller_cli = self.create_client(SwitchController, '/controller_manager/switch_controller')

    # Callback to active components/controllers
    self.recovery_timer = self.create_timer(self.check_period_sec, self.recovery_callback)

    # Subscription to update local state of controllers
    self.subscription = self.create_subscription(
      ControllerManagerActivity,
      '/controller_manager/activity',
      self.update_local_state,
      10
    )

  def update_local_state(self, msg) -> None:
    for controller in msg.controllers:
      self.current_controller_state[controller.name] = controller.state.label

    # Check to see if controllers should attempt to be started
    self.controllers_active = all(
      self.current_controller_state.get(name) == 'active'
      for name in self.controllers
    )

    for hw_comp in msg.hardware_components:
      self.current_component_state[hw_comp.name] = hw_comp.state.label

    # Check to make sure all components are on before starting to enable controllers
    self.components_active = all(
      self.current_component_state.get(name) == 'active'
      for name in self.hardware_components
    )

  def recovery_callback(self) -> None:
    # Activate hardware components
    for component_name, state in self.current_component_state.items():
      if state != 'active':
        self.get_logger().info(f'{component_name} is currently in the \'{state}\' state. Attempting to activate.')

        if not self.set_hw_comp_state_cli.service_is_ready():
          self.get_logger().info('Service not available at this moment.')
          return

        req = SetHardwareComponentState.Request()
        req.name = component_name
        req.target_state = State(id=State.PRIMARY_STATE_ACTIVE, label='active')

        if self.hw_future == None or self.hw_future.done():
          self.hw_future = self.set_hw_comp_state_cli.call_async(req)

    inactive_controllers = []

    # Check for controllers that aren't activated
    if not self.controllers_active:
      for controller_name, state in self.current_controller_state.items():
        if state != 'active':
          inactive_controllers.append(controller_name)

    # Activate inactive controllers if hardware components are active
    if self.components_active and inactive_controllers:
      self.get_logger().info(f'Attempting to activate inactive controllers.')

      if not self.switch_controller_cli.wait_for_service(timeout_sec=2.5):
        self.get_logger().info('Service not available at this moment.')
        return

      req = SwitchController.Request()
      req.activate_controllers = inactive_controllers
      req.strictness = SwitchController.Request.STRICT

      if self.controller_future == None or self.controller_future.done():
        self.controller_future = self.switch_controller_cli.call_async(req)

def main():
  rclpy.init()
  node = Ros2ControlRecovery()

  rclpy.spin(node)

  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  raise SystemExit(main())