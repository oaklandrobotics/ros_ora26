#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from lifecycle_msgs.msg import (
  State
)

from controller_manager_msgs.srv import (
  ListHardwareComponents,
  ListControllers,
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

    # Flags to only do recovery after components/controllers have been activated initially
    self.initial_component_active = False
    self.initial_controller_active = False
    self.enable_recovery = False

    self.hw_future = None
    self.set_hw_future = None
    self.controller_future = None
    self.set_controller_future = None

    self.current_component_state = dict()
    self.current_controller_state = dict()

    for component in self.hardware_components:
      self.current_component_state[component] = self.hardware_components_default_state

    for controller in self.controllers:
      self.current_controller_state[controller] = self.controllers_default_state

    # Service Client to get component/controller state
    self.list_hw_comps = self.create_client(ListHardwareComponents, '/controller_manager/list_hardware_components')
    self.list_controllers = self.create_client(ListControllers, '/controller_manager/list_controllers')

    # Service Clients to update components/controllers
    self.set_hw_comp_state_cli = self.create_client(SetHardwareComponentState, '/controller_manager/set_hardware_component_state')
    self.switch_controller_cli = self.create_client(SwitchController, '/controller_manager/switch_controller')

    # Callback to active components/controllers
    self.recovery_timer = self.create_timer(self.check_period_sec, self.recovery_callback)

  def recovery_callback(self) -> None:
    # Get updated component state
    req = ListHardwareComponents.Request()

    if self.hw_future == None:
      self.hw_future = self.list_hw_comps.call_async(req)

    if self.hw_future.done():
      for component in self.hw_future.result().component:
        if (component.name in self.hardware_components):
          self.current_component_state[component.name] = component.state.label

      self.hw_future = self.list_hw_comps.call_async(req)

    # Get updated controller states
    req = ListControllers.Request()

    if self.controller_future == None:
      self.controller_future = self.list_controllers.call_async(req)

    if self.controller_future.done():
      for controller in self.controller_future.result().controller:
        if (controller.name in self.controllers):
          self.current_controller_state[controller.name] = controller.state

          self.controller_future = self.list_controllers.call_async(req)

    # Only enable recovery if the components have reached the initial active state from ros2_control
    if not self.enable_recovery:
      if not self.initial_component_active:
        temp_active = True

        for name, state in self.current_component_state.items():
          temp_active = (temp_active and (state == 'active'))

        self.initial_component_active = temp_active

      if not self.initial_controller_active:
        temp_active = True

        for name, state in self.current_controller_state.items():
          temp_active = (temp_active and state == 'active')

        self.initial_controller_active = temp_active

      self.enable_recovery = (self.initial_component_active and self.initial_controller_active)

      if self.enable_recovery:
        self.get_logger().info(f'ros2_control initialized for the first time. Enabling recovery node.')

    if self.enable_recovery:
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

          if self.set_hw_future == None or self.set_hw_future.done():
            self.set_hw_future = self.set_hw_comp_state_cli.call_async(req)

      inactive_controllers = []

      # Check for controllers that aren't activated
      for controller_name, state in self.current_controller_state.items():
        if state != 'active':
          inactive_controllers.append(controller_name)

      # Activate inactive controllers if hardware components are active
      if inactive_controllers:
        self.get_logger().info(f'Attempting to activate inactive controllers.')

        if not self.switch_controller_cli.wait_for_service(timeout_sec=2.5):
          self.get_logger().info('Service not available at this moment.')
          return

        req = SwitchController.Request()
        req.activate_controllers = inactive_controllers
        req.strictness = SwitchController.Request.STRICT

        if self.set_controller_future == None or self.set_controller_future.done():
          self.set_controller_future = self.switch_controller_cli.call_async(req)

def main():
  rclpy.init()
  node = Ros2ControlRecovery()

  rclpy.spin(node)

  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  raise SystemExit(main())