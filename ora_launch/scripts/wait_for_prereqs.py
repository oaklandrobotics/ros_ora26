#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

import tf2_ros

class WaitForPrereqs(Node):
  def __init__(self):
    super().__init__('wait_for_prereqs')

    # Default parameters
    #Format is similar to other topics ".../ns/.../topic"
    self.declare_parameter('required_topics', [''])

    # Format is "target_frame:source_frame" where it checks for the tf from source frame into target frame
    # i.e. odom:base_link to check if odom -> base_link exists
    self.declare_parameter('required_transforms', [''])

    self.declare_parameter('check_period_sec', 0.5)

    # Get parameters
    self.required_topics = list(
      self.get_parameter('required_topics').value
    )

    self.required_transforms = list(
      self.get_parameter('required_transforms').value
    )

    self.check_period_sec = float(
      self.get_parameter('check_period_sec').value
    )

    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

  def topic_exists(self, topic_name: str) -> bool:
    topics = dict(self.get_topic_names_and_types())
    return topic_name in topics

  def transform_exists(self, transform: str) -> bool:
    try:
      # Empty means no transform
      if (transform == ''):
        return True

      target_frame, source_frame = transform.split(':')
    except ValueError:
      self.get_logger().error(
        f'Invalid transform format "{transform}". '
        f'Expected "target_frame:source_frame".'
      )
      return False

    return self.tf_buffer.can_transform(
      target_frame,
      source_frame,
      Time(),
      timeout=Duration(seconds=0.1)
    )

  def wait(self) -> int:
    self.get_logger().info('Waiting for required topics and TF transforms...')

    while rclpy.ok():
      missing_topics = [
        topic for topic in self.required_topics
        if not self.topic_exists(topic)
      ]

      missing_transforms = [
        transform for transform in self.required_transforms
        if not self.transform_exists(transform)
      ]

      if not missing_topics and not missing_transforms:
        self.get_logger().info(
          'All required topics and TF transforms are available.'
        )
        return 0

      if missing_topics:
        self.get_logger().info(f'Missing topics: {missing_topics}')

      if missing_transforms:
        self.get_logger().info(
          f'Missing TF transforms: {missing_transforms}'
        )

      rclpy.spin_once(self, timeout_sec=self.check_period_sec)
      time.sleep(self.check_period_sec)

    return 1


def main():
  rclpy.init()
  node = WaitForPrereqs()

  try:
    exit_code = node.wait()
  finally:
    node.destroy_node()
    rclpy.shutdown()

  return exit_code


if __name__ == '__main__':
  raise SystemExit(main())