#!/usr/bin/env python3

import rclpy
import message_filters
from rclpy.node import Node 
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs_py import point_cloud2 as pc2

# Topic names
EDGE_POINTS_TOPIC = '/filtered/lines'
FREE_SPACE_TOPIC = '/filtered/free_space'
MASKED_IMG_TOPIC = '/masked_image'

# Parameters
BLUR_KERNEL_SIZE = 5
LINE_LOWER_BOUND = 200
LINE_UPPER_BOUND = 255
HORIZON_LINE = 0.278

MIN_DEPTH = 0
MAX_DEPTH = 10

class EdgeDetectionNode(Node):
  def __init__(self):
    super().__init__('edge_detection_node')

    self.bridge = CvBridge()
    self.camera_model = None

    # Set the topic name based on use_sim_time
    self.use_sim_time = self.get_parameter('use_sim_time').value

    if self.use_sim_time == True: # Simulated camera
        raw_image_topic = '/depth_camera/image_raw'
        depth_image_topic = '/depth_camera/depth_image_raw'
        camera_info_topic = '/depth_camera/camera_info'
    else: # Real Camera - ZED2i (left channel)
        raw_image_topic = '/zed/zed_node/rgb/color/rect/image'
        depth_image_topic = '/zed/zed_node/depth/depth_registered'
        camera_info_topic = '/zed/zed_node/depth/camera_info'

    # Topic subscriptions
    self.image_sub = message_filters.Subscriber(self, Image, raw_image_topic)
    self.depth_sub = message_filters.Subscriber(self, Image, depth_image_topic)
    self.info_sub = self.create_subscription(CameraInfo, camera_info_topic, self.info_callback, 10)

    # Topic publishers
    self.masked_img_pub = self.create_publisher(Image, MASKED_IMG_TOPIC, 10)

    # Line for marking, free for clearing
    # Free will be a bitwise not of the Line image
    self.line_publisher = self.create_publisher(PointCloud2, EDGE_POINTS_TOPIC, 10)
    self.free_space_publisher = self.create_publisher(PointCloud2, FREE_SPACE_TOPIC, 10)

    # Messages from image_sub and depth_sub must be synced for accuracy.
    self.time_sync = message_filters.ApproximateTimeSynchronizer((self.image_sub, self.depth_sub), 10, slop=0.1)
    self.time_sync.registerCallback(self.detect_edges)

    self.get_logger().info('Edge detection node started.')

  def info_callback(self, msg):
    # Extract camera information from the CameraInfo message
    # K matrix: [fx 0 cx; 0 fy cy; 0 0 1]
    # fy,fx are focal lengths
    # cx, cy are the optical center (middle of image)
    self.camera_model = {
        'fx': msg.k[0],
        'fy': msg.k[4],
        'cx': msg.k[2],
        'cy': msg.k[5]
    }

  def image_to_depth(self, thresholded_image: Image, image_depth: Image, stride: int = 1):
    # Find every pixel (u,v) within the threshold and get it's depth(z)
    v_indices, u_indices = np.where(thresholded_image > 0)

    # Downsample the image if stride > 1
    # This should take 1 pixel every stride, reducing the amount of work
    if stride > 1:
      v_indices = v_indices[::stride]
      u_indices = u_indices[::stride]

    depths = image_depth[v_indices, u_indices]

    # Filter out invalid depth (0 or NaN)
    # Increases performance by only rendering points that are part of a 'line'
    # Adjust (depths > NUM) to change the minimum depth (stop robot from appearing as a line)
    valid_mask = (depths > MIN_DEPTH) & (depths < MAX_DEPTH) & np.isfinite(depths)
    z = depths[valid_mask]
    u = u_indices[valid_mask]
    v = v_indices[valid_mask]

    if len(z) == 0: # No points to display
      return np.empty((0, 3), dtype=np.float32)

    # Pinhole Camera Math: X = (u - cx) * Z / fx
    # Used to convert the 2D pixel coordinates back to 3D coordinates
    fx, fy = self.camera_model['fx'], self.camera_model['fy']
    cx, cy = self.camera_model['cx'], self.camera_model['cy']

    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    # Create a stack of points for pointcloud2 to display
    points = np.column_stack((x, y, z)).astype(np.float32)

    return points

  def detect_edges(self, image_raw: Image, image_depth: Image):
    if self.camera_model is None:
      self.get_logger().warn('Waiting for CameraInfo...')
      return

    # Convert ROS 2 images to OpenCV
    cv_raw = self.bridge.imgmsg_to_cv2(image_raw, 'bgr8')
    cv_depth = self.bridge.imgmsg_to_cv2(image_depth, '32FC1')

    # Image Processing (Grayscale + Blur)
    grey_scale = cv2.cvtColor(cv_raw, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(grey_scale, (BLUR_KERNEL_SIZE, BLUR_KERNEL_SIZE), 0)

    # Masking (Top of frame to near horizon line)
    mask = np.zeros(cv_raw.shape[:2], dtype=np.uint8)
    h, w = cv_raw.shape[:2]
    cv2.rectangle(mask, (0, int(HORIZON_LINE * h)), (w, h), 255, -1)

    # Masking visible part of robot in frame
    # [Bottom left, Top left, Top right, Bottom right]
    if self.use_sim_time == True: #Simulated camera
      robot_mask_points = np.array([[0.256, 1], [0.367, 0.556], [0.634, 0.556], [0.746, 1]])
    else: # Real camera
      robot_mask_points = np.array([[0.230, 1], [0.322, 0.675], [0.748, 0.675], [0.867, 1]])

    pixel_points = robot_mask_points.copy()
    pixel_points[:, 0] *= (w - 1)
    pixel_points[:, 1] *= (h - 1)
    pixel_points = pixel_points.astype(np.int32)

    pixel_points = pixel_points.reshape((4, 1, 2))
    cv2.fillPoly(mask, [pixel_points], 0)

    # Pubished the masked *color* image
    color_mask_cv = cv2.bitwise_and(cv_raw, cv_raw, mask=mask)
    color_mask_ros = Image()
    color_mask_ros = self.bridge.cv2_to_imgmsg(color_mask_cv, 'bgr8')
    color_mask_ros.header = image_raw.header
    self.masked_img_pub.publish(color_mask_ros)

    # Thresholding to find 'White' edges
    _, lines_threshold = cv2.threshold(blur, LINE_LOWER_BOUND, LINE_UPPER_BOUND, cv2.THRESH_BINARY)

    # Create a mask to convert into depth
    lines_mask = cv2.bitwise_and(lines_threshold, mask)

    # Morphological operations to clean mask
    # Opening followed by closing should reduce noise and re-fill lines
    clean_lines = cv2.morphologyEx(
      lines_mask,
      cv2.MORPH_OPEN,
      np.ones((3, 3), np.uint8),
      iterations=1
    )
    clean_lines = cv2.morphologyEx(
      lines_mask,
      cv2.MORPH_CLOSE,
      np.ones((5, 5), np.uint8),
      iterations=1
    )

    # Dilate the clean lines to create a layer between the lines and free space
    protected_lines = cv2.dilate(clean_lines, np.ones((5, 5), np.uint8), iterations=1)
    protected_lines = cv2.bitwise_and(protected_lines, mask)

    free_mask = cv2.bitwise_and(cv2.bitwise_not(protected_lines), mask)

    lines_points = self.image_to_depth(clean_lines, cv_depth, stride=2)
    free_points = self.image_to_depth(free_mask, cv_depth, stride=10)

    # Create and Publish PointCloud2
    lines_pc_msg = pc2.create_cloud_xyz32(image_depth.header, lines_points)
    free_pc_msg = pc2.create_cloud_xyz32(image_depth.header, free_points)

    if self.use_sim_time == True:
      lines_pc_msg.header.frame_id = 'camera_link_optical'
      free_pc_msg.header.frame_id = 'camera_link_optical'
    else:
      lines_pc_msg.header.stamp = self.get_clock().now().to_msg()
      lines_pc_msg.header.frame_id = 'zed_left_camera_frame_optical'
      free_pc_msg.header.stamp = self.get_clock().now().to_msg()
      free_pc_msg.header.frame_id = 'zed_left_camera_frame_optical'

    self.line_publisher.publish(lines_pc_msg)
    self.free_space_publisher.publish(free_pc_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EdgeDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()