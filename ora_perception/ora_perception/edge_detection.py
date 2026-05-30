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
EDGE_POINTS_TOPIC = "/depth_camera/filtered/points"
MASKED_IMG_TOPIC = "/masked_image"

# Parameters
BLUR_KERNEL_SIZE = 5
# Values closer to 255 are colors closer to white (white is 255)
LINE_LOWER_BOUND = 200
LINE_UPPER_BOUND = 255

MIN_DEPTH = 1
MAX_DEPTH = 8

class EdgeDetectionNode(Node):
    def __init__(self):
        super().__init__("edge_detection_node")

        self.bridge = CvBridge()
        self.camera_model = None

        # Set the topic name based on use_sim_time
        self.use_sim_time = self.get_parameter('use_sim_time').value
        if self.use_sim_time == True: #Simulated camera
            raw_image_topic = "/depth_camera/image_raw"
            depth_image_topic = "/depth_camera/depth_image_raw"
            camera_into_topic = "/depth_camera/camera_info"
        else: # Real Camera - ZED2i (left channel)
            raw_image_topic = "/zed/zed_node/rgb/color/rect/image"
            depth_image_topic = "/zed/zed_node/depth/depth_registered"
            camera_into_topic = "/zed/zed_node/depth/camera_info"

        # Topic subscriptions
        self.image_sub = message_filters.Subscriber(self, Image, raw_image_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_image_topic)
        self.info_sub = self.create_subscription(CameraInfo, camera_into_topic, self.info_callback, 10)

        # Topic publishers
        self.edge_publisher = self.create_publisher(PointCloud2, EDGE_POINTS_TOPIC, 10)
        self.masked_img_pub = self.create_publisher(Image, MASKED_IMG_TOPIC, 10)

        # Messages from image_sub and depth_sub must be synced for accuracy.
        #   ts stands for time synchronization
        self.ts = message_filters.ApproximateTimeSynchronizer((self.image_sub, self.depth_sub), 10, slop=0.1)
        self.ts.registerCallback(self.detect_edges)

        self.get_logger().info('Hello from edge detection')

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

    def detect_edges(self, image_raw: Image, image_depth: Image):
        if self.camera_model is None:
            self.get_logger().warn("Waiting for CameraInfo...")
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
        cv2.rectangle(mask, (0, 200), (w, h), 255, -1)

        # Masking visible part of robot in frame
        # [Bottom left, Top left, Top right, Bottom right]
        if self.use_sim_time == True: #Simulated camera
            robot_mask_points = np.array([[328, 720], [470, 400], [812, 400], [955, 720]])
        else: # Real camera
            robot_mask_points = np.array([[295, 720], [412, 551], [957, 551], [1110, 720]])
        robot_mask_points = robot_mask_points.reshape((4, 1, 2))
        cv2.fillPoly(mask, [robot_mask_points], 0)

        # Pubished the masked *color* image
        color_mask_cv = cv2.bitwise_and(cv_raw, cv_raw, mask=mask)
        color_mask_ros = Image()
        color_mask_ros = self.bridge.cv2_to_imgmsg(color_mask_cv, 'bgr8')
        color_mask_ros.header = image_raw.header
        self.masked_img_pub.publish(color_mask_ros)

        # Thresholding to find "White" edges
        _, threshold = cv2.threshold(blur, LINE_LOWER_BOUND, LINE_UPPER_BOUND, cv2.THRESH_BINARY)
        final_mask = cv2.bitwise_and(threshold, mask)

        # Find every pixel (u,v) within the threshold and get it's depth(z)
        v_indices, u_indices = np.where(final_mask > 0)
        depths = cv_depth[v_indices, u_indices]

        # Filter out invalid depth (0 or NaN)
        # Increases performance by only rendering points that are part of a 'line'
        # Adjust (depths > NUM) to change the minimum depth (stop robot from appearing as a line)
        valid_mask = (depths > MIN_DEPTH) & (depths < MAX_DEPTH) & np.isfinite(depths)
        z = depths[valid_mask]
        u = u_indices[valid_mask]
        v = v_indices[valid_mask]

        if len(z) == 0: # No points to display
            return

        # Pinhole Camera Math: X = (u - cx) * Z / fx
        # Used to convert the 2D pixel coordinates back to 3D coordinates
        fx, fy = self.camera_model['fx'], self.camera_model['fy']
        cx, cy = self.camera_model['cx'], self.camera_model['cy']

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        # Create a stack of points for pointcloud2 to display
        points = np.column_stack((x, y, z)).astype(np.float32)

        # Create and Publish PointCloud2
        pc_msg = pc2.create_cloud_xyz32(image_depth.header, points)
        if self.use_sim_time == True:
            pc_msg.header.frame_id = "camera_link_optical"
        else:
            pc_msg.header.frame_id = "zed_left_camera_link"
        self.edge_publisher.publish(pc_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EdgeDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()