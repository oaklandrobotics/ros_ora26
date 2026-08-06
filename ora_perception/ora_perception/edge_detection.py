#!/usr/bin/env python3

import traceback

import rclpy
import message_filters
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs_py import point_cloud2 as pc2

cv2.setNumThreads(1)

# Topic names
EDGE_POINTS_TOPIC = "/depth_camera/filtered/points"
CLEAR_POINTS_TOPIC = "/depth_camera/clear/points"
MASKED_IMG_TOPIC = "/masked_image"

# Parameters
BLUR_KERNEL_SIZE = 5
LINE_LOWER_BOUND = 200
LINE_UPPER_BOUND = 255
HORIZON_LINE = 0.278

MIN_DEPTH = 1.0
MAX_DEPTH = 8.0

# Keep PointCloud2 messages from becoming huge
MAX_LINE_POINTS = 3000
MAX_CLEAR_POINTS = 3000

# Set false during autonomous runs if you do not need image debugging
PUBLISH_MASKED_IMAGE = False


class EdgeDetectionNode(Node):
    def __init__(self):
        super().__init__("edge_detection_node")

        self.bridge = CvBridge()
        self.camera_model = None

        self.received_camera_info = False
        self.detect_callback_count = 0
        self.no_valid_points_count = 0
        self.line_cloud_publish_count = 0
        self.clear_cloud_publish_count = 0
        self.masked_publish_count = 0
        self.cleanup_started = False

        self.use_sim_time = self.get_parameter("use_sim_time").value

        if self.use_sim_time:
            raw_image_topic = "/depth_camera/image_raw"
            depth_image_topic = "/depth_camera/depth_image_raw"
            camera_info_topic = "/depth_camera/camera_info"
        else:
            raw_image_topic = "/zed/zed_node/rgb/color/rect/image"
            depth_image_topic = "/zed/zed_node/depth/depth_registered"
            camera_info_topic = "/zed/zed_node/depth/camera_info"

        self.get_logger().info("Starting edge detection")
        self.get_logger().info(f"use_sim_time: {self.use_sim_time}")
        self.get_logger().info(f"Raw image topic: {raw_image_topic}")
        self.get_logger().info(f"Depth image topic: {depth_image_topic}")
        self.get_logger().info(f"CameraInfo topic: {camera_info_topic}")
        self.get_logger().info(f"Line cloud output topic: {EDGE_POINTS_TOPIC}")
        self.get_logger().info(f"Clear cloud output topic: {CLEAR_POINTS_TOPIC}")
        self.get_logger().info(f"Masked image output topic: {MASKED_IMG_TOPIC}")

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.image_sub = message_filters.Subscriber(
            self,
            Image,
            raw_image_topic,
            qos_profile=sensor_qos,
        )

        self.depth_sub = message_filters.Subscriber(
            self,
            Image,
            depth_image_topic,
            qos_profile=sensor_qos,
        )

        self.info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.info_callback,
            10,
        )

        self.edge_publisher = self.create_publisher(PointCloud2, EDGE_POINTS_TOPIC, 10)
        self.clear_publisher = self.create_publisher(PointCloud2, CLEAR_POINTS_TOPIC, 10)
        self.masked_img_pub = self.create_publisher(Image, MASKED_IMG_TOPIC, 10)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            (self.image_sub, self.depth_sub),
            queue_size=10,
            slop=0.2,
        )
        self.ts.registerCallback(self.detect_edges)

        self.get_logger().info("Edge detection initialized")

    def info_callback(self, msg):
        fx = msg.k[0]
        fy = msg.k[4]
        cx = msg.k[2]
        cy = msg.k[5]

        if fx == 0.0 or fy == 0.0:
            self.get_logger().error(
                f"Invalid CameraInfo received: frame={msg.header.frame_id}, "
                f"fx={fx}, fy={fy}, cx={cx}, cy={cy}"
            )
            return

        if not self.received_camera_info:
            self.get_logger().info(
                f"Received CameraInfo: frame={msg.header.frame_id}, "
                f"fx={fx}, fy={fy}, cx={cx}, cy={cy}"
            )
            self.received_camera_info = True

        self.camera_model = {
            "fx": fx,
            "fy": fy,
            "cx": cx,
            "cy": cy,
        }

    def pixels_to_points(self, u, v, z):
        fx = self.camera_model["fx"]
        fy = self.camera_model["fy"]
        cx = self.camera_model["cx"]
        cy = self.camera_model["cy"]

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        return np.column_stack((x, y, z)).astype(np.float32)

    def downsample_points(self, points, max_points):
        if len(points) > max_points:
            step = max(1, len(points) // max_points)
            points = points[::step]

        return points

    def publish_cloud(self, publisher, image_depth, points):
        cloud_msg = pc2.create_cloud_xyz32(image_depth.header, points)
        cloud_msg.header.frame_id = image_depth.header.frame_id

        # Keeps Nav2 from rejecting old ZED image timestamps.
        cloud_msg.header.stamp = self.get_clock().now().to_msg()

        publisher.publish(cloud_msg)

    def detect_edges(self, image_raw: Image, image_depth: Image):
        try:
            self.detect_callback_count += 1

            if self.camera_model is None:
                if self.detect_callback_count == 1:
                    self.get_logger().warn("Waiting for CameraInfo...")
                return

            cv_raw = self.bridge.imgmsg_to_cv2(image_raw, "bgr8")
            cv_depth = self.bridge.imgmsg_to_cv2(image_depth, "32FC1")

            if cv_raw.shape[:2] != cv_depth.shape[:2]:
                self.get_logger().error(
                    f"Image/depth size mismatch: "
                    f"image_shape={cv_raw.shape[:2]}, depth_shape={cv_depth.shape[:2]}"
                )
                return

            if self.detect_callback_count == 1:
                self.get_logger().info(
                    f"First synced image/depth callback: "
                    f"image_frame={image_raw.header.frame_id}, "
                    f"depth_frame={image_depth.header.frame_id}, "
                    f"depth_encoding={image_depth.encoding}"
                )

            grey_scale = cv2.cvtColor(cv_raw, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(grey_scale, (BLUR_KERNEL_SIZE, BLUR_KERNEL_SIZE), 0)

            mask = np.zeros(cv_raw.shape[:2], dtype=np.uint8)
            h, w = cv_raw.shape[:2]

            # Keep lower part of image below horizon
            cv2.rectangle(mask, (0, int(HORIZON_LINE * h)), (w, h), 255, -1)

            # Mask visible robot body
            if self.use_sim_time:
                robot_mask_points = np.array([
                    [0.256, 1],
                    [0.367, 0.556],
                    [0.634, 0.556],
                    [0.746, 1],
                ])
            else:
                robot_mask_points = np.array([
                    [0.230, 1],
                    [0.322, 0.675],
                    [0.748, 0.675],
                    [0.867, 1],
                ])

            pixel_points = robot_mask_points.copy()
            pixel_points[:, 0] *= (w - 1)
            pixel_points[:, 1] *= (h - 1)
            pixel_points = pixel_points.astype(np.int32)
            pixel_points = pixel_points.reshape((4, 1, 2))
            cv2.fillPoly(mask, [pixel_points], 0)

            if PUBLISH_MASKED_IMAGE:
                color_mask_cv = cv2.bitwise_and(cv_raw, cv_raw, mask=mask)
                color_mask_ros = self.bridge.cv2_to_imgmsg(color_mask_cv, "bgr8")
                color_mask_ros.header = image_raw.header
                self.masked_img_pub.publish(color_mask_ros)
                self.masked_publish_count += 1

            # Thresholding to find white line pixels
            _, threshold = cv2.threshold(
                blur,
                LINE_LOWER_BOUND,
                LINE_UPPER_BOUND,
                cv2.THRESH_BINARY,
            )

            # final_mask = detected line pixels inside valid ROI
            final_mask = cv2.bitwise_and(threshold, mask)

            # clear_mask = valid ROI pixels that are NOT detected as lines
            clear_mask = cv2.bitwise_and(cv2.bitwise_not(final_mask), mask)

            # -------------------------
            # Publish clearing cloud
            # -------------------------
            clear_v, clear_u = np.where(clear_mask > 0)
            clear_depths = cv_depth[clear_v, clear_u]

            clear_valid_mask = (
                (clear_depths > MIN_DEPTH) &
                (clear_depths < MAX_DEPTH) &
                np.isfinite(clear_depths)
            )

            clear_z = clear_depths[clear_valid_mask]
            clear_u = clear_u[clear_valid_mask]
            clear_v = clear_v[clear_valid_mask]

            if len(clear_z) > 0:
                clear_points = self.pixels_to_points(clear_u, clear_v, clear_z)
                clear_points = self.downsample_points(clear_points, MAX_CLEAR_POINTS)

                self.publish_cloud(self.clear_publisher, image_depth, clear_points)
                self.clear_cloud_publish_count += 1

            # -------------------------
            # Publish line cloud
            # -------------------------
            v_indices, u_indices = np.where(final_mask > 0)
            depths = cv_depth[v_indices, u_indices]

            valid_mask = (
                (depths > MIN_DEPTH) &
                (depths < MAX_DEPTH) &
                np.isfinite(depths)
            )

            z = depths[valid_mask]
            u = u_indices[valid_mask]
            v = v_indices[valid_mask]

            if len(z) == 0:
                self.no_valid_points_count += 1

                if self.no_valid_points_count == 1 or self.no_valid_points_count % 100 == 0:
                    finite_depth_pixels = int(np.count_nonzero(np.isfinite(depths)))
                    valid_depth_pixels = int(np.count_nonzero(valid_mask))

                    self.get_logger().warn(
                        f"No valid line points. "
                        f"threshold_pixels={len(u_indices)}, "
                        f"finite_depth_pixels={finite_depth_pixels}, "
                        f"valid_depth_pixels={valid_depth_pixels}"
                    )

                return

            line_points = self.pixels_to_points(u, v, z)
            line_points = self.downsample_points(line_points, MAX_LINE_POINTS)

            self.publish_cloud(self.edge_publisher, image_depth, line_points)
            self.line_cloud_publish_count += 1

            if self.line_cloud_publish_count == 1 or self.line_cloud_publish_count % 500 == 0:
                self.get_logger().info(
                    f"Published line/clear clouds: "
                    f"line_clouds={self.line_cloud_publish_count}, "
                    f"clear_clouds={self.clear_cloud_publish_count}, "
                    f"line_points={len(line_points)}"
                )

        except Exception:
            self.get_logger().error(
                "Exception in detect_edges callback:\n" + traceback.format_exc()
            )

    def destroy_message_filter_subscriber(self, sub_obj, name):
        if sub_obj is None:
            return

        try:
            ros_sub = getattr(sub_obj, "sub", None)

            if ros_sub is not None:
                self.destroy_subscription(ros_sub)

        except Exception:
            self.get_logger().warn(
                f"Failed to destroy {name} subscription:\n" + traceback.format_exc()
            )

    def cleanup(self):
        if self.cleanup_started:
            return

        self.cleanup_started = True
        self.get_logger().info("Cleaning up edge detection resources...")

        try:
            self.ts = None
        except Exception:
            pass

        try:
            self.destroy_message_filter_subscriber(self.image_sub, "image")
            self.image_sub = None
        except Exception:
            pass

        try:
            self.destroy_message_filter_subscriber(self.depth_sub, "depth")
            self.depth_sub = None
        except Exception:
            pass

        try:
            if self.info_sub is not None:
                self.destroy_subscription(self.info_sub)
                self.info_sub = None
        except Exception:
            pass

        try:
            if self.edge_publisher is not None:
                self.destroy_publisher(self.edge_publisher)
                self.edge_publisher = None
        except Exception:
            pass

        try:
            if self.clear_publisher is not None:
                self.destroy_publisher(self.clear_publisher)
                self.clear_publisher = None
        except Exception:
            pass

        try:
            if self.masked_img_pub is not None:
                self.destroy_publisher(self.masked_img_pub)
                self.masked_img_pub = None
        except Exception:
            pass

        self.get_logger().info("Edge detection cleanup complete")


def main(args=None):
    rclpy.init(args=args)

    node = EdgeDetectionNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()

    except ExternalShutdownException:
        pass

    except BaseException as exc:
        if isinstance(exc, Exception):
            node.get_logger().fatal(
                "Fatal exception in edge detection node:\n" + traceback.format_exc()
            )

    finally:
        try:
            executor.remove_node(node)
        except Exception:
            pass

        try:
            node.cleanup()
        except Exception:
            print("Failed during edge detection cleanup:")
            print(traceback.format_exc())

        try:
            node.destroy_node()
        except Exception:
            print("Failed to destroy edge detection node:")
            print(traceback.format_exc())

        try:
            executor.shutdown(timeout_sec=1.0)
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()