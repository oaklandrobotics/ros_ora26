import rclpy
import message_filters
from rclpy.node import Node 
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs_py import point_cloud2 as pc2

# Topic names
RAW_IMAGE_TOPIC = "/depth_camera/image_raw"
DEPTH_IMAGE_TOPIC = "/depth_camera/depth_image_raw"
CAMERA_INFO_TOPIC = "/depth_camera/camera_info"
EDGE_POINTS_TOPIC = "/depth_camera/filtered/points"

# Parameters
BLUR_KERNEL_SIZE = 5
# Values closer to 255 are colors closer to white (white is 255)
LINE_LOWER_BOUND = 200
LINE_UPPER_BOUND = 255

class EdgeDetectionNode(Node):
    def __init__(self):
        super().__init__("edge_detection_node")
        self.bridge = CvBridge()
        self.camera_model = None

        # Topic subscriptions
        self.image_sub = message_filters.Subscriber(
            self, 
            Image, 
            RAW_IMAGE_TOPIC
        )
        self.depth_sub = message_filters.Subscriber(
            self, 
            Image, 
            DEPTH_IMAGE_TOPIC
        )
        self.info_sub = self.create_subscription(
            CameraInfo, 
            CAMERA_INFO_TOPIC, 
            self.info_callback, 
            10
        )

        # Topic publisher
        self.edge_publisher = self.create_publisher(PointCloud2, EDGE_POINTS_TOPIC, 10)

        # Messages from image_sub and depth_sub must be synced for accuracy.
        #   ts stands for time synchronization
        self.ts = message_filters.ApproximateTimeSynchronizer(
            (self.image_sub, self.depth_sub), 10, slop=0.1)
        self.ts.registerCallback(self.detect_edges)

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

    def detect_edges(self, image_raw, image_depth):
        if self.camera_model is None:
            self.get_logger().warn("Waiting for CameraInfo...")
            return

        # Convert ROS 2 images to OpenCV
        cv_raw = self.bridge.imgmsg_to_cv2(image_raw, 'bgr8')
        cv_depth = self.bridge.imgmsg_to_cv2(image_depth, '32FC1')

        # Image Processing (Grayscale + Blur)
        grey_scale = cv2.cvtColor(cv_raw, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(grey_scale, (BLUR_KERNEL_SIZE, BLUR_KERNEL_SIZE), 0)

        # Masking (Bottom half)
        mask = np.zeros(cv_raw.shape[:2], dtype=np.uint8)
        h, w = cv_raw.shape[:2]
        cv2.rectangle(mask, (0, int(h/2)), (w, h), 255, -1)
        
        # Thresholding to find "White" edges
        _, threshold = cv2.threshold(blur, LINE_LOWER_BOUND, LINE_UPPER_BOUND, cv2.THRESH_BINARY)
        final_mask = cv2.bitwise_and(threshold, mask)

        # Find every pixel (u,v) within the threshold and get it's depth(z)
        v_indices, u_indices = np.where(final_mask > 0)
        depths = cv_depth[v_indices, u_indices]

        # Filter out invalid depth (0 or NaN)
        # Increases performance by only rendering points that are part of a 'line'
        # Adjust (depths > NUM) to change the minimum depth (stop robot from appearing as a line)
        valid_mask = (depths > 1) & (~np.isnan(depths))
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
        points = np.column_stack((x, y, z))

        # Create and Publish PointCloud2
        pc_msg = pc2.create_cloud_xyz32(image_depth.header, points)
        pc_msg.header.frame_id = "camera_link_optical"
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