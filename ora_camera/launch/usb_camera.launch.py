from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_ora_camera = get_package_share_directory('ora_camera')

    declare_video_device_cmd = DeclareLaunchArgument(
        'video_device', default_value='/dev/video0')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_ora_camera, 'config', 'ora_camera_params.yaml'))

    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        namespace='ora_camera',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('image_raw', 'image_raw'),
            ('camera_info', 'camera_info'),
        ]
    )

    # Compressed images (recommended for bandwidth)
    compress_node = Node(
        package='image_transport',
        executable='republish',
        name='republish_compressed',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', '/ora_camera/image_raw'),
            ('out', '/ora_camera/image_raw/compressed')
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_video_device_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(usb_cam_node)
    ld.add_action(compress_node)
    return ld
