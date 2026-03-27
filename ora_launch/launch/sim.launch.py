import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer


def generate_launch_description():
    # Package share directories
    launch_pkg_share = get_package_share_directory('ora_launch')
    desc_pkg_share = get_package_share_directory('ora_description')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    ora_camera_share = get_package_share_directory('ora_camera')

    # Gazebo / world files
    world_path = os.path.join(desc_pkg_share, 'worlds', 'igvc_world.sdf')
    bridge_config_path = os.path.join(desc_pkg_share, 'config', 'bridge_config.yaml')
    gz_spawn_model_launch_source = os.path.join(
        ros_gz_sim_share, "launch", "gz_spawn_model.launch.py"
    )

    # ────────────────────────────────────────────────
    # Launch Arguments
    # ────────────────────────────────────────────────
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_use_camera = DeclareLaunchArgument(
        name='use_camera',
        default_value='true',
        description='Launch the USB camera driver (real hardware)'
    )

    # ────────────────────────────────────────────────
    # Robot State Publisher (from rsp.launch.py)
    # ────────────────────────────────────────────────
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg_share, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_ros2_control': 'true'
        }.items()
    )

    # ────────────────────────────────────────────────
    # USB Camera (real hardware driver)
    # ────────────────────────────────────────────────
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ora_camera_share, 'launch', 'usb_camera.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # You can override video_device here if needed:
            # 'video_device': '/dev/video0'
        }.items(),
        #condition=launch.conditions.IfCondition(LaunchConfiguration('use_camera'))
    )

    # ────────────────────────────────────────────────
    # Gazebo Server & Bridge
    # ────────────────────────────────────────────────
    gz_server = GzServer(
        world_sdf_file=world_path,
        container_name='ros_gz_container',
        create_own_container='True',
        use_composition='True',
    )

    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=bridge_config_path,
        container_name='ros_gz_container',
        create_own_container='False',
        use_composition='True',
    )

    # Spawn robot in Gazebo
    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_spawn_model_launch_source),
        launch_arguments={
            'world': 'igvc_world',
            'topic': '/robot_description',
            'entity_name': 'whoknows_bot',
            'z': '0.65',
        }.items(),
    )

    # Delay spawn to avoid race condition
    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_entity]
    )

    # ────────────────────────────────────────────────
    # Launch Description
    # ────────────────────────────────────────────────
    return LaunchDescription([
        # Arguments
        declare_use_sim_time,
        declare_use_camera,

        # Robot description & TF
        rsp,

        # Camera (real)
        camera_launch,

        # Gazebo GUI (runs separately)
        ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen'),

        # Gazebo server + bridge
        gz_server,
        ros_gz_bridge,

        # Delayed robot spawn
        delayed_spawn,
    ])
