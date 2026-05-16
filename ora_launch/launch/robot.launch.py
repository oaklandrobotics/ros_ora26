from launch import LaunchDescription
from launch.actions import (
DeclareLaunchArgument,
EmitEvent,
IncludeLaunchDescription,
RegisterEventHandler,
LogInfo,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit

from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  use_sim_time = LaunchConfiguration('use_sim_time')
  model_name = LaunchConfiguration('model_name')
  rviz_config = LaunchConfiguration('rviz_config')
  world_name = LaunchConfiguration('world_name')
  joy_config = LaunchConfiguration('joy_config')
  fusioncore_config = LaunchConfiguration('fusioncore_config')

  declare_use_sim_time = DeclareLaunchArgument(
    'use_sim_time',
    default_value='true',
    description='Use simulation time'
  )

  declare_model_name = DeclareLaunchArgument(
    'model_name',
    default_value='anteater.urdf.xacro',
    description='Robot model xacro file'
  )

  declare_rviz_config = DeclareLaunchArgument(
    'rviz_config',
    default_value='config.rviz',
    description='RViz config file'
  )

  declare_world_name = DeclareLaunchArgument(
    'world_name',
    default_value='igvc_world.sdf',
    description='Gazebo world file'
  )

  declare_joy_config = DeclareLaunchArgument(
    'joy_config',
    default_value='wired_xbox.yaml',
    description='Joystick config file'
  )

  declare_fusioncore_config = DeclareLaunchArgument(
    'joy_config',
    default_value='wired_xbox.yaml',
    description='Joystick config file'
  )

  ####################
  #     Includes     #
  ####################
  # Robot State Publisher
  rsp_launch = IncludeLaunchDescription(
    AnyLaunchDescriptionSource(
      PathJoinSubstitution([
        FindPackageShare('ora_description'), 'launch', 'rsp.launch.yaml'
      ])
    ),
    launch_arguments={
      'use_sim_time': use_sim_time,
      'model_name': model_name,
      'rviz_config': rviz_config
    }.items()
  )

  # Includes joy and twist_mux
  teleop_launch = IncludeLaunchDescription(
    AnyLaunchDescriptionSource(
      PathJoinSubstitution([
        FindPackageShare('ora_teleop'), 'launch', 'teleop.launch.yaml'
      ])
    ),
    launch_arguments={
      'use_sim_time': use_sim_time,
      'joy_config': joy_config
    }.items()
  )

  # Gazebo
  gazebo_launch = IncludeLaunchDescription(
    AnyLaunchDescriptionSource(
      PathJoinSubstitution([
        FindPackageShare('ora_sim'), 'launch', 'gazebo.launch.yaml'
      ])
    ),
    launch_arguments={
      'world_name': world_name
    }.items(),

    # Launch Gazebo only if use_sim_time is true
    condition=IfCondition(use_sim_time)
  )

  # LiDAR Filter
  filter_launch = IncludeLaunchDescription(
    AnyLaunchDescriptionSource(
      PathJoinSubstitution([
        FindPackageShare('ora_lidar'), 'launch', 'filter.launch.yaml',
      ])
    ),
    launch_arguments={
      'use_sim_time': use_sim_time
    }.items()
  )

  # fusioncore
  fusioncore_launch = IncludeLaunchDescription(
    AnyLaunchDescriptionSource(
      PathJoinSubstitution([
        FindPackageShare('ora_localization'), 'launch', 'fusioncore.launch.py',
      ])
    ),
    launch_arguments={
      'use_sim_time': use_sim_time
    }.items()
  )

  nav_launch = IncludeLaunchDescription(
    AnyLaunchDescriptionSource(
      PathJoinSubstitution([
        FindPackageShare('ora_navigation'), 'launch', 'nav.launch.yaml',
      ])
    ),
    launch_arguments={
      'use_sim_time': use_sim_time
    }.items()
  )

  ####################
  #      Nodes       #
  ####################
  diff_drive_node = Node(
    package='controller_manager',
    executable='spawner',
    name='diff_drive_spawner',
    output='screen',
    arguments=[
      'diff_cont'
    ],
    parameters=[
      {
        'use_sim_time': use_sim_time
      }
    ]
  )

  joint_broad_node = Node(
    package='controller_manager',
    executable='spawner',
    name='joint_broad_spawner',
    output='screen',
    arguments=[
      'joint_broad'
    ],
    parameters=[
      {
        'use_sim_time': use_sim_time
      }
    ]
  )

  edge_detection_node = Node(
    package='ora_edge_detection',
    executable='edge_detection',
    name='ora_edge_detection'
  )

  # Rotates frame such that the Robot X axis is aligned with image Z axis
  optical_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='camera_optical_frame_publisher',
    arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'camera_link', 'camera_link_optical']
  )

  ####################
  #   PreReq Nodes   #
  ####################
  filter_prereq_node = Node(
    package='ora_launch',
    executable='wait_for_prereqs.py',
    name='filter_prereq',
    output='screen',
    parameters=[
      {
        'required_topics': [
          '/scan'
        ]
      }
    ]
  )

  localization_prereq_node = Node(
    package='ora_launch',
    executable='wait_for_prereqs.py',
    name='localization_prereq',
    output='screen',
    parameters=[
      {
        'required_topics': [
          '/imu',
          '/diff_cont/odom',
          '/gnss/fix'
        ]
      }
    ]
  )

  nav_prereq_node = Node(
    package='ora_launch',
    executable='wait_for_prereqs.py',
    name='nav_prereq',
    output='screen',
    parameters=[
      {
        'required_topics': [
          '/fusion/odom',
          '/fusion/pose',
          '/scan_filtered'
        ],
        'required_transforms': [
          'odom:base_link'
        ]
      }
    ]
  )

  ####################
  #      Events      #
  ####################
  filter_launch_event = RegisterEventHandler(
    OnProcessExit(
      target_action=filter_prereq_node,
      on_exit=[
        LogInfo(msg='Starting scan filter'),
        filter_launch
      ]
    )
  )

  localization_launch_event = RegisterEventHandler(
    OnProcessExit(
      target_action=localization_prereq_node,
      on_exit=[
        LogInfo(msg='Starting fusioncore'),
        fusioncore_launch
      ]
    )
  )

  nav_launch_event = RegisterEventHandler(
    OnProcessExit(
      target_action=nav_prereq_node,
      on_exit=[
        LogInfo(msg='Starting Nav2'),
        nav_launch
      ]
    )
  )

  return LaunchDescription([
    declare_use_sim_time,
    declare_model_name,
    declare_rviz_config,
    declare_world_name,
    declare_joy_config,
    declare_fusioncore_config,

    # Generic
    rsp_launch,
    teleop_launch,

    # Sim Specific
    gazebo_launch,

    # Ros2 Control
    diff_drive_node,
    joint_broad_node,

    # Edge/line detection
    edge_detection_node,
    optical_tf_node,

    # Prerequisite Nodes
    filter_prereq_node,
    localization_prereq_node,
    nav_prereq_node,

    # Launch Events on Prereq Exit
    filter_launch_event,
    localization_launch_event,
    nav_launch_event
  ])
