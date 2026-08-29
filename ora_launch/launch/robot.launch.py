import os
from launch import LaunchDescription
from launch.actions import (
DeclareLaunchArgument,
EmitEvent,
IncludeLaunchDescription,
RegisterEventHandler,
LogInfo,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, Command
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit

from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

# Environment variables for parameters
# This allows you to add env variables to your ~/.bashrc file rather 
#   than specifying them every launch
#  For example, to set use_sim_time to true on your system, 
#    append 'export USE_SIM_TIME=true' to the end of your ~/.bashrc file.
#    After changing an environment file, you must source it with 'source ~/.bashrc'
USE_SIM_TIME = os.getenv('USE_SIM_TIME', 'false')
MODEL_NAME = os.getenv('MODEL_NAME', 'anteater.urdf.xacro')
RVIZ_CONFIG = os.getenv('RVIZ_CONFIG', 'config.rviz')
WORLD_NAME = os.getenv('WORLD_NAME', 'igvc_world.sdf')
JOY_CONFIG = os.getenv('JOY_CONFIG', 'bt_xbox.yaml')
FUSIONCORE_CONFIG = os.getenv('FUSIONCORE_CONFIG', 'fusioncore_config.yaml')
USE_SEMANTIC_SEGMENTATION = os.getenv('USE_SEMANTIC_SEGMENTATION', 'true')

def generate_launch_description():
  use_sim_time = LaunchConfiguration('use_sim_time')
  model_name = LaunchConfiguration('model_name')
  rviz_config = LaunchConfiguration('rviz_config')
  world_name = LaunchConfiguration('world_name')
  joy_config = LaunchConfiguration('joy_config')
  fusioncore_config = LaunchConfiguration('fusioncore_config')
  use_semantic_segmentation = LaunchConfiguration('use_semantic_segmentation')

  declare_use_sim_time = DeclareLaunchArgument(
    'use_sim_time',
    default_value=USE_SIM_TIME,
    description='Use simulation time'
  )

  declare_model_name = DeclareLaunchArgument(
    'model_name',
    default_value=MODEL_NAME,
    description='Robot model xacro file'
  )

  declare_rviz_config = DeclareLaunchArgument(
    'rviz_config',
    default_value=RVIZ_CONFIG,
    description='RViz config file'
  )

  declare_world_name = DeclareLaunchArgument(
    'world_name',
    default_value=WORLD_NAME,
    description='Gazebo world file'
  )

  declare_joy_config = DeclareLaunchArgument(
    'joy_config',
    default_value=JOY_CONFIG,
    description='Joystick config file'
  )

  declare_fusioncore_config = DeclareLaunchArgument(
    'fusioncore_config',
    default_value=FUSIONCORE_CONFIG,
    description='FusionCore config file'
  )

  declare_use_semantic_segmentation = DeclareLaunchArgument(
    'use_semantic_segmentation',
    default_value=USE_SEMANTIC_SEGMENTATION,
    description='Launch with/without semantic segmentation node'
  )

  robot_description = ParameterValue(
    Command([
      'xacro',
      ' ',
      PathJoinSubstitution([
        FindPackageShare('ora_description'),
        'description',
        model_name
      ]),
      ' ',
      'use_sim_time:=false'
    ]),
    value_type=str
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

  #LiDAR
  lidar_launch = IncludeLaunchDescription(
    AnyLaunchDescriptionSource(
      PathJoinSubstitution([
        FindPackageShare('lidar'), 'launch', 'VLP16.launch.py'
      ])
    ),
    condition=UnlessCondition(use_sim_time)
  )

  # LiDAR Filter
  filter_launch = IncludeLaunchDescription(
    AnyLaunchDescriptionSource(
      PathJoinSubstitution([
        FindPackageShare('lidar'), 'launch', 'filter.launch.yaml',
      ])
    ),
    launch_arguments={
      'use_sim_time': use_sim_time
    }.items()
  )

  # GNSS
  gnss_launch = IncludeLaunchDescription(
    AnyLaunchDescriptionSource(
      PathJoinSubstitution([
        FindPackageShare('ora_navigation'), 'launch', 'ublox.launch.yaml'
      ])
    ),
    launch_arguments={
      'use_sim_time': use_sim_time
    }.items(),

    condition=UnlessCondition(use_sim_time)
  )

  # fusioncore
  fusioncore_launch = IncludeLaunchDescription(
    AnyLaunchDescriptionSource(
      PathJoinSubstitution([
        FindPackageShare('ora_launch'), 'launch', 'fusioncore.launch.py',
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
      'diff_cont',
      '--controller-manager',
      '/controller_manager'
    ],
  )

  joint_broad_node = Node(
    package='controller_manager',
    executable='spawner',
    name='joint_broad_spawner',
    output='screen',
    arguments=[
      'joint_broad',
      '--controller-manager',
      '/controller_manager'
    ],
  )

  controller_manager = Node(
    package='controller_manager',
    executable='ros2_control_node',
    parameters=[
      {
        'robot_description': robot_description
      },
      PathJoinSubstitution([
        FindPackageShare('ora_launch'),
        'config',
        'real_controller.yaml'
      ])
    ],
    output='screen',
    condition=UnlessCondition(use_sim_time)
  )

  edge_detection_node = Node(
    package='edge_detection',
    executable='edge_detection',
    name='edge_detection',
    parameters=[
      {
        'use_sim_time': use_sim_time
      }
    ]
  )

  semantic_segmentation_node = Node(
    package='semantic_segmentation',
    executable='segmentation_node',
    name='semantic_segmentation',
    parameters=[
      {
        'use_sim_time': use_sim_time
      }
    ],
    condition=IfCondition(use_semantic_segmentation)
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
          '/esp/imu',
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
    declare_use_semantic_segmentation,

    # Generic
    rsp_launch,
    teleop_launch,

    # Sim Specific
    gazebo_launch,

    # Ros2 Control
    controller_manager,
    diff_drive_node,
    joint_broad_node,

    # LiDAR
    lidar_launch,

    # GNSS
    gnss_launch,

    # Edge/line detection
    edge_detection_node,

    # Semantic segmentation
    semantic_segmentation_node,

    # Prerequisite Nodes
    filter_prereq_node,
    localization_prereq_node,
    nav_prereq_node,

    # Launch Events on Prereq Exit
    filter_launch_event,
    localization_launch_event,
    nav_launch_event
  ])
