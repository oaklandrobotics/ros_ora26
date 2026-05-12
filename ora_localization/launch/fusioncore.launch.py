from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():
  use_sim_time = LaunchConfiguration('use_sim_time')
  param_file = LaunchConfiguration('param_file')

  declare_use_sim_time = DeclareLaunchArgument(
    'use_sim_time',
    default_value='true',
    description='Use simulation time'
  )

  declare_param_file = DeclareLaunchArgument(
    'param_file',
    default_value='fusioncore_config.yaml',
    description='Fusioncore configuration file'
  )

  fusioncore_node = LifecycleNode(
    package='fusioncore_ros',
    executable='fusioncore_node',
    name='fusioncore',
    namespace='',
    output='screen',
    parameters=[
      PathJoinSubstitution([
        FindPackageShare('ora_localization'), 'config', 'fusioncore_config.yaml',
      ]),
      {
        'use_sim_time': use_sim_time,
      }
    ],
    remappings=[
      ('/imu/data', '/imu'),
      ('/odom/wheels', '/diff_cont/odom'),
      ('/gnss/fix', '/gnss/fix'),
    ]
  )

  configure_fusioncore = TimerAction(
    period=2.0,
    actions=[EmitEvent(event=ChangeState(
      lifecycle_node_matcher=lambda a: a is fusioncore_node,
      transition_id=Transition.TRANSITION_CONFIGURE
    ))]
  )

  activate_fusioncore = RegisterEventHandler(OnStateTransition(
    target_lifecycle_node=fusioncore_node,
    start_state='configuring',
    goal_state='inactive',
    entities=[EmitEvent(event=ChangeState(
      lifecycle_node_matcher=lambda a: a is fusioncore_node,
      transition_id=Transition.TRANSITION_ACTIVATE
    ))]
  ))

  return LaunchDescription([
    declare_use_sim_time,
    declare_param_file,

    fusioncore_node,
    configure_fusioncore,
    activate_fusioncore
  ])
