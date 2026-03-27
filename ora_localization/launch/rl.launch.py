import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchService, LaunchContext
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.event_handlers import OnProcessStart
from launch.events.process import ProcessStarted

from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer

def generate_launch_description():
  localization_share = get_package_share_directory('ora_localization')

  ekf_path = os.path.join(localization_share, 'config/sim_rl_gps_ekf.yaml')

  robot_localization_node = Node(
      package='robot_localization',
      executable='ekf_node',
      name='ekf_filter_node',
      output='screen',
      parameters=[ekf_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
  )

  return LaunchDescription([
    # Launch Args
    DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),

    # Start the robot localization node
    robot_localization_node
  ])