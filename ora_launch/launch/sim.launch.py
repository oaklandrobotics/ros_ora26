import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchService, LaunchContext
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.event_handlers import OnProcessStart
from launch.events.process import ProcessStarted

from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer

def generate_launch_description():
  # Packages for launching
  launch_pkg_share = get_package_share_directory('ora_launch')
  desc_pkg_share = get_package_share_directory('ora_description')
  ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
  
  # Gazebo files
  world_path = os.path.join(desc_pkg_share, 'worlds', 'igvc_world.sdf')
  bridge_config_path = os.path.join(desc_pkg_share, 'config', 'bridge_config.yaml')
  gz_spawn_model_launch_source = os.path.join(ros_gz_sim_share, "launch", "gz_spawn_model.launch.py")

  ##############################################
  #                                            #
  #                     RSP                    #
  #                                            #
  ##############################################
  rsp = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(
        desc_pkg_share, 'launch', 'rsp.launch.py'
      )
    ]),
    launch_arguments={
      'use_sim_time' : 'false',
      'use_ros2_control' : 'true'
    }.items()
  )

  ##############################################
  #                                            #
  #                   Gazebo                   #
  #                                            #
  ##############################################
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
  spawn_entity = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(gz_spawn_model_launch_source),
      launch_arguments={
          'world': 'igvc_world',
          'topic': '/robot_description',
          'entity_name': 'whoknows_bot',
          'z': '0.65',
      }.items(),
  )
    
  return LaunchDescription([
    # Start the robot state publisher
    rsp,
    ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen'),
    gz_server,
    ros_gz_bridge,
    spawn_entity,
  ])