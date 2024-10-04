import os
from pathlib import Path
import launch
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

CURRENT_PATH = Path(os.getcwd())
PACKAGE_PATH = CURRENT_PATH / Path('autonomy/jupiter/robotics/jupiter-desktop/examples/ros2/launch')
DEFAULT_CONFIG_YAML_PATH = str(PACKAGE_PATH / 'halo_cameras.yaml')


def generate_launch_description():

    ld = launch.LaunchDescription(
        [

            DeclareLaunchArgument(name="ip", default_value="127.0.0.1"),
            DeclareLaunchArgument(name="port", default_value="41452"),
            DeclareLaunchArgument(name='config_yaml', default_value = DEFAULT_CONFIG_YAML_PATH),
            DeclareLaunchArgument(name='env_ros_domain_id', default_value = '11'),
            SetEnvironmentVariable(name='ROS_DOMAIN_ID', value=LaunchConfiguration('env_ros_domain_id')),
            Node(
                executable="autonomy/jupiter/callisto/jupiter_factesim/fs_camera_manager",
                output="screen",
                name="fs_camera_manager",
                arguments = '--dds_domain_id 11'.split(),
                parameters=[
                               {"ip": LaunchConfiguration("ip")},
                               {"port": LaunchConfiguration("port")},
                               {'config_yaml': LaunchConfiguration('config_yaml')},
                           ]
            )
        ]
    )

    return ld