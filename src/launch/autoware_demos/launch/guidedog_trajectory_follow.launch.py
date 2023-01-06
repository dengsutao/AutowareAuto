# Copyright 2020-2021 the Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.


"""
This launch file launches all the nodes necessary to produce object tracks.

Based on raw lidar points and 2d detections from a single camera in the SVL simulator.
Also launches rviz.
Uses the lgsvl-sensors-camera.json sensor configuration file for the simulator.
"""


import os

from ament_index_python import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch.substitutions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import Shutdown
from launch.conditions import IfCondition
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration


def get_param_file(package_name, file_name):
    """Pass the given param file as a LaunchConfiguration."""
    file_path = os.path.join(
        get_package_share_directory(package_name),
        'param',
        file_name)
    return launch.substitutions.LaunchConfiguration(
        'params', default=[file_path])


def get_lexus_robot_description(filename):
    # Setup robot state publisher
    vehicle_description_pkg_path = get_package_share_directory(
        'lexus_rx_450h_description')
    urdf_path = os.path.join(vehicle_description_pkg_path, 'urdf',
                             filename)
    with open(urdf_path, 'r') as infp:
        urdf_file = infp.read()

    return urdf_file


def generate_launch_description():
    autoware_launch_pkg_prefix = get_package_share_directory('autoware_auto_launch')

    # Sensors
    imu_gps_runner = Node(
        package='imu_gps',
        executable='imu_gps_node',
        parameters=[{
            'port': "/dev/imu_gps_usb",
            'baudrate': 115200,
            "debug": False
         }])
    gps_runner = Node(
        package='gps',
        executable='gps_node',
        parameters=[{
            'port': "/dev/gps",
            'baudrate': 9600,
            "debug": False
         }])
    wheel_imu_runner = Node(
        package='wheel_imu',
        executable='wheel_imu_node',
        parameters=[{
            'debug': False,
         }],
        remappings=[("cmd_vel", "/control/output/control_cmd")]
    )


    #"/localization/goal_pose"
    #"/localization/cur_pose"
    #"/localization/init_gps"
    eskf_runner = Node(
        package='eskf',
        executable='eskf_node',
        namespace='localization',
        parameters=[{
            'mode': 0,
            'debug': False,
         }],
        remappings=[
            ("imu1", "/imu1"),
            ("odom", "/odom"),
            ("gps", "/gps")
        ]
    )

    pure_pursuit_node = Node(
        package='pure_pursuit_nodes',
        executable='pure_pursuit_node_exe',
        name='pure_pursuit',
        namespace='control',
        output='screen',
        parameters=[get_param_file('pure_pursuit_nodes',
                                   'pure_pursuit.param.yaml')],
        remappings=[
            ("input/current_trajectory", "/planning/trajectory"),
            ("input/current_state", "/vehicle/vehicle_kinematic_state"),
        ]
    )

    return launch.LaunchDescription([
        imu_gps_runner,
        gps_runner,
        wheel_imu_runner,
        eskf_runner,
        pure_pursuit_node
    ])
