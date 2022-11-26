# Copyright 2021 the Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory

import os


def generate_launch_description():
    f1tenth_launch_pkg_prefix = get_package_share_directory('f1tenth_launch')

    # param files
    vesc_to_odom_node_param_file = os.path.join(
        f1tenth_launch_pkg_prefix, 'param/vesc_to_odom_node.param.yaml')
    odom_to_state_param_file = os.path.join(
        f1tenth_launch_pkg_prefix, 'param/odom_to_state_conversion.param.yaml')
    joy_translator_param_file = os.path.join(
        f1tenth_launch_pkg_prefix, 'param/logitech_f710_basic.param.yaml')
    joy_param_file = os.path.join(
        f1tenth_launch_pkg_prefix, 'param/joy.param.yaml')
    vesc_config = os.path.join(
        f1tenth_launch_pkg_prefix, 'param/vesc.param.yaml')
    hokuyo_param_file = os.path.join(
        f1tenth_launch_pkg_prefix, 'param/urg_node.param.yaml')
    ekf_param_file = os.path.join(
        f1tenth_launch_pkg_prefix, "param/ekf.param.yaml")
    imu_param_file = os.path.join(
        f1tenth_launch_pkg_prefix, "param/razor.param.yaml")
    laser_filter_param_file = os.path.join(
        f1tenth_launch_pkg_prefix, "param/laser_filter.param.yaml")
    f1tenth_urdf = os.path.join(
        f1tenth_launch_pkg_prefix, 'urdf/f1tenth_vesc.urdf')

    with open(f1tenth_urdf, 'r') as infp:
        urdf_file = infp.read()

    vesc_interface_param_file = os.path.join(
        f1tenth_launch_pkg_prefix, 'param/vesc_config.param.yaml'
    )

    with_joy_param = DeclareLaunchArgument(
        'with_joy',
        default_value='True',
        description='Launch joystick_interface in addition to other nodes'
    )

    # Nodes
    vesc_driver_launcher = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        namespace='vehicle',
        output='screen',
        parameters=[vesc_config]
    )

    vesc_to_odom_launcher = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        namespace='vehicle',
        output='screen',
        parameters=[vesc_to_odom_node_param_file],
        remappings=[
            ('odom', "vesc_odom")
        ]
    )

    # joystick driver node
    joy = Node(
        package='joy_linux',
        executable='joy_linux_node',
        output='screen',
        parameters=[joy_param_file]
    )

    # joystick translator node
    joy_translator = Node(
        package='joystick_vehicle_interface_nodes',
        executable='joystick_vehicle_interface_node_exe',
        output='screen',
        parameters=[joy_translator_param_file],
        remappings=[
            ("basic_command", "/vehicle/vehicle_command"),
            ("auto_basic_command", "/vehicle/auto_vehicle_command"),
            ("state_command", "/vehicle/state_command")
        ],
        condition=IfCondition(LaunchConfiguration('with_joy'))
    )

    vesc_interface_node = Node(
        package='vesc_interface',
        executable='vesc_interface_node_exe',
        namespace='vehicle',
        output='screen',
        parameters=[vesc_interface_param_file],
    )

    odom_to_state_node = Node(
        package='odom_to_state_conversion_nodes',
        executable='odom_to_state_conversion_nodes_exe',
        namespace='vehicle',
        output='screen',
        parameters=[odom_to_state_param_file],
        remappings=[
            ("vehicle_state", "/vehicle/vehicle_kinematic_state"),
            ("odometry", "/vehicle/odometry"),
            ("odom", "/vehicle/odometry/filtered")
        ]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        namespace='vehicle',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[ekf_param_file],
    )

    razor_node = Node(
        package='razor_imu_ros2',
        executable='razor_imu_ros2_exe',
        output='screen',
        parameters=[imu_param_file],
        remappings=[
            ('imu', '/razor/imu'),
        ]
    )

    hokuyo_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        namespace='lidar',
        output='screen',
        parameters=[hokuyo_param_file],
        remappings=[
            ('scan', '/lidar/scan_raw')
        ]
    )

    laser_filter_node = Node(
        package="laser_filters",
        namespace='lidar',
        executable="scan_to_scan_filter_chain",
        parameters=[laser_filter_param_file],
        remappings=[
            ('output', 'scan'),
            ('scan', '/lidar/scan_raw')
        ]
    )

    robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='vehicle',
        output='screen',
        parameters=[{'robot_description': urdf_file}],
    )

    return launch.LaunchDescription([
        with_joy_param,
        vesc_driver_launcher,
        vesc_to_odom_launcher,
        odom_to_state_node,
        joy,
        joy_translator,
        vesc_interface_node,
        hokuyo_node,
        laser_filter_node,
        robot_publisher,
        ekf_node,
        razor_node
    ])
