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
    avp_demo_pkg_prefix = get_package_share_directory('autoware_demos')
    autoware_launch_pkg_prefix = get_package_share_directory('autoware_auto_launch')

    behavior_planner_param_file = os.path.join(
        autoware_launch_pkg_prefix, 'param/behavior_planner.param.yaml')
    vehicle_characteristics_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/vehicle_characteristics.param.yaml')
    vehicle_constants_manager_param_file = os.path.join(
        autoware_launch_pkg_prefix, 'param/lexus_rx_hybrid_2016.param.yaml')
    costmap_generator_param_file = os.path.join(
        autoware_launch_pkg_prefix, 'param/costmap_generator.param.yaml')
    freespace_planner_param_file = os.path.join(
        autoware_launch_pkg_prefix, 'param/freespace_planner.param.yaml')
    

    ###############################
    # Arguments
    ###############################
    
    behavior_planner_param = DeclareLaunchArgument(
        'behavior_planner_param_file',
        default_value=behavior_planner_param_file,
        description='Path to parameter file for behavior planner'
    )
    vehicle_characteristics_param = DeclareLaunchArgument(
        'vehicle_characteristics_param_file',
        default_value=vehicle_characteristics_param_file,
        description='Path to config file for vehicle characteristics'
    )
    vehicle_constants_manager_param = DeclareLaunchArgument(
        'vehicle_constants_manager_param_file',
        default_value=vehicle_constants_manager_param_file,
        description='Path to parameter file for vehicle_constants_manager'
    )
    costmap_generator_param = DeclareLaunchArgument(
        'costmap_generator_param_file',
        default_value=costmap_generator_param_file,
        description='Path to parameter file for costmap generator'
    )
    freespace_planner_param = DeclareLaunchArgument(
        'freespace_planner_param_file',
        default_value=freespace_planner_param_file,
        description='Path to parameter file for freespace_planner'
    )

    ###############################
    # Nodes
    ###############################
    
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
    
    #/planning/global_path
    gaode_api_global_planner_node_runner = Node(
        package='gaode_api_global_planner_nodes',
        executable='gaode_api_global_planner_node_exe',
        namespace='planning',
        remappings=[
            ("goal_pose", "/localization/goal_pose"),
            ("cur_pose", "/localization/cur_pose"),
            ("init_gps", "/localization/init_gps")
        ]
    )

    costmap_generator = Node(
        package='costmap_generator_nodes',
        executable='costmap_generator_node_exe',
        name='costmap_generator_node',
        namespace='planning',
        output='screen',
        parameters=[
            LaunchConfiguration('costmap_generator_param_file'),
        ],
        remappings=[
            ('~/client/HAD_Map_Service', '/had_maps/HAD_Map_Service')
        ]
    )
    
    freespace_planner = Node(
        package='freespace_planner_nodes',
        executable='freespace_planner_node_exe',
        name='freespace_planner',
        namespace='planning',
        output='screen',
        parameters=[
            LaunchConfiguration('freespace_planner_param_file'),
            LaunchConfiguration('vehicle_constants_manager_param_file')
        ]
    )

    behavior_planner = Node(
        package='behavior_planner_nodes',
        name='behavior_planner_node',
        namespace='planning',
        executable='behavior_planner_node_exe',
        parameters=[
            LaunchConfiguration('behavior_planner_param_file'),
            {'enable_object_collision_estimator': True},
            LaunchConfiguration('vehicle_characteristics_param_file'),
        ],
        output='screen',
        remappings=[
            ('HAD_Map_Service', '/had_maps/HAD_Map_Service'),
            ('vehicle_state', '/localization/cur_pose'),
            ('route', 'global_path'),
            ('gear_report', '/vehicle/gear_report'),
            ('gear_command', '/vehicle/gear_command')
        ]
    )

    

    return launch.LaunchDescription([
        vehicle_characteristics_param,
        vehicle_constants_manager_param,
        costmap_generator_param,
        freespace_planner_param,
        behavior_planner_param,
        eskf_runner,
        gaode_api_global_planner_node_runner,
        costmap_generator,
        freespace_planner,
        behavior_planner,
    ])
