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
    ###############################
    # Files
    ###############################
    avp_demo_pkg_prefix = get_package_share_directory('autoware_demos')
    autoware_launch_pkg_prefix = get_package_share_directory('autoware_auto_launch')
    
    vehicle_characteristics_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/vehicle_characteristics.param.yaml')
    vehicle_constants_manager_param_file = os.path.join(
        autoware_launch_pkg_prefix, 'param/guide_dog_robot.param.yaml')
    costmap_generator_param_file = os.path.join(
        autoware_launch_pkg_prefix, 'param/costmap_generator.param.yaml')
    freespace_planner_param_file = os.path.join(
        autoware_launch_pkg_prefix, 'param/freespace_planner.param.yaml')
    behavior_planner_param_file = os.path.join(
        autoware_launch_pkg_prefix, 'param/behavior_planner.param.yaml')

    ###############################
    # Arguments
    ###############################
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

    behavior_planner_param = DeclareLaunchArgument(
        'behavior_planner_param_file',
        default_value=behavior_planner_param_file,
        description='Path to parameter file for behavior planner'
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
        executable='eskf_node_exe',
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

    euclidean_cluster_node_runner = Node(
        package='euclidean_cluster_nodes',
        executable='euclidean_cluster_node_exe',
        namespace='lidars',
        parameters=[
            get_param_file('euclidean_cluster_nodes', 'guide_dog_cluster.param.yaml'),
            {'use_detected_objects': True}],
        remappings=[
            ("points_in", "points_nonground"),
            ("points_clustered", "cluster_points")
        ])

    # ray ground filter runner definition.
    ray_ground_runner = Node(
        package='ray_ground_classifier_nodes',
        executable='ray_ground_classifier_cloud_node_exe',
        namespace='lidars',
        parameters=[get_param_file('ray_ground_classifier_nodes',
                                   'vlp16_sim_lexus_ray_ground.param.yaml')],
        remappings=[("points_in", "/lidar_front/points_filtered")])

    # point cloud filter transform param file
    filter_transform_param = get_param_file(
            'point_cloud_filter_transform_nodes',
            'guide_dog_filter_transform.param.yaml')

    # point cloud filter transform runner definition for front lidar
    filter_transform_front_runner = Node(
        package='point_cloud_filter_transform_nodes',
        executable='point_cloud_filter_transform_node_exe',
        name='filter_transform_vlp16_front',
        namespace='lidar_front',
        parameters=[filter_transform_param],
        # remappings=[("points_in", "points_raw")])
        remappings=[("points_in", "/hesai/pandar"),
                    ("imu_in", "/imu1")])

    multi_object_tracker = Node(
        executable='multi_object_tracker_node_exe',
        name='multi_object_tracker',
        namespace='perception',
        on_exit=Shutdown(),
        package='tracking_nodes',
        parameters=[
            get_param_file('autoware_demos',
                           'multi_object_tracker.param.yaml'),
            {
                'use_ndt': False,
                'track_frame_id': "odom",
                'use_vision': False,
                'num_vision_topics': 1
            }
        ],
        remappings=[
            ("detected_objects", "/lidars/lidar_detected_objects"),
            ("ego_state", "/localization/fused_pose1"),
            ("classified_rois1", "/perception/ground_truth_detections_2d"),
            ("clusters", "/lidars/cluster_points")
        ]
    )

    single_camera_robot_state_publisher_runner = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': get_lexus_robot_description('navi_car.urdf')}],
    )

    # Run rviz
    examples_pkg_path = get_package_share_directory(
        'autoware_demos')
    rviz_cfg_path = os.path.join(examples_pkg_path, 'rviz2',
                                 'guide_dog_predict.rviz')
    rviz_runner = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(rviz_cfg_path)])
    
    # prediction
    prediction = Node(
        executable='prediction_nodes_node_exe',
        name='prediction',
        namespace='prediction',
        output="screen",
        package='prediction_nodes',
        parameters=[get_param_file('autoware_demos', 'prediction.param.yaml')],
        remappings=[
            ("tracked_objects", "/perception/tracked_objects")
        ],
    )

    ###############################
    # Planning Nodes
    ###############################
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
            ('predicted_objects', '/prediction/predicted_objects')
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

    global_path_mapping = Node(
        package='global_path_mapping',
        executable='global_path_mapping_node_exe',
        name='global_path_mapping',
        namespace='planning',
    )

    return launch.LaunchDescription([
        vehicle_characteristics_param,
        vehicle_constants_manager_param,
        costmap_generator_param,
        freespace_planner_param,
        # behavior_planner_param,
        eskf_runner,
        euclidean_cluster_node_runner,
        ray_ground_runner,
        filter_transform_front_runner,
        multi_object_tracker,
        prediction,
        costmap_generator,
        single_camera_robot_state_publisher_runner,
        freespace_planner,
        # behavior_planner,
        rviz_runner,
        global_path_mapping
    ])
