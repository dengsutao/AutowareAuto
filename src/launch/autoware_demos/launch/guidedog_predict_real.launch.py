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

    costmap_generator_param_file = os.path.join(
        autoware_launch_pkg_prefix, 'param/costmap_generator.param.yaml')

    costmap_generator_param = DeclareLaunchArgument(
        'costmap_generator_param_file',
        default_value=costmap_generator_param_file,
        description='Path to parameter file for costmap generator'
    )

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
         }])

    hesai_node = Node(
            package ='hesai_lidar',
            node_namespace ='hesai',
            node_executable ='hesai_lidar_node',
            name ='hesai_node',
            output ="screen",
            parameters=[
                {"pcap_file": ""},
                {"server_ip"  : "10.81.81.99"},
                {"lidar_recv_port"  : 2368},
                {"gps_port"  : 10110},
                {"start_angle"  : 0.0},
                {"lidar_type"  : "PandarQT"},
                {"frame_id"  : "PandarQT"},
                {"pcldata_type"  : 0},
                {"publish_type"  : "both"},
                {"timestamp_type"  : "''"},
                {"data_type"  : "''"},
                {"lidar_correction_file"  : "./src/HesaiLidar_General_ROS/config/PandarQT.csv"},
                {"multicast_ip"  : "''"},
                {"coordinate_correction_flag"  : False},
                {"fixed_frame"  : "''"},
                {"target_frame_frame"  : "''"}
            ]
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

    # euclidean cluster node execution definition.
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
            'vlp16_sim_lexus_filter_transform.param.yaml')

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

    #
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

    return launch.LaunchDescription([
        imu_gps_runner,
        gps_runner,
        wheel_imu_runner,
        hesai_node,
        costmap_generator_param,
        eskf_runner,
        euclidean_cluster_node_runner,
        ray_ground_runner,
        filter_transform_front_runner,
        multi_object_tracker,
        prediction,
        costmap_generator,
        single_camera_robot_state_publisher_runner,
        rviz_runner,
    ])
