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

"""Launch perception stack using lidar data from the SVL simulator."""

import os

from ament_index_python import get_package_share_directory
import launch.substitutions
import launch_ros.actions


# assumes the param file has the given file_name and is under
# "param" folder inside the given package_name.
def get_param_file(package_name, file_name):
    """Get the specified param file."""
    file_path = os.path.join(
        get_package_share_directory(package_name),
        'param',
        file_name)
    return launch.substitutions.LaunchConfiguration(
        'params', default=[file_path])


def generate_launch_description():
    """Launch all the needed perception nodes."""
    # euclidean cluster node execution definition.
    euclidean_cluster_node_runner = launch_ros.actions.Node(
        package='euclidean_cluster_nodes',
        executable='euclidean_cluster_node_exe',
        namespace='lidars',
        parameters=[
            get_param_file('euclidean_cluster_nodes', 'vlp16_sim_lexus_cluster.param.yaml'),
            {'use_detected_objects': True}],
        remappings=[
            ("points_in", "points_nonground"),
            ("points_clustered", "cluster_points")
        ])

    # ray ground filter runner definition.
    ray_ground_runner = launch_ros.actions.Node(
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
    filter_transform_front_runner = launch_ros.actions.Node(
        package='point_cloud_filter_transform_nodes',
        executable='point_cloud_filter_transform_node_exe',
        name='filter_transform_vlp16_front',
        namespace='lidar_front',
        parameters=[filter_transform_param],
        # remappings=[("points_in", "points_raw")])
        remappings=[("points_in", "/hesai/pandar"),
                    ("imu_in", "/imu1")])

    # point cloud fusion runner to fuse front and rear lidar
    fuser_runner = launch_ros.actions.Node(
        package='point_cloud_fusion_nodes',
        executable='pointcloud_fusion_node_exe',
        namespace='lidars',
        parameters=[get_param_file('point_cloud_fusion_nodes',
                                   'vlp16_sim_lexus_pc_fusion.param.yaml')],
        remappings=[
            ("output_topic", "points_filtered"),
            ("input_topic1", "/lidar_front/points_filtered"),
            ("input_topic2", "/lidar_rear/points_filtered")
        ])

    lgsvl_interface = launch_ros.actions.Node(
        package='lgsvl_interface',
        executable='lgsvl_interface_exe',
        namespace='vehicle',
        output='screen',
        parameters=[get_param_file('autoware_demos', 'lgsvl_interface.param.yaml')],
        remappings=[
            ("gnss_odom", "/lgsvl/gnss_odom"),
            ("vehicle_odom", "/lgsvl/vehicle_odom")
        ])

    # Setup robot state publisher
    vehicle_description_pkg_path = get_package_share_directory(
        'lexus_rx_450h_description')
    urdf_path = os.path.join(
        vehicle_description_pkg_path,
        'urdf',
        # 'lexus_rx_450h.urdf'
        'navi_car.urdf'
    )
    print("urdf path:",urdf_path)
    with open(urdf_path, 'r') as infp:
        urdf_file = infp.read()
    robot_state_publisher_runner = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': urdf_file}],
    )

    # Run rviz
    examples_pkg_path = get_package_share_directory(
        'autoware_demos')
    rviz_cfg_path = os.path.join(
        examples_pkg_path,
        'rviz2',
        'lidar_bounding_boxes_lgsvl.rviz'
    )
    rviz_runner = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(rviz_cfg_path)])

    return launch.LaunchDescription([
        filter_transform_front_runner,
        ray_ground_runner,
        # lgsvl_interface,
        euclidean_cluster_node_runner,
        robot_state_publisher_runner,
        rviz_runner
        ])
