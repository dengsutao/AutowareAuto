# Copyright 2020-2022, The Autoware Foundation
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

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


import os


def generate_launch_description():
    f1tenth_launch_pkg_prefix = get_package_share_directory('f1tenth_launch')

    localization_param_file = os.path.join(
        f1tenth_launch_pkg_prefix, 'param/slam_localization.param.yaml')
    localization_param = DeclareLaunchArgument(
        'localization_param_file',
        default_value=localization_param_file,
        description='Path to config file for localization nodes'
    )

    map_file_path = os.path.join(
        f1tenth_launch_pkg_prefix, 'data/red_bull_ring_racetrack.posegraph')
    map_file = DeclareLaunchArgument(
        'map',
        default_value=map_file_path,
        description='Path to .posegraph map serialized by slam_toolbox'
    )

    print(LaunchConfiguration("map_file"))
    slam_launch = Node(
        parameters=[
            LaunchConfiguration('localization_param_file'),
            {'map_file_name': LaunchConfiguration('map')}
        ],
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    return LaunchDescription([
        localization_param,
        map_file,
        slam_launch,
    ])
