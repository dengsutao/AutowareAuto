# Copyright 2021 The Autoware Foundation
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
# Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.


import ament_index_python
import launch
import launch_ros.actions


def generate_launch_description():
    """Launch costmap_generator node with default configuration."""
    # -------------------------------- Nodes-----------------------------------
    #outputs:/control/output/control_cmd
    pure_pursuit_node = launch_ros.actions.Node(
        package='pure_pursuit_nodes',
        executable='pure_pursuit_node_exe',
        name='pure_pursuit',
        namespace='control',
        output='screen',
        parameters=[
            "{}/param/pure_pursuit.param.yaml".format(
                ament_index_python.get_package_share_directory(
                    "pure_pursuit_nodes"
                )
            ),
        ],
        remappings=[
            ("input/current_trajectory", "/planning/trajectory"),
            ("input/current_state", "/vehicle/vehicle_kinematic_state"),
        ]
    )

    ld = launch.LaunchDescription([pure_pursuit_node])
    return ld
