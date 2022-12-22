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

"""Launch gaode api global planner."""

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
    gaode_api_global_planner_node_runner = launch_ros.actions.Node(
        package='gaode_api_global_planner',
        executable='gaode_api_global_planner_node_exe',
        namespace='gaode_api',
        )


    return launch.LaunchDescription([
        gaode_api_global_planner_node_runner
        ])
