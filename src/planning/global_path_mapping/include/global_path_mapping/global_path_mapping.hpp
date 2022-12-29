// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the global_path_mapping class.

#ifndef GLOBAL_PATH_MAPPING__GLOBAL_PATH_MAPPING_HPP_
#define GLOBAL_PATH_MAPPING__GLOBAL_PATH_MAPPING_HPP_

#include <global_path_mapping/visibility_control.hpp>
#include <gaode_api_global_planner_nodes/gaode_api_global_planner_node.hpp>
#include <gaode_api_route_msgs/msg/gaode_api_route.hpp>
#include <cstdint>

namespace autoware
{
namespace planning
{
/// \brief TODO(dengsutao1996): Document namespaces!
namespace global_path_mapping
{

/// \brief TODO(dengsutao1996): Document your functions
int32_t GLOBAL_PATH_MAPPING_PUBLIC print_hello();


}  // namespace global_path_mapping
}  // namespace planning
}  // namespace autoware

#endif  // GLOBAL_PATH_MAPPING__GLOBAL_PATH_MAPPING_HPP_
