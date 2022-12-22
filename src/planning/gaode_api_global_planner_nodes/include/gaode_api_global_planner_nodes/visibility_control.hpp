// Copyright 2019 the Autoware Foundation
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

#ifndef GAODE_API_GLOBAL_PLANNER_NODES__VISIBILITY_CONTROL_HPP_
#define GAODE_API_GLOBAL_PLANNER_NODES__VISIBILITY_CONTROL_HPP_

#if defined(__WIN32)
  #if defined(GAODE_API_GLOBAL_PLANNER_NODES_BUILDING_DLL) || \
  defined(GAODE_API_GLOBAL_PLANNER_NODES_EXPORTS)
    #define GAODE_API_GLOBAL_PLANNER_NODES_PUBLIC __declspec(dllexport)
    #define GAODE_API_GLOBAL_PLANNER_NODES_LOCAL
  #else
    #define GAODE_API_GLOBAL_PLANNER_NODES_PUBLIC __declspec(dllimport)
    #define GAODE_API_GLOBAL_PLANNER_NODES_LOCAL
  #endif
#elif defined(__linux__)
  #define GAODE_API_GLOBAL_PLANNER_NODES_PUBLIC __attribute__((visibility("default")))
  #define GAODE_API_GLOBAL_PLANNER_NODES_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define GAODE_API_GLOBAL_PLANNER_NODES_PUBLIC __attribute__((visibility("default")))
  #define GAODE_API_GLOBAL_PLANNER_NODES_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif
#endif  // GAODE_API_GLOBAL_PLANNER__VISIBILITY_CONTROL_HPP_
