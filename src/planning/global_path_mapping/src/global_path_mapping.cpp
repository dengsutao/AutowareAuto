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

#include "global_path_mapping/global_path_mapping.hpp"

#include <iostream>

namespace autoware
{
namespace planning
{
int32_t global_path_mapping::print_hello()
{
  std::cout << "Hello World" << std::endl;
  return 0;
}
}  // namespace planning
}  // namespace autoware