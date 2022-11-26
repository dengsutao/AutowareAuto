// Copyright 2022 The Autoware Foundation
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

/// \copyright Copyright 2022 The Autoware Foundation
/// \file
/// \brief This file defines the lgsvl_laser_adapter_nodes_node class.

#ifndef LGSVL_LASER_ADAPTER_NODES__LGSVL_LASER_ADAPTER_NODES_HPP_
#define LGSVL_LASER_ADAPTER_NODES__LGSVL_LASER_ADAPTER_NODES_HPP_

#include <memory>
#include <string>

#include "lgsvl_laser_adapter_nodes/visibility_control.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"

using sensor_msgs::msg::LaserScan;

namespace lgsvl_laser_adapter_nodes
{
/// \class LgsvlLaserAdapterNode
class LGSVL_LASER_ADAPTER_NODES_PUBLIC LgsvlLaserAdapterNode : public rclcpp::Node
{
public:
  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  explicit LgsvlLaserAdapterNode(const rclcpp::NodeOptions & options);

protected:
  rclcpp::Subscription<LaserScan>::SharedPtr m_laser_sub{};
  rclcpp::Publisher<LaserScan>::SharedPtr m_laser_pub{};

  LGSVL_LASER_ADAPTER_NODES_LOCAL void on_laser(const LaserScan::SharedPtr msg);
};
}  // namespace lgsvl_laser_adapter_nodes

#endif  // LGSVL_LASER_ADAPTER_NODES__LGSVL_LASER_ADAPTER_NODES_HPP_
