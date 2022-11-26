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

#include <string>
#include <memory>

#include "lgsvl_laser_adapter_nodes/lgsvl_laser_adapter_nodes.hpp"

using std::placeholders::_1;

namespace lgsvl_laser_adapter_nodes
{

LgsvlLaserAdapterNode::LgsvlLaserAdapterNode(const rclcpp::NodeOptions & options)
:  Node("lgsvl_laser_adapter_node", options)
{
  m_laser_pub =
    create_publisher<LaserScan>(
    "scan_out", rclcpp::QoS{10});
  m_laser_sub =
    create_subscription<LaserScan>(
    "scan_in", rclcpp::QoS{10},
    std::bind(&LgsvlLaserAdapterNode::on_laser, this, _1));
}


void LgsvlLaserAdapterNode::on_laser(const LaserScan::SharedPtr msg)
{
  msg->angle_max -= msg->angle_increment;
  m_laser_pub->publish(*msg);
}
}  // namespace lgsvl_laser_adapter_nodes

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(lgsvl_laser_adapter_nodes::LgsvlLaserAdapterNode)
