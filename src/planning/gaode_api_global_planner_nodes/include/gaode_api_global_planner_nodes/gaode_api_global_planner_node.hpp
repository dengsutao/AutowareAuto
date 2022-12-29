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

#ifndef  GAODE_API_GLOBAL_PLANNER_NODES__GAODE_API_GLOBAL_PLANNER_NODE_HPP_
#define  GAODE_API_GLOBAL_PLANNER_NODES__GAODE_API_GLOBAL_PLANNER_NODE_HPP_
// ros2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>
#include <time_utils/time_utils.hpp>

// autoware
#include <gaode_api_global_planner_nodes/visibility_control.hpp>
#include <common/types.hpp>
#include <autoware_auto_mapping_msgs/srv/had_map_service.hpp>
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_planning_msgs/msg/had_map_route.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_kinematic_state.hpp>
#include <had_map_utils/had_map_conversion.hpp>
#include <common/types.hpp>

// c++
#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <cmath>

// msg
#include <gaode_api_route_msgs/msg/gaode_api_route.hpp>
#include "sensor_msgs/msg/nav_sat_fix.hpp"

// geographiclib
#include "GeographicLib/LocalCartesian.hpp"

// eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

// json
#include <jsoncpp/json/json.h>
// cpr
#include <cpr/cpr.h>

using namespace std::chrono_literals;
using autoware::common::types::bool8_t;
using autoware::common::types::float64_t;
using std::vector;

namespace autoware
{
namespace planning
{
namespace gaode_api_global_planner_nodes
{
class GAODE_API_GLOBAL_PLANNER_NODES_PUBLIC GaodeApiGlobalPlannerNode : public rclcpp::Node
{
public:
  explicit GaodeApiGlobalPlannerNode(const rclcpp::NodeOptions & node_options);

  void goal_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  // void current_pose_cb(const autoware_auto_vehicle_msgs::msg::VehicleKinematicState::SharedPtr msg);
  void current_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void send_global_path(
    const geometry_msgs::msg::Pose & start_enu_pose, 
    const geometry_msgs::msg::Pose & end_enu_pose, 
    const vector<geometry_msgs::msg::Pose> & enu_route,
    const sensor_msgs::msg::NavSatFix & start_gps_pose, 
    const sensor_msgs::msg::NavSatFix & end_gps_pose, 
    const vector<sensor_msgs::msg::NavSatFix> & gps_route,
    const std_msgs::msg::Header & header);
  bool8_t transform_pose_to_odom(
    const geometry_msgs::msg::PoseStamped & pose_in, geometry_msgs::msg::PoseStamped & pose_out);
  
  bool8_t plan_route(
    geometry_msgs::msg::Pose & start_enu_pose, 
    geometry_msgs::msg::Pose & end_enu_pose, 
    vector<geometry_msgs::msg::Pose> & enu_route,
    sensor_msgs::msg::NavSatFix & start_gps_pose, 
    sensor_msgs::msg::NavSatFix & end_gps_pose, 
    vector<sensor_msgs::msg::NavSatFix> & gps_route);
  void to_enu(
    geometry_msgs::msg::Pose & enu_pose, sensor_msgs::msg::NavSatFix & gps_pose);
  void to_gps(
    geometry_msgs::msg::Pose & enu_pose, sensor_msgs::msg::NavSatFix & gps_pose);
  void init_gps_cb(
    const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void add_pose(
    geometry_msgs::msg::Pose & enu_pose, 
    sensor_msgs::msg::NavSatFix & gps_pose,
    vector<geometry_msgs::msg::Pose> & enu_route,
    vector<sensor_msgs::msg::NavSatFix> & gps_route);
  bool8_t to_gaode_gps(
    std::string & gps_str,
    bool8_t is_start);
  bool8_t visualize(
    const gaode_api_route_msgs::msg::GaodeApiRoute & msg);
  bool8_t parse(
    std::string & url, Json::Value & data);


private:
  // rclcpp::Client<autoware_auto_mapping_msgs::srv::HADMapService>::SharedPtr map_client;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_ptr;
  // rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VehicleKinematicState>::SharedPtr
  //   current_pose_sub_ptr;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
    current_pose_sub_ptr;
  rclcpp::Publisher<gaode_api_route_msgs::msg::GaodeApiRoute>::SharedPtr global_path_pub_ptr;
  geometry_msgs::msg::PoseStamped start_pose;
  geometry_msgs::msg::PoseStamped goal_pose;
  bool8_t start_pose_init;
  tf2::BufferCore tf_buffer;
  tf2_ros::TransformListener tf_listener;
  static GeographicLib::LocalCartesian geo_converter_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr init_gps_sub_ptr;
  vector<double> gps_offset;
  
};
}  // namespace gaode_api_global_planner_nodes
}  // namespace planning
}  // namespace autoware

#endif  // GAODE_API_GLOBAL_PLANNER_NODES__GAODE_API_GLOBAL_PLANNER_NODE_HPP_
