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
/// \brief This file defines the global_path_mapping_node class.

#ifndef GLOBAL_PATH_MAPPING__GLOBAL_PATH_MAPPING_NODE_HPP_
#define GLOBAL_PATH_MAPPING__GLOBAL_PATH_MAPPING_NODE_HPP_

#include <global_path_mapping/global_path_mapping.hpp>
#include <autoware_auto_planning_msgs/action/plan_trajectory.hpp>
#include <autoware_auto_planning_msgs/action/planner_costmap.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>

#include <global_path_mapping_action/action/global_path_mapping_action.hpp>
#include <gaode_api_route_msgs/msg/gaode_api_route.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2/LinearMath/Transform.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>


namespace autoware
{
namespace planning
{
namespace global_path_mapping
{
using GlobalPathMappingAction = global_path_mapping_action::action::GlobalPathMappingAction;
using GaodeApiRoute = gaode_api_route_msgs::msg::GaodeApiRoute;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using Pose = geometry_msgs::msg::Pose;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Point = geometry_msgs::msg::Point;
using Occupancy_Grid = nav_msgs::msg::OccupancyGrid;
/// \class GlobalPathMappingNode
/// \brief ROS 2 Node for hello world.
class GLOBAL_PATH_MAPPING_PUBLIC GlobalPathMappingNode : public rclcpp::Node
{
public:
  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  explicit GlobalPathMappingNode(const rclcpp::NodeOptions & options);
  

private:
  rclcpp_action::Server<GlobalPathMappingAction>::SharedPtr
    mapping_action_server_;

  // action
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const GlobalPathMappingAction::Goal>);
  rclcpp_action::CancelResponse handleCancel(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<GlobalPathMappingAction>>);
  void handleAccepted(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<GlobalPathMappingAction>>);
  bool8_t find_object_with_direction(
    Eigen::Vector2d & start_point, 
    Eigen::Vector2d & result_point, 
    Eigen::Vector2d & direction,
    int32_t step);
  bool8_t find_no_obj_with_direction(
    Eigen::Vector2d & start_point, 
    Eigen::Vector2d & result_point, 
    Eigen::Vector2d & direction,
    float64_t & find_dist,
    int32_t step
  );

  std::shared_ptr<rclcpp_action::ServerGoalHandle<GlobalPathMappingAction>> goal_handle_{nullptr};
  Occupancy_Grid cur_grid_;
  GaodeApiRoute cur_route_;
  TransformStamped cur_tf_;
  Pose cur_pose_;
  size_t pose_id_=0;
  size_t map_w;
  size_t map_h;
  float64_t resolution;
  Pose result_start_pose_;
  Pose result_end_pose_;

  // rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug_occupancy_grid_publisher_;

  
  
};
}  // namespace global_path_mapping
}  // namespace planning
}  // namespace autoware

#endif  // GLOBAL_PATH_MAPPING__GLOBAL_PATH_MAPPING_NODE_HPP_
