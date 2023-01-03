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

#include "global_path_mapping/global_path_mapping_node.hpp"

#include <algorithm>
#include <deque>
#include <memory>
#include <string>
#include <utility>

#include <deque>
#include <vector>
#include <iostream>
#include <chrono>

using namespace std::chrono_literals;

namespace autoware
{
namespace planning
{
namespace global_path_mapping
{
float64_t distance(Pose pose1, Pose pose2)
{
  return sqrt(pow(pose1.position.x-pose2.position.x,2)+
              pow(pose1.position.y-pose2.position.y,2)+
              pow(pose1.position.z-pose2.position.z,2));
}
float64_t distance(Eigen::Vector2d pose1, Eigen::Vector2d pose2)
{
  return (pose1-pose2).norm();
}
Eigen::Vector2d direction(Point from, Point to)
{
  Eigen::Vector2d result;
  result.x() = to.x - from.x;
  result.y() = to.y - from.y;
  result.normalize();
  return result;
}
// right direction
Eigen::Vector2d normal_direction(Eigen::Vector2d diretion)
{
  Eigen::Vector2d result = diretion;
  result.normalize();
  // 顺时针转九十度就是右手法向方向
  float64_t theta = -M_PI/2;
  auto x = result.x(), y = result.y();
  result.x() = cos(theta)*x-sin(theta)*y;
  result.y() = sin(theta)*x+cos(theta)*y;
  return result;
}
const Pose to_pose(
  Eigen::Vector2d vec)
{
  Pose pose;
  pose.position.x = vec.x();
  pose.position.y = vec.y();
  return pose;
}
void tf_transform_pose(Pose & pose, TransformStamped & tf)
{
  PoseStamped ps_in, ps_out;
  ps_in.pose = pose;
  tf2::doTransform(ps_in, ps_out, tf);
  pose = ps_out.pose;
}

TransformStamped tf_inverse(TransformStamped & tf)
{
  tf2::Stamped<tf2::Transform> stamped_transform;
  tf2::fromMsg(tf, stamped_transform);
  TransformStamped out;
  out.header = tf.header;
  out.child_frame_id = tf.child_frame_id;
  out.transform = tf2::toMsg(stamped_transform.inverse());
  return out;
}
GlobalPathMappingNode::GlobalPathMappingNode(const rclcpp::NodeOptions & options)
:  Node("global_path_mapping", options)
{
  // declare_parameter<float64_t>("distance_th", 5.0);
  declare_parameter<float64_t>("len_forward_per_step", 2.0);
  // declare_parameter<float64_t>("distance_obj_th", 1.0);
  // debug_occupancy_grid_publisher_ =
  //   this->create_publisher<nav_msgs::msg::OccupancyGrid>("/debug/occupancy_grid", 1);
  // Action server
  mapping_action_server_ = rclcpp_action::create_server<GlobalPathMappingAction>(
    this->get_node_base_interface(), this->get_node_clock_interface(),
    this->get_node_logging_interface(), this->get_node_waitables_interface(), "global_path_mapping_service",
    [this](auto uuid, auto goal) {return this->handleGoal(uuid, goal);},
    [this](auto goal_handle) {return this->handleCancel(goal_handle);},
    [this](auto goal_handle) {return this->handleAccepted(goal_handle);});
}
rclcpp_action::GoalResponse GlobalPathMappingNode::handleGoal(
  const rclcpp_action::GoalUUID &,
  const std::shared_ptr<const GlobalPathMappingAction::Goal>)
{
  RCLCPP_INFO(get_logger(), "Received new global path mapping request.");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GlobalPathMappingNode::handleCancel(
  const std::shared_ptr<
    rclcpp_action::ServerGoalHandle<GlobalPathMappingAction>>)
{
  RCLCPP_WARN(get_logger(), "Received action cancellation, rejecting.");
  return rclcpp_action::CancelResponse::REJECT;
}

void GlobalPathMappingNode::handleAccepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<GlobalPathMappingAction>>
  goal_handle)
{
  RCLCPP_INFO(get_logger(), "handleAccepted for global path mapping request.");


  auto feedback = std::make_shared<GlobalPathMappingAction::Feedback>();
  auto result = std::make_shared<GlobalPathMappingAction::Result>();

  goal_handle_ = goal_handle;
  cur_grid_ = goal_handle_->get_goal()->occupancy_grid;
  cur_route_ = goal_handle_->get_goal()->gaode_api_route;
  cur_tf_ = goal_handle_->get_goal()->tf_odom2baselink;

  //publish 
  // RCLCPP_INFO(get_logger(), "occupancy_grid pose:"+
  //     std::to_string(cur_grid_.info.origin.position.x)+","+
  //     std::to_string(cur_grid_.info.origin.position.y)+","+
  //     std::to_string(cur_grid_.info.origin.position.z));
  // debug_occupancy_grid_publisher_->publish(cur_grid_);

  // first, do transform to each gps point(odom->base_link)
  for (auto it = cur_route_.enu_route.begin();it!=cur_route_.enu_route.end();it++)
  {
    Pose & pose_i = *it;
    tf_transform_pose(pose_i, cur_tf_);
  }

  // cur_pose_ should always be [0,0,0], in base_link frame
  map_w = static_cast<size_t>(cur_grid_.info.width);
  map_h = static_cast<size_t>(cur_grid_.info.height);
  resolution = static_cast<float64_t>(cur_grid_.info.resolution);

  Eigen::Vector2d cur_pose{0,0};
  Eigen::Vector2d target_pose;

  // direction and normal direction
  Eigen::Vector2d cur_route_direction = direction(cur_route_.enu_route[pose_id_].position, cur_route_.enu_route[pose_id_+1].position);
  Eigen::Vector2d cur_route_direction_right = normal_direction(cur_route_direction);
  Eigen::Vector2d cur_route_direction_left = -cur_route_direction_right;
  // Eigen::Vector2d cur_route_direction_normal = normal_direction(cur_route_direction);

  target_pose.x() = cur_pose.x() + cur_route_direction.x()*get_parameter("len_forward_per_step").as_double();
  target_pose.y() = cur_pose.y() + cur_route_direction.y()*get_parameter("len_forward_per_step").as_double();
  
  Eigen::Vector2d front_pose, right_pose, left_pose, final_pose;
  float64_t front_dist, right_dist, left_dist;
  float64_t min_dist=1e8;
  bool is_find=false;
  if (find_no_obj_with_direction(target_pose, front_pose, cur_route_direction, front_dist, 1) && min_dist>front_dist)
  {
    min_dist = front_dist;
    final_pose = front_pose;
    is_find = true;
  }
  if (find_no_obj_with_direction(target_pose, right_pose, cur_route_direction_right, right_dist, 1) && min_dist>right_dist)
  {
    min_dist = right_dist;
    final_pose = right_pose;
    is_find = true;
  }
  if (find_no_obj_with_direction(target_pose, left_pose, cur_route_direction_left, left_dist, 1) && min_dist>left_dist)
  {
    min_dist = left_dist;
    final_pose = left_pose;
    is_find = true;
  }
  if (!is_find)
  {
    RCLCPP_WARN(get_logger(), "cannot find free space for target pose!");
    goal_handle_->abort(result);
    return;
  }
  target_pose = final_pose;
  
  auto inverse_tf = tf_inverse(cur_tf_);
  feedback->feedback.start_enu_pose = to_pose(cur_pose);
  tf_transform_pose(feedback->feedback.start_enu_pose, inverse_tf);
  feedback->feedback.end_enu_pose = to_pose(target_pose);
  tf_transform_pose(feedback->feedback.end_enu_pose, inverse_tf);
  feedback->feedback.enu_route.push_back(feedback->feedback.start_enu_pose);
  feedback->feedback.enu_route.push_back(feedback->feedback.end_enu_pose);
  goal_handle_->publish_feedback(feedback);
  goal_handle_->succeed(result);
  RCLCPP_INFO(get_logger(),
              "start x,y:"+
              std::to_string(feedback->feedback.start_enu_pose.position.x)+","+
              std::to_string(feedback->feedback.start_enu_pose.position.y)+","+
              "end x,y:"+
              std::to_string(feedback->feedback.end_enu_pose.position.x)+","+
              std::to_string(feedback->feedback.end_enu_pose.position.y)+",");
  return;
}

bool8_t GlobalPathMappingNode::find_no_obj_with_direction(
  Eigen::Vector2d & start_point, 
  Eigen::Vector2d & result_point, 
  Eigen::Vector2d & direction,
  float64_t & find_dist,
  int32_t step
)
{
  // find first object point in normal direction
  float64_t x_offset = static_cast<float64_t>(map_w)/2;
  float64_t y_offset = static_cast<float64_t>(map_h)/2;
  float64_t start_x = start_point.x()/resolution+x_offset;
  float64_t start_y = start_point.y()/resolution+y_offset;
  float64_t x = start_x, y = start_y;
  size_t int_x = static_cast<size_t>(x);
  size_t int_y = static_cast<size_t>(y);
  // size_t start_x = int_x, start_y = int_y;
  // cur point not object
  if (cur_grid_.data[int_y*map_w+int_x]==100)
  {
    result_point.x() = start_point.x();
    result_point.y() = start_point.y();
    find_dist = 0;
    return true;
  }

  int32_t cur_dist = step;
  while(x>=0 && x<static_cast<float64_t>(map_w) && 
        y>=0 && y<static_cast<float64_t>(map_h))
  {
    x = start_x + direction.x() * cur_dist;
    y = start_y + direction.y() * cur_dist;
    int_x = static_cast<size_t>(x);
    int_y = static_cast<size_t>(y);
    if (cur_grid_.data[int_y*map_w+int_x]==100)
    {
      result_point.x() = (static_cast<float64_t>(int_x)-x_offset)*resolution;
      result_point.y() = (static_cast<float64_t>(int_y)-y_offset)*resolution;
      find_dist = static_cast<float64_t>(cur_dist)*resolution;
      return true;
    }
    cur_dist += step;
  }
  find_dist = -1;
  return false;
  
}

bool8_t GlobalPathMappingNode::find_object_with_direction(
  Eigen::Vector2d & start_point, 
  Eigen::Vector2d & result_point, 
  Eigen::Vector2d & direction,
  int32_t step
)
{
  // find first object point in normal direction
  float64_t x_offset = static_cast<float64_t>(map_w)/2;
  float64_t y_offset = static_cast<float64_t>(map_h)/2;
  float64_t x = start_point.x()/resolution+x_offset;
  float64_t y = start_point.y()/resolution+y_offset;
  size_t int_x = static_cast<size_t>(x);
  size_t int_y = static_cast<size_t>(y);
  size_t start_x = int_x, start_y = int_y;
  int_y = static_cast<size_t>(y);
  bool label = false;
  while(x>=0 && x<static_cast<float64_t>(map_w) && 
        y>=0 && y<static_cast<float64_t>(map_h))
  {
    x += direction.x() * step;
    y += direction.y() * step;
    int_x = static_cast<size_t>(x);
    int_y = static_cast<size_t>(y);
    if (cur_grid_.data[int_y*map_w+int_x]==0)
    {
      label = true;
      break;
    }
  }
  if (label)
  {
    result_point.x() = (static_cast<float64_t>(int_x)-x_offset)*resolution;
    result_point.y() = (static_cast<float64_t>(int_y)-y_offset)*resolution;
    return true;
  }
  else
  {
    // if we cannot find right hand object, we dont move!
    result_point.x() = (static_cast<float64_t>(start_x)-x_offset)*resolution;
    result_point.y() = (static_cast<float64_t>(start_y)-y_offset)*resolution;
    RCLCPP_WARN(get_logger(), "no matched point in current position! cannot find object in right hand normal direction!");
    return false;
  }
  
}

}  // namespace global_path_mapping
}  // namespace planning
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::planning::global_path_mapping::GlobalPathMappingNode)
