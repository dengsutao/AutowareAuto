// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.

#include "freespace_planner/base_planning_algorithm.hpp"

#include <vector>

namespace autoware
{
namespace planning
{
namespace freespace_planner
{

double normalizeRadian(const double rad, const double min_rad)
{
  double value = autoware::common::helper_functions::wrap_angle(rad);
  if (value < min_rad) {
    value += 2 * M_PI;
  }
  return value;
}

int discretizeAngle(const double theta, const size_t theta_size)
{
  const double angle_increment_rad = 2.0 * M_PI / static_cast<double>(theta_size);
  return static_cast<int>(normalizeRadian(theta, 0.0) / angle_increment_rad) %
         static_cast<int>(theta_size);
}

geometry_msgs::msg::Pose transformPose(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::TransformStamped & transform)
{
  geometry_msgs::msg::PoseStamped transformed_pose;
  geometry_msgs::msg::PoseStamped pose_orig;
  pose_orig.pose = pose;
  tf2::doTransform(pose_orig, transformed_pose, transform);

  return transformed_pose.pose;
}

geometry_msgs::msg::Pose global2local(
  const nav_msgs::msg::OccupancyGrid & costmap, const geometry_msgs::msg::Pose & pose_global)
{
  tf2::Transform tf_origin;
  tf2::convert(costmap.info.origin, tf_origin);

  geometry_msgs::msg::TransformStamped transform;
  transform.transform = tf2::toMsg(tf_origin.inverse());

  return transformPose(pose_global, transform);
}

geometry_msgs::msg::Pose local2global(
  const nav_msgs::msg::OccupancyGrid & costmap, const geometry_msgs::msg::Pose & pose_local)
{
  tf2::Transform tf_origin;
  tf2::convert(costmap.info.origin, tf_origin);

  geometry_msgs::msg::TransformStamped transform;
  transform.transform = tf2::toMsg(tf_origin);

  return transformPose(pose_local, transform);
}

IndexXYT pose2index(
  const geometry_msgs::msg::Pose & pose_local,
  const float & costmap_resolution,
  const size_t theta_size)
{
  const auto resolution = static_cast<double>(costmap_resolution);
  const int index_x = static_cast<int>(std::floor(pose_local.position.x / resolution));
  const int index_y = static_cast<int>(std::floor(pose_local.position.y / resolution));
  const int index_theta = discretizeAngle(tf2::getYaw(pose_local.orientation), theta_size);
  return {index_x, index_y, index_theta};
}

geometry_msgs::msg::Pose index2pose(
  const IndexXYT & index, const float & costmap_resolution, const size_t theta_size)
{
  geometry_msgs::msg::Pose pose_local;

  pose_local.position.x = static_cast<float>(index.x) * costmap_resolution;
  pose_local.position.y = static_cast<float>(index.y) * costmap_resolution;

  const double angle_increment_rad = 2.0 * M_PI / static_cast<double>(theta_size);
  const double yaw = index.theta * angle_increment_rad;
  tf2::Quaternion quat;
  quat.setRPY(0, 0, yaw);
  tf2::convert(quat, pose_local.orientation);

  return pose_local;
}

void BasePlanningAlgorithm::setOccupancyGrid(const nav_msgs::msg::OccupancyGrid & costmap)
{
  costmap_ = costmap;
  const auto height = costmap_.info.height;
  const auto width = costmap_.info.width;

  // Initialize status
  std::vector<std::vector<bool>> is_obstacle_table;
  is_obstacle_table.resize(height);
  for (uint32_t i = 0; i < height; i++) {
    is_obstacle_table.at(i).resize(width);
    for (uint32_t j = 0; j < width; j++) {
      const int cost = costmap_.data[i * width + j];

      // if (cost < 0 || planner_common_param_.obstacle_threshold <= cost) {
      //   is_obstacle_table[i][j] = true;
      // }
      if (cost <= 0) {
        is_obstacle_table[i][j] = true;
      }
    }
  }
  is_obstacle_table_ = is_obstacle_table;

  // construct collision indexes table
  coll_indexes_table_.clear();
  for (int i = 0; i < static_cast<int>(planner_common_param_.theta_size); i++) {
    std::vector<IndexXY> indexes_2d;
    computeCollisionIndexes(i, indexes_2d);
    coll_indexes_table_.push_back(indexes_2d);
  }
}


bool BasePlanningAlgorithm::hasObstacleOnTrajectory(
  const geometry_msgs::msg::PoseArray & trajectory) const
{
  for (const auto & pose : trajectory.poses) {
    const auto pose_local = global2local(costmap_, pose);
    const auto index = pose2index(
      pose_local, costmap_.info.resolution, planner_common_param_.theta_size);

    if (detectCollision(index)) {
      return true;
    }
  }

  return false;
}

void BasePlanningAlgorithm::computeCollisionIndexes(
  int theta_index, std::vector<IndexXY> & indexes_2d)
{
  IndexXYT base_index{0, 0, theta_index};
  const VehicleShape & vehicle_shape = planner_common_param_.vehicle_shape;

  // Define the robot as rectangle
  const double back = -1.0 * vehicle_shape.cg2back;
  const double front = vehicle_shape.length - vehicle_shape.cg2back;
  const double right = -1.0 * vehicle_shape.width / 2.0;
  const double left = vehicle_shape.width / 2.0;

  const auto base_pose = index2pose(
    base_index, costmap_.info.resolution, planner_common_param_.theta_size);
  const auto base_theta = tf2::getYaw(base_pose.orientation);

  // Convert each point to index and check if the node is Obstacle
  const auto costmap_resolution = static_cast<double>(costmap_.info.resolution);
  for (double x = back; x <= front; x += costmap_resolution) {
    for (double y = right; y <= left; y += costmap_resolution) {
      // Calculate offset in rotated frame
      const double offset_x = std::cos(base_theta) * x - std::sin(base_theta) * y;
      const double offset_y = std::sin(base_theta) * x + std::cos(base_theta) * y;

      geometry_msgs::msg::Pose pose_local;
      pose_local.position.x = base_pose.position.x + offset_x;
      pose_local.position.y = base_pose.position.y + offset_y;

      const auto index = pose2index(
        pose_local, costmap_.info.resolution, planner_common_param_.theta_size);
      const auto index_2d = IndexXY{index.x, index.y};
      indexes_2d.push_back(index_2d);
    }
  }
}

bool BasePlanningAlgorithm::detectCollision(const IndexXYT & base_index) const
{
  const auto & coll_indexes_2d = coll_indexes_table_[static_cast<size_t>(base_index.theta)];
  for (const auto & coll_index_2d : coll_indexes_2d) {
    int idx_theta = 0;  // whatever. Yaw is nothing to do with collision detection between grids.
    IndexXYT coll_index{coll_index_2d.x, coll_index_2d.y, idx_theta};
    // must slide to current base position
    coll_index.x += base_index.x;
    coll_index.y += base_index.y;

    if (isOutOfRange(coll_index) || isObs(coll_index)) {
      return true;
    }
  }
  return false;
}


}  // namespace freespace_planner
}  // namespace planning
}  // namespace autoware
