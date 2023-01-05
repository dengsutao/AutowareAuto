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

#ifndef PURE_PURSUIT_NODES__PURE_PURSUIT_NODE_HPP_
#define PURE_PURSUIT_NODES__PURE_PURSUIT_NODE_HPP_

#include <controller_common_nodes/controller_base_node.hpp>
#include <pure_pursuit/pure_pursuit.hpp>

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "motion_common/motion_common.hpp"
#include "motion_common/trajectory_common.hpp"

#include "trajectory_follower/longitudinal_controller_utils.hpp"
#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "pure_pursuit_nodes/visibility_control.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "trajectory_follower/smooth_stop.hpp"

#include "pure_pursuit/pure_pursuit.hpp"

namespace autoware
{
namespace motion
{
namespace control
{
/// \brief Boilerplate Apex.OS nodes around pure_pursuit
namespace pure_pursuit_nodes
{
using autoware::common::types::float64_t;
namespace motion_common = ::autoware::motion::motion_common;
using ::autoware::motion::control::pure_pursuit::PurePursuit;
using TrajectoryPointStamped = autoware_auto_vehicle_msgs::msg::VehicleKinematicState;

/// \brief Boilerplate node that subscribes to the current pose and
/// publishes a vehicle control command
class PURE_PURSUIT_NODES_PUBLIC PurePursuitNode
  : public ::motion::control::controller_common_nodes::ControllerBaseNode
{
public:
  /// \brief Parameter constructor
  /// \param[in] node_name Name of the node, controls which parameter set from the file is matched
  /// \param[in] node_namespace Name of the node's namespace, controls which parameters are used
  PurePursuitNode(
    const std::string & node_name,
    const std::string & node_namespace = "");

  /// \brief Explicit constructor
  /// \param[in] node_name Name of the node
  /// \param[in] cfg Configuration object for PurePursuit
  /// \param[in] node_namespace Namespace of this node
  PurePursuitNode(
    const std::string & node_name,
    const pure_pursuit::Config & cfg,
    const std::string & node_namespace = "");
  
  // timer callback
  float64_t m_control_rate;
  float64_t m_emergency_state_traj_trans_dev;
  float64_t m_emergency_state_traj_rot_dev;
  float64_t m_emergency_acceleration;

  std::unique_ptr<PurePursuit> m_ppcontroller_;

  rclcpp::TimerBase::SharedPtr m_timer_control;

  // for calculating dt
  std::shared_ptr<rclcpp::Time> m_prev_control_time{nullptr};

  // ros variables
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VehicleKinematicState>::SharedPtr
    m_sub_current_state;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr m_sub_trajectory;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
    m_pub_control_cmd;

  
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // pointers for ros topic
  std::shared_ptr<autoware_auto_vehicle_msgs::msg::VehicleKinematicState> m_current_state_ptr{
    nullptr};
  std::shared_ptr<autoware_auto_vehicle_msgs::msg::VehicleKinematicState> m_prev_state_ptr{nullptr};
  std::shared_ptr<autoware_auto_planning_msgs::msg::Trajectory> m_trajectory_ptr{nullptr};

  struct Motion
  {
    float64_t vel{0.0};//longitudinal speed
    float64_t acc{0.0};//longitudinal acceleration
    float64_t ang_z{0.0};//roation z speed
  };

  enum class Shift { Forward = 0, Reverse };
    
  // shift mode
  Shift m_prev_shift{Shift::Forward};

  struct ControlData
  {
    bool8_t is_far_from_trajectory{false};
    size_t nearest_idx{0};  // nearest_idx = 0 when nearest_idx is not found with findNearestIdx
    Motion current_motion{};
    Shift shift{Shift::Forward};  // shift is used only to calculate the sign of pitch compensation
    float64_t stop_dist{0.0};  // signed distance that is positive when car is before the stopline
    float64_t slope_angle{0.0};
    float64_t dt{0.0};
  };

  // delay compensation
  float64_t m_delay_compensation_time;

  // enable flags
  bool8_t m_enable_smooth_stop;
  bool8_t m_enable_overshoot_emergency;

  // smooth stop
  trajectory_follower::SmoothStop m_smooth_stop;
  
  // smooth stop transition
  struct StateTransitionParams
  {
    // drive
    float64_t drive_state_stop_dist;
    float64_t drive_state_offset_stop_dist;
    // stopping
    float64_t stopping_state_stop_dist;
    // stop
    float64_t stopped_state_entry_vel;
    float64_t stopped_state_entry_acc;
    // emergency
    float64_t emergency_state_overshoot_stop_dist;
    float64_t emergency_state_traj_trans_dev;
    float64_t emergency_state_traj_rot_dev;
    float64_t emergency_acceleration;
  };
  StateTransitionParams m_state_transition_params;

   // stop
  struct StoppedStateParams
  {
    float64_t vel;
    float64_t acc;
    float64_t jerk;
  };
  StoppedStateParams m_stopped_state_params;
  
  // control state
  enum class ControlState { DRIVE = 0, STOPPING, STOPPED, EMERGENCY };
  ControlState m_control_state{ControlState::STOPPED};

  // diff limit
  Motion m_prev_ctrl_cmd{};      // with slope compensation
  Motion m_prev_raw_ctrl_cmd{};  // without slope compensation
  std::vector<std::pair<rclcpp::Time, float64_t>> m_vel_hist;

  std::shared_ptr<rclcpp::Time> m_last_running_time{std::make_shared<rclcpp::Time>(this->now())};

  // buffer of send command
  // std::vector<geometry_msgs::msg::Twist> m_ctrl_cmd_vec;

  /**
   * @brief compute control command, and publish periodically
   */
  void callbackTimerControl();

  /**
   * @brief set current and previous velocity with received message
   * @param [in] msg current state message
   */
  void callbackCurrentState(
    const autoware_auto_vehicle_msgs::msg::VehicleKinematicState::ConstSharedPtr msg);

  /**
   * @brief set reference trajectory with received message
   * @param [in] msg trajectory message
   */
  void callbackTrajectory(const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr msg);

  /**
   * @brief calculate data for controllers whose type is ControlData
   * @param [in] current_pose current ego pose
   */
  ControlData getControlData(const geometry_msgs::msg::Pose & current_pose);

  /**
   * @brief calculate time between current and previous one
   */
  float64_t getDt();

  /**
   * @brief calculate current velocity and acceleration
   */
  Motion getCurrentMotion() const;

  /**
   * @brief calculate direction (forward or backward) that vehicle moves
   * @param [in] nearest_idx nearest index on trajectory to vehicle
   */
  enum Shift getCurrentShift(const size_t nearest_idx) const;

  /**
   * @brief calculate control command in emergency state
   * @param [in] dt time between previous and current one
   */
  Motion calcEmergencyCtrlCmd(const float64_t dt) const;

  /**
   * @brief publish control command
   * @param [in] ctrl_cmd calculated control command to control velocity
   * @param [in] current_vel current velocity of the vehicle
   */
  void publishCtrlCmd(const Motion & ctrl_cmd, const float64_t current_vel);

  /**
   * @brief update control state according to the current situation
   * @param [in] current_control_state current control state
   * @param [in] control_data control data
   */
  ControlState updateControlState(
    const ControlState current_control_state, const ControlData & control_data);
  
  /**
   * @brief calculate predicted velocity after time delay based on past control commands
   * @param [in] current_motion current velocity and acceleration of the vehicle
   * @param [in] delay_compensation_time predicted time delay
   */
  float64_t predictedVelocityInTargetPoint(
    const Motion current_motion, const float64_t delay_compensation_time) const;
  
  /**
   * @brief calculate control command based on the current control state
   * @param [in] current_control_state current control state
   * @param [in] current_pose current ego pose
   * @param [in] control_data control data
   */
  Motion calcCtrlCmd(
    const ControlState & current_control_state, const TrajectoryPointStamped & current_pose,
    const ControlData & control_data);

};
}  // namespace pure_pursuit_nodes
}  // namespace control
}  // namespace motion
}  // namespace autoware

#endif  // PURE_PURSUIT_NODES__PURE_PURSUIT_NODE_HPP_
