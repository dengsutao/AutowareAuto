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

#include <common/types.hpp>

#include <memory>
#include <string>
#include "motion_common/motion_common.hpp"

#include "time_utils/time_utils.hpp"
#include "pure_pursuit_nodes/pure_pursuit_node.hpp"

using autoware::common::types::float64_t;
using autoware::common::types::float32_t;
using autoware::common::types::bool8_t;

namespace autoware
{
namespace motion
{
namespace control
{
/// \brief Resources relating to the pure pursuit node package
namespace pure_pursuit_nodes
{
PurePursuitNode::PurePursuitNode(
  const std::string & node_name,
  const std::string & node_namespace)
: ControllerBaseNode{node_name, node_namespace, "legacy_ctrl_cmd", "legacy_current_pose",
    "legacy_tf", "legacy_trajectory", "legacy_ctrl_diag"}
{
  using std::placeholders::_1;
  
  pure_pursuit::Config cfg{
    static_cast<float32_t>(declare_parameter(
      "controller.minimum_lookahead_distance").get<float64_t>()),
    static_cast<float32_t>(declare_parameter(
      "controller.maximum_lookahead_distance").get<float64_t>()),
    static_cast<float32_t>(
      declare_parameter("controller.speed_to_lookahead_ratio").get<float64_t>()),
    declare_parameter("controller.is_interpolate_lookahead_point").get<bool8_t>(),
    declare_parameter("controller.is_delay_compensation").get<bool8_t>(),
    static_cast<float32_t>(declare_parameter(
      "controller.emergency_stop_distance").get<float64_t>()),
    static_cast<float32_t>(declare_parameter(
      "controller.speed_thres_traveling_direction").get<float64_t>()),
    static_cast<float32_t>(declare_parameter(
      "controller.maximum_acceleration").get<float64_t>()),
    static_cast<float32_t>(declare_parameter(
      "controller.dist_front_rear_wheels").get<float64_t>())};
  
  // m_ppcontroller = PurePursuit{cfg};
  // set_controller(std::make_unique<pure_pursuit::PurePursuit>(cfg));
  m_ppcontroller_ = std::make_unique<PurePursuit>(cfg);

  m_control_rate = declare_parameter("control_rate").get<float64_t>();
  m_enable_overshoot_emergency = declare_parameter("enable_overshoot_emergency").get<bool8_t>();
  m_enable_smooth_stop = declare_parameter("enable_smooth_stop").get<bool8_t>();
  m_delay_compensation_time = declare_parameter("delay_compensation_time").get<float64_t>();

  // parameters for smooth stop state
  {
    const float64_t max_strong_acc{declare_parameter(
        "smooth_stop_max_strong_acc").get<float64_t>()};         // [m/s^2]
    const float64_t min_strong_acc{declare_parameter(
        "smooth_stop_min_strong_acc").get<float64_t>()};         // [m/s^2]
    const float64_t weak_acc{declare_parameter(
        "smooth_stop_weak_acc").get<float64_t>()};               // [m/s^2]
    const float64_t weak_stop_acc{declare_parameter(
        "smooth_stop_weak_stop_acc").get<float64_t>()};          // [m/s^2]
    const float64_t strong_stop_acc{declare_parameter(
        "smooth_stop_strong_stop_acc").get<float64_t>()};        // [m/s^2]

    const float64_t max_fast_vel{declare_parameter(
        "smooth_stop_max_fast_vel").get<float64_t>()};            // [m/s]
    const float64_t min_running_vel{declare_parameter(
        "smooth_stop_min_running_vel").get<float64_t>()};        // [m/s]
    const float64_t min_running_acc{declare_parameter(
        "smooth_stop_min_running_acc").get<float64_t>()};        // [m/s^2]
    const float64_t weak_stop_time{declare_parameter(
        "smooth_stop_weak_stop_time").get<float64_t>()};          // [s]

    const float64_t weak_stop_dist{declare_parameter(
        "smooth_stop_weak_stop_dist").get<float64_t>()};         // [m]
    const float64_t strong_stop_dist{declare_parameter(
        "smooth_stop_strong_stop_dist").get<float64_t>()};       // [m]

    m_smooth_stop.setParams(
      max_strong_acc, min_strong_acc, weak_acc, weak_stop_acc, strong_stop_acc, max_fast_vel,
      min_running_vel, min_running_acc, weak_stop_time, weak_stop_dist, strong_stop_dist);
  }
  
  // parameters for state transition
  {
    auto & p = m_state_transition_params;
    // drive
    p.drive_state_stop_dist = declare_parameter("state_transition.drive_state_stop_dist").get<float64_t>();  // [m]
    p.drive_state_offset_stop_dist = declare_parameter(
      "state_transition.drive_state_offset_stop_dist").get<float64_t>();  // [m]
    // stopping
    p.stopping_state_stop_dist =
      declare_parameter("state_transition.stopping_state_stop_dist").get<float64_t>();  // [m]
    // stop
    p.stopped_state_entry_vel =
      declare_parameter("state_transition.stopped_state_entry_vel").get<float64_t>();  // [m/s]
    p.stopped_state_entry_acc =
      declare_parameter("state_transition.stopped_state_entry_acc").get<float64_t>();  // [m/s²]
    // emergency
    p.emergency_state_overshoot_stop_dist = declare_parameter(
      "state_transition.emergency_state_overshoot_stop_dist").get<float64_t>();  // [m]
    p.emergency_state_traj_trans_dev = declare_parameter(
      "state_transition.emergency_state_traj_trans_dev").get<float64_t>();  // [m]
    p.emergency_state_traj_rot_dev = declare_parameter(
      "state_transition.emergency_state_traj_rot_dev").get<float64_t>();  // [m]
     p.emergency_acceleration = declare_parameter(
      "state_transition.emergency_acceleration").get<float64_t>();  // [m/s²]
  }

  // parameters for stop state
  {
    auto & p = m_stopped_state_params;
    p.vel = declare_parameter("stopped_vel").get<float64_t>();   // [m/s]
    p.acc = declare_parameter("stopped_acc").get<float64_t>();  // [m/s^2]
    p.jerk = declare_parameter("stopped_jerk").get<float64_t>();  // [m/s^3]
  }

  // Timer
  auto timer_callback = std::bind(&PurePursuitNode::callbackTimerControl, this);
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<float64_t>(1.0 / m_control_rate));
  m_timer_control = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(m_timer_control, nullptr);


  // subscriber, publisher
  m_sub_current_state = create_subscription<autoware_auto_vehicle_msgs::msg::VehicleKinematicState>(
    "input/current_state", rclcpp::QoS{1},
    std::bind(&PurePursuitNode::callbackCurrentState, this, _1));
  m_sub_trajectory = create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
    "input/current_trajectory", rclcpp::QoS{1},
    std::bind(&PurePursuitNode::callbackTrajectory, this, _1));
  m_pub_control_cmd = create_publisher<geometry_msgs::msg::Twist>(
    "output/control_cmd", rclcpp::QoS{1});

  // TF
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *tf_buffer_, std::shared_ptr<rclcpp::Node>(this, [](auto) {}), false);
  }
}

void PurePursuitNode::callbackTimerControl()
{
  // wait for initial pointers
  if (
    !m_current_state_ptr || !m_prev_state_ptr || !m_trajectory_ptr ||
    !tf_buffer_->canTransform(
      m_trajectory_ptr->header.frame_id,
      "base_link",
      tf2::TimePointZero))
  {
    return;
  }
  
  // // transform state to the same frame as the trajectory
  geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
    m_trajectory_ptr->header.frame_id,
    "base_link",
    tf2::TimePointZero);
  autoware_auto_planning_msgs::msg::TrajectoryPoint current_state_tf;
  ::motion::motion_common::doTransform(m_current_state_ptr->state, current_state_tf, tf);
  // calculate current pose and control data
  geometry_msgs::msg::Pose current_pose;
  current_pose.position.x = tf.transform.translation.x;
  current_pose.position.y = tf.transform.translation.y;
  current_pose.position.z = tf.transform.translation.z;
  current_pose.orientation = tf.transform.rotation;

  TrajectoryPointStamped current_states_stamped_tf;
  current_states_stamped_tf.header = m_current_state_ptr->header;
  current_states_stamped_tf.state = current_state_tf;

  const auto control_data = getControlData(current_pose);

  // self pose is far from trajectory
  if (control_data.is_far_from_trajectory) {
    m_control_state = ControlState::EMERGENCY;                           // update control state
    const Motion raw_ctrl_cmd = calcEmergencyCtrlCmd(control_data.dt);  // calculate control command
    m_prev_raw_ctrl_cmd = raw_ctrl_cmd;
    publishCtrlCmd(raw_ctrl_cmd, control_data.current_motion.vel);  // publish control command
    // publishDebugData(raw_ctrl_cmd, control_data);     // publish debug data
    return;
  }

  // update control state
  m_control_state = updateControlState(m_control_state, control_data);

  // calculate control command
  const Motion raw_ctrl_cmd = calcCtrlCmd(m_control_state, current_states_stamped_tf, control_data);

  // // publish control command
  publishCtrlCmd(raw_ctrl_cmd, control_data.current_motion.vel);

  // // publish debug data
  // publishDebugData(ctrl_cmd, control_data);
}

float64_t PurePursuitNode::getDt()
{
  float64_t dt;
  if (!m_prev_control_time) {
    dt = 1.0 / m_control_rate;
    m_prev_control_time = std::make_shared<rclcpp::Time>(this->now());
  } else {
    dt = (this->now() - *m_prev_control_time).seconds();
    *m_prev_control_time = this->now();
  }
  const float64_t max_dt = 1.0 / m_control_rate * 2.0;
  const float64_t min_dt = 1.0 / m_control_rate * 0.5;
  return std::max(std::min(dt, max_dt), min_dt);
}

PurePursuitNode::Motion PurePursuitNode::getCurrentMotion() const
{
  const float64_t dv = m_current_state_ptr->state.longitudinal_velocity_mps -
    m_prev_state_ptr->state.longitudinal_velocity_mps;
  const float64_t dt =
    std::max(
    (rclcpp::Time(m_current_state_ptr->header.stamp) -
    rclcpp::Time(m_prev_state_ptr->header.stamp)).seconds(), 1e-03);
  const float64_t accel = dv / dt;

  const float64_t current_vel = m_current_state_ptr->state.longitudinal_velocity_mps;
  // const float64_t current_acc = m_lpf_acc->filte(accel);
  const float64_t current_acc = accel;

  return Motion{current_vel, current_acc};
}

enum PurePursuitNode::Shift PurePursuitNode::getCurrentShift(const size_t nearest_idx)
const
{
  constexpr float64_t epsilon = 1e-5;

  const float64_t target_vel = m_trajectory_ptr->points.at(nearest_idx).longitudinal_velocity_mps;

  if (target_vel > epsilon) {
    return Shift::Forward;
  } else if (target_vel < -epsilon) {
    return Shift::Reverse;
  }

  return m_prev_shift;
}

PurePursuitNode::ControlData PurePursuitNode::getControlData(
  const geometry_msgs::msg::Pose & current_pose)
{
  ControlData control_data{};

  // dt
  control_data.dt = getDt();

  // current velocity and acceleration
  control_data.current_motion = getCurrentMotion();

  // nearest idx
  const float64_t max_dist = m_state_transition_params.emergency_state_traj_trans_dev;
  const float64_t max_yaw = m_state_transition_params.emergency_state_traj_rot_dev;
  const auto nearest_idx_opt =
    motion_common::findNearestIndex(m_trajectory_ptr->points, current_pose, max_dist, max_yaw);

  // return here if nearest index is not found
  if (!nearest_idx_opt) {
    control_data.is_far_from_trajectory = true;
    return control_data;
  }
  control_data.nearest_idx = *nearest_idx_opt;

  // shift
  control_data.shift = getCurrentShift(control_data.nearest_idx);
  if (control_data.shift != m_prev_shift) {
    // Guidedog only walk forward,won't reverse. So shift mode was not used.
    // m_pid_vel.reset();
  }
  m_prev_shift = control_data.shift;
 
  // distance to stopline
  control_data.stop_dist =
    trajectory_follower::longitudinal_utils::calcStopDistance(
    current_pose.position,
    *m_trajectory_ptr);
  
  // slope compensation not used
  // pitch
  // const float64_t raw_pitch = trajectory_follower::longitudinal_utils::getPitchByPose(
  //   current_pose.orientation);
  // const float64_t traj_pitch = trajectory_follower::longitudinal_utils::getPitchByTraj(
  //   *m_trajectory_ptr, control_data.nearest_idx, 1.0);
  // control_data.slope_angle = m_use_traj_for_pitch ? traj_pitch : m_lpf_pitch->filter(raw_pitch);
  // // updatePitchDebugValues(control_data.slope_angle, traj_pitch, raw_pitch);

  return control_data;
}

PurePursuitNode::Motion PurePursuitNode::calcEmergencyCtrlCmd(const float64_t dt)
const
{
  // These accelerations are without slope compensation

  const float64_t m_emergency_acceleration = m_state_transition_params.emergency_acceleration;
  const float64_t v_decrease = m_emergency_acceleration * dt;
  return Motion{std::max(m_prev_raw_ctrl_cmd.vel - v_decrease, 0.0), m_emergency_acceleration, 0.0};
}

void PurePursuitNode::callbackCurrentState(
  const autoware_auto_vehicle_msgs::msg::VehicleKinematicState::ConstSharedPtr msg)
{
  if (m_current_state_ptr) {
    m_prev_state_ptr = m_current_state_ptr;
  }
  m_current_state_ptr = std::make_shared<autoware_auto_vehicle_msgs::msg::VehicleKinematicState>(
    *msg);
}


void PurePursuitNode::callbackTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  if (!trajectory_follower::longitudinal_utils::isValidTrajectory(*msg)) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "received invalid trajectory. ignore.");
    return;
  }

  if (msg->points.size() < 2) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Unexpected trajectory size < 2. Ignored.");
    return;
  }

  m_trajectory_ptr = std::make_shared<autoware_auto_planning_msgs::msg::Trajectory>(*msg);
  m_ppcontroller_->set_trajectory(*m_trajectory_ptr);
}

// Do not use nearest_idx here
void PurePursuitNode::publishCtrlCmd(const Motion & ctrl_cmd, float64_t current_vel)
{
  geometry_msgs::msg::Twist cmd;

  // // publish control command
  cmd.linear.x = static_cast<decltype(cmd.linear.x)>(ctrl_cmd.vel);
  cmd.linear.y = 0.0;
  cmd.linear.z = 0.0;
  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;
  cmd.angular.z = static_cast<decltype(cmd.angular.z)>(ctrl_cmd.ang_z);
  while(true){
    m_pub_control_cmd->publish(cmd);
  }
  
  // store current velocity history
  m_vel_hist.push_back({this->now(), current_vel});
  while (m_vel_hist.size() > static_cast<size_t>(m_control_rate * 0.5)) {
    m_vel_hist.erase(m_vel_hist.begin());
  }

  m_prev_ctrl_cmd = ctrl_cmd;
}

PurePursuitNode::ControlState PurePursuitNode::updateControlState(
  const ControlState current_control_state, const ControlData & control_data)
{
  const float64_t current_vel = control_data.current_motion.vel;
  const float64_t current_acc = control_data.current_motion.acc;
  const float64_t stop_dist = control_data.stop_dist;

  // flags for state transition
  const auto & p = m_state_transition_params;

  const bool8_t departure_condition_from_stopping =
    stop_dist > p.drive_state_stop_dist + p.drive_state_offset_stop_dist;
  const bool8_t departure_condition_from_stopped = stop_dist > p.drive_state_stop_dist;

  const bool8_t stopping_condition = stop_dist < p.stopping_state_stop_dist;

  if (
    std::fabs(current_vel) > p.stopped_state_entry_vel ||
    std::fabs(current_acc) > p.stopped_state_entry_acc)
  {
    m_last_running_time = std::make_shared<rclcpp::Time>(this->now());
  }
  const bool8_t stopped_condition =
    m_last_running_time ? (this->now() - *m_last_running_time).seconds() > 0.5 : false;

  const bool8_t emergency_condition =
    m_enable_overshoot_emergency && stop_dist < -p.emergency_state_overshoot_stop_dist;

  // transit state
  if (current_control_state == ControlState::DRIVE) {
    if (emergency_condition) {
      return ControlState::EMERGENCY;
    }

    if (m_enable_smooth_stop) {
      // if (stopping_condition) {
      //   // predictions after input time delay
      //   const float64_t pred_vel_in_target =
      //     predictedVelocityInTargetPoint(control_data.current_motion, m_delay_compensation_time);
      //   const float64_t pred_stop_dist =
      //     control_data.stop_dist -
      //     0.5 * (pred_vel_in_target + current_vel) * m_delay_compensation_time;
      //   m_smooth_stop.init(pred_vel_in_target, pred_stop_dist);
      //   return ControlState::STOPPING;
      // }
    } else {
      if (stopped_condition && !departure_condition_from_stopped) {
        return ControlState::STOPPED;
      }
    }
  } else if (current_control_state == ControlState::STOPPING) {
    if (emergency_condition) {
      return ControlState::EMERGENCY;
    }

    if (stopped_condition) {
      return ControlState::STOPPED;
    }

    if (departure_condition_from_stopping) {
      // m_pid_vel.reset();
      // m_lpf_vel_error->reset(0.0);
      return ControlState::DRIVE;
    }
  } else if (current_control_state == ControlState::STOPPED) {
    if (departure_condition_from_stopped) {
      // m_pid_vel.reset();
      // m_lpf_vel_error->reset(0.0);
      return ControlState::DRIVE;
    }
  } else if (m_control_state == ControlState::EMERGENCY) {
    if (stopped_condition && !emergency_condition) {
      return ControlState::STOPPED;
    }
  }

  return current_control_state;
}

// float64_t PurePursuitNode::predictedVelocityInTargetPoint(
//   const Motion current_motion, const float64_t delay_compensation_time) const
// {
//   const float64_t current_vel = current_motion.vel;
//   const float64_t current_acc = current_motion.acc;

//   if (std::fabs(current_vel) < 1e-01) {  // when velocity is low, no prediction
//     return current_vel;
//   }

//   const float64_t current_vel_abs = std::fabs(current_vel);
//   if (m_ctrl_cmd_vec.size() == 0) {
//     const float64_t pred_vel = current_vel + current_acc * delay_compensation_time;
//     // avoid to change sign of current_vel and pred_vel
//     return pred_vel > 0 ? std::copysign(pred_vel, current_vel) : 0.0;
//   }

//   float64_t pred_vel = current_vel_abs;

//   const auto past_delay_time =
//     this->now() - rclcpp::Duration::from_seconds(delay_compensation_time);
//   for (std::size_t i = 0; i < m_ctrl_cmd_vec.size(); ++i) {
//     if (
//       (this->now() - m_ctrl_cmd_vec.at(i).stamp).seconds() <
//       m_delay_compensation_time)
//     {
//       if (i == 0) {
//         // size of m_ctrl_cmd_vec is less than m_delay_compensation_time
//         pred_vel =
//           current_vel_abs + static_cast<float64_t>(m_ctrl_cmd_vec.at(i).acceleration) *
//           delay_compensation_time;
//         return pred_vel > 0 ? std::copysign(pred_vel, current_vel) : 0.0;
//       }
//       // add velocity to accel * dt
//       const float64_t acc = m_ctrl_cmd_vec.at(i - 1).acceleration;
//       const auto curr_time_i = rclcpp::Time(m_ctrl_cmd_vec.at(i).stamp);
//       const float64_t time_to_next_acc = std::min(
//         (curr_time_i - rclcpp::Time(m_ctrl_cmd_vec.at(i - 1).stamp)).seconds(),
//         (curr_time_i - past_delay_time).seconds());
//       pred_vel += acc * time_to_next_acc;
//     }
//   }

//   const float64_t last_acc = m_ctrl_cmd_vec.at(m_ctrl_cmd_vec.size() - 1).acceleration;
//   const float64_t time_to_current =
//     (this->now() - m_ctrl_cmd_vec.at(m_ctrl_cmd_vec.size() - 1).stamp).seconds();
//   pred_vel += last_acc * time_to_current;

//   // avoid to change sign of current_vel and pred_vel
//   return pred_vel > 0 ? std::copysign(pred_vel, current_vel) : 0.0;
// }

PurePursuitNode::Motion PurePursuitNode::calcCtrlCmd(
  const ControlState & current_control_state, const TrajectoryPointStamped & current_pose,
  const ControlData & control_data)
{
  
  // const auto cmd = m_ppcontroller_->compute_command(current_pose);
  // Motion raw_ctrl_cmd{cmd.velocity_mps, cmd.long_accel_mps2, cmd.rear_wheel_angle_rad};
  // return Motion{0.0,0.0,0.0};


  const float64_t current_vel = control_data.current_motion.vel;
  const float64_t current_acc = control_data.current_motion.acc;

  // velocity and acceleration command
  Motion raw_ctrl_cmd{};
  Motion target_motion{};
  if (current_control_state == ControlState::DRIVE) {
    const auto cmd = m_ppcontroller_->compute_command(current_pose);
    raw_ctrl_cmd.vel = cmd.velocity_mps;
    raw_ctrl_cmd.acc = cmd.long_accel_mps2;
    raw_ctrl_cmd.ang_z = cmd.rear_wheel_angle_rad;
    RCLCPP_DEBUG(
      get_logger(),
      "[feedback control]  vel: %3.3f, ang_z: %3.3f, dt: %3.3f, v_curr: %3.3f, v_ref: %3.3f "
      "feedback_ctrl_cmd.ac: %3.3f",
      raw_ctrl_cmd.vel, raw_ctrl_cmd.ang_z, control_data.dt, current_vel, target_motion.vel,
      raw_ctrl_cmd.acc);
  } else if (current_control_state == ControlState::STOPPING) {
    raw_ctrl_cmd.acc = m_smooth_stop.calculate(
      control_data.stop_dist, current_vel, current_acc, m_vel_hist, m_delay_compensation_time);
    raw_ctrl_cmd.vel = m_stopped_state_params.vel;
    raw_ctrl_cmd.ang_z = 0.0;
    RCLCPP_DEBUG(
      get_logger(),
      "[smooth stop]: Smooth stopping. vel: %3.3f, ang_z: %3.3f",
      raw_ctrl_cmd.vel, raw_ctrl_cmd.ang_z);
  } else if (current_control_state == ControlState::STOPPED) {
    // This acceleration is without slope compensation
    const auto & p = m_stopped_state_params;
    raw_ctrl_cmd.vel = trajectory_follower::longitudinal_utils::applyDiffLimitFilter(
      p.vel,
      m_prev_raw_ctrl_cmd.vel,
      control_data.dt, p.acc);
    raw_ctrl_cmd.acc = trajectory_follower::longitudinal_utils::applyDiffLimitFilter(
      p.acc,
      m_prev_raw_ctrl_cmd.acc,
      control_data.dt, p.jerk);
    raw_ctrl_cmd.ang_z = 0.0;
    RCLCPP_DEBUG(
      get_logger(), "[Stopped]. vel: %3.3f, acc: %3.3f",
      raw_ctrl_cmd.vel, raw_ctrl_cmd.acc);
  } else if (current_control_state == ControlState::EMERGENCY) {
    raw_ctrl_cmd = calcEmergencyCtrlCmd(control_data.dt);
  }

  // store acceleration without slope compensation
  m_prev_raw_ctrl_cmd = raw_ctrl_cmd;

  // // apply slope compensation and filter acceleration and jerk
  // const float64_t filtered_acc_cmd = calcFilteredAcc(raw_ctrl_cmd.acc, control_data);
  // const Motion filtered_ctrl_cmd{raw_ctrl_cmd.vel, filtered_acc_cmd};

  // update debug visualization
  // updateDebugVelAcc(target_motion, current_pose, control_data);

  return raw_ctrl_cmd;
}

////////////////////////////////////////////////////////////////////////////////
PurePursuitNode::PurePursuitNode(
  const std::string & node_name,
  const pure_pursuit::Config & cfg,
  const std::string & node_namespace)
: ControllerBaseNode{node_name, node_namespace, "ctrl_cmd", "current_pose",
    "tf", "trajectory", "ctrl_diag"}
{
  set_controller(std::make_unique<pure_pursuit::PurePursuit>(cfg));
}
}  // namespace pure_pursuit_nodes
}  // namespace control
}  // namespace motion
}  // namespace autoware
