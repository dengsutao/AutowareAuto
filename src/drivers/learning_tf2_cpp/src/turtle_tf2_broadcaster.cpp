// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include "nav_msgs/msg/odometry.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlesim/msg/pose.hpp"

class FramePublisher : public rclcpp::Node
{
public:
  FramePublisher()
  : Node("turtle_tf2_frame_publisher")
  {
    // Declare and acquire `turtlename` parameter
    turtlename_ = this->declare_parameter<std::string>("turtlename", "/fused_pose1");

    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
    // callback function on each message
    std::ostringstream stream;
    stream <<  turtlename_.c_str();
    std::string topic_name = stream.str();

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      topic_name, 100,
      std::bind(&FramePublisher::handle_turtle_pose, this, std::placeholders::_1));

  }

private:
  void handle_turtle_pose(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
  {
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = msg->header.frame_id;
    t.child_frame_id = msg->child_frame_id;

    // Turtle only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0
    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;

    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message
    // tf2::Quaternion q;
    // q.setRPY(0, 0, msg->theta);
    t.transform.rotation.x = msg->pose.pose.orientation.x;
    t.transform.rotation.y = msg->pose.pose.orientation.y;
    t.transform.rotation.z = msg->pose.pose.orientation.z;
    t.transform.rotation.w = msg->pose.pose.orientation.w;

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
  }

  //rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string turtlename_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FramePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;


//   try{
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<FramePublisher>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
//   }catch (rclcpp::exceptions::RCLError &e) {
//     if (rclcpp::ok()) {
//         // handle the exception / report to user
//       std::cout<<1;
//     } else {
//         // ignore, ROS is shutting down
//         std::cout<<1;
//     }
// }

}
