// Copyright 2017-2020 the Autoware Foundation
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
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <common/types.hpp>
#include <point_cloud_filter_transform_nodes/point_cloud_filter_transform_node.hpp>
#include <lidar_utils/point_cloud_utils.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <memory>
#include <string>
#include <map>
#include <vector>

namespace autoware
{
namespace perception
{
namespace filters
{
/// \brief Boilerplate Apex.OS nodes around point_cloud_filter_transform_nodes
namespace point_cloud_filter_transform_nodes
{
using autoware::common::lidar_utils::has_intensity_and_throw_if_no_xyz;
using autoware::common::lidar_utils::sanitize_point_cloud;
using autoware::common::lidar_utils::CloudModifier;
using autoware::common::types::float64_t;
using autoware::common::types::PointXYZI;
using autoware::common::types::PointXYZIF;
using geometry_msgs::msg::TransformStamped;
using sensor_msgs::msg::PointCloud2;

TransformStamped get_transform(
  const std::string & input_frame_id,
  const std::string & output_frame_id,
  float64_t r_x, float64_t r_y, float64_t r_z, float64_t r_w, float64_t t_x,
  float64_t t_y, float64_t t_z)
{
  Eigen::Quaterniond tmp{r_w,r_x,r_y,r_z};
  tmp.normalize();
  TransformStamped ret;
  ret.header.frame_id = input_frame_id;
  ret.child_frame_id = output_frame_id;
  ret.transform.rotation.x = tmp.x();
  ret.transform.rotation.y = tmp.y();
  ret.transform.rotation.z = tmp.z();
  ret.transform.rotation.w = tmp.w();
  ret.transform.translation.x = t_x;
  ret.transform.translation.y = t_y;
  ret.transform.translation.z = t_z;
  return ret;
}

PointCloud2FilterTransformNode::PointCloud2FilterTransformNode(
  const rclcpp::NodeOptions & node_options)
: Node("point_cloud_filter_transform_node", node_options),
  m_angle_filter{
    static_cast<float32_t>(declare_parameter("start_angle").get<float64_t>()),
    static_cast<float32_t>(declare_parameter("end_angle").get<float64_t>())},
  m_distance_filter{
    static_cast<float32_t>(declare_parameter("min_radius").get<float64_t>()),
    static_cast<float32_t>(declare_parameter("max_radius").get<float64_t>())},
  m_input_frame_id{declare_parameter("input_frame_id").get<std::string>()},
  m_output_frame_id{declare_parameter("output_frame_id").get<std::string>()},
  m_init_timeout{std::chrono::milliseconds{declare_parameter("init_timeout_ms").get<int32_t>()}},
  m_timeout{std::chrono::milliseconds{declare_parameter("timeout_ms").get<int32_t>()}},
  m_sub_ptr{create_subscription<PointCloud2>(
      // "/hesai/pandar" qos reliablity=best_effort, 
      // default is reliable, change code below to  best_effort
      "points_in", rclcpp::QoS{10}.best_effort(), 
      std::bind(
        &PointCloud2FilterTransformNode::process_filtered_transformed_message, this, _1))},
  m_pub_ptr{create_publisher<PointCloud2>("points_filtered", rclcpp::QoS{10})},
  tf_publisher_{std::make_shared<tf2_ros::StaticTransformBroadcaster>(this)},
  m_expected_num_publishers{
    static_cast<size_t>(declare_parameter("expected_num_publishers").get<int32_t>())},
  m_expected_num_subscribers{
    static_cast<size_t>(declare_parameter("expected_num_subscribers").get<int32_t>())},
  m_pcl_size{static_cast<std::uint32_t>(declare_parameter("pcl_size").get<uint32_t>())},
  m_sub_imu_ptr{create_subscription<Imu>(
      "imu_in", rclcpp::QoS{10}.best_effort(), 
      std::bind(
        &PointCloud2FilterTransformNode::process_imu_message, this, _1))}
{  /// Declare transform parameters with the namespace
  this->declare_parameter("static_transformer.quaternion.x");
  this->declare_parameter("static_transformer.quaternion.y");
  this->declare_parameter("static_transformer.quaternion.z");
  this->declare_parameter("static_transformer.quaternion.w");
  this->declare_parameter("static_transformer.translation.x");
  this->declare_parameter("static_transformer.translation.y");
  this->declare_parameter("static_transformer.translation.z");

  /// Declare objects to hold transform parameters
  rclcpp::Parameter quat_x_param;
  rclcpp::Parameter quat_y_param;
  rclcpp::Parameter quat_z_param;
  rclcpp::Parameter quat_w_param;
  rclcpp::Parameter trans_x_param;
  rclcpp::Parameter trans_y_param;
  rclcpp::Parameter trans_z_param;

  


  /// If transform parameters exist in the param file use them
  if (this->get_parameter("static_transformer.quaternion.x", quat_x_param) &&
    this->get_parameter("static_transformer.quaternion.y", quat_y_param) &&
    this->get_parameter("static_transformer.quaternion.z", quat_z_param) &&
    this->get_parameter("static_transformer.quaternion.w", quat_w_param) &&
    this->get_parameter("static_transformer.translation.x", trans_x_param) &&
    this->get_parameter("static_transformer.translation.y", trans_y_param) &&
    this->get_parameter("static_transformer.translation.z", trans_z_param))
  {
    RCLCPP_INFO(get_logger(), "param quat:"+std::to_string(quat_w_param.as_double())+","
                              +std::to_string(quat_x_param.as_double())+","
                              +std::to_string(quat_y_param.as_double())+","
                              +std::to_string(quat_z_param.as_double()));
    RCLCPP_WARN(get_logger(), "Using transform from file.");
    m_static_transformer = std::make_unique<StaticTransformer>(
      get_transform(
        m_input_frame_id, "axis",
        quat_x_param.as_double(),
        quat_y_param.as_double(),
        quat_z_param.as_double(),
        quat_w_param.as_double(),
        trans_x_param.as_double(),
        trans_y_param.as_double(),
        trans_z_param.as_double()).transform);
  } else {  /// Else lookup transform being published on /tf or /static_tf topics
    /// TF buffer
    tf2_ros::Buffer tf2_buffer(this->get_clock());
    /// TF listener
    tf2_ros::TransformListener tf2_listener(tf2_buffer);
    while (rclcpp::ok()) {
      try {
        RCLCPP_INFO(get_logger(), "Looking up the transform.");
        m_static_transformer = std::make_unique<StaticTransformer>(
          tf2_buffer.lookupTransform(
            m_output_frame_id, m_input_frame_id,
            tf2::TimePointZero).transform);
        break;
      } catch (const std::exception & transform_exception) {
        RCLCPP_INFO(get_logger(), "No transform was available. Retrying after 100 ms.");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
    }
  }
  m_imu_transformer = std::make_unique<StaticTransformer>(
      get_transform(
        "axis", m_output_frame_id,
        0.0,
        0.0,
        0.0,
        1.0, // w
        0.0,
        0.0,
        0.0).transform);
  TransformStamped ts;
  ts.header.stamp = get_clock()->now();
  ts.header.frame_id = "base_link";
  ts.child_frame_id = "axis";
  tf_publisher_->sendTransform(ts);
  using autoware::common::types::PointXYZI;
  CloudModifier{
    m_filtered_transformed_msg, m_output_frame_id}.resize(m_pcl_size);
  
}

const PointCloud2 & PointCloud2FilterTransformNode::filter_and_transform(const PointCloud2 & msg)
{
  // // Verify frame_id
  // if (msg.header.frame_id != m_input_frame_id) {
  //   throw std::runtime_error(
  //           "Raw topic from unexpected frame. Expected: " +
  //           m_input_frame_id + ", got: " + msg.header.frame_id);
  // }

  sensor_msgs::PointCloud2ConstIterator<float32_t> x_it(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float32_t> y_it(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float32_t> z_it(msg, "z");

  auto && intensity_it = autoware::common::lidar_utils::IntensityIteratorWrapper(msg);

  using autoware::common::types::PointXYZI;
  CloudModifier modifier{m_filtered_transformed_msg};
  modifier.clear();
  modifier.reserve(m_pcl_size);

  m_filtered_transformed_msg.header.stamp = msg.header.stamp;
  // bool flag = false;
  for (size_t it = 0; it < (msg.data.size() / 16); it++) {
    PointXYZI pt;
    pt.x = *x_it;
    pt.y = *y_it;
    pt.z = *z_it;
    intensity_it.get_current_value(pt.intensity);

    if (point_not_filtered(pt)) {
      auto transformed_point = transform_point(pt);
      transformed_point = transform_point_imu(transformed_point);
      transformed_point.intensity = pt.intensity;
      modifier.push_back(transformed_point);
    }

    ++x_it;
    ++y_it;
    ++z_it;
    intensity_it.next();

    if (intensity_it.eof()) {
      break;
    }
  }
  return m_filtered_transformed_msg;
}

void
PointCloud2FilterTransformNode::process_filtered_transformed_message(
  const PointCloud2::SharedPtr msg)
{
  const auto filtered_transformed_msg = filter_and_transform(*msg);
  m_pub_ptr->publish(filtered_transformed_msg);
}

void
PointCloud2FilterTransformNode::process_imu_message(
  const Imu::SharedPtr msg)
{
  // 输出小车到大地的四元数转换,分为两步:1.小车到imu;2.imu到大地
  // 以小车前方为n的话,小车坐标系nwu;imu坐标系enu;大地坐标系nwu;

  float pi = static_cast<float>(M_PI);
  // 小车到imu
  Eigen::Quaternionf quat_nwu2enu = Eigen::Quaternionf{static_cast<float>(pow(0.5,0.5)),
                                                      0.0,
                                                      0.0,
                                                      static_cast<float>(pow(0.5,0.5))};
  // // imu到大地坐标系，先根据imu读数旋转，然后imu的enu坐标系转大地坐标系nwu
  Eigen::Quaternionf imu_quat_enu = Eigen::Quaternionf{static_cast<float>(msg->orientation.w),
                                                      static_cast<float>(msg->orientation.x),
                                                      static_cast<float>(msg->orientation.y),
                                                      static_cast<float>(msg->orientation.z)};
  Eigen::Vector3f euler_tmp = imu_quat_enu.toRotationMatrix().eulerAngles(2,1,0);
  RCLCPP_INFO(get_logger(), "imu x,y,z="+std::to_string(euler_tmp.x()*180/pi)+","+std::to_string(euler_tmp.y()*180/pi)+","+std::to_string(euler_tmp.z()*180/pi));
  Eigen::Quaternionf quat_enu2nwu = Eigen::Quaternionf{static_cast<float>(pow(0.5,0.5)),
                                                      0.0,
                                                      0.0,
                                                      static_cast<float>(-pow(0.5,0.5))};
  Eigen::Quaternionf imu_quat = quat_enu2nwu * imu_quat_enu * quat_nwu2enu;
  euler_tmp = imu_quat.toRotationMatrix().eulerAngles(2,1,0);
  RCLCPP_INFO(get_logger(), "car2ground x,y,z="+std::to_string(euler_tmp.x()*180/pi)+","+std::to_string(euler_tmp.y()*180/pi)+","+std::to_string(euler_tmp.z()*180/pi));

  // // 以下为获取欧拉角,并修正eulerangles的x默认必须是[0,pi]的问题
  // // eulerAngles()得到的欧拉角,x范围限制在了[0,pi],即如果小车向左倾斜0.1rad,x值不是-0.1,而是pi-0.1
  // // 在这种情况下,eulerAngles()的y与实际y0的关系为y0=pi-y;z0=pi+z;
  // // 注意即使进行以下转换后eulerangles1与eulerangles对应的四元数是一样的,只是两者xyz的数值不同而已.
  // auto eulerangles = imu_quat.toRotationMatrix().eulerAngles(0,1,2);
  // Eigen::Vector3f eulerangles1{eulerangles.x(),eulerangles.y(),eulerangles.z()};
  // float pi = static_cast<float>(M_PI);
  // if (eulerangles.x()>pi/2)
  // {
  //   eulerangles1.x() = eulerangles.x() - pi;

  //   auto y = pi - eulerangles.y();
  //   y = fmod(y + pi, 2 * pi) - pi;
  //   eulerangles1.y() = y;
    
  //   auto z = pi + eulerangles.z();
  //   z = fmod(z + pi, 2 * pi) - pi;
  //   eulerangles1.z() = z;
  // }
  // // RCLCPP_INFO(get_logger(), "imu x,y="+std::to_string(eulerangles1.x()*180/pi)+","+std::to_string(eulerangles1.y()*180/pi));
  // Eigen::Quaternionf imu_quat1 = Eigen::AngleAxisf(eulerangles1.x(),Eigen::Vector3f::UnitX()) *
  //                                   Eigen::AngleAxisf(eulerangles1.y(),Eigen::Vector3f::UnitY()) *
  //                                   Eigen::AngleAxisf(eulerangles1.z(),Eigen::Vector3f::UnitZ());
  m_imu_transformer = std::make_unique<StaticTransformer>(
      get_transform(
        "axis", m_output_frame_id, 
        imu_quat.x(),
        imu_quat.y(),
        imu_quat.z(),
        imu_quat.w(),
        0.0,
        0.0,
        0.0).transform);
  TransformStamped ts;
  ts.header.stamp = msg->header.stamp;
  ts.header.frame_id = "base_link";
  ts.child_frame_id = "axis";
  ts.transform.rotation.w = imu_quat.w();
  ts.transform.rotation.x = imu_quat.x();
  ts.transform.rotation.y = imu_quat.y();
  ts.transform.rotation.z = imu_quat.z();
  tf_publisher_->sendTransform(ts);
}

}  // namespace point_cloud_filter_transform_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::perception::filters::point_cloud_filter_transform_nodes::PointCloud2FilterTransformNode)
