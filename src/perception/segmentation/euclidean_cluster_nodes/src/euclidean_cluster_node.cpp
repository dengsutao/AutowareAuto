// Copyright 2019-2021 the Autoware Foundation
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
/// \file
/// \brief This file implements a clustering node that published colored point clouds and convex
///        hulls

#include <euclidean_cluster_nodes/euclidean_cluster_node.hpp>

#include <autoware_auto_tf2/tf2_autoware_auto_msgs.hpp>
#include <autoware_auto_perception_msgs/msg/bounding_box_array.hpp>
#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <common/types.hpp>
#include <lidar_utils/point_cloud_utils.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <memory>
#include <string>
#include <utility>

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::perception::segmentation::euclidean_cluster::details::BboxMethod;
using BoundingBoxArray = autoware_auto_perception_msgs::msg::BoundingBoxArray;
using DetectedObjects = autoware_auto_perception_msgs::msg::DetectedObjects;

namespace autoware
{
namespace perception
{
namespace segmentation
{
namespace euclidean_cluster_nodes
{
////////////////////////////////////////////////////////////////////////////////
EuclideanClusterNode::EuclideanClusterNode(
  const rclcpp::NodeOptions & node_options)
: Node("euclidean_cluster_cloud_node", node_options),
  m_cloud_sub_ptr{create_subscription<PointCloud2>(
      "points_in",
      rclcpp::QoS(10),
      [this](const PointCloud2::SharedPtr msg) {handle(msg);})},
  m_cluster_pub_ptr{declare_parameter("use_cluster").get<bool8_t>() ?
  create_publisher<Clusters>(
    "points_clustered",
    rclcpp::QoS(10)) : nullptr},
m_box_pub_ptr{declare_parameter("use_box").get<bool8_t>() ?
  create_publisher<BoundingBoxArray>(
    "lidar_bounding_boxes", rclcpp::QoS{10}) :
  nullptr},
m_marker_pub_ptr{get_parameter("use_box").as_bool() ?
  create_publisher<MarkerArray>(
    "lidar_bounding_boxes_viz", rclcpp::QoS{10}) :
  nullptr},
m_cluster_alg{
  euclidean_cluster::Config{
    declare_parameter("cluster.frame_id").get<std::string>().c_str(),
    static_cast<std::size_t>(declare_parameter(
      "cluster.min_number_of_points_in_cluster").get<std::size_t>()),
    static_cast<std::size_t>(declare_parameter("cluster.max_num_clusters").get<std::size_t>()),
    static_cast<float32_t>(declare_parameter("cluster.min_cluster_threshold_m").get<float32_t>()),
    static_cast<float32_t>(declare_parameter("cluster.max_cluster_threshold_m").get<float32_t>()),
    static_cast<float32_t>(declare_parameter("cluster.threshold_saturation_distance_m")
    .get<float32_t>())
  },
  euclidean_cluster::HashConfig{
    static_cast<float32_t>(declare_parameter("hash.min_x").get<float32_t>()),
    static_cast<float32_t>(declare_parameter("hash.max_x").get<float32_t>()),
    static_cast<float32_t>(declare_parameter("hash.min_y").get<float32_t>()),
    static_cast<float32_t>(declare_parameter("hash.max_y").get<float32_t>()),
    static_cast<float32_t>(declare_parameter("hash.side_length").get<float32_t>()),
    static_cast<std::size_t>(declare_parameter("max_cloud_size").get<std::size_t>())
  },
  euclidean_cluster::FilterConfig{
    static_cast<float32_t>(declare_parameter("filter.min_x").get<float32_t>()),
    static_cast<float32_t>(declare_parameter("filter.min_y").get<float32_t>()),
    static_cast<float32_t>(declare_parameter("filter.min_z").get<float32_t>()),
    static_cast<float32_t>(declare_parameter("filter.max_x").get<float32_t>()),
    static_cast<float32_t>(declare_parameter("filter.max_y").get<float32_t>()),
    static_cast<float32_t>(declare_parameter("filter.max_z").get<float32_t>())
  }
},
m_clusters{},
m_voxel_ptr{nullptr},  // Because voxel config's Point types don't accept positional arguments
m_use_lfit{declare_parameter("use_lfit").get<bool8_t>()},
m_use_z{declare_parameter("use_z").get<bool8_t>()},
m_filter_output_by_size{declare_parameter("filter_output_by_size").get<bool8_t>()}
{
  if (declare_parameter("use_detected_objects").get<bool8_t>()) {
    detected_objects_shape_type = get_detected_object_type_from_string(
      declare_parameter(
        "detected_objects_shape_type").get<std::string>());
    if (detected_objects_shape_type == DetectedObjectsShape::POLYGON_PRISM) {
      m_detected_objects_pub_ptr = create_publisher<DetectedObjects>(
        "lidar_polygon_prisms", rclcpp::QoS{10});
    } else if (detected_objects_shape_type == DetectedObjectsShape::BOUNDING_BOX) {
      m_detected_objects_pub_ptr = create_publisher<DetectedObjects>(
        "lidar_detected_objects", rclcpp::QoS{10});
    }
  }

  // Sanity check
  if ((!m_detected_objects_pub_ptr) && (!m_box_pub_ptr) && (!m_cluster_pub_ptr)) {
    throw std::domain_error{"EuclideanClusterNode: No publisher topics provided"};
  }
  // Setup Tf Buffer with listener
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
    *tf_buffer_,
    std::shared_ptr<rclcpp::Node>(this, [](auto) {}), false);

  // Initialize voxel grid
  if (declare_parameter("downsample").get<bool8_t>()) {
    filters::voxel_grid::PointXYZ min_point;
    filters::voxel_grid::PointXYZ max_point;
    filters::voxel_grid::PointXYZ voxel_size;
    min_point.x = static_cast<float32_t>(declare_parameter("voxel.min_point.x").get<float32_t>());
    min_point.y = static_cast<float32_t>(declare_parameter("voxel.min_point.y").get<float32_t>());
    min_point.z = static_cast<float32_t>(declare_parameter("voxel.min_point.z").get<float32_t>());
    max_point.x = static_cast<float32_t>(declare_parameter("voxel.max_point.x").get<float32_t>());
    max_point.y = static_cast<float32_t>(declare_parameter("voxel.max_point.y").get<float32_t>());
    max_point.z = static_cast<float32_t>(declare_parameter("voxel.max_point.z").get<float32_t>());
    voxel_size.x = static_cast<float32_t>(declare_parameter("voxel.voxel_size.x").get<float32_t>());
    voxel_size.y = static_cast<float32_t>(declare_parameter("voxel.voxel_size.y").get<float32_t>());
    voxel_size.z = static_cast<float32_t>(declare_parameter("voxel.voxel_size.z").get<float32_t>());
    // Aggressive downsampling if not using z
    if (!m_use_z) {
      voxel_size.z = (max_point.z - min_point.z) + 1.0F;
      // Info
      RCLCPP_INFO(get_logger(), "z is not used, height aspect is fully downsampled away");
    }
    m_voxel_ptr = std::make_unique<VoxelAlgorithm>(
      filters::voxel_grid::Config{
              min_point,
              max_point,
              voxel_size,
              static_cast<std::size_t>(get_parameter("max_cloud_size").as_int())
            });
  }
}

////////////////////////////////////////////////////////////////////////////////
void EuclideanClusterNode::insert_plain(const PointCloud2 & cloud)
{
  using euclidean_cluster::PointXYZI;
  const auto indices = common::lidar_utils::sanitize_point_cloud(cloud);
  if (indices.point_step != cloud.point_step) {
    std::cout << "Using only a subset of Point cloud fields" << std::endl;
  }
  if (indices.data_length != cloud.data.size()) {
    std::cout << "Misaligned data: Using only a subset of Point cloud data" << std::endl;
  }
  // Insert via memcpy to ensure proper aliasing
  for (auto idx = 0U; idx < indices.data_length; idx += cloud.point_step) {
    using euclidean_cluster::PointXYZI;
    PointXYZI pt;
    void * const dest = &pt;
    const void * const src = &cloud.data[idx];
    (void)std::memcpy(dest, src, indices.point_step);
    m_cluster_alg.insert(euclidean_cluster::PointXYZIR{pt});
  }
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanClusterNode::insert_voxel(const PointCloud2 & cloud)
{
  m_voxel_ptr->insert(cloud);
  insert_plain(m_voxel_ptr->get());
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanClusterNode::insert(const PointCloud2 & cloud)
{
  if (m_voxel_ptr) {
    insert_voxel(cloud);
  } else {
    insert_plain(cloud);
  }
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanClusterNode::publish_clusters(
  Clusters & clusters,
  const std_msgs::msg::Header & header)
{
  m_clusters.header = header;
  m_clusters.header.stamp = get_clock()->now();
  if (m_clusters.header.stamp.sec==0)
  {
    RCLCPP_WARN(get_logger(), "--------------------m_clusters.header.stamp.sec==0--------------------");
  }
  m_cluster_pub_ptr->publish(clusters);
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanClusterNode::handle_clusters(
  Clusters & clusters,
  const std_msgs::msg::Header & header)
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(
      m_cluster_alg.get_config().frame_id(),
      header.frame_id, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
    return;
  }

  Clusters clusters_out = clusters;
  tf2::doTransform(clusters, clusters_out, tf);

  if (m_cluster_pub_ptr) {
    std_msgs::msg::Header header_cluster = header;
    header_cluster.frame_id = m_cluster_alg.get_config().frame_id();
    publish_clusters(clusters_out, header_cluster);
  }

  if (!m_box_pub_ptr && !m_detected_objects_pub_ptr) {
    return;
  }

  BoundingBoxArray boxes;
  if (m_use_lfit) {
    boxes = euclidean_cluster::details::compute_bounding_boxes(
      clusters_out, BboxMethod::LFit, m_use_z, true, m_cluster_alg.get_filter_config());
    // boxes = euclidean_cluster::details::compute_bounding_boxes(
    //   clusters_out, BboxMethod::LFit, m_use_z);
  } else {
    boxes = euclidean_cluster::details::compute_bounding_boxes(
      clusters_out, BboxMethod::Eigenbox,
      m_use_z, true, m_cluster_alg.get_filter_config());
  }

  if (m_box_pub_ptr) {
    //boxes.header.stamp = header.stamp;
    boxes.header.stamp = get_clock()->now();
    boxes.header.frame_id = header.frame_id;
    m_box_pub_ptr->publish(boxes);
  }

  if (m_detected_objects_pub_ptr) {
    if (detected_objects_shape_type == DetectedObjectsShape::POLYGON_PRISM) {
      // Publish detected objects as a polygon prisms shape
      const auto detected_objects_polygon_msg =
        euclidean_cluster::details::convert_to_polygon_prisms(clusters);
      m_detected_objects_pub_ptr->publish(detected_objects_polygon_msg);
    } else if (detected_objects_shape_type == DetectedObjectsShape::BOUNDING_BOX) {
      // Publish detected objects as a bounding box shape
      const auto detected_objects_msg =
        euclidean_cluster::details::convert_to_detected_objects(boxes);
      m_detected_objects_pub_ptr->publish(detected_objects_msg);
    }
  }

  if (m_marker_pub_ptr) {
    // Also publish boxes for visualization
    uint32_t id_counter = 0;
    MarkerArray marker_array;
    for (const auto & box : boxes.boxes) {
      Marker m{};
      m.header.stamp = rclcpp::Time(0);
      m.header.frame_id = header.frame_id;
      m.ns = "bbox";
      m.id = static_cast<int>(id_counter);
      m.type = Marker::CUBE;
      m.action = Marker::ADD;
      m.pose.position.x = static_cast<float64_t>(box.centroid.x);
      m.pose.position.y = static_cast<float64_t>(box.centroid.y);
      m.pose.position.z = static_cast<float64_t>(box.centroid.z);
      m.pose.orientation.x = static_cast<float64_t>(box.orientation.x);
      m.pose.orientation.y = static_cast<float64_t>(box.orientation.y);
      m.pose.orientation.z = static_cast<float64_t>(box.orientation.z);
      m.pose.orientation.w = static_cast<float64_t>(box.orientation.w);
      // X and Y scale are swapped between these two message types
      m.scale.x = static_cast<float64_t>(box.size.y);
      m.scale.y = static_cast<float64_t>(box.size.x);
      m.scale.z = static_cast<float64_t>(box.size.z);
      m.color.r = 1.0;
      m.color.g = 0.5;
      m.color.b = 0.0;
      m.color.a = 0.75;
      m.lifetime.sec = 0;
      m.lifetime.nanosec = 500000000;
      marker_array.markers.push_back(m);
      id_counter++;
    }
    m_marker_pub_ptr->publish(marker_array);
  }
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanClusterNode::handle(const PointCloud2::SharedPtr msg_ptr)
{
  try {
    try {
      insert(*msg_ptr);
    } catch (const std::length_error & e) {
      // Hit limits of inserting, can still cluster, but in bad state
      RCLCPP_WARN(get_logger(), e.what());
    }
    m_cluster_alg.cluster(m_clusters);
    //lint -e{523} NOLINT empty functions to make this modular
    handle_clusters(m_clusters, msg_ptr->header);
    m_cluster_alg.throw_stored_error();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), e.what());
  } catch (...) {
    RCLCPP_FATAL(get_logger(), "EuclideanClusterNode: Unexpected error occurred!");
    throw;
  }
}

}  // namespace euclidean_cluster_nodes
}  // namespace segmentation
}  // namespace perception
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::perception::segmentation::euclidean_cluster_nodes::EuclideanClusterNode)
