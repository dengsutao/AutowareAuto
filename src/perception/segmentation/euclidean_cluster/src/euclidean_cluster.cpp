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
#include <lidar_utils/point_cloud_utils.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <cstring>
//lint -e537 NOLINT Repeated include file: pclint vs cpplint
#include <algorithm>
#include <string>
//lint -e537 NOLINT Repeated include file: pclint vs cpplint
#include <utility>
#include <list>
#include "euclidean_cluster/euclidean_cluster.hpp"
#include "geometry/bounding_box_2d.hpp"

namespace autoware
{
namespace perception
{
namespace segmentation
{
namespace euclidean_cluster
{
////////////////////////////////////////////////////////////////////////////////
PointXYZIR::PointXYZIR(const common::types::PointXYZIF & pt)
: m_point{pt.x, pt.y, pt.z, pt.intensity},
  m_r_xy{sqrtf((pt.x * pt.x) + (pt.y * pt.y))}
{
}
////////////////////////////////////////////////////////////////////////////////
PointXYZIR::PointXYZIR(const PointXYZI & pt)
: m_point{pt},
  m_r_xy{sqrtf((pt.x * pt.x) + (pt.y * pt.y))}
{
}
////////////////////////////////////////////////////////////////////////////////
PointXYZIR::PointXYZIR(
  const float32_t x,
  const float32_t y,
  const float32_t z,
  const float32_t intensity)
: m_point{x, y, z, intensity},
  m_r_xy{std::hypotf(x, y)}
{
}
////////////////////////////////////////////////////////////////////////////////
float32_t PointXYZIR::get_r() const
{
  return m_r_xy;
}
////////////////////////////////////////////////////////////////////////////////
const PointXYZI & PointXYZIR::get_point() const
{
  return m_point;
}
////////////////////////////////////////////////////////////////////////////////
PointXYZIR::operator autoware_auto_perception_msgs::msg::PointXYZIF() const
{
  /*lint -e{1793} it's safe to call non-const member function on a temporary in this case*/
  autoware_auto_perception_msgs::msg::PointXYZIF ret;

  ret.x = m_point.x;
  ret.y = m_point.y;
  ret.z = m_point.z;
  ret.intensity = m_point.intensity;
  return ret;
}
////////////////////////////////////////////////////////////////////////////////
Config::Config(
  const std::string & frame_id,
  const std::size_t min_number_of_points_in_cluster,
  const std::size_t max_num_clusters,
  const float32_t min_cluster_threshold_m,
  const float32_t max_cluster_threshold_m,
  const float32_t cluster_threshold_saturation_distance_m)
: m_frame_id(frame_id),
  m_min_number_of_points_in_cluster(min_number_of_points_in_cluster),
  m_max_num_clusters(max_num_clusters),
  m_min_thresh_m(min_cluster_threshold_m),
  m_max_distance_m(cluster_threshold_saturation_distance_m),
  m_thresh_rate((max_cluster_threshold_m - min_cluster_threshold_m) /
    cluster_threshold_saturation_distance_m)
{
  // TODO(c.ho) sanity checking
}
////////////////////////////////////////////////////////////////////////////////
FilterConfig::FilterConfig(
  const float32_t min_x,
  const float32_t min_y,
  const float32_t min_z,
  const float32_t max_x,
  const float32_t max_y,
  const float32_t max_z)
: m_min_filter_x(min_x),
  m_min_filter_y(min_y),
  m_min_filter_z(min_z),
  m_max_filter_x(max_x),
  m_max_filter_y(max_y),
  m_max_filter_z(max_z)
{
  if (min_x > min_y || max_x > max_y) {
    throw std::runtime_error(
            "width(x) must be smaller than the length(y) for min/max filter");
  }
}
////////////////////////////////////////////////////////////////////////////////
std::size_t Config::min_number_of_points_in_cluster() const
{
  return m_min_number_of_points_in_cluster;
}
////////////////////////////////////////////////////////////////////////////////
std::size_t Config::max_num_clusters() const
{
  return m_max_num_clusters;
}
////////////////////////////////////////////////////////////////////////////////
float32_t Config::threshold(const PointXYZIR & pt) const
{
  return threshold(pt.get_r());
}
////////////////////////////////////////////////////////////////////////////////
float32_t Config::threshold(const float32_t r) const
{
  return m_min_thresh_m + (std::min(m_max_distance_m, r) * m_thresh_rate);
}
////////////////////////////////////////////////////////////////////////////////
const std::string & Config::frame_id() const
{
  return m_frame_id;
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
EuclideanCluster::EuclideanCluster(
  const Config & cfg, const HashConfig & hash_cfg,
  const FilterConfig & filter_cfg)
: m_config(cfg),
  m_hash(hash_cfg),
  m_filter_config(filter_cfg),
  m_last_error(Error::NONE)
{}
////////////////////////////////////////////////////////////////////////////////
bool Config::match_clusters_size(const Clusters & clusters) const
{
  bool ret = true;
  if (clusters.cluster_boundary.capacity() < m_max_num_clusters) {
    ret = false;
  }
  if (clusters.points.capacity() < (m_max_num_clusters * m_min_number_of_points_in_cluster)) {
    ret = false;
  }
  return ret;
}
////////////////////////////////////////////////////////////////////////////////
float32_t FilterConfig::min_filter_x() const
{
  return m_min_filter_x;
}
////////////////////////////////////////////////////////////////////////////////
float32_t FilterConfig::min_filter_y() const
{
  return m_min_filter_y;
}
////////////////////////////////////////////////////////////////////////////////
float32_t FilterConfig::min_filter_z() const
{
  return m_min_filter_z;
}
////////////////////////////////////////////////////////////////////////////////
float32_t FilterConfig::max_filter_x() const
{
  return m_max_filter_x;
}
////////////////////////////////////////////////////////////////////////////////
float32_t FilterConfig::max_filter_y() const
{
  return m_max_filter_y;
}
////////////////////////////////////////////////////////////////////////////////
float32_t FilterConfig::max_filter_z() const
{
  return m_max_filter_z;
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanCluster::insert(const PointXYZIR & pt)
{
  // can't do anything with return values
  (void)m_hash.insert(pt);
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanCluster::cluster(Clusters & clusters)
{
  // Clean the previous clustering result
  clusters.points.clear();
  clusters.cluster_boundary.clear();
  cluster_impl(clusters);
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanCluster::throw_stored_error() const
{
  // Error handling after publishing
  switch (get_error()) {
    case Error::TOO_MANY_CLUSTERS:
      throw std::runtime_error{"EuclideanCluster: Too many clusters"};
    case Error::NONE:
    default:
      break;
  }
}
////////////////////////////////////////////////////////////////////////////////
EuclideanCluster::Error EuclideanCluster::get_error() const
{
  return m_last_error;
}
////////////////////////////////////////////////////////////////////////////////
const Config & EuclideanCluster::get_config() const
{
  return m_config;
}
////////////////////////////////////////////////////////////////////////////////
const FilterConfig & EuclideanCluster::get_filter_config() const
{
  return m_filter_config;
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanCluster::cluster_impl(Clusters & clusters)
{
  m_last_error = Error::NONE;
  auto it = m_hash.begin();
  while (it != m_hash.end()) {
    cluster(clusters, it);
    // Go to next point still in hash; points assigned to a cluster are removed from hash
    it = m_hash.begin();
  }
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanCluster::cluster(Clusters & clusters, const Hash::IT it)
{
  // init new cluster
  if (clusters.cluster_boundary.size() >= m_config.max_num_clusters()) {
    m_last_error = Error::TOO_MANY_CLUSTERS;
    // Flush the remaining points in the hash for the new scan
    m_hash.clear();
  } else {
    // initialize new cluster in clusters
    if (clusters.cluster_boundary.empty()) {
      clusters.cluster_boundary.emplace_back(0U);
    } else {
      clusters.cluster_boundary.emplace_back(clusters.cluster_boundary.back());
    }
    // Seed cluster with new point
    add_point_to_last_cluster(clusters, it->second);
    // Erase returns the element after the removed element but it is not useful here
    (void)m_hash.erase(it);
    // Start clustering process
    std::size_t last_cls_pt_idx = 0U;
    while (last_cls_pt_idx < last_cluster_size(clusters)) {
      const auto pt = get_point_from_last_cluster(clusters, last_cls_pt_idx);
      add_neighbors_to_last_cluster(clusters, pt);
      // Increment seed point
      ++last_cls_pt_idx;
    }
    // check if cluster is large enough
    if (last_cls_pt_idx < m_config.min_number_of_points_in_cluster()) {
      // reject the cluster if too small
      clusters.cluster_boundary.pop_back();
    }
  }
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanCluster::add_neighbors_to_last_cluster(
  Clusters & clusters,
  const EuclideanCluster::PointXY pt)
{
  // TODO(c.ho) make this more generic... also duplicated work..
  const float32_t r = sqrtf((pt.x * pt.x) + (pt.y * pt.y));
  const float32_t thresh1 = m_config.threshold(r);
  // z is not needed since it's a 2d hash
  const auto & nbrs = m_hash.near(pt.x, pt.y, thresh1);
  // For each point within a fixed radius, check for connectivity
  for (const auto itd : nbrs) {
    const auto & qt = itd.get_point();
    // Ensure that threshold is satisfied bidirectionally
    const float32_t thresh2 = m_config.threshold(qt);
    if (itd.get_distance() <= thresh2) {
      // Add to the last cluster
      add_point_to_last_cluster(clusters, qt);
      // Remove from hash: point is already assigned to a cluster; never need to see again
      // (equivalent to marking a point as "seen")
      (void)m_hash.erase(itd);
    }
  }
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanCluster::add_point_to_last_cluster(Clusters & clusters, const PointXYZIR & pt)
{
  // If there are non-valid points in the container due to rejecting small clusters,
  // overwrite the non-valid points, otherwise emplace new point
  if (clusters.cluster_boundary.back() < clusters.points.size()) {
    clusters.points[clusters.cluster_boundary.back()] =
      static_cast<autoware_auto_perception_msgs::msg::PointXYZIF>(pt);
  } else {
    clusters.points.emplace_back(static_cast<autoware_auto_perception_msgs::msg::PointXYZIF>(pt));
  }
  clusters.cluster_boundary.back() = clusters.cluster_boundary.back() + 1U;
}
////////////////////////////////////////////////////////////////////////////////
EuclideanCluster::PointXY EuclideanCluster::get_point_from_last_cluster(
  const Clusters & clusters,
  const std::size_t cls_pt_idx)
{
  const std::size_t num_of_clusters = clusters.cluster_boundary.size();
  // num_of_clusters will be at least 1
  const auto points_idx = (num_of_clusters < 2U) ?
    cls_pt_idx : (cls_pt_idx + clusters.cluster_boundary[num_of_clusters - 2U]);
  return PointXY{clusters.points[points_idx].x, clusters.points[points_idx].y};
}
////////////////////////////////////////////////////////////////////////////////
std::size_t EuclideanCluster::last_cluster_size(const Clusters & clusters)
{
  const std::size_t num_of_clusters = clusters.cluster_boundary.size();
  if (num_of_clusters < 2U) {
    return clusters.cluster_boundary.front();
  } else {
    const auto cluster_size =
      clusters.cluster_boundary[num_of_clusters - 1U] -
      clusters.cluster_boundary[num_of_clusters - 2U];
    return cluster_size;
  }
}
////////////////////////////////////////////////////////////////////////////////
namespace details
{
BoundingBoxArray compute_bounding_boxes(
  Clusters & clusters, const BboxMethod method,
  const bool compute_height, const bool size_filter,
  const FilterConfig & filter_config)
{
  BoundingBoxArray boxes;
  for (uint32_t cls_id = 0U; cls_id < clusters.cluster_boundary.size(); cls_id++) {
    try {
      const auto iter_pair = common::lidar_utils::get_cluster(clusters, cls_id);
      if (iter_pair.first == iter_pair.second) {
        continue;
      }
      BoundingBox box;
      switch (method) {
        case BboxMethod::Eigenbox: 
            box = common::geometry::bounding_box::eigenbox_2d(
              iter_pair.first,
              iter_pair.second);
          break;
        case BboxMethod::LFit:     
            box = common::geometry::bounding_box::lfit_bounding_box_2d(
              iter_pair.first,
              iter_pair.second);
          break;
      }
      if (compute_height) {
        common::geometry::bounding_box::compute_height(
          iter_pair.first, iter_pair.second, box);
      }

      // filter the bounding box if it does not satisfy the specified size
      if (size_filter) {
        BoundingBox & box_to_filter = box;
        uint32_t num_cls;
        auto filtered_boxes = filter_boxes(box_to_filter,cls_id,num_cls,iter_pair,clusters,method,compute_height,filter_config);
        if (filtered_boxes.boxes.size()>0)
          boxes.boxes.insert(boxes.boxes.end(), filtered_boxes.boxes.begin(), filtered_boxes.boxes.end());
        cls_id+=num_cls-1U;
      }
    } catch (const std::exception & e) {
      std::cerr << e.what() << std::endl;
    }
  }
  return boxes;
}

BoundingBoxArray filter_boxes(
  BoundingBox & box, const uint32_t cls_id, uint32_t & num_cls,
  const std::pair<autoware_auto_perception_msgs::msg::PointClusters::_points_type::iterator,
  autoware_auto_perception_msgs::msg::PointClusters::_points_type::iterator> & iter_pair,
  Clusters & clusters, const BboxMethod method, const bool compute_height,
  const FilterConfig & filter_config, const float32_t eps)
{
  num_cls = 1U;
  BoundingBoxArray new_boxes;
  // remove too small box
  bool erase_box = false;
  if (box.size.x < filter_config.min_filter_x() ||
    box.size.y < filter_config.min_filter_y() ) {erase_box = true;}
  if (compute_height &&
    (box.size.z < filter_config.min_filter_z()) ) {erase_box = true;}
  if (erase_box) return new_boxes;
  
  // large box splitting
  if (box.size.x > filter_config.max_filter_x() ||
    box.size.y > filter_config.max_filter_y())
  {
    // first split box corners
    BoundingBox box1, box2;
    decltype(BoundingBox::corners) corners1, corners2;
    if (box.size.x > filter_config.max_filter_x())
    {
      float32_t max_x = filter_config.max_filter_x()-eps;
      corners1[0] = box.corners[0];
      corners1[3] = box.corners[3];
      corners1[1].z = box.corners[1].z;
      corners1[2].z = box.corners[2].z;
      float32_t x_offset = (box.corners[1].x - box.corners[0].x) * 
                        max_x / box.size.x;
      float32_t y_offset = (box.corners[1].y - box.corners[0].y) * 
                        max_x / box.size.x;
      corners1[1].x = box.corners[0].x + x_offset;
      corners1[1].y = box.corners[0].y + y_offset;

      corners1[2].x = box.corners[3].x + x_offset;
      corners1[2].y = box.corners[3].y + y_offset;

      corners2[0] = corners1[1];
      corners2[1] = box.corners[1];
      corners2[2] = box.corners[2];
      corners2[3] = corners1[2];

    }
    else if (box.size.y > filter_config.max_filter_y())
    {
      float32_t max_y = filter_config.max_filter_y()-eps;
      corners1[0] = box.corners[0];
      corners1[1] = box.corners[1];
      corners1[2].z = box.corners[2].z;
      corners1[3].z = box.corners[3].z;
      float32_t x_offset = (box.corners[2].x - box.corners[1].x) * 
                        max_y / box.size.y;
      float32_t y_offset = (box.corners[2].y - box.corners[1].y) * 
                        max_y / box.size.y;
      corners1[2].x = box.corners[1].x + x_offset;
      corners1[2].y = box.corners[1].y + y_offset;

      corners1[3].x = box.corners[0].x + x_offset;
      corners1[3].y = box.corners[0].y + y_offset;

      corners2[0] = corners1[3];
      corners2[1] = corners1[2];
      corners2[2] = box.corners[2];
      corners2[3] = box.corners[3];

    }

    // build box1, box2
    autoware::common::geometry::bounding_box::details::finalize_box(corners1, box1);
    autoware::common::geometry::bounding_box::details::size_2d(corners1, box1.size);
    autoware::common::geometry::bounding_box::details::finalize_box(corners2, box2);
    autoware::common::geometry::bounding_box::details::size_2d(corners2, box2.size);
    if (compute_height)
    {
      box1.centroid.z = box.centroid.z;
      for (auto & corner : box1.corners) {
        corner.z = box.centroid.z;
      }
      box1.size.z = box.size.z;

      box2.centroid.z = box.centroid.z;
      for (auto & corner : box2.corners) {
        corner.z = box.centroid.z;
      }
      box2.size.z = box.size.z;
    }

    // calculate which point in box1, else in box2
    std::vector<autoware_auto_perception_msgs::msg::PointXYZIF> points1;
    std::vector<autoware_auto_perception_msgs::msg::PointXYZIF> points2;
    for (auto it = iter_pair.first; it != iter_pair.second; ++it) {
      float32_t x = (*it).x, y = (*it).y;
      float32_t x_vec1 = x - box1.centroid.x, y_vec1 = y - box1.centroid.y;
      float32_t x_vec2 = x - box2.centroid.x, y_vec2 = y - box2.centroid.y;
      Eigen::Vector3f vec1(x_vec1,y_vec1,0.0), vec2(x_vec2,y_vec2,0.0);
      Eigen::Quaternionf quat(box1.orientation.w,
                              box1.orientation.x,
                              box1.orientation.y,
                              box1.orientation.z);
      // 将vec进行旋转，旋转到矩形z姿态角为0时的表示(enu)
      // 此时box.size坐标系为seu，即x方向向下，y方向向右，idx[0,1,2,3]逆时针方向
      vec1 = quat.inverse() * vec1;
      vec2 = quat.inverse() * vec2;
      bool is_in_box1 = false;
      if (vec1.x()>=-box1.size.y/2-eps/2&&
          vec1.x()<=box1.size.y/2+eps/2&&
          vec1.y()>=-box1.size.x/2-eps/2&&
          vec1.y()<=box1.size.x/2+eps/2) is_in_box1 = true;
      if (is_in_box1) points1.push_back(*it);
      else points2.push_back(*it);
    }
    uint32_t num_points_1 = static_cast<uint32_t>(points1.size());
    uint32_t num_points_2 = static_cast<uint32_t>(points2.size());
    bool is_valid1 = num_points_1>=2U? true:false, is_valid2 = num_points_2>=2U? true:false;
    uint32_t begin_offset = static_cast<uint32_t>(iter_pair.first - clusters.points.begin());
    uint32_t end_offset = static_cast<uint32_t>(iter_pair.second - clusters.points.begin());
    // 拆分clusters为两半
    for (auto i = begin_offset; i<end_offset;i++)
    {
      if (i<begin_offset+num_points_1) clusters.points[i] = points1[i-begin_offset];
      else clusters.points[i] = points2[i-begin_offset-num_points_1];
    }
    clusters.cluster_boundary.insert(
            clusters.cluster_boundary.begin()+cls_id,
            begin_offset+num_points_1);
    // 重新计算box，因为拆分后，两个box不应该还是原来的大小，需要根据点云分布缩紧
    uint32_t num_cls1=1U, num_cls2=1U;
    if (is_valid1)
    {
      BoundingBox new_box1;
      switch (method) {
        case BboxMethod::Eigenbox: 
            new_box1 = common::geometry::bounding_box::eigenbox_2d(
              clusters.points.begin()+begin_offset,
              clusters.points.begin()+begin_offset+num_points_1);
          break;
        case BboxMethod::LFit:     
            new_box1 = common::geometry::bounding_box::lfit_bounding_box_2d(
              clusters.points.begin()+begin_offset,
              clusters.points.begin()+begin_offset+num_points_1);
          break;
      }
      if (compute_height) {
        common::geometry::bounding_box::compute_height(
          clusters.points.begin()+begin_offset,
          clusters.points.begin()+begin_offset+num_points_1, new_box1);
      }
      // 递归操作，分别重新计算分开的两个box是否需要递归拆分，得到拆分后的boxes
      // 注意第二个boxes的cls_id应该根据前一个动态计算，因为clusters是动态变化的。
      const std::pair<autoware_auto_perception_msgs::msg::PointClusters::_points_type::iterator,
      autoware_auto_perception_msgs::msg::PointClusters::_points_type::iterator>
      new_iter_pair1 = {clusters.points.begin() + begin_offset, 
                              clusters.points.begin()+begin_offset+num_points_1};
      uint32_t cls_id1 = cls_id;
      BoundingBoxArray new_boxes1 = filter_boxes(new_box1, cls_id1, num_cls1, new_iter_pair1, clusters, method, compute_height, filter_config);
      if (new_boxes1.boxes.size()>0)
        new_boxes.boxes.insert(new_boxes.boxes.end(), new_boxes1.boxes.begin(), new_boxes1.boxes.end());
    }
    if (is_valid2)
    {
      BoundingBox new_box2;
      switch (method) {
        case BboxMethod::Eigenbox: 
            new_box2 = common::geometry::bounding_box::eigenbox_2d(
              clusters.points.begin()+begin_offset+num_points_1,
              clusters.points.begin()+end_offset);
          break;
        case BboxMethod::LFit:     
            new_box2 = common::geometry::bounding_box::lfit_bounding_box_2d(
              clusters.points.begin()+begin_offset+num_points_1,
              clusters.points.begin()+end_offset);
          break;
      }
      if (compute_height) {
        common::geometry::bounding_box::compute_height(
          clusters.points.begin()+begin_offset+num_points_1,
        clusters.points.begin()+end_offset, new_box2);
      }
      // 递归操作，分别重新计算分开的两个box是否需要递归拆分，得到拆分后的boxes
      // 注意第二个boxes的cls_id应该根据前一个动态计算，因为clusters是动态变化的。
      const std::pair<autoware_auto_perception_msgs::msg::PointClusters::_points_type::iterator,
      autoware_auto_perception_msgs::msg::PointClusters::_points_type::iterator>
      new_iter_pair2 = {clusters.points.begin()+begin_offset+num_points_1,
                      clusters.points.begin()+end_offset};
      uint32_t cls_id2 = cls_id + num_cls1;
      BoundingBoxArray new_boxes2 = filter_boxes(new_box2, cls_id2, num_cls2, new_iter_pair2, clusters, method, compute_height, filter_config);
      if (new_boxes2.boxes.size()>0)
        new_boxes.boxes.insert(new_boxes.boxes.end(), new_boxes2.boxes.begin(), new_boxes2.boxes.end());
    }
    num_cls = num_cls1 + num_cls2;
  }
  else
  {
    new_boxes.boxes.push_back(box);
    num_cls = 1U;
  }
  return new_boxes;
}

////////////////////////////////////////////////////////////////////////////////
BoundingBoxArray compute_lfit_bounding_boxes(Clusters & clusters, const bool compute_height)
{
  BoundingBoxArray boxes;
  for (uint32_t cls_id = 0U; cls_id < clusters.cluster_boundary.size(); cls_id++) {
    try {
      const auto iter_pair = common::lidar_utils::get_cluster(clusters, cls_id);
      if (iter_pair.first == iter_pair.second) {
        continue;
      }
      boxes.boxes.push_back(
        common::geometry::bounding_box::lfit_bounding_box_2d(iter_pair.first, iter_pair.second));
      if (compute_height) {
        common::geometry::bounding_box::compute_height(
          iter_pair.first, iter_pair.second, boxes.boxes.back());
      }
    } catch (const std::exception & e) {
      std::cerr << e.what() << std::endl;
    }
  }
  return boxes;
}
////////////////////////////////////////////////////////////////////////////////
DetectedObjects convert_to_detected_objects(const BoundingBoxArray & boxes)
{
  DetectedObjects detected_objects;
  detected_objects.objects.reserve(boxes.boxes.size());
  detected_objects.header = boxes.header;
  std::transform(
    boxes.boxes.begin(), boxes.boxes.end(), std::back_inserter(detected_objects.objects),
    common::geometry::bounding_box::details::make_detected_object);
  return detected_objects;
}
////////////////////////////////////////////////////////////////////////////////
DetectedObjects convert_to_polygon_prisms(const Clusters & clusters)
{
  DetectedObjects detected_objects;
  for (uint32_t cls_id = 0U; cls_id < clusters.cluster_boundary.size(); cls_id++) {
    try {
      autoware_auto_perception_msgs::msg::DetectedObject detected_object;
      detected_object.existence_probability = 1.0F;
      const auto iter_pair = common::lidar_utils::get_cluster(clusters, cls_id);
      if (iter_pair.first == iter_pair.second) {
        continue;
      }
      std::list<autoware_auto_perception_msgs::msg::PointClusters::_points_type::value_type>
      point_list{
        iter_pair.first, iter_pair.second};
      const auto hull_end_iter = common::geometry::convex_hull(point_list);
      for (auto iter = point_list.begin(); iter != hull_end_iter; ++iter) {
        const auto & hull_point = *iter;
        geometry_msgs::msg::Point32 polygon_point;
        polygon_point.x = hull_point.x;
        polygon_point.y = hull_point.y;
        polygon_point.z = 0.0F;
        detected_object.shape.polygon.points.push_back(polygon_point);
      }
      common::geometry::bounding_box::compute_height(
        iter_pair.first,
        iter_pair.second,
        detected_object.shape);

      // Compute the centroid
      geometry_msgs::msg::Point32 sum;
      for (const auto & point : detected_object.shape.polygon.points) {
        sum = common::geometry::plus_2d(sum, point);
      }
      const auto centroid = common::geometry::times_2d(
        sum, 1.0F / static_cast<float>(detected_object.shape.polygon.points.size()));
      auto & detected_object_position =
        detected_object.kinematics.pose_with_covariance.pose.position;
      detected_object_position.x = static_cast<decltype(detected_object_position.x)>(centroid.x);
      detected_object_position.y = static_cast<decltype(detected_object_position.y)>(centroid.y);
      detected_object_position.z = static_cast<decltype(detected_object_position.z)>(centroid.z);
      for (auto & point : detected_object.shape.polygon.points) {
        // We assume here a zero orientation as we don't care about the orientation of the convex
        // hull. This then becomes a poor man's transformation into the object-local coordinates.
        point = common::geometry::minus_2d(point, centroid);
      }

      autoware_auto_perception_msgs::msg::ObjectClassification label;
      label.classification = autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN;
      label.probability = 1.0F;
      detected_object.classification.emplace_back(label);

      detected_objects.objects.push_back(detected_object);
    } catch (const std::exception & e) {
      std::cerr << e.what() << std::endl;
    }
  }

  detected_objects.header.stamp = clusters.header.stamp;
  detected_objects.header.frame_id = clusters.header.frame_id;

  return detected_objects;
}
////////////////////////////////////////////////////////////////////////////////
}  // namespace details
}  // namespace euclidean_cluster
}  // namespace segmentation
}  // namespace perception
}  // namespace autoware
