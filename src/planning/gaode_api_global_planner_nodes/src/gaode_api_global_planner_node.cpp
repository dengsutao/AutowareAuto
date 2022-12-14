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

#include <rclcpp/node_options.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/buffer_core.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <time_utils/time_utils.hpp>
#include <motion_common/motion_common.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <gaode_api_global_planner_nodes/gaode_api_global_planner_node.hpp>
#include <std_msgs/msg/string.hpp>
#include <common/types.hpp>

#include <chrono>
#include <cmath>
#include <string>
#include <memory>
#include <vector>


using namespace std::chrono_literals;
using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::common::types::TAU;
using std::placeholders::_1;
using std::vector;
using std::to_string;

//地址到地理编码信息，如地址到经纬度
const std::string url_address2geo_prefix = "https://restapi.amap.com/v3/geocode/geo?";
//地理编码信息到逆地理编码，如经纬度->省市等地址信息
const std::string url_geo2regeo_prefix = "https://restapi.amap.com/v3/geocode/regeo?";
//步行规划，起始经纬度->目标经纬度
const std::string url_walking_prefix = "https://restapi.amap.com/v5/direction/walking?";
//gps经纬度坐标到高德坐标
const std::string url_to_gaode_coord = "https://restapi.amap.com/v3/assistant/coordinate/convert?";
//静态地图
const std::string url_static_map = "https://restapi.amap.com/v3/staticmap?";
//输入提示
const std::string url_tips = "https://restapi.amap.com/v3/assistant/inputtips?";

const std::string apikey = "29ad5efe8c382d9f42e70a246748d0ce";

namespace autoware
{
namespace planning
{
namespace gaode_api_global_planner_nodes
{

double lerp(double s, double a, double b)
{
  return a+s*(b-a);
}

GeographicLib::LocalCartesian GaodeApiGlobalPlannerNode::geo_converter_{32.0, 120.0, 0.0};
GaodeApiGlobalPlannerNode::GaodeApiGlobalPlannerNode(
  const rclcpp::NodeOptions & node_options)
: Node("gaode_api_global_planner_node", node_options),
  tf_listener(tf_buffer, std::shared_ptr<rclcpp::Node>(this, [](auto) {}), false)
{
  start_pose_init = false;
  // Subcribers Goal Pose
  goal_pose_sub_ptr =
    this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose", rclcpp::QoS(10),
    std::bind(&GaodeApiGlobalPlannerNode::goal_pose_cb, this, _1));

  current_pose_sub_ptr =
    this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "cur_pose", rclcpp::QoS(10),
    std::bind(&GaodeApiGlobalPlannerNode::current_pose_cb, this, _1));

  // Global path publisher
  global_path_pub_ptr =
    this->create_publisher<gaode_api_route_msgs::msg::GaodeApiRoute>(
    "global_path", rclcpp::QoS(10));
  
  // Subcribers init lon-lat-alt pose
  init_gps_sub_ptr =
    this->create_subscription<sensor_msgs::msg::NavSatFix>(
    "init_gps", rclcpp::QoS(10),
    std::bind(&GaodeApiGlobalPlannerNode::init_gps_cb, this, _1));
  
  declare_parameter<bool>("vis",false);
  
}

void GaodeApiGlobalPlannerNode::goal_pose_cb(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!start_pose_init) {
    RCLCPP_ERROR(this->get_logger(), "Current pose has not been set!");
    return;
  }
  // transform and set the starting and goal point in the map frame
  goal_pose.header = msg->header;
  goal_pose.pose = msg->pose;
  geometry_msgs::msg::PoseStamped goal_pose_map = goal_pose;

  if (goal_pose.header.frame_id != "odom") {
    if (!transform_pose_to_odom(goal_pose, goal_pose_map)) {
      // return: nothing happen
      return;
    } else {
      goal_pose = goal_pose_map;
    }
  }

  auto start = start_pose.pose;
  auto end = goal_pose.pose;


  // get routes
  geometry_msgs::msg::Pose start_enu_pose = start;
  geometry_msgs::msg::Pose end_enu_pose = end;
  vector<geometry_msgs::msg::Pose> enu_route;
  sensor_msgs::msg::NavSatFix start_gps_pose;
  sensor_msgs::msg::NavSatFix end_gps_pose;
  vector<sensor_msgs::msg::NavSatFix> gps_route;
  if (plan_route(start_enu_pose, end_enu_pose, enu_route,
                start_gps_pose, end_gps_pose, gps_route)) {
    // send out the global path
    std_msgs::msg::Header msg_header;
    msg_header.stamp = rclcpp::Clock().now();
    msg_header.frame_id = "odom";
    send_global_path(start_enu_pose, end_enu_pose, enu_route,
                    start_gps_pose, end_gps_pose, gps_route, msg_header);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Global route has not been found!");
  }
}

void GaodeApiGlobalPlannerNode::current_pose_cb(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // convert msg to geometry_msgs::msg::Pose
  // start_pose.pose = msg->state.pose;
  // start_pose.header = msg->header;
  start_pose.pose = msg->pose;
  start_pose.header = msg->header;

  // transform to "map" frame if needed
  if (start_pose.header.frame_id != "odom") {
    geometry_msgs::msg::PoseStamped start_pose_map = start_pose;

    if (!transform_pose_to_odom(start_pose, start_pose_map)) {
      // transform failed
      start_pose_init = false;
    } else {
      // transform ok: set start_pose to the pose in map
      start_pose = start_pose_map;
      start_pose_init = true;
    }
  } else {
    // No transform required
    start_pose_init = true;
  }
}

bool8_t GaodeApiGlobalPlannerNode::plan_route(
  geometry_msgs::msg::Pose & start_enu_pose, 
  geometry_msgs::msg::Pose & end_enu_pose, 
  vector<geometry_msgs::msg::Pose> & enu_route,
  sensor_msgs::msg::NavSatFix & start_gps_pose, 
  sensor_msgs::msg::NavSatFix & end_gps_pose, 
  vector<sensor_msgs::msg::NavSatFix> & gps_route)
{
  std::string isindoor = "0"; // 是否需要室内算路:0,不需要;1,需要
  // odom pose to gps pose
  to_gps(start_enu_pose, start_gps_pose);
  to_gps(end_enu_pose, end_gps_pose);
  // to string
  std::string lon_lat_start = to_string(start_gps_pose.longitude)+","+to_string(start_gps_pose.latitude);
  if (!to_gaode_gps(lon_lat_start, true)) return false;
  std::string lon_lat_end = to_string(end_gps_pose.longitude)+","+to_string(end_gps_pose.latitude);
  if (!to_gaode_gps(lon_lat_end, true)) return false;
  RCLCPP_DEBUG(get_logger(),lon_lat_start+"-------->"+lon_lat_end);
  RCLCPP_DEBUG(get_logger(),"gps_offset, gaode gps vs ori gps(degree):"+to_string(gps_offset[0])+","+to_string(gps_offset[1]));
  // request url
  std::string route_url = url_walking_prefix+"key="+apikey+
                                              "&isindoor="+isindoor+
                                              "&origin="+lon_lat_start+
                                              "&destination="+lon_lat_end+
                                              "&show_fields=polyline";
  Json::Value data;
  if (!parse(route_url, data)) return false;
  
  // concat gps poses in polylines
  Json::Value steps = data["route"]["paths"][0]["steps"];
  for (size_t i=0;i<steps.size();i++)
  {
    Json::Value step_i = steps[static_cast<int>(i)];
    std::string polyline = step_i["polyline"].asString();
    // int32_t step_distance = step_i["step_distance"].asInt();

    size_t p=0,q=0;
    while ((q=polyline.find(";",p))!=std::string::npos)
    {
      std::string lon_lat = polyline.substr(p,q-p);
      sensor_msgs::msg::NavSatFix gps_pose;
      geometry_msgs::msg::Pose enu_pose;
      size_t idx = lon_lat.find_first_of(',');
      // 为了减少调用次数，这里高德转原始gps时直接根据gps_offset相减了
      gps_pose.longitude = std::stod(lon_lat.substr(0,idx)) - gps_offset[0];
      gps_pose.latitude = std::stod(lon_lat.substr(idx+1,lon_lat.size()-idx-1)) - gps_offset[1];
      add_pose(enu_pose, gps_pose, enu_route, gps_route);
      p = q+1;
    }
    std::string lon_lat = polyline.substr(p,polyline.size()-p);
    sensor_msgs::msg::NavSatFix gps_pose;
    geometry_msgs::msg::Pose enu_pose;
    size_t idx = lon_lat.find_first_of(',');
    gps_pose.longitude = std::stod(lon_lat.substr(0,idx)) - gps_offset[0];
    gps_pose.latitude = std::stod(lon_lat.substr(idx+1,lon_lat.size()-idx-1)) - gps_offset[1];
    add_pose(enu_pose, gps_pose, enu_route, gps_route);
  }

  std::setprecision(10);
  for (size_t i=0;i<gps_route.size();i++)
  {
    auto x1 = enu_route[i].position.x;
    auto y1 = enu_route[i].position.y;
    auto x2 = gps_route[i].longitude;
    auto y2 = gps_route[i].latitude;
    RCLCPP_DEBUG(get_logger(),"enu_pose:"+to_string(x1)+","+to_string(y1)
                +"\tgps_pose:"+to_string(x2)+","+to_string(y2));
  }
  return true;
}

void GaodeApiGlobalPlannerNode::add_pose(
  geometry_msgs::msg::Pose & enu_pose, 
  sensor_msgs::msg::NavSatFix & gps_pose,
  vector<geometry_msgs::msg::Pose> & enu_route,
  vector<sensor_msgs::msg::NavSatFix> & gps_route)
{
  if (gps_route.size()==0)
  {
    to_enu(enu_pose, gps_pose);
    gps_route.push_back(gps_pose);
    enu_route.push_back(enu_pose);
    return;
  }
  else if (!(
    gps_pose.longitude==gps_route[gps_route.size()-1].longitude &&
    gps_pose.latitude==gps_route[gps_route.size()-1].latitude)
    )
  {
    to_enu(enu_pose, gps_pose);
    if (!(
      abs(enu_pose.position.x-enu_route[enu_route.size()-1].position.x)<1.0 &&
      abs(enu_pose.position.y-enu_route[enu_route.size()-1].position.y)<1.0))
    { 
      auto x1 = enu_route[enu_route.size()-1].position.x;
      auto y1 = enu_route[enu_route.size()-1].position.y;
      auto x2 = enu_pose.position.x;
      auto y2 = enu_pose.position.y;
      double dist = sqrt(pow(abs(x1-x2),2)+pow(abs(y1-y2),2));
      double min_dist = 10.0;
      // if dist<2*min_dist, dont split
      int num = static_cast<int>(dist / min_dist)-1;
      double inter = min_dist / dist;
      double inter_i=inter;
      for (int j=0;j<num;j++)
      {
        double xj = lerp(inter_i,x1,x2);
        double yj = lerp(inter_i,y1,y2);
        geometry_msgs::msg::Pose enu_pose_i;
        enu_pose_i.position.x = xj;
        enu_pose_i.position.y = yj;
        sensor_msgs::msg::NavSatFix gps_pose_i;
        to_gps(enu_pose_i, gps_pose_i);

        gps_route.push_back(gps_pose_i);
        enu_route.push_back(enu_pose_i);

        inter_i += inter;
      }
      gps_route.push_back(gps_pose);
      enu_route.push_back(enu_pose);
    }
  }
}

void GaodeApiGlobalPlannerNode::to_gps(
  geometry_msgs::msg::Pose & enu_pose, sensor_msgs::msg::NavSatFix & gps_pose)
{
  double curr_lat=0, curr_lon=0, curr_alt=0;
  GaodeApiGlobalPlannerNode::geo_converter_.Reverse(
    enu_pose.position.x, enu_pose.position.y, enu_pose.position.z, 
    curr_lat, curr_lon, curr_alt);
  gps_pose.latitude = curr_lat;
  gps_pose.longitude = curr_lon;
  gps_pose.altitude = curr_alt;
}

void GaodeApiGlobalPlannerNode::to_enu(
  geometry_msgs::msg::Pose & enu_pose, sensor_msgs::msg::NavSatFix & gps_pose)
{
  double curr_e=0, curr_n=0, curr_u=0;
  GaodeApiGlobalPlannerNode::geo_converter_.Forward(
    gps_pose.latitude, gps_pose.longitude, gps_pose.altitude,
    curr_e, curr_n, curr_u);
  enu_pose.position.x = curr_e;
  enu_pose.position.y = curr_n;
  enu_pose.position.z = curr_u;
}

void GaodeApiGlobalPlannerNode::send_global_path(
    const geometry_msgs::msg::Pose & start_enu_pose, 
    const geometry_msgs::msg::Pose & end_enu_pose, 
    const vector<geometry_msgs::msg::Pose> & enu_route,
    const sensor_msgs::msg::NavSatFix & start_gps_pose, 
    const sensor_msgs::msg::NavSatFix & end_gps_pose, 
    const vector<sensor_msgs::msg::NavSatFix> & gps_route,
    const std_msgs::msg::Header & header)
{
  if (gps_route.size() > 10000) {
    RCLCPP_ERROR(this->get_logger(), "Route size is exceeded the limit of 10000");
    return;
  }
  gaode_api_route_msgs::msg::GaodeApiRoute gaode_route;
  gaode_route.header = header;
  gaode_route.start_enu_pose = start_enu_pose;
  gaode_route.end_enu_pose = end_enu_pose;
  gaode_route.enu_route = enu_route;
  gaode_route.start_gps_pose = start_gps_pose;
  gaode_route.end_gps_pose = end_gps_pose;
  gaode_route.gps_route = gps_route;
  global_path_pub_ptr->publish(gaode_route);
  if (get_parameter("vis").as_bool()) visualize(gaode_route);
  
}

bool8_t GaodeApiGlobalPlannerNode::transform_pose_to_odom(
  const geometry_msgs::msg::PoseStamped & pose_in,
  geometry_msgs::msg::PoseStamped & pose_out)
{
  std::string source_frame = pose_in.header.frame_id;
  // lookup transform validity
  if (!tf_buffer.canTransform("odom", source_frame, tf2::TimePointZero)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to transform Pose to odom frame");
    return false;
  }

  geometry_msgs::msg::TransformStamped tf_odom;
  try {
    tf_odom = tf_buffer.lookupTransform(
      "odom", source_frame,
      time_utils::from_message(pose_in.header.stamp));
  } catch (const tf2::ExtrapolationException &) {
    // currently falls back to retrive newest transform available for availability,
    // Do validation of time stamp in the future
    tf_odom = tf_buffer.lookupTransform("odom", source_frame, tf2::TimePointZero);
  }

  // apply transform
  tf2::doTransform(pose_in, pose_out, tf_odom);
  return true;
}

void GaodeApiGlobalPlannerNode::init_gps_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  GaodeApiGlobalPlannerNode::geo_converter_ = 
    GeographicLib::LocalCartesian{msg->latitude, msg->longitude, msg->altitude};
  RCLCPP_INFO(get_logger(), "init gps position 'lon,lat,alt':"
    +to_string(msg->longitude)+","
    +to_string(msg->latitude)+","
    +to_string(msg->altitude));
}

bool8_t GaodeApiGlobalPlannerNode::to_gaode_gps(
  std::string & gps_str,
  bool8_t is_start)
{
  // request url
  std::string trans_url = url_to_gaode_coord+"key="+apikey+
                                              "&locations="+gps_str+
                                              "&coordsys=gps";
  Json::Value data;
  if (!parse(trans_url, data)) return false;
  std::string ori_str(gps_str);
  gps_str = data["locations"].asString();
  if (is_start)
  {
    gps_offset.clear();
    size_t idx0 = gps_str.find_first_of(',');
    size_t idx1 = ori_str.find_first_of(',');
    gps_offset.push_back(
      std::stod(gps_str.substr(0,idx0))-
      std::stod(ori_str.substr(0,idx1)));
    gps_offset.push_back(
      std::stod(gps_str.substr(idx0+1, gps_str.size()-idx0-1))-
      std::stod(ori_str.substr(idx1+1, ori_str.size()-idx1-1)));
  }
  return true;

}

bool8_t GaodeApiGlobalPlannerNode::visualize(const gaode_api_route_msgs::msg::GaodeApiRoute & msg)
{
  std::string location = to_string(msg.start_gps_pose.longitude)+","+to_string(msg.start_gps_pose.latitude);
  // request url
  // location和zoom在有折线数据的条件下可选
  // weight, color, transparency, fillcolor, fillTransparency
  std::string paths_style = "10,0x0000ff,1,,:";
  std::string locations("");
  auto gps_route = msg.gps_route;
  for (size_t i=0;i<gps_route.size();i++)
  {
    locations+=to_string(gps_route[i].longitude+gps_offset[0])+","+to_string(gps_route[i].latitude+gps_offset[1]);
    if (i<gps_route.size()-1) locations+=";";
  }
  std::string paths = paths_style+locations;
  std::string url = url_static_map+"key="+apikey+
                    // "&locations="+location+ 
                    "&zoom=17"+
                    "&size=1000*1000"+
                    "&scale=1"+
                    "&paths="+paths
                    ;
  RCLCPP_INFO(get_logger(),url);
  return true;
}

bool8_t GaodeApiGlobalPlannerNode::parse(std::string & url, Json::Value & data)
{
  RCLCPP_DEBUG(get_logger(), "request url="+url);
  cpr::Response r = cpr::Get(cpr::Url{url});

  if (r.status_code!=200)
  {
    RCLCPP_DEBUG(get_logger(), "wrong response code:"+r.status_code);
    return false;
  }
  // json parse
  Json::Reader reader;
  reader.parse(r.text, data);

  Json::StreamWriterBuilder builder;
  builder["indentation"] = ""; // If you want whitespace-less output
  const std::string output = Json::writeString(builder, data);
  RCLCPP_DEBUG(get_logger(), output);
  if (data["info"].asString()!="ok" && data["info"].asString()!="OK")
  {
    RCLCPP_WARN(get_logger(), "gaode api response:"+data["info"].asString());
    return false;
  }
  return true;
}

}  // namespace gaode_api_global_planner_nodes
}  // namespace planning
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::planning::gaode_api_global_planner_nodes::GaodeApiGlobalPlannerNode)
