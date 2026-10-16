// Copyright 2024 pradyum
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

#include "dual_laser_merger/dual_laser_merger.hpp"

#include <cmath>
#include <limits>
#include <string>
#include <thread>

using namespace std::chrono_literals;

namespace merger_node
{
MergerNode::MergerNode(const rclcpp::NodeOptions & options)
: Node("dual_laser_merger", options), merger(params)
{
  declare_param();

  if (params.target_frame.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Target Frame cannot be Empty");
  } else {
    RCLCPP_INFO(this->get_logger(), "Target Frame: %s", params.target_frame.c_str());
  }

  merged_scan_pub =
    this->create_publisher<sensor_msgs::msg::LaserScan>(this->get_parameter(
      "merged_scan_topic").as_string(), rclcpp::SensorDataQoS());
  merged_cloud_pub =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(this->get_parameter(
      "merged_cloud_topic").as_string(), rclcpp::SensorDataQoS());
  laser_1_sub.subscribe(this, this->get_parameter("laser_1_topic").as_string(),
      rmw_qos_profile_sensor_data);
  laser_2_sub.subscribe(this, this->get_parameter("laser_2_topic").as_string(),
      rmw_qos_profile_sensor_data);

  tf2_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf2_listener = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer, this);
  message_filter = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(params.input_queue_size), laser_1_sub, laser_2_sub);
  message_filter->setAgePenalty(params.tolerance);
  message_filter->registerCallback(
    std::bind(&MergerNode::sub_callback, this, std::placeholders::_1, std::placeholders::_2));
  tf2_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
}

void MergerNode::declare_param()
{
  this->declare_parameter("laser_1_topic", "laser_1");
  this->declare_parameter("laser_2_topic", "laser_2");
  this->declare_parameter("merged_scan_topic", "merged");
  this->declare_parameter("merged_cloud_topic", "merged_cloud");
  params.target_frame = this->declare_parameter("target_frame", "");
  params.tolerance = this->declare_parameter("tolerance", 0.01);
  params.input_queue_size =
    this->declare_parameter("queue_size", static_cast<int>(std::thread::hardware_concurrency()));
  params.min_height = this->declare_parameter("min_height", std::numeric_limits<double>::min());
  params.max_height = this->declare_parameter("max_height", std::numeric_limits<double>::max());
  params.angle_min = this->declare_parameter("angle_min", -M_PI);
  params.angle_max = this->declare_parameter("angle_max", M_PI);
  params.angle_increment = this->declare_parameter("angle_increment", M_PI / 180.0);
  params.scan_time = this->declare_parameter("scan_time", 1.0 / 30.0);
  params.range_min = this->declare_parameter("range_min", 0.0);
  params.range_max = this->declare_parameter("range_max", std::numeric_limits<double>::max());
  params.inf_epsilon = this->declare_parameter("inf_epsilon", 1.0);
  params.use_inf = this->declare_parameter("use_inf", true);
  params.enable_calibration = this->declare_parameter("enable_calibration", false);
  params.laser_1_x_offset = this->declare_parameter("laser_1_x_offset", 0.0);
  params.laser_1_y_offset = this->declare_parameter("laser_1_y_offset", 0.0);
  params.laser_1_yaw_offset = this->declare_parameter("laser_1_yaw_offset", 0.0);
  params.laser_2_x_offset = this->declare_parameter("laser_2_x_offset", 0.0);
  params.laser_2_y_offset = this->declare_parameter("laser_2_y_offset", 0.0);
  params.laser_2_yaw_offset = this->declare_parameter("laser_2_yaw_offset", 0.0);
  params.allowed_radius = this->declare_parameter("allowed_radius", 1.0);
  params.enable_shadow_filter = this->declare_parameter("enable_shadow_filter", false);
  params.enable_average_filter = this->declare_parameter("enable_average_filter", false);
}

void MergerNode::refresh_param()
{
  this->get_parameter("tolerance", params.tolerance);
  this->get_parameter("queue_size", params.input_queue_size);
  this->get_parameter("min_height", params.min_height);
  this->get_parameter("max_height", params.max_height);
  this->get_parameter("angle_min", params.angle_min);
  this->get_parameter("angle_max", params.angle_max);
  this->get_parameter("angle_increment", params.angle_increment);
  this->get_parameter("scan_time", params.scan_time);
  this->get_parameter("range_min", params.range_min);
  this->get_parameter("range_max", params.range_max);
  this->get_parameter("inf_epsilon", params.inf_epsilon);
  this->get_parameter("use_inf", params.use_inf);
  this->get_parameter("laser_1_x_offset", params.laser_1_x_offset);
  this->get_parameter("laser_1_y_offset", params.laser_1_y_offset);
  this->get_parameter("laser_1_yaw_offset", params.laser_1_yaw_offset);
  this->get_parameter("laser_2_x_offset", params.laser_2_x_offset);
  this->get_parameter("laser_2_y_offset", params.laser_2_y_offset);
  this->get_parameter("laser_2_yaw_offset", params.laser_2_yaw_offset);
  this->get_parameter("allowed_radius", params.allowed_radius);
  this->get_parameter("enable_shadow_filter", params.enable_shadow_filter);
  this->get_parameter("enable_average_filter", params.enable_average_filter);
}

bool MergerNode::calibrate_and_transform(
  sensor_msgs::msg::PointCloud2 & cloud,
  double x_offset, double y_offset, double yaw_offset,
  const std::string & laser_name)
{
  geometry_msgs::msg::TransformStamped tf2_msg;
  tf2::Quaternion tf2_quaternion;

  tf2_msg.header = cloud.header;
  tf2_msg.child_frame_id = cloud.header.frame_id + "_calibrated";
  tf2_msg.transform.translation.x = x_offset;
  tf2_msg.transform.translation.y = y_offset;
  tf2_msg.transform.translation.z = 0.0;
  tf2_quaternion.setRPY(0, 0, yaw_offset);
  tf2_msg.transform.rotation.x = tf2_quaternion.x();
  tf2_msg.transform.rotation.y = tf2_quaternion.y();
  tf2_msg.transform.rotation.z = tf2_quaternion.z();
  tf2_msg.transform.rotation.w = tf2_quaternion.w();
  tf2_broadcaster->sendTransform(tf2_msg);
  cloud.header.frame_id = tf2_msg.child_frame_id;

  try {
    cloud = tf2_buffer->transform(
      cloud, params.target_frame, tf2::durationFromSec(params.tolerance));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR_STREAM(
      this->get_logger(), "Transform failure, " << laser_name << ": " << ex.what());
    return false;
  }

  return true;
}

void MergerNode::sub_callback(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & lidar_1_msg,
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & lidar_2_msg)
{
  if (params.target_frame.empty()) {
    rclcpp::shutdown();
    return;
  }

  this->get_parameter<bool>("enable_calibration", params.enable_calibration);
  if (params.enable_calibration) {
    refresh_param();
  }

  if (params.enable_average_filter) {
    projector.projectLaser(ScanMerger::average_filter(*lidar_1_msg), cloud_in_1);
    projector.projectLaser(ScanMerger::average_filter(*lidar_2_msg), cloud_in_2);
  } else {
    projector.projectLaser(*lidar_1_msg, cloud_in_1);
    projector.projectLaser(*lidar_2_msg, cloud_in_2);
  }

  if (lidar_1_msg->header.frame_id != params.target_frame) {
    if (!calibrate_and_transform(
        cloud_in_1, params.laser_1_x_offset, params.laser_1_y_offset,
        params.laser_1_yaw_offset, "Laser 1"))
    {
      return;
    }
  }

  if (lidar_2_msg->header.frame_id != params.target_frame) {
    if (!calibrate_and_transform(
        cloud_in_2, params.laser_2_x_offset, params.laser_2_y_offset,
        params.laser_2_yaw_offset, "Laser 2"))
    {
      return;
    }
  }

  if (!merger.merge(cloud_in_1, cloud_in_2, cloud_out, merged)) {
    return;
  }

  merged_cloud_pub->publish(cloud_out);
  merged_scan_pub->publish(merged);
}

}  // namespace merger_node

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(merger_node::MergerNode)
