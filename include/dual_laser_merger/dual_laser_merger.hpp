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

#ifndef DUAL_LASER_MERGER__DUAL_LASER_MERGER_HPP_
#define DUAL_LASER_MERGER__DUAL_LASER_MERGER_HPP_

#include <memory>
#include <string>

#include "dual_laser_merger/merger_params.hpp"
#include "dual_laser_merger/scan_merger.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "message_filters/subscriber.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"
#include "message_filters/synchronizer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace merger_node
{

// ROS plumbing only: parameters, synchronized subscriptions, tf2 and the
// publishers. The merging itself lives in ScanMerger.
class MergerNode : public rclcpp::Node
{
public:
  explicit MergerNode(const rclcpp::NodeOptions & options);

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan>;

  void declare_param();
  void refresh_param();

  void sub_callback(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & lidar_1_msg,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & lidar_2_msg);

  // Broadcasts the calibration offset for this laser and transforms the cloud
  // into the target frame. Returns false when the lookup fails.
  bool calibrate_and_transform(
    sensor_msgs::msg::PointCloud2 & cloud,
    double x_offset, double y_offset, double yaw_offset,
    const std::string & laser_name);

  std::shared_ptr<tf2_ros::Buffer> tf2_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> message_filter;
  message_filters::Subscriber<sensor_msgs::msg::LaserScan> laser_1_sub;
  message_filters::Subscriber<sensor_msgs::msg::LaserScan> laser_2_sub;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr merged_scan_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_cloud_pub;
  laser_geometry::LaserProjection projector;

  MergerParams params;
  ScanMerger merger;

  sensor_msgs::msg::LaserScan merged;
  sensor_msgs::msg::PointCloud2 cloud_in_1;
  sensor_msgs::msg::PointCloud2 cloud_in_2;
  sensor_msgs::msg::PointCloud2 cloud_out;
};

}  // namespace merger_node

#endif  // DUAL_LASER_MERGER__DUAL_LASER_MERGER_HPP_
