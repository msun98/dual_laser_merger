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

#ifndef DUAL_LASER_MERGER__SCAN_MERGER_HPP_
#define DUAL_LASER_MERGER__SCAN_MERGER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "dual_laser_merger/merger_params.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace merger_node
{

// Pure merging logic: no rclcpp, no tf2, no publishers. Everything it needs
// comes from the parameters it is constructed with and from its arguments.
class ScanMerger
{
public:
  explicit ScanMerger(const MergerParams & params);

  // Smooths every range against its two neighbours, wrapping around the scan.
  static sensor_msgs::msg::LaserScan average_filter(const sensor_msgs::msg::LaserScan & scan_in);

  // Concatenates both clouds, optionally drops shadow points, and rasterizes
  // the result into a laser scan. Returns false when there is nothing to merge.
  bool merge(
    const sensor_msgs::msg::PointCloud2 & cloud_in_1,
    const sensor_msgs::msg::PointCloud2 & cloud_in_2,
    sensor_msgs::msg::PointCloud2 & cloud_out,
    sensor_msgs::msg::LaserScan & scan_out);

private:
  // Drops points that have no neighbour within a range-scaled radius.
  void shadow_filter(pcl::PointCloud<pcl::PointXYZ> & cloud);

  void to_laser_scan(
    const sensor_msgs::msg::PointCloud2 & cloud_out,
    sensor_msgs::msg::LaserScan & scan_out) const;

  const MergerParams & params_;

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud_in_1_;
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud_in_2_;
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud_out_;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
};

}  // namespace merger_node

#endif  // DUAL_LASER_MERGER__SCAN_MERGER_HPP_
