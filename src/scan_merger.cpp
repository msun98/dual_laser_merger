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

#include "dual_laser_merger/scan_merger.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace merger_node
{

ScanMerger::ScanMerger(const MergerParams & params)
: params_(params)
{
}

sensor_msgs::msg::LaserScan ScanMerger::average_filter(
  const sensor_msgs::msg::LaserScan & scan_in)
{
  sensor_msgs::msg::LaserScan scan_avg = scan_in;
  const auto & ranges = scan_in.ranges;

  for (size_t i = 0; i <= ranges.size(); i++) {
    if (i == 0) {
      scan_avg.ranges[i] = (ranges[ranges.size() - 1] + ranges[i] + ranges[i + 1]) / 3;
    } else if (i == (ranges.size() - 1)) {
      scan_avg.ranges[i] = (ranges[i - 1] + ranges[i] + ranges[0]) / 3;
    } else {
      scan_avg.ranges[i] = (ranges[i - 1] + ranges[i] + ranges[i + 1]) / 3;
    }
  }

  return scan_avg;
}

bool ScanMerger::merge(
  const sensor_msgs::msg::PointCloud2 & cloud_in_1,
  const sensor_msgs::msg::PointCloud2 & cloud_in_2,
  sensor_msgs::msg::PointCloud2 & cloud_out,
  sensor_msgs::msg::LaserScan & scan_out)
{
  pcl::fromROSMsg(cloud_in_1, pcl_cloud_in_1_);
  pcl::fromROSMsg(cloud_in_2, pcl_cloud_in_2_);

  if (pcl_cloud_in_1_.points.empty() || pcl_cloud_in_2_.points.empty()) {
    return false;
  }

  pcl_cloud_out_ = pcl_cloud_in_1_;
  pcl_cloud_out_ += pcl_cloud_in_2_;

  if (params_.enable_shadow_filter) {
    shadow_filter(pcl_cloud_out_);
  }

  pcl::toROSMsg(pcl_cloud_out_, cloud_out);
  to_laser_scan(cloud_out, scan_out);

  return true;
}

void ScanMerger::shadow_filter(pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  if (cloud.points.empty()) {
    return;
  }

  const double allowed_radius_scaled = params_.allowed_radius / params_.range_max;

  kdtree_.setInputCloud(cloud.makeShared());

  std::vector<int> point_indices;
  std::vector<float> point_distances;

  // A point whose only neighbour is itself is a shadow artifact: drop it
  // outright instead of pushing it to infinity, which would otherwise leak
  // non-finite coordinates into the published cloud.
  const auto is_isolated = [&](const pcl::PointXYZ & point) {
      const double dist_from_origin = std::hypot(point.x, point.y);
      const int num_nearby_points = kdtree_.radiusSearch(
        point, allowed_radius_scaled * dist_from_origin, point_indices, point_distances) - 1;
      return num_nearby_points == 0;
    };

  auto & points = cloud.points;
  points.erase(std::remove_if(points.begin(), points.end(), is_isolated), points.end());

  cloud.width = static_cast<uint32_t>(points.size());
  cloud.height = 1;
  cloud.is_dense = true;
}

void ScanMerger::to_laser_scan(
  const sensor_msgs::msg::PointCloud2 & cloud_out,
  sensor_msgs::msg::LaserScan & scan_out) const
{
  scan_out.header = cloud_out.header;
  scan_out.header.frame_id = params_.target_frame;

  scan_out.angle_min = params_.angle_min;
  scan_out.angle_max = params_.angle_max;
  scan_out.angle_increment = params_.angle_increment;
  scan_out.time_increment = 0.0;
  scan_out.scan_time = params_.scan_time;
  scan_out.range_min = params_.range_min;
  scan_out.range_max = params_.range_max;

  const uint32_t ranges_size =
    std::ceil((scan_out.angle_max - scan_out.angle_min) / scan_out.angle_increment);

  if (params_.use_inf) {
    scan_out.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
  } else {
    scan_out.ranges.assign(ranges_size, scan_out.range_max + params_.inf_epsilon);
  }

  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_out, "x"),
    iter_y(cloud_out, "y"), iter_z(cloud_out, "z");
    iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    if (
      std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z) ||
      *iter_z > params_.max_height || *iter_z < params_.min_height)
    {
      continue;
    }

    const double range = hypot(*iter_x, *iter_y);
    if (range < scan_out.range_min || range > scan_out.range_max) {
      continue;
    }

    const double angle = atan2(*iter_y, *iter_x);
    if (angle < scan_out.angle_min || angle > scan_out.angle_max) {
      continue;
    }

    const int index = (angle - scan_out.angle_min) / scan_out.angle_increment;
    if (range < scan_out.ranges[index]) {
      scan_out.ranges[index] = range;
    }
  }
}

}  // namespace merger_node
