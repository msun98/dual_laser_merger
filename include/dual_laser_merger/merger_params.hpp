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

#ifndef DUAL_LASER_MERGER__MERGER_PARAMS_HPP_
#define DUAL_LASER_MERGER__MERGER_PARAMS_HPP_

#include <string>

namespace merger_node
{

// Plain data holder, intentionally free of any rclcpp dependency so that the
// merging logic can be exercised without a ROS node.
struct MergerParams
{
  std::string target_frame;
  int input_queue_size;
  double tolerance;
  double min_height;
  double max_height;
  double angle_min;
  double angle_max;
  double angle_increment;
  double scan_time;
  double range_min;
  double range_max;
  double inf_epsilon;
  double laser_1_x_offset;
  double laser_1_y_offset;
  double laser_1_yaw_offset;
  double laser_2_x_offset;
  double laser_2_y_offset;
  double laser_2_yaw_offset;
  double allowed_radius;
  bool use_inf;
  bool enable_calibration;
  bool enable_shadow_filter;
  bool enable_average_filter;
};

}  // namespace merger_node

#endif  // DUAL_LASER_MERGER__MERGER_PARAMS_HPP_
