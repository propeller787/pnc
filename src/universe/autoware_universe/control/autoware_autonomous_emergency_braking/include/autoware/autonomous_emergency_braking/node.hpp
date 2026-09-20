// Copyright 2023 TIER IV, Inc.
// Modified: 近场 AEB —— 订阅 /perception/near_obstacle/tracked_objects，
// 连续 N 帧非空后请求停车；objects 持续空 release_time_sec 后再放行。
// 不走 MRM：向 vehicle_cmd_gate 下发 stop 标志和 v=0 / 最大减速度指令。
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

#ifndef AUTOWARE__AUTONOMOUS_EMERGENCY_BRAKING__NODE_HPP_
#define AUTOWARE__AUTONOMOUS_EMERGENCY_BRAKING__NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <std_msgs/msg/bool.hpp>

namespace autoware::motion::control::autonomous_emergency_braking
{
class AEB : public rclcpp::Node
{
public:
  explicit AEB(const rclcpp::NodeOptions & options);

private:
  void on_tracked_objects(
    const autoware_perception_msgs::msg::TrackedObjects::ConstSharedPtr msg);
  void on_timer();
  void publish_stop_flag(bool stop);
  void publish_stop_cmd();

  rclcpp::Subscription<autoware_perception_msgs::msg::TrackedObjects>::SharedPtr sub_objects_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_aeb_stop_;
  rclcpp::Publisher<autoware_control_msgs::msg::Control>::SharedPtr pub_control_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;

  int enable_frames_{2};
  double release_time_sec_{2.0};
  double stop_acceleration_{-3.0};
  bool enable_aeb_{false};
  int non_empty_count_{0};
  rclcpp::Time last_non_empty_time_{0, 0, RCL_ROS_TIME};
};
}  // namespace autoware::motion::control::autonomous_emergency_braking

#endif  // AUTOWARE__AUTONOMOUS_EMERGENCY_BRAKING__NODE_HPP_
