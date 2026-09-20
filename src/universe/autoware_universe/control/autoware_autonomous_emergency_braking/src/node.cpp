// Copyright 2023 TIER IV, Inc.
// Modified: 近场 AEB —— 直接吃 /perception/near_obstacle/tracked_objects。
// 逻辑：
//   - 连续 enable_frames_ 帧 objects 非空 -> enable_aeb_ = true
//   - enable 期间向 vehicle_cmd_gate 下发 stop=true，以及 v=0 / 最大减速度
//   - objects 持续空 release_time_sec_ 秒 -> enable_aeb_ = false，放行
// 不发布 MRM / emergency 话题。
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

#include "autoware/autonomous_emergency_braking/node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace autoware::motion::control::autonomous_emergency_braking
{
namespace
{
rclcpp::NodeOptions allow_undeclared(const rclcpp::NodeOptions & options)
{
  auto opts = options;
  opts.allow_undeclared_parameters(true);  // 兼容 launch 里传入的旧 AEB 参数
  return opts;
}
}  // namespace

AEB::AEB(const rclcpp::NodeOptions & options)
: Node("autonomous_emergency_braking", allow_undeclared(options))
{
  using std::placeholders::_1;

  sub_objects_ = create_subscription<autoware_perception_msgs::msg::TrackedObjects>(
    "/perception/near_obstacle/tracked_objects", rclcpp::QoS{1},
    std::bind(&AEB::on_tracked_objects, this, _1));

  rclcpp::QoS stop_qos{1};
  stop_qos.transient_local();
  pub_aeb_stop_ = create_publisher<std_msgs::msg::Bool>("~/output/aeb_stop", stop_qos);
  pub_control_cmd_ = create_publisher<autoware_control_msgs::msg::Control>(
    "~/output/control_cmd", rclcpp::QoS{1});

  enable_frames_ = declare_parameter<int>("enable_frames", 2);
  release_time_sec_ = declare_parameter<double>("release_time_sec", 2.0);
  stop_acceleration_ = declare_parameter<double>("stop_acceleration", -3.0);
  const double aeb_hz = std::max(1.0, declare_parameter<double>("aeb_hz", 10.0));
  const auto period_ms = static_cast<int>(std::lround(1000.0 / aeb_hz));

  last_non_empty_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

  timer_ = create_wall_timer(
    std::chrono::milliseconds(period_ms), std::bind(&AEB::on_timer, this));

  publish_stop_flag(false);
}

void AEB::on_tracked_objects(
  const autoware_perception_msgs::msg::TrackedObjects::ConstSharedPtr msg)
{
  if (!msg->objects.empty()) {
    ++non_empty_count_;
    last_non_empty_time_ = now();
  } else {
    non_empty_count_ = 0;
  }
}

void AEB::on_timer()
{
  if (enable_aeb_) {
    if ((now() - last_non_empty_time_).seconds() > release_time_sec_) {
      enable_aeb_ = false;
      non_empty_count_ = 0;
      RCLCPP_WARN(get_logger(), "[AEB] released");
    }
  } else if (non_empty_count_ >= enable_frames_) {
    enable_aeb_ = true;
    RCLCPP_WARN(get_logger(), "[AEB] engaged by near obstacle");
  }

  if (enable_aeb_) {
    publish_stop_cmd();
  }
  publish_stop_flag(enable_aeb_);
}

void AEB::publish_stop_flag(const bool stop)
{
  std_msgs::msg::Bool msg;
  msg.data = stop;
  pub_aeb_stop_->publish(msg);
}

void AEB::publish_stop_cmd()
{
  autoware_control_msgs::msg::Control msg;
  msg.stamp = now();
  msg.control_time = msg.stamp;
  msg.lateral.stamp = msg.stamp;
  msg.lateral.control_time = msg.stamp;
  msg.longitudinal.stamp = msg.stamp;
  msg.longitudinal.control_time = msg.stamp;
  msg.longitudinal.velocity = 0.0F;
  msg.longitudinal.acceleration = static_cast<float>(stop_acceleration_);
  msg.longitudinal.jerk = 0.0F;
  msg.longitudinal.is_defined_acceleration = true;
  msg.longitudinal.is_defined_jerk = false;
  pub_control_cmd_->publish(msg);
}
}  // namespace autoware::motion::control::autonomous_emergency_braking

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::motion::control::autonomous_emergency_braking::AEB)
