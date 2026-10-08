// Copyright 2026 Tier IV, Inc., Leo Drive Teknoloji A.Ş.
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

#ifndef PARAM_UTILS_HPP_
#define PARAM_UTILS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <map>
#include <string>

namespace autoware::behavior_velocity_planner
{

// Reads `name` from the node, or declares it with `default_value` when the parameter file does not
// provide it.
//
// autoware_utils::get_or_declare_parameter() falls back to declare_parameter<T>(name), i.e. the
// statically typed overload *without* a default value, which throws
// rclcpp::exceptions::UninitializedStaticallyTypedParameterException when no override exists. A
// single missing key therefore takes down the whole planner container, leaving the vehicle unable
// to move (observed with a stale installed speed_bump.param.yaml that did not contain
// `deceleration`). Falling back to an explicit default keeps the module running in a degraded state
// and names the parameter that has to be added to the parameter file.
//
// The lookup goes through the node's parameter *overrides* rather than has_parameter(), which only
// reports parameters that were already declared on the node: an override coming from the parameter
// file is not "declared" until declare_parameter() is called, so has_parameter() would report every
// configured parameter as missing.
template <typename T>
T get_parameter_or_default(
  rclcpp::Node & node, const std::string & name, const T & default_value)
{
  const std::map<std::string, rclcpp::ParameterValue> & overrides =
    node.get_node_parameters_interface()->get_parameter_overrides();

  if (overrides.find(name) == overrides.cend()) {
    RCLCPP_WARN(
      node.get_logger().get_child("speed_bump"),
      "parameter '%s' is not set, falling back to the built-in default", name.c_str());
  }

  // The override wins when it exists, otherwise `default_value` is used.
  return node.declare_parameter<T>(name, default_value);
}

}  // namespace autoware::behavior_velocity_planner

#endif  // PARAM_UTILS_HPP_
