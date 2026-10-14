// Copyright 2020 Tier IV, Inc., Leo Drive Teknoloji A.Ş.
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

#include "manager.hpp"
#include "param_utils.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/speed_bump.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <tf2/utils.hpp>

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner
{
using lanelet::autoware::SpeedBump;

SpeedBumpModuleManager::SpeedBumpModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName())
{
  // A parameter missing from the parameter file falls back to the built-in default below (with a
  // warning) instead of taking down the whole planner container. The defaults degrade rather than
  // fail: the ramps are switched off, and a missing speed_calculation block still slows the ego
  // down to the low speed before the bump.
  std::string ns(SpeedBumpModuleManager::getModuleName());
  planner_param_.slow_start_margin =
    get_parameter_or_default<double>(node, ns + ".slow_start_margin", 0.0);
  planner_param_.slow_end_margin =
    get_parameter_or_default<double>(node, ns + ".slow_end_margin", 0.0);

  // approach/exit acceleration magnitudes [m/s^2] to build a decelerate -> cruise -> accelerate
  // profile with constant acceleration
  planner_param_.deceleration = get_parameter_or_default<double>(node, ns + ".deceleration", 0.0);
  planner_param_.acceleration = get_parameter_or_default<double>(node, ns + ".acceleration", 0.0);

  // Object handling (pedestrian stop / static obstacle lateral avoidance)
  planner_param_.object_handling.enable =
    get_parameter_or_default<bool>(node, ns + ".object_handling_enable", false);
  planner_param_.object_handling.pedestrian_stop_enable =
    get_parameter_or_default<bool>(node, ns + ".pedestrian_stop_enable", false);
  planner_param_.object_handling.static_object_avoid_enable =
    get_parameter_or_default<bool>(node, ns + ".static_object_avoid_enable", false);
  planner_param_.object_handling.detect_expansion_margin =
    get_parameter_or_default<double>(node, ns + ".detect_expansion_margin", 0.0);
  planner_param_.object_handling.stop_margin =
    get_parameter_or_default<double>(node, ns + ".stop_margin", 0.0);
  planner_param_.object_handling.lateral_max_offset =
    get_parameter_or_default<double>(node, ns + ".lateral_max_offset", 0.0);
  planner_param_.object_handling.lateral_avoid_length =
    get_parameter_or_default<double>(node, ns + ".lateral_avoid_length", 0.0);
  planner_param_.object_handling.lateral_clearance =
    get_parameter_or_default<double>(node, ns + ".lateral_clearance", 0.0);
  planner_param_.object_handling.lateral_blocking_width =
    get_parameter_or_default<double>(node, ns + ".lateral_blocking_width", 0.0);
  planner_param_.object_handling.static_velocity_threshold =
    get_parameter_or_default<double>(node, ns + ".static_velocity_threshold", 0.0);

  planner_param_.print_debug_info =
    get_parameter_or_default<bool>(node, ns + ".print_debug_info", false);

  // limits for speed bump height and slow down speed
  ns += ".speed_calculation";
  planner_param_.speed_calculation_min_height =
    static_cast<float>(get_parameter_or_default<double>(node, ns + ".min_height", 0.05));
  planner_param_.speed_calculation_max_height =
    static_cast<float>(get_parameter_or_default<double>(node, ns + ".max_height", 0.30));
  planner_param_.speed_calculation_min_speed =
    static_cast<float>(get_parameter_or_default<double>(node, ns + ".min_speed", 0.6389));
  planner_param_.speed_calculation_max_speed =
    static_cast<float>(get_parameter_or_default<double>(node, ns + ".max_speed", 0.6389));
}

void SpeedBumpModuleManager::launchNewModules(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  for (const auto & speed_bump_with_lane_id : planning_utils::getRegElemMapOnPath<SpeedBump>(
         path, planner_data_->route_handler_->getLaneletMapPtr(),
         planner_data_->current_odometry->pose)) {
    const auto lane_id = speed_bump_with_lane_id.second.id();
    const auto module_id = speed_bump_with_lane_id.first->id();
    if (!isModuleRegistered(module_id)) {
      registerModule(
        std::make_shared<SpeedBumpModule>(
          module_id, lane_id, *speed_bump_with_lane_id.first, planner_param_,
          logger_.get_child("speed_bump_module"), clock_, time_keeper_,
          planning_factor_interface_));
    }
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
SpeedBumpModuleManager::getModuleExpiredFunction(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  const auto speed_bump_id_set = planning_utils::getRegElemIdSetOnPath<SpeedBump>(
    path, planner_data_->route_handler_->getLaneletMapPtr(), planner_data_->current_odometry->pose);

  return [speed_bump_id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return speed_bump_id_set.count(scene_module->getModuleId()) == 0;
  };
}

}  // namespace autoware::behavior_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_velocity_planner::SpeedBumpModulePlugin,
  autoware::behavior_velocity_planner::PluginInterface)
