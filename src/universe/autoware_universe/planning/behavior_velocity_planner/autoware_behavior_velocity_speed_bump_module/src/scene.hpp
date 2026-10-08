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

#ifndef SCENE_HPP_
#define SCENE_HPP_

#include "util.hpp"

#include <autoware/behavior_velocity_planner_common/scene_module_interface.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/speed_bump.hpp>
#include <autoware_utils/system/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <optional>
#include <tuple>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner
{
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using autoware_internal_planning_msgs::msg::PathPointWithLaneId;

class SpeedBumpModule : public SceneModuleInterface
{
public:
  struct DebugData
  {
    double base_link2front;
    PathPolygonIntersectionStatus path_polygon_intersection_status;
    std::vector<geometry_msgs::msg::Pose> slow_start_poses;
    std::vector<geometry_msgs::msg::Point> slow_end_points;
    std::vector<geometry_msgs::msg::Point> speed_bump_polygon;
    // positions where the ego is asked to stop while waiting for a pedestrian to leave the zone
    std::vector<geometry_msgs::msg::Point> object_stop_poses;
  };

  struct PlannerParam
  {
    double slow_start_margin;
    double slow_end_margin;
    // Acceleration magnitudes [m/s^2] of the approach/exit ramp around the cruise (low-speed)
    // zone. The module applies a constant-deceleration profile v(s) = sqrt(v_slow^2 + 2*a*ds) to
    // ramp the original speed down to the speed_bump speed, keeps the low speed over the speed
    // bump, then ramps back up with constant acceleration. This produces an explicit
    // decelerate -> cruise -> accelerate profile instead of relying only on downstream smoothing.
    double deceleration;
    double acceleration;
    bool print_debug_info;
    float speed_calculation_min_height;
    float speed_calculation_max_height;
    float speed_calculation_min_speed;
    float speed_calculation_max_speed;

    // Object handling inside the slow zone: stop for a pedestrian, avoid a static obstacle.
    ObjectHandlingParam object_handling;
  };

  SpeedBumpModule(
    const int64_t module_id, const int64_t lane_id,
    const lanelet::autoware::SpeedBump & speed_bump_reg_elem, const PlannerParam & planner_param,
    const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr clock,
    const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
    const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
      planning_factor_interface);

  bool modifyPathVelocity(PathWithLaneId * path) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;
  autoware::motion_utils::VirtualWalls createVirtualWalls() override;

private:
  int64_t module_id_;
  int64_t lane_id_;

  // Speed Bump Regulatory Element
  const lanelet::autoware::SpeedBump & speed_bump_reg_elem_;

  // Parameter
  PlannerParam planner_param_;

  // Debug
  DebugData debug_data_;

  bool applySlowDownSpeed(
    PathWithLaneId & output, const float speed_bump_speed,
    const PathPolygonIntersectionStatus & path_polygon_intersection_status);

  // Stop for a pedestrian inside the zone, or drive around a static obstacle blocking it.
  void applyObjectHandling(PathWithLaneId & output);

  float speed_bump_slow_down_speed_;

  // cruise part of the zone, recorded by applySlowDownSpeed and used by the object handling
  SlowZone slow_zone_;
};

}  // namespace autoware::behavior_velocity_planner

#endif  // SCENE_HPP_
