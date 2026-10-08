// Copyright 2025 Tier IV, Inc., Leo Drive Teknoloji A.Ş.
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

#ifndef EXPERIMENTAL__SCENE_HPP_
#define EXPERIMENTAL__SCENE_HPP_

#include "../util.hpp"

#include <autoware/behavior_velocity_planner_common/experimental/scene_module_interface.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/speed_bump.hpp>
#include <autoware_utils/system/time_keeper.hpp>

#include <memory>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{
using autoware_internal_planning_msgs::msg::PathWithLaneId;

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
    // zone. See src/scene.hpp for the full description of the profile.
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
    const lanelet::Id module_id, const lanelet::autoware::SpeedBump & speed_bump_reg_elem,
    const PlannerParam & planner_param, const rclcpp::Logger & logger,
    const rclcpp::Clock::SharedPtr clock,
    const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
    const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
      planning_factor_interface);

  bool modifyPathVelocity(
    Trajectory & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
    const std::vector<geometry_msgs::msg::Point> & right_bound,
    const PlannerData & planner_data) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;
  autoware::motion_utils::VirtualWalls createVirtualWalls() override;

private:
  lanelet::Id module_id_;

  // Speed Bump Regulatory Element
  const lanelet::autoware::SpeedBump & speed_bump_reg_elem_;

  // Parameter
  PlannerParam planner_param_;

  // Debug
  DebugData debug_data_;

  bool applySlowDownSpeed(
    PathWithLaneId & output, const float speed_bump_speed,
    const PathPolygonIntersectionStatus & path_polygon_intersection_status,
    const PlannerData & planner_data);

  // Stop for a pedestrian inside the zone, or drive around a static obstacle blocking it.
  void applyObjectHandling(
    PathWithLaneId & output, const std::vector<geometry_msgs::msg::Point> & left_bound,
    const std::vector<geometry_msgs::msg::Point> & right_bound, const PlannerData & planner_data);

  float speed_bump_slow_down_speed_;

  // cruise part of the zone, recorded by applySlowDownSpeed and used by the object handling
  SlowZone slow_zone_;
};

}  // namespace autoware::behavior_velocity_planner::experimental

#endif  // EXPERIMENTAL__SCENE_HPP_
