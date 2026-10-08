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

#ifndef UTIL_HPP_
#define UTIL_HPP_

#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#define EIGEN_MPL2_ONLY

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/point32.hpp>

#include <lanelet2_core/primitives/Polygon.h>

namespace autoware::behavior_velocity_planner
{

namespace bg = boost::geometry;

using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Point32;

// the status of intersection between path and speed bump
struct PathPolygonIntersectionStatus
{
  bool is_path_inside_of_polygon = false;  // true if path is completely inside the speed bump
                                           // polygon (no intersection point)
  boost::optional<geometry_msgs::msg::Point> first_intersection_point;
  boost::optional<geometry_msgs::msg::Point> second_intersection_point;
};

// Object handling inside the speed bump slow zone, see the module README.
struct ObjectHandlingParam
{
  bool enable;                       // yaml: object_handling_enable (master switch)
  bool pedestrian_stop_enable;       // yaml: pedestrian_stop_enable
  bool static_object_avoid_enable;   // yaml: static_object_avoid_enable
  double detect_expansion_margin;    // yaml: detect_expansion_margin [m]
  double stop_margin;                // yaml: stop_margin [m]
  double lateral_max_offset;         // yaml: lateral_max_offset [m]
  double lateral_avoid_length;       // yaml: lateral_avoid_length [m]
  double lateral_clearance;          // yaml: lateral_clearance [m]
  double lateral_blocking_width;     // yaml: lateral_blocking_width [m]
  double static_velocity_threshold;  // yaml: static_velocity_threshold [m/s]
};

// The cruise part of the speed bump zone: the path points limited to `slow_down_speed`.
struct SlowZone
{
  bool valid{false};
  size_t start_idx{0};
  size_t end_idx{0};
};

// A static obstacle the ego should drive around.
struct ZoneObstacle
{
  geometry_msgs::msg::Point center;
  double lateral_offset{0.0};  // signed, positive = left of the path
  double half_width{0.0};
};

struct ZoneReaction
{
  bool stop_for_pedestrian{false};
  std::optional<ZoneObstacle> static_obstacle;
};

enum class LateralShiftResult {
  kNotNeeded,        // nothing blocked the zone, or the shift was too small to matter
  kApplied,          // the path was shifted around the obstacle
  kInsufficientRoom  // no room to clear the obstacle; the caller must stop instead
};

struct ObjectHandlingResult
{
  LateralShiftResult shift_result{LateralShiftResult::kNotNeeded};
  // stop position chosen for the pedestrian case, for debug visualisation
  std::optional<geometry_msgs::msg::Point> stop_position;
};

PathPolygonIntersectionStatus getPathPolygonIntersectionStatus(
  const PathWithLaneId & ego_path, const lanelet::BasicPolygon2d & polygon,
  const geometry_msgs::msg::Point & ego_pos, const size_t max_num);

bool isNoRelation(const PathPolygonIntersectionStatus & status);

// Applies a decelerate -> cruise -> accelerate profile around a speed bump:
//   - [start_idx, end_idx] is capped at `speed`                                (cruise)
//   - before start_idx the path ramps down to `speed` with constant deceleration `deceleration`
//   - after end_idx it ramps back up with constant acceleration `acceleration`
// The constant-acceleration profile is v(s) = sqrt(v_slow^2 + 2*a*ds), so the ramp length
// (v_ref^2 - v_slow^2) / (2*a) follows from the path's own velocity v_ref. Every point keeps the
// stricter of its original and the interpolated velocity, so a limit that is already lower (e.g. a
// downstream curve limit) is never raised. Returns false if the range is empty or inconsistent.
bool insertSlowDownSpeedProfile(
  std::vector<PathPointWithLaneId> & output, const size_t start_idx, const size_t end_idx,
  const float speed, const double deceleration, const double acceleration);

// Minimum 2D distance from `point` to the polyline `line`, or nullopt if it has fewer than two
// points.
std::optional<double> calcDistanceToPolyline(
  const geometry_msgs::msg::Point & point, const std::vector<geometry_msgs::msg::Point> & line);

// Look for objects that require a reaction inside (or next to) the slow zone.
ZoneReaction findZoneReaction(
  const std::vector<PathPointWithLaneId> & path, const SlowZone & zone,
  const ObjectHandlingParam & param,
  const autoware_perception_msgs::msg::PredictedObjects & objects);

// Set the velocity of the path point closest to `stop_margin` before the slow zone to zero.
// Returns the stop position if a point was found. Unlike planning_utils::insertStopPoint this
// reuses an existing point instead of inserting one.
std::optional<geometry_msgs::msg::Point> insertStopPointBeforeSlowZone(
  std::vector<PathPointWithLaneId> & output, const SlowZone & zone, const double stop_margin);

// Shift the path laterally around `obstacle`, clipping the shift to the drivable area given by
// `left_bound` / `right_bound` minus `half_vehicle_width`. `path_position` is the path geometry
// before any shift and is used as the reference for arc length and tangents.
LateralShiftResult applyLateralAvoidance(
  std::vector<PathPointWithLaneId> & output,
  const std::vector<geometry_msgs::msg::Point> & path_position,
  const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound, const SlowZone & zone,
  const ZoneObstacle & obstacle, const ObjectHandlingParam & param,
  const double half_vehicle_width);

// Apply the pedestrian stop / static obstacle avoidance behaviour to the slow zone. Stops the ego
// outside the zone if the obstacle cannot be avoided within the drivable area.
ObjectHandlingResult handleObjectsInSlowZone(
  PathWithLaneId & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound, const SlowZone & zone,
  const ObjectHandlingParam & param,
  const autoware_perception_msgs::msg::PredictedObjects & objects, const double half_vehicle_width);

std::optional<size_t> insertPointWithOffset(
  const geometry_msgs::msg::Point & src_point, const double longitudinal_offset,
  std::vector<PathPointWithLaneId> & output, const double overlap_threshold = 1e-3);

// returns y (speed) for y=mx+b
float calcSlowDownSpeed(const Point32 & p1, const Point32 & p2, const float speed_bump_height);

}  // namespace autoware::behavior_velocity_planner

#endif  // UTIL_HPP_
