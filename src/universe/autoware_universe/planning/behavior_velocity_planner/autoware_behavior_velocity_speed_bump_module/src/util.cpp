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

#include "util.hpp"

#include "autoware/motion_utils/trajectory/path_with_lane_id.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner
{

namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;
using Line = bg::model::linestring<Point>;

using autoware::motion_utils::calcLongitudinalOffsetPoint;
using autoware::motion_utils::calcSignedArcLength;
using autoware::motion_utils::findNearestSegmentIndex;
using autoware::motion_utils::insertTargetPoint;
using autoware_utils::create_point;

PathPolygonIntersectionStatus getPathPolygonIntersectionStatus(
  const PathWithLaneId & ego_path, const lanelet::BasicPolygon2d & polygon,
  const geometry_msgs::msg::Point & ego_pos, const size_t max_num)
{
  PathPolygonIntersectionStatus polygon_intersection_status;
  std::vector<Point> intersects{};

  bool found_max_num = false;
  for (size_t i = 0; i < ego_path.points.size() - 1; ++i) {
    const auto & p_back = ego_path.points.at(i).point.pose.position;
    const auto & p_front = ego_path.points.at(i + 1).point.pose.position;
    const Line segment{{p_back.x, p_back.y}, {p_front.x, p_front.y}};

    std::vector<Point> tmp_intersects{};
    bg::intersection(segment, polygon, tmp_intersects);

    for (const auto & p : tmp_intersects) {
      intersects.push_back(p);
      if (intersects.size() == max_num) {
        found_max_num = true;
        break;
      }
    }

    if (found_max_num) {
      break;
    }
  }

  const auto compare = [&](const Point & p1, const Point & p2) {
    const auto dist_l1 =
      calcSignedArcLength(ego_path.points, size_t(0), create_point(p1.x(), p1.y(), ego_pos.z));

    const auto dist_l2 =
      calcSignedArcLength(ego_path.points, size_t(0), create_point(p2.x(), p2.y(), ego_pos.z));

    return dist_l1 < dist_l2;
  };

  std::sort(intersects.begin(), intersects.end(), compare);

  const auto & p_last = ego_path.points.back().point.pose.position;
  const auto & p_first = ego_path.points.front().point.pose.position;
  const Point & last_path_point{p_last.x, p_last.y};
  const Point & first_path_point{p_first.x, p_first.y};

  auto const & is_first_path_point_inside_polygon = bg::within(first_path_point, polygon);
  auto const & is_last_path_point_inside_polygon = bg::within(last_path_point, polygon);

  // classify first and second intersection points
  if (intersects.empty()) {
    if (is_first_path_point_inside_polygon && is_last_path_point_inside_polygon) {
      polygon_intersection_status.is_path_inside_of_polygon = true;
    } else {
      // do nothing
    }
  } else if (intersects.size() == 1) {
    const auto & p = intersects.at(0);
    if (is_last_path_point_inside_polygon) {
      polygon_intersection_status.first_intersection_point = create_point(p.x(), p.y(), ego_pos.z);
    } else if (is_first_path_point_inside_polygon) {
      polygon_intersection_status.second_intersection_point = create_point(p.x(), p.y(), ego_pos.z);
    } else {
      // do nothing
    }
  } else if (intersects.size() == 2) {
    const auto & p0 = intersects.at(0);
    const auto & p1 = intersects.at(1);
    polygon_intersection_status.first_intersection_point = create_point(p0.x(), p0.y(), ego_pos.z);
    polygon_intersection_status.second_intersection_point = create_point(p1.x(), p1.y(), ego_pos.z);
  } else {
    // do nothing
  }

  return polygon_intersection_status;
}

bool isNoRelation(const PathPolygonIntersectionStatus & status)
{
  return !status.is_path_inside_of_polygon && !status.first_intersection_point &&
         !status.second_intersection_point;
}

bool insertSlowDownSpeedProfile(
  std::vector<PathPointWithLaneId> & output, const size_t start_idx, const size_t end_idx,
  const float speed, const double deceleration, const double acceleration)
{
  if (output.empty() || start_idx > end_idx) {
    return false;
  }

  const size_t n = output.size();
  const size_t cruise_start = std::min(start_idx, n - 1);
  const size_t cruise_end = std::min(end_idx, n - 1);
  const float v_slow = std::max(speed, 0.0f);

  // accumulated arc length along the path
  std::vector<double> arc(n, 0.0);
  for (size_t i = 1; i < n; ++i) {
    arc[i] = arc[i - 1] + autoware::universe_utils::calcDistance2d(
                            output[i - 1].point.pose.position, output[i].point.pose.position);
  }

  // never raise a limit that is already lower than the value we want to write
  const auto limit = [&output](const size_t idx, const float v) {
    output[idx].point.longitudinal_velocity_mps =
      std::min(output[idx].point.longitudinal_velocity_mps, v);
  };

  for (size_t i = cruise_start; i <= cruise_end; ++i) {
    limit(i, speed);
  }

  // constant-deceleration ramp before the cruise zone:
  //   ramp length (v_ref^2 - v_slow^2) / (2*a) follows from the path's own velocity v_ref, so the
  //   vehicle decelerates uniformly and reaches `speed` exactly at cruise_start. A path that starts
  //   inside the ramp is simply decelerated harder (i.e. slower, never faster).
  if (deceleration > 0.0 && cruise_start > 0) {
    size_t decel_start = cruise_start;
    while (decel_start > 0) {
      const float v_ref = output[decel_start - 1].point.longitudinal_velocity_mps;
      if (v_ref <= v_slow) {
        break;
      }
      const double ramp_len = (v_ref * v_ref - v_slow * v_slow) / (2.0 * deceleration);
      if (arc[cruise_start] - arc[decel_start - 1] <= ramp_len) {
        --decel_start;
      } else {
        break;
      }
    }
    if (decel_start < cruise_start) {
      for (size_t i = decel_start; i < cruise_start; ++i) {
        const double dist = std::max(arc[cruise_start] - arc[i], 0.0);
        const double v_ramp = std::sqrt(v_slow * v_slow + 2.0 * deceleration * dist);
        limit(i, static_cast<float>(v_ramp));
      }
    }
  }

  // constant-acceleration ramp after the cruise zone, symmetric to the approach ramp
  if (acceleration > 0.0 && cruise_end + 1 < n) {
    size_t accel_end = cruise_end;
    while (accel_end + 1 < n) {
      const float v_ref = output[accel_end + 1].point.longitudinal_velocity_mps;
      if (v_ref <= v_slow) {
        break;
      }
      const double ramp_len = (v_ref * v_ref - v_slow * v_slow) / (2.0 * acceleration);
      if (arc[accel_end + 1] - arc[cruise_end] <= ramp_len) {
        ++accel_end;
      } else {
        break;
      }
    }
    if (accel_end > cruise_end) {
      for (size_t i = cruise_end + 1; i <= accel_end; ++i) {
        const double dist = std::max(arc[i] - arc[cruise_end], 0.0);
        const double v_ramp = std::sqrt(v_slow * v_slow + 2.0 * acceleration * dist);
        limit(i, static_cast<float>(v_ramp));
      }
    }
  }

  return true;
}

std::optional<double> calcDistanceToPolyline(
  const geometry_msgs::msg::Point & point, const std::vector<geometry_msgs::msg::Point> & line)
{
  if (line.size() < 2) {
    return std::nullopt;
  }

  double min_distance = std::numeric_limits<double>::max();
  for (size_t i = 0; i + 1 < line.size(); ++i) {
    const double dx = line[i + 1].x - line[i].x;
    const double dy = line[i + 1].y - line[i].y;
    const double length_squared = dx * dx + dy * dy;
    // project onto the segment, clamped to its two ends
    double t = 0.0;
    if (length_squared > 1e-12) {
      t = std::clamp(
        ((point.x - line[i].x) * dx + (point.y - line[i].y) * dy) / length_squared, 0.0, 1.0);
    }
    min_distance = std::min(
      min_distance, std::hypot(point.x - (line[i].x + t * dx), point.y - (line[i].y + t * dy)));
  }

  return min_distance;
}

namespace
{
uint8_t getHighestProbLabel(
  const std::vector<autoware_perception_msgs::msg::ObjectClassification> & classifications)
{
  if (classifications.empty()) {
    return autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
  }
  uint8_t best_label = classifications.front().label;
  float best_prob = classifications.front().probability;
  for (const auto & classification : classifications) {
    if (classification.probability > best_prob) {
      best_prob = classification.probability;
      best_label = classification.label;
    }
  }
  return best_label;
}

std::vector<geometry_msgs::msg::Point> toPointList(const std::vector<PathPointWithLaneId> & points)
{
  std::vector<geometry_msgs::msg::Point> positions;
  positions.reserve(points.size());
  for (const auto & p : points) {
    positions.push_back(p.point.pose.position);
  }
  return positions;
}
}  // namespace

ZoneReaction findZoneReaction(
  const std::vector<PathPointWithLaneId> & path, const SlowZone & zone,
  const ObjectHandlingParam & param,
  const autoware_perception_msgs::msg::PredictedObjects & objects)
{
  ZoneReaction reaction;
  if (
    !param.enable || !zone.valid || path.empty() || zone.start_idx >= path.size() ||
    zone.end_idx >= path.size() || zone.end_idx <= zone.start_idx || objects.objects.empty()) {
    return reaction;
  }

  const auto path_position = toPointList(path);
  const double zone_len = calcSignedArcLength(path, zone.start_idx, zone.end_idx);

  for (const auto & object : objects.objects) {
    const auto & obj_pos = object.kinematics.initial_pose_with_covariance.pose.position;
    const double obj_arc = calcSignedArcLength(path, zone.start_idx, obj_pos);
    if (std::isnan(obj_arc)) {
      continue;
    }
    // only react to objects inside (or near) the slow zone
    if (
      obj_arc < -param.detect_expansion_margin ||
      obj_arc > zone_len + param.detect_expansion_margin) {
      continue;
    }
    const double obj_lat = autoware::motion_utils::calcLateralOffset(path_position, obj_pos);
    if (std::isnan(obj_lat)) {
      continue;
    }

    const uint8_t label = getHighestProbLabel(object.classification);
    if (
      param.pedestrian_stop_enable &&
      label == autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN) {
      // a pedestrian stands inside the zone -> stop outside and wait
      reaction.stop_for_pedestrian = true;
      break;
    }

    if (param.static_object_avoid_enable) {
      const double obj_vel =
        std::abs(object.kinematics.initial_twist_with_covariance.twist.linear.x);
      if (
        obj_vel < param.static_velocity_threshold &&
        std::abs(obj_lat) < param.lateral_blocking_width) {
        // NOTE: like the original implementation, the last matching object wins when several
        // obstacles block the zone.
        reaction.static_obstacle =
          ZoneObstacle{obj_pos, obj_lat, static_cast<double>(object.shape.dimensions.y) / 2.0};
      }
    }
  }

  return reaction;
}

std::optional<geometry_msgs::msg::Point> insertStopPointBeforeSlowZone(
  std::vector<PathPointWithLaneId> & output, const SlowZone & zone, const double stop_margin)
{
  if (output.empty() || !zone.valid || zone.start_idx >= output.size()) {
    return std::nullopt;
  }
  const double zone_start_arc = calcSignedArcLength(output, size_t(0), zone.start_idx);
  const double stop_arc_goal = std::max(0.0, zone_start_arc - stop_margin);

  // pick the existing point closest to the target stop arc; never beyond the zone start
  size_t stop_idx = 0;
  double best = std::numeric_limits<double>::max();
  for (size_t j = 0; j <= zone.start_idx; ++j) {
    const double arc = calcSignedArcLength(output, size_t(0), j);
    const double d = std::abs(arc - stop_arc_goal);
    if (d < best) {
      best = d;
      stop_idx = j;
    }
  }
  stop_idx = std::min(stop_idx, zone.start_idx);

  output.at(stop_idx).point.longitudinal_velocity_mps = 0.0f;
  return output.at(stop_idx).point.pose.position;
}

LateralShiftResult applyLateralAvoidance(
  std::vector<PathPointWithLaneId> & output,
  const std::vector<geometry_msgs::msg::Point> & path_position,
  const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound, const SlowZone & zone,
  const ZoneObstacle & obstacle, const ObjectHandlingParam & param, const double half_vehicle_width)
{
  if (
    output.empty() || !zone.valid || path_position.size() != output.size() ||
    zone.start_idx >= output.size()) {
    return LateralShiftResult::kNotNeeded;
  }

  const double clearance = obstacle.half_width + param.lateral_clearance;
  // a positive lateral offset means the object sits on the left of the path
  const double shift_dir = (obstacle.lateral_offset >= 0.0) ? -1.0 : 1.0;
  const double shift_needed = std::abs(obstacle.lateral_offset) + clearance;

  if (shift_needed < 1e-3) {
    return LateralShiftResult::kNotNeeded;
  }
  if (shift_needed > param.lateral_max_offset) {
    return LateralShiftResult::kInsufficientRoom;
  }
  // The shifted path has to stay inside the drivable area. Without bounds we cannot guarantee
  // that, so stop instead of silently leaving the lane.
  if (left_bound.size() < 2 || right_bound.size() < 2) {
    return LateralShiftResult::kInsufficientRoom;
  }

  const size_t n = output.size();
  const double avoid_half_len = param.lateral_avoid_length / 2.0;
  if (avoid_half_len < 1e-3) {
    return LateralShiftResult::kNotNeeded;
  }

  // accumulated arc length along the path as it was before any shift
  std::vector<double> arc(n, 0.0);
  for (size_t i = 1; i < n; ++i) {
    arc[i] =
      arc[i - 1] + autoware::universe_utils::calcDistance2d(path_position[i - 1], path_position[i]);
  }

  const double obj_arc =
    arc[zone.start_idx] + calcSignedArcLength(output, zone.start_idx, obstacle.center);

  size_t obj_idx = 0;
  double nearest = std::numeric_limits<double>::max();
  for (size_t i = 0; i < n; ++i) {
    const double d = std::abs(arc[i] - obj_arc);
    if (d < nearest) {
      nearest = d;
      obj_idx = i;
    }
  }

  // lateral room [m] left for the vehicle centre on each side of the path. Only the points that
  // actually get shifted need their room, plus the object point used for the feasibility check.
  std::vector<double> room_left(n, 0.0);
  std::vector<double> room_right(n, 0.0);
  for (size_t i = 0; i < n; ++i) {
    if (i != obj_idx && std::abs(arc[i] - obj_arc) >= avoid_half_len) {
      continue;
    }
    const auto to_left = calcDistanceToPolyline(path_position[i], left_bound);
    const auto to_right = calcDistanceToPolyline(path_position[i], right_bound);
    if (!to_left || !to_right) {
      return LateralShiftResult::kInsufficientRoom;
    }
    room_left[i] = std::max(0.0, *to_left - half_vehicle_width);
    room_right[i] = std::max(0.0, *to_right - half_vehicle_width);
  }

  // the ego has to reach the full shift at the object, otherwise it would still clip it
  const double room_at_object = (shift_dir > 0.0) ? room_left[obj_idx] : room_right[obj_idx];
  if (room_at_object + 1e-3 < shift_needed) {
    return LateralShiftResult::kInsufficientRoom;
  }

  // smooth raised-cosine lateral shift centred on the obstacle, clipped to the drivable area
  std::vector<bool> shifted(n, false);
  for (size_t i = 0; i < n; ++i) {
    const double a = std::abs(arc[i] - obj_arc);
    if (a >= avoid_half_len) {
      continue;
    }
    const double w = 0.5 * (1.0 + std::cos(M_PI * a / avoid_half_len));
    const double off = std::clamp(shift_dir * shift_needed * w, -room_right[i], room_left[i]);
    if (std::abs(off) < 1e-3) {
      continue;
    }
    // tangent taken from the unshifted path so the result does not depend on the loop order
    const auto & p1 = path_position[std::min(i, n - 2)];
    const auto & p2 = path_position[std::min(i + 1, n - 1)];
    const double dx = p2.x - p1.x;
    const double dy = p2.y - p1.y;
    const double norm = std::hypot(dx, dy);
    if (norm < 1e-6) {
      continue;
    }
    // (-dy, dx) is the left-hand normal of the direction of travel, matching the sign of `off`
    output[i].point.pose.position.x += (-dy / norm) * off;
    output[i].point.pose.position.y += (dx / norm) * off;
    shifted[i] = true;
  }

  // recompute the heading, but only for the segments whose geometry actually changed
  for (size_t i = 0; i + 1 < n; ++i) {
    if (!shifted[i] && !shifted[i + 1]) {
      continue;
    }
    const double dx = output[i + 1].point.pose.position.x - output[i].point.pose.position.x;
    const double dy = output[i + 1].point.pose.position.y - output[i].point.pose.position.y;
    if (std::hypot(dx, dy) < 1e-6) {
      continue;
    }
    output[i].point.pose.orientation =
      autoware::universe_utils::createQuaternionFromYaw(std::atan2(dy, dx));
  }

  return LateralShiftResult::kApplied;
}

ObjectHandlingResult handleObjectsInSlowZone(
  PathWithLaneId & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound, const SlowZone & zone,
  const ObjectHandlingParam & param,
  const autoware_perception_msgs::msg::PredictedObjects & objects, const double half_vehicle_width)
{
  ObjectHandlingResult result;
  if (!param.enable || !zone.valid) {
    return result;
  }

  const auto reaction = findZoneReaction(path.points, zone, param, objects);

  if (reaction.stop_for_pedestrian) {
    result.stop_position = insertStopPointBeforeSlowZone(path.points, zone, param.stop_margin);
    return result;
  }
  if (!reaction.static_obstacle) {
    return result;
  }

  result.shift_result = applyLateralAvoidance(
    path.points, toPointList(path.points), left_bound, right_bound, zone, *reaction.static_obstacle,
    param, half_vehicle_width);

  if (result.shift_result == LateralShiftResult::kInsufficientRoom) {
    result.stop_position = insertStopPointBeforeSlowZone(path.points, zone, param.stop_margin);
  }
  return result;
}

std::optional<size_t> insertPointWithOffset(
  const geometry_msgs::msg::Point & src_point, const double longitudinal_offset,
  std::vector<PathPointWithLaneId> & output, const double overlap_threshold)
{
  const auto & path = output;
  const auto & target_point = calcLongitudinalOffsetPoint(path, src_point, longitudinal_offset);

  if (!target_point) {
    return {};
  }

  const auto & target_segment_idx = findNearestSegmentIndex(path, *target_point);

  const auto & target_point_idx =
    insertTargetPoint(target_segment_idx, *target_point, output, overlap_threshold);

  return target_point_idx;
}

float calcSlowDownSpeed(const Point32 & p1, const Point32 & p2, const float speed_bump_height)
{
  const float m = (p1.y - p2.y) / (p1.x - p2.x);
  const float b = p1.y - (m * p1.x);

  // y=mx+b
  auto speed = m * speed_bump_height + b;

  return std::clamp(speed, p2.y, p1.y);
}

}  // namespace autoware::behavior_velocity_planner
