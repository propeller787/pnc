// Copyright 2026 The Autoware Contributors
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

#include "autoware/map_loader/uturn_lanelet_injector.hpp"

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace autoware::map_loader
{
namespace
{
using BasicPoint2d = lanelet::BasicPoint2d;

constexpr double kMinChordLength = 1e-3;  // [m] entry/exit cross sections closer than this are bogus
constexpr size_t kMinSamples = 8;
constexpr size_t kMaxSamples = 1000;

double normalize_angle(const double angle)
{
  double normalized = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (normalized < 0.0) {
    normalized += 2.0 * M_PI;
  }
  return normalized - M_PI;
}

BasicPoint2d to_2d(const lanelet::ConstPoint3d & point) { return {point.x(), point.y()}; }

//! left normal of a heading, i.e. the direction the turn center of a left turn lies in
BasicPoint2d left_normal(const double heading)
{
  return {-std::sin(heading), std::cos(heading)};
}

BasicPoint2d right_normal(const double heading) { return {std::sin(heading), -std::cos(heading)}; }

double heading_at_end(const lanelet::ConstLanelet & lanelet)
{
  const auto & centerline = lanelet.centerline();
  const auto p1 = to_2d(centerline.back());
  const auto p0 = to_2d(centerline[centerline.size() - 2]);
  return std::atan2(p1.y() - p0.y(), p1.x() - p0.x());
}

double heading_at_start(const lanelet::ConstLanelet & lanelet)
{
  const auto & centerline = lanelet.centerline();
  const auto p0 = to_2d(centerline.front());
  const auto p1 = to_2d(centerline[1]);
  return std::atan2(p1.y() - p0.y(), p1.x() - p0.x());
}

std::string format_speed_limit(const double speed_limit_kmh)
{
  const auto rounded = std::round(speed_limit_kmh);
  if (std::abs(speed_limit_kmh - rounded) < 1e-3) {
    return std::to_string(static_cast<int>(rounded));
  }
  return std::to_string(speed_limit_kmh);
}

//! hands out deterministic ids that do not collide with the ids of the loaded map
class IdAllocator
{
public:
  IdAllocator(lanelet::Id first_id, const lanelet::LaneletMapPtr & map)
  : next_id_(first_id), map_(map)
  {
  }

  lanelet::Id next()
  {
    while (
      map_->laneletLayer.exists(next_id_) || map_->pointLayer.exists(next_id_) ||
      map_->lineStringLayer.exists(next_id_) || map_->polygonLayer.exists(next_id_) ||
      map_->areaLayer.exists(next_id_)) {
      ++next_id_;
    }
    return next_id_++;
  }

private:
  lanelet::Id next_id_;
  lanelet::LaneletMapPtr map_;
};

//! geometry of the single arc that connects the entry end edge with the exit start edge
struct TurnGeometry
{
  BasicPoint2d center;
  BasicPoint2d entry_mid;
  BasicPoint2d exit_mid;
  double radius{0.0};
  double sweep{0.0};  //!< signed heading change, positive for a left turn
  double phi0{0.0};   //!< angle of the entry cross section center around the turn center
  double z_entry{0.0};
  double z_exit{0.0};
  double entry_width{0.0};
  double exit_width{0.0};
  bool is_left{true};
};

std::optional<TurnGeometry> compute_turn_geometry(
  const lanelet::ConstLanelet & entry, const lanelet::ConstLanelet & exit,
  const UturnParams & params, const rclcpp::Logger & logger, const std::string & label)
{
  if (entry.id() == exit.id()) {
    RCLCPP_WARN(
      logger, "[uturn] '%s': entry and exit are the same lanelet. Skipped.", label.c_str());
    return std::nullopt;
  }
  if (
    entry.centerline().size() < 2 || exit.centerline().size() < 2 || entry.leftBound().size() < 1 ||
    exit.leftBound().size() < 1) {
    RCLCPP_WARN(logger, "[uturn] '%s': degraded lanelet geometry. Skipped.", label.c_str());
    return std::nullopt;
  }

  TurnGeometry geometry;
  const auto entry_left = to_2d(entry.leftBound().back());
  const auto entry_right = to_2d(entry.rightBound().back());
  const auto exit_left = to_2d(exit.leftBound().front());
  const auto exit_right = to_2d(exit.rightBound().front());

  geometry.entry_mid = 0.5 * (entry_left + entry_right);
  geometry.exit_mid = 0.5 * (exit_left + exit_right);
  geometry.entry_width = (entry_right - entry_left).norm();
  geometry.exit_width = (exit_right - exit_left).norm();
  geometry.z_entry = 0.5 * (entry.leftBound().back().z() + entry.rightBound().back().z());
  geometry.z_exit = 0.5 * (exit.leftBound().front().z() + exit.rightBound().front().z());

  const double chord = (geometry.exit_mid - geometry.entry_mid).norm();
  if (chord < kMinChordLength) {
    RCLCPP_WARN(
      logger, "[uturn] '%s': entry end edge and exit start edge coincide (%.3f m). Skipped.",
      label.c_str(), chord);
    return std::nullopt;
  }

  const double heading0 = heading_at_end(entry);
  const double heading1 = heading_at_start(exit);
  const double turn_angle = std::abs(normalize_angle(heading1 - heading0));
  const double turn_deg = turn_angle * 180.0 / M_PI;
  if (turn_deg < params.min_turn_angle_deg || turn_deg > params.max_turn_angle_deg) {
    RCLCPP_WARN(
      logger,
      "[uturn] '%s': heading change is %.1f deg, outside [%.1f, %.1f] deg. Skipped. "
      "entry and exit must be opposite directions.",
      label.c_str(), turn_deg, params.min_turn_angle_deg, params.max_turn_angle_deg);
    return std::nullopt;
  }

  // The side of the U-turn is decided by the position of the exit relative to the entry heading.
  // The sign of the heading difference cannot be used, since it is ambiguous at exactly 180 deg.
  const auto entry_chord = geometry.exit_mid - geometry.entry_mid;
  const auto heading_dir = BasicPoint2d{std::cos(heading0), std::sin(heading0)};
  geometry.is_left =
    (heading_dir.x() * entry_chord.y() - heading_dir.y() * entry_chord.x()) > 0.0;
  geometry.sweep = (geometry.is_left ? 1.0 : -1.0) * turn_angle;
  geometry.radius = chord / (2.0 * std::sin(0.5 * turn_angle));

  const double min_radius = params.wheel_base / std::tan(params.max_steer_angle);
  if (geometry.radius < params.radius_margin * min_radius) {
    RCLCPP_WARN(
      logger,
      "[uturn] '%s': radius %.2f m is smaller than the vehicle minimum %.2f m "
      "(r_min %.2f m x margin %.2f). Skipped.",
      label.c_str(), geometry.radius, params.radius_margin * min_radius, min_radius,
      params.radius_margin);
    return std::nullopt;
  }

  const auto normal =
    geometry.is_left ? left_normal(heading0) : right_normal(heading0);
  geometry.center = geometry.entry_mid + geometry.radius * normal;
  const auto radial = geometry.entry_mid - geometry.center;
  geometry.phi0 = std::atan2(radial.y(), radial.x());

  // the exit cross section center must sit on the very same circle, otherwise the single arc
  // assumption does not hold and the synthesized lanelet would be geometrically inconsistent
  const double exit_radius = (geometry.exit_mid - geometry.center).norm();
  if (std::abs(exit_radius - geometry.radius) > 0.05 * geometry.radius) {
    RCLCPP_WARN(
      logger,
      "[uturn] '%s': exit cross section is %.2f m away from the arc of radius %.2f m. Skipped.",
      label.c_str(), exit_radius, geometry.radius);
    return std::nullopt;
  }

  return geometry;
}

size_t sample_count(const TurnGeometry & geometry, const UturnParams & params)
{
  const double arc_length = geometry.radius * std::abs(geometry.sweep);
  const auto samples =
    static_cast<size_t>(std::ceil(arc_length / std::max(params.arc_resolution, 1e-3)));
  return std::clamp(samples, kMinSamples, kMaxSamples);
}

//! samples one bound of the U-turn lanelet.
//!
//! The first and the last point are the *existing* points of the neighbouring lanelets (shared
//! data, which is what makes lanelet2's routing graph accept the lanelet as a successor or
//! predecessor). The bound sweeps from the first cross section to the second one, so the lanelet
//! covers the whole area that the cross sections sweep during the turn.
//!
//! Note: the cross sections of real maps are often wider than the turn. Both of their end points
//! must be pinned for the routing graph, and then the lanelet polygon is self intersecting in the
//! middle of the turn. Such a lanelet still routes and drives correctly (the driving line is the
//! centerline), and it covers the driving line, which a narrow ribbon would not. It is reported as
//! a warning instead of being dropped, see is_simple_polygon().
lanelet::Points3d sample_bound(
  const lanelet::ConstPoint3d & first, const lanelet::ConstPoint3d & last,
  const TurnGeometry & geometry, const size_t samples, IdAllocator & ids)
{
  const auto p0 = to_2d(first);
  const auto p1 = to_2d(last);
  const auto r0 = p0 - geometry.center;
  const auto r1 = p1 - geometry.center;
  const double phi_start = std::atan2(r0.y(), r0.x());
  const double phi_delta = normalize_angle(std::atan2(r1.y(), r1.x()) - phi_start);
  const double radius_start = r0.norm();
  const double radius_end = r1.norm();

  lanelet::Points3d points;
  points.reserve(samples + 1);
  for (size_t i = 0; i <= samples; ++i) {
    if (i == 0) {
      points.push_back(lanelet::Point3d(first));
      continue;
    }
    if (i == samples) {
      points.push_back(lanelet::Point3d(last));
      continue;
    }
    const double t = static_cast<double>(i) / static_cast<double>(samples);
    const double phi = phi_start + phi_delta * t;
    const double radius = radius_start + (radius_end - radius_start) * t;
    const double z = first.z() + (last.z() - first.z()) * t;
    points.emplace_back(
      ids.next(), geometry.center.x() + radius * std::cos(phi),
      geometry.center.y() + radius * std::sin(phi), z);
  }
  return points;
}

//! proper intersection test of two segments
bool segments_intersect(
  const lanelet::BasicPoint2d & p1, const lanelet::BasicPoint2d & p2,
  const lanelet::BasicPoint2d & p3, const lanelet::BasicPoint2d & p4)
{
  const auto cross = [](const lanelet::BasicPoint2d & a, const lanelet::BasicPoint2d & b,
                        const lanelet::BasicPoint2d & c) {
    return (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x());
  };
  const double d1 = cross(p3, p4, p1);
  const double d2 = cross(p3, p4, p2);
  const double d3 = cross(p1, p2, p3);
  const double d4 = cross(p1, p2, p4);
  return ((d1 > 0.0) != (d2 > 0.0)) && ((d3 > 0.0) != (d4 > 0.0));
}

//! The polygon of the two bounds must not intersect itself. A degenerate polygon makes
//! visualization and geometry queries of the lanelet fail, so such a candidate is rejected.
bool is_simple_polygon(const lanelet::Points3d & left, const lanelet::Points3d & right)
{
  std::vector<lanelet::BasicPoint2d> polygon;
  polygon.reserve(left.size() + right.size());
  for (const auto & point : left) {
    polygon.emplace_back(point.x(), point.y());
  }
  for (auto it = right.rbegin(); it != right.rend(); ++it) {
    polygon.emplace_back(it->x(), it->y());
  }

  const size_t size = polygon.size();
  for (size_t i = 0; i < size; ++i) {
    for (size_t j = i + 1; j < size; ++j) {
      if ((i + 1) % size == j || (j + 1) % size == i) {
        continue;  // adjacent edges share a vertex
      }
      if (segments_intersect(
            polygon[i], polygon[(i + 1) % size], polygon[j], polygon[(j + 1) % size])) {
        return false;
      }
    }
  }
  return true;
}

//! samples the driving line of the U-turn: a circular arc from the entry cross section center to
//! the exit cross section center
lanelet::Points3d sample_centerline(
  const TurnGeometry & geometry, const size_t samples, IdAllocator & ids)
{
  lanelet::Points3d points;
  points.reserve(samples + 1);
  for (size_t i = 0; i <= samples; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(samples);
    const double phi = geometry.phi0 + geometry.sweep * t;
    const double z = geometry.z_entry + (geometry.z_exit - geometry.z_entry) * t;
    points.emplace_back(
      ids.next(), geometry.center.x() + geometry.radius * std::cos(phi),
      geometry.center.y() + geometry.radius * std::sin(phi), z);
  }
  return points;
}

lanelet::AttributeMap make_attributes(const TurnGeometry & geometry, const double speed_limit_kmh)
{
  lanelet::AttributeMap attributes;
  attributes["type"] = "lanelet";
  attributes["subtype"] = "road";
  attributes["one_way"] = "yes";
  attributes["location"] = "urban";
  attributes["speed_limit"] = format_speed_limit(speed_limit_kmh);
  // turn_direction drives the turn indicator during the U-turn. Note: it also makes the behavior
  // modules register a turn factor at the entry, but that was measured not to be the reason for
  // the vehicle stopping at the entry (see the notes on the polygon construction above).
  attributes["turn_direction"] = geometry.is_left ? "left" : "right";
  attributes[kUturnAttribute] = "yes";
  return attributes;
}
}  // namespace

size_t inject_uturn_lanelets(
  const lanelet::LaneletMapPtr & map, const std::vector<UturnCandidate> & candidates,
  const UturnParams & params, const rclcpp::Logger & logger)
{
  if (!map) {
    RCLCPP_ERROR(logger, "[uturn] map is a null pointer. Nothing is injected.");
    return 0;
  }

  IdAllocator ids(params.id_base, map);
  size_t injected = 0;
  size_t degenerate_polygons = 0;
  for (const auto & candidate : candidates) {
    const auto geometry =
      compute_turn_geometry(candidate.entry, candidate.exit, params, logger, candidate.id);
    if (!geometry) {
      continue;
    }

    const size_t samples = sample_count(*geometry, params);
    auto left_points = sample_bound(
      candidate.entry.leftBound().back(), candidate.exit.leftBound().front(), *geometry, samples,
      ids);
    auto right_points = sample_bound(
      candidate.entry.rightBound().back(), candidate.exit.rightBound().front(), *geometry, samples,
      ids);
    auto centerline_points = sample_centerline(*geometry, samples, ids);

    // Pinning both ends of both cross sections is required for the routing graph. If a cross
    // section is wider than the turn, the resulting polygon is self intersecting in the middle of
    // the turn. Such a spot still routes and drives correctly (the driving line is the centerline
    // and it stays inside the polygon), so it is accepted; the count is reported once at the end.
    if (!is_simple_polygon(left_points, right_points)) {
      ++degenerate_polygons;
    }

    const lanelet::Id lanelet_id = ids.next();
    lanelet::LineString3d left_bound(ids.next(), left_points);
    lanelet::LineString3d right_bound(ids.next(), right_points);
    lanelet::LineString3d centerline(ids.next(), centerline_points);

    const double speed_limit_kmh =
      candidate.speed_limit_kmh > 0.0 ? candidate.speed_limit_kmh : params.speed_limit_kmh;
    lanelet::Lanelet lanelet(
      lanelet_id, left_bound, right_bound, make_attributes(*geometry, speed_limit_kmh));
    lanelet.setCenterline(centerline);
    // The map lanelets carry a `waypoints` attribute (created by the map loader because
    // use_waypoints is true). RouteHandler::getCenterLinePath() trims the reference path with the
    // waypoints of the neighbouring lanelets, and a lanelet without waypoints loses its reference
    // points there, which makes the path end at that lanelet and stops the vehicle. Tag the
    // synthesized lanelet the same way as the map lanelets.
    lanelet.setAttribute("waypoints", centerline.id());

    map->add(lanelet);
    if (!map->lineStringLayer.exists(centerline.id())) {
      map->add(centerline);
    }

    ++injected;
    RCLCPP_INFO(
      logger,
      "[uturn] injected lanelet %ld for '%s': %s turn, R = %.2f m, heading change = %.1f deg, "
      "speed limit = %s, %zu centerline points",
      lanelet_id, candidate.id.c_str(), geometry->is_left ? "left" : "right", geometry->radius,
      geometry->sweep * 180.0 / M_PI, format_speed_limit(speed_limit_kmh).c_str(), samples + 1);
  }

  if (!candidates.empty()) {
    RCLCPP_INFO(
      logger, "[uturn] %zu of %zu whitelisted U-turn spot(s) were injected.", injected,
      candidates.size());
  }
  if (degenerate_polygons > 0) {
    RCLCPP_WARN(
      logger,
      "[uturn] %zu of the injected lanelets have a cross section wider than the turn, so their "
      "polygon is self intersecting and their visualization may look odd. Routing and the driving "
      "line are not affected.",
      degenerate_polygons);
  }
  return injected;
}

std::optional<lanelet::ConstLanelet> resolve_uturn_lanelet(
  const YAML::Node & node, const lanelet::LaneletMapPtr & map, const std::string & label,
  const rclcpp::Logger & logger)
{
  if (!node) {
    RCLCPP_WARN(logger, "[uturn] '%s': lanelet definition is missing. Skipped.", label.c_str());
    return std::nullopt;
  }

  if (node["lanelet_id"]) {
    const auto id = node["lanelet_id"].as<lanelet::Id>();
    if (!map->laneletLayer.exists(id)) {
      RCLCPP_WARN(
        logger, "[uturn] '%s': lanelet id %ld does not exist in the map. Skipped.", label.c_str(),
        id);
      return std::nullopt;
    }
    return map->laneletLayer.get(id);
  }

  if (node["x"] && node["y"]) {
    const lanelet::BasicPoint2d point{node["x"].as<double>(), node["y"].as<double>()};
    const auto candidates = lanelet::geometry::findWithin2d(map->laneletLayer, point, 3.0);
    for (const auto & candidate : candidates) {
      if (lanelet::geometry::inside(candidate.second, point)) {
        return candidate.second;
      }
    }
    RCLCPP_WARN(
      logger, "[uturn] '%s': no lanelet contains the coordinate (%.2f, %.2f). Skipped.",
      label.c_str(), point.x(), point.y());
    return std::nullopt;
  }

  RCLCPP_WARN(
    logger, "[uturn] '%s': neither 'lanelet_id' nor 'x'/'y' is given. Skipped.", label.c_str());
  return std::nullopt;
}

std::vector<UturnCandidate> load_uturn_candidates(
  const std::string & file_path, const lanelet::LaneletMapPtr & map, const rclcpp::Logger & logger)
{
  std::vector<UturnCandidate> candidates;
  if (file_path.empty()) {
    return candidates;
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(file_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      logger, "[uturn] failed to read the U-turn whitelist '%s': %s", file_path.c_str(), e.what());
    return candidates;
  }

  const auto list = root["uturn"]["candidates"];
  if (!list) {
    RCLCPP_WARN(
      logger, "[uturn] '%s' has no 'uturn.candidates' entry. No U-turn spot is added.",
      file_path.c_str());
    return candidates;
  }
  if (!list.IsSequence()) {
    RCLCPP_ERROR(logger, "[uturn] 'uturn.candidates' of '%s' must be a list.", file_path.c_str());
    return candidates;
  }

  size_t index = 0;
  for (const auto & item : list) {
    const std::string label =
      item["id"] ? item["id"].as<std::string>() : ("candidate_" + std::to_string(index));
    ++index;

    const auto entry = resolve_uturn_lanelet(item["entry"], map, label + ".entry", logger);
    const auto exit = resolve_uturn_lanelet(item["exit"], map, label + ".exit", logger);
    if (!entry || !exit) {
      continue;
    }

    UturnCandidate candidate;
    candidate.id = label;
    candidate.entry = *entry;
    candidate.exit = *exit;
    if (item["speed_limit_kmh"]) {
      candidate.speed_limit_kmh = item["speed_limit_kmh"].as<double>();
    }
    candidates.push_back(candidate);
  }

  RCLCPP_INFO(
    logger, "[uturn] loaded %zu U-turn spot(s) from '%s'", candidates.size(), file_path.c_str());
  return candidates;
}
}  // namespace autoware::map_loader
