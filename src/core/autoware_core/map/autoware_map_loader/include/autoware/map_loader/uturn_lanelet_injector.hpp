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

#ifndef AUTOWARE__MAP_LOADER__UTURN_LANELET_INJECTOR_HPP_
#define AUTOWARE__MAP_LOADER__UTURN_LANELET_INJECTOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <yaml-cpp/yaml.h>

#include <optional>
#include <string>
#include <vector>

namespace autoware::map_loader
{
/// @brief attribute that marks a lanelet synthesized by this module.
constexpr const char * kUturnAttribute = "autoware_uturn";

/// @brief One U-turn spot taken from the operator maintained whitelist.
///
/// The vehicle drives `entry` in its own direction, leaves it at its end edge, and joins `exit`
/// at its start edge.
struct UturnCandidate
{
  std::string id;               //!< label used in log messages
  lanelet::ConstLanelet entry;  //!< lanelet the vehicle leaves
  lanelet::ConstLanelet exit;   //!< opposite direction lanelet the vehicle joins
  double speed_limit_kmh{5.0};  //!< speed limit tagged on the synthesized lanelet
};

/// @brief Parameters of the synthesized U-turn geometry.
struct UturnParams
{
  double wheel_base{1.0};        //!< [m] must match vehicle_info.param.yaml
  double max_steer_angle{0.70};  //!< [rad] must match vehicle_info.param.yaml
  double radius_margin{1.2};     //!< a candidate is accepted only if R >= radius_margin * r_min
  double arc_resolution{0.2};    //!< [m] sampling resolution of the synthesized centerline
  double speed_limit_kmh{5.0};   //!< default speed limit of the synthesized lanelet
  double min_turn_angle_deg{150.0};  //!< accepted |heading change| window
  double max_turn_angle_deg{210.0};
  lanelet::Id id_base{2000000};  //!< first id used for synthesized lanelets
};

/// @brief Adds one single arc U-turn lanelet per candidate to the map.
///
/// The synthesized lanelet reuses the *point objects* of the neighbouring lanelets, which is what
/// lanelet2's routing graph requires to accept it as a successor/predecessor. Candidates that fail
/// the geometric or vehicle feasibility check are skipped with a warning; map loading never fails.
///
/// @return number of injected lanelets
size_t inject_uturn_lanelets(
  const lanelet::LaneletMapPtr & map, const std::vector<UturnCandidate> & candidates,
  const UturnParams & params, const rclcpp::Logger & logger);

/// @brief Reads the whitelist YAML and resolves every entry/exit into a lanelet of the map.
///
/// Supported file layout:
/// @code{.yaml}
/// uturn:
///   candidates:
///     - id: "junction_1"
///       entry: {lanelet_id: 802}   # or {x: 1.0, y: 2.0} in the map frame
///       exit:  {lanelet_id: 826}
///       speed_limit_kmh: 5.0       # optional
/// @endcode
std::vector<UturnCandidate> load_uturn_candidates(
  const std::string & file_path, const lanelet::LaneletMapPtr & map, const rclcpp::Logger & logger);

/// @brief Resolves a whitelist entry into a lanelet: either a direct id or the lanelet containing
/// the given map frame coordinate.
std::optional<lanelet::ConstLanelet> resolve_uturn_lanelet(
  const YAML::Node & node, const lanelet::LaneletMapPtr & map, const std::string & label,
  const rclcpp::Logger & logger);
}  // namespace autoware::map_loader

#endif  // AUTOWARE__MAP_LOADER__UTURN_LANELET_INJECTOR_HPP_
