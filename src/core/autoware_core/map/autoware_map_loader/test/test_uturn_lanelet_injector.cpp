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

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace
{
using autoware::map_loader::inject_uturn_lanelets;
using autoware::map_loader::kUturnAttribute;
using autoware::map_loader::UturnCandidate;
using autoware::map_loader::UturnParams;

constexpr lanelet::Id kEntryId = 1;
constexpr lanelet::Id kExitId = 2;
constexpr lanelet::Id kParallelId = 3;

lanelet::LineString3d make_bound(
  const lanelet::Id id, const std::vector<std::pair<double, double>> & points)
{
  lanelet::Points3d lanelet_points;
  lanelet::Id point_id = id * 100;
  for (const auto & point : points) {
    lanelet_points.emplace_back(point_id++, point.first, point.second, 0.0);
  }
  return lanelet::LineString3d(id, lanelet_points);
}

//! A minimal U-turn pair: `entry` drives east, `exit` drives west and is `gap` meters north of
//! the entry end edge. With gap = 4.12 the single arc has a radius of 2.06 m.
struct TestMap
{
  lanelet::LaneletMapPtr map{std::make_shared<lanelet::LaneletMap>()};

  explicit TestMap(const double gap = 4.12)
  {
    // entry: 4 m long, 4 m wide, heading east
    const auto entry_left = make_bound(11, {{0.0, 2.0}, {4.0, 2.0}});
    const auto entry_right = make_bound(12, {{0.0, -2.0}, {4.0, -2.0}});
    lanelet::Lanelet entry(kEntryId, entry_left, entry_right);
    entry.setAttribute("type", "lanelet");
    entry.setAttribute("subtype", "road");
    entry.setAttribute("one_way", "yes");
    entry.setAttribute("location", "urban");
    entry.setCenterline(make_bound(13, {{0.0, 0.0}, {4.0, 0.0}}));
    map->add(entry);

    // exit: heading west, start edge centered at (4, gap)
    const auto exit_left = make_bound(21, {{4.0, gap - 2.0}, {0.0, gap - 2.0}});
    const auto exit_right = make_bound(22, {{4.0, gap + 2.0}, {0.0, gap + 2.0}});
    lanelet::Lanelet exit(kExitId, exit_left, exit_right);
    exit.setAttribute("type", "lanelet");
    exit.setAttribute("subtype", "road");
    exit.setAttribute("one_way", "yes");
    exit.setAttribute("location", "urban");
    exit.setCenterline(make_bound(23, {{4.0, gap}, {0.0, gap}}));
    map->add(exit);

    // a lanelet that is parallel to the entry, i.e. it can never be the exit of a U-turn
    const auto parallel_left = make_bound(31, {{0.0, 10.0}, {4.0, 10.0}});
    const auto parallel_right = make_bound(32, {{0.0, 6.0}, {4.0, 6.0}});
    lanelet::Lanelet parallel(kParallelId, parallel_left, parallel_right);
    parallel.setAttribute("type", "lanelet");
    parallel.setAttribute("subtype", "road");
    parallel.setAttribute("one_way", "yes");
    parallel.setAttribute("location", "urban");
    parallel.setCenterline(make_bound(33, {{0.0, 8.0}, {4.0, 8.0}}));
    map->add(parallel);
  }

  UturnCandidate candidate() const
  {
    UturnCandidate candidate;
    candidate.id = "test_spot";
    candidate.entry = map->laneletLayer.get(kEntryId);
    candidate.exit = map->laneletLayer.get(kExitId);
    candidate.speed_limit_kmh = 5.0;
    return candidate;
  }

  std::optional<lanelet::ConstLanelet> uturn_lanelet() const
  {
    for (const auto & lanelet : map->laneletLayer) {
      if (lanelet.hasAttribute(kUturnAttribute)) {
        return lanelet;
      }
    }
    return std::nullopt;
  }

  std::unique_ptr<lanelet::routing::RoutingGraph> routing_graph() const
  {
    return lanelet::routing::RoutingGraph::build(
      *map,
      *lanelet::traffic_rules::TrafficRulesFactory::create(
        lanelet::Locations::Germany, lanelet::Participants::Vehicle));
  }
};

UturnParams default_params()
{
  UturnParams params;
  params.wheel_base = 1.0;
  params.max_steer_angle = 0.70;
  params.radius_margin = 1.2;
  return params;
}

TEST(TestUturnLaneletInjector, injectsSingleArcLanelet)
{
  const TestMap test_map;
  const auto params = default_params();

  const size_t injected = inject_uturn_lanelets(
    test_map.map, {test_map.candidate()}, params, rclcpp::get_logger("test"));
  ASSERT_EQ(injected, 1U);

  const auto uturn = test_map.uturn_lanelet();
  ASSERT_TRUE(uturn.has_value());

  // The synthesized lanelet must not be tagged as a turning lane: a turn direction makes the
  // behavior modules register a turn factor with a zero velocity control point at the U-turn
  // entry, which stops the vehicle there.
  EXPECT_EQ(uturn->attribute("turn_direction").value(), "left");
  EXPECT_EQ(uturn->attribute("one_way").value(), "yes");
  EXPECT_EQ(uturn->attribute("speed_limit").value(), "5");
  EXPECT_TRUE(uturn->hasCustomCenterline());

  // the bounds must be pinned to the *existing* points of the neighbouring lanelets, this is what
  // lanelet2 requires to accept the lanelet as a successor/predecessor
  const auto entry = test_map.map->laneletLayer.get(kEntryId);
  const auto exit = test_map.map->laneletLayer.get(kExitId);
  EXPECT_TRUE(uturn->leftBound().front() == entry.leftBound().back());
  EXPECT_TRUE(uturn->rightBound().front() == entry.rightBound().back());
  EXPECT_TRUE(uturn->leftBound().back() == exit.leftBound().front());
  EXPECT_TRUE(uturn->rightBound().back() == exit.rightBound().front());
}

TEST(TestUturnLaneletInjector, centerlineIsTheExpectedSingleArc)
{
  const TestMap test_map;
  const auto params = default_params();
  ASSERT_EQ(
    inject_uturn_lanelets(test_map.map, {test_map.candidate()}, params, rclcpp::get_logger("test")),
    1U);

  const auto uturn = test_map.uturn_lanelet();
  ASSERT_TRUE(uturn.has_value());

  // entry end edge center is (4, 0), exit start edge center is (4, 4.12), so the arc has a radius
  // of 2.06 m and its center is at (4, 2.06)
  const lanelet::BasicPoint2d center{4.0, 2.06};
  const double expected_radius = 2.06;

  const auto & centerline = uturn->centerline();
  ASSERT_GE(centerline.size(), 8U);
  for (const auto & point : centerline) {
    const lanelet::BasicPoint2d p{point.x(), point.y()};
    EXPECT_NEAR((p - center).norm(), expected_radius, 1e-6);
  }

  // driving direction: from the entry to the exit
  EXPECT_NEAR(centerline.front().x(), 4.0, 1e-6);
  EXPECT_NEAR(centerline.front().y(), 0.0, 1e-6);
  EXPECT_NEAR(centerline.back().x(), 4.0, 1e-6);
  EXPECT_NEAR(centerline.back().y(), 4.12, 1e-6);

  // the arc must be drivable with the minimum turning radius of the vehicle
  const double min_radius = params.wheel_base / std::tan(params.max_steer_angle);
  EXPECT_GE(expected_radius, params.radius_margin * min_radius);
}

TEST(TestUturnLaneletInjector, isRoutable)
{
  const TestMap test_map;
  const auto params = default_params();
  ASSERT_EQ(
    inject_uturn_lanelets(test_map.map, {test_map.candidate()}, params, rclcpp::get_logger("test")),
    1U);

  const auto uturn = test_map.uturn_lanelet();
  ASSERT_TRUE(uturn.has_value());
  const auto graph = test_map.routing_graph();

  const auto entry = test_map.map->laneletLayer.get(kEntryId);
  const auto exit = test_map.map->laneletLayer.get(kExitId);

  // the U-turn lanelet is a successor of the entry and a predecessor of the exit
  const auto following_entry = graph->following(entry);
  EXPECT_NE(
    std::find(following_entry.begin(), following_entry.end(), *uturn), following_entry.end());
  const auto following_uturn = graph->following(*uturn);
  EXPECT_NE(std::find(following_uturn.begin(), following_uturn.end(), exit), following_uturn.end());

  // a route from the entry to the exit exists and goes through the U-turn lanelet
  const auto route = graph->getRoute(entry, exit, 0);
  ASSERT_TRUE(!!route);
  const auto path = route->shortestPath();
  EXPECT_NE(std::find(path.begin(), path.end(), *uturn), path.end());

  // one_way: the U-turn must not be drivable in the opposite direction
  EXPECT_FALSE(!!graph->getRoute(exit, entry, 0));
}

TEST(TestUturnLaneletInjector, survivesMapBinSerialization)
{
  const TestMap test_map;
  const auto params = default_params();
  ASSERT_EQ(
    inject_uturn_lanelets(test_map.map, {test_map.candidate()}, params, rclcpp::get_logger("test")),
    1U);

  autoware_map_msgs::msg::LaneletMapBin map_bin;
  lanelet::utils::conversion::toBinMsg(test_map.map, &map_bin);

  auto deserialized = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(map_bin, deserialized);

  const auto uturn = [&]() -> std::optional<lanelet::ConstLanelet> {
    for (const auto & lanelet : deserialized->laneletLayer) {
      if (lanelet.hasAttribute(kUturnAttribute)) {
        return lanelet;
      }
    }
    return std::nullopt;
  }();
  ASSERT_TRUE(uturn.has_value()) << "the synthesized lanelet must survive the map bin";
  EXPECT_TRUE(uturn->hasCustomCenterline());

  // the relations must survive as well, which requires that the shared points are still shared
  const auto entry = deserialized->laneletLayer.get(kEntryId);
  const auto exit = deserialized->laneletLayer.get(kExitId);
  const auto graph = lanelet::routing::RoutingGraph::build(
    *deserialized,
    *lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::Locations::Germany, lanelet::Participants::Vehicle));
  EXPECT_TRUE(!!graph->getRoute(entry, exit, 0));
  EXPECT_TRUE(!!graph->getRoute(entry, *uturn, 0));
}

TEST(TestUturnLaneletInjector, rejectsCandidatesThatTheVehicleCannotDrive)
{
  const auto params = default_params();

  // the gap is too small for the minimum turning radius (R = 0.5 m < 1.2 * 1.187 m)
  {
    const TestMap test_map(1.0);
    EXPECT_EQ(
      inject_uturn_lanelets(
        test_map.map, {test_map.candidate()}, params, rclcpp::get_logger("test")),
      0U);
    EXPECT_FALSE(test_map.uturn_lanelet().has_value());
  }

  // the entry and the exit are parallel lanes instead of opposite directions
  {
    TestMap test_map;
    auto candidate = test_map.candidate();
    candidate.exit = test_map.map->laneletLayer.get(kParallelId);
    EXPECT_EQ(
      inject_uturn_lanelets(test_map.map, {candidate}, params, rclcpp::get_logger("test")), 0U);
    EXPECT_FALSE(test_map.uturn_lanelet().has_value());
  }

  // the entry is used as exit as well
  {
    TestMap test_map;
    auto candidate = test_map.candidate();
    candidate.exit = test_map.map->laneletLayer.get(kEntryId);
    EXPECT_EQ(
      inject_uturn_lanelets(test_map.map, {candidate}, params, rclcpp::get_logger("test")), 0U);
    EXPECT_FALSE(test_map.uturn_lanelet().has_value());
  }
}

TEST(TestUturnLaneletInjector, rejectsEmptyCandidateList)
{
  const TestMap test_map;
  EXPECT_EQ(
    inject_uturn_lanelets(test_map.map, {}, default_params(), rclcpp::get_logger("test")), 0U);
  EXPECT_FALSE(test_map.uturn_lanelet().has_value());
}
}  // namespace
