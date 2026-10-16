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

#include <../src/lanelet2_plugins/default_planner.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_planning_msgs::msg::LaneletRoute;
using geometry_msgs::msg::Pose;
using RoutePoints = std::vector<geometry_msgs::msg::Pose>;

constexpr lanelet::Id kEntryId = 101;
constexpr lanelet::Id kExitId = 102;
constexpr lanelet::Id kUturnId = 103;
constexpr lanelet::Id kLong1Id = 104;
constexpr lanelet::Id kTurnaroundId = 105;
constexpr lanelet::Id kLong2Id = 106;
//! the length of the entry lanelet, the U-turn spot is at its end edge
constexpr double kEntryLength = 12.0;
//! the cross section centers of the entry end edge and the exit start edge are this far apart,
//! which results in a single arc of radius 2.06 m
constexpr double kGap = 4.12;

lanelet::Point3d new_point(const lanelet::Id id, const double x, const double y)
{
  return lanelet::Point3d(id, x, y, 0.0);
}

lanelet::LineString3d make_line(const lanelet::Id id, const lanelet::Points3d & points)
{
  return lanelet::LineString3d(id, points);
}

lanelet::Lanelet make_lanelet(
  const lanelet::Id id, const lanelet::Points3d & left, const lanelet::Points3d & right,
  const lanelet::Points3d & centerline)
{
  lanelet::Lanelet lanelet(id, make_line(id * 10 + 1, left), make_line(id * 10 + 2, right));
  lanelet.setAttribute("type", "lanelet");
  lanelet.setAttribute("subtype", "road");
  lanelet.setAttribute("one_way", "yes");
  lanelet.setAttribute("location", "urban");
  lanelet.setCenterline(make_line(id * 10 + 3, centerline));
  return lanelet;
}

//! samples a single arc of `radius` around `center` from `phi0` by `sweep` radians. The first and
//! the last point are the given points, so that the lanelet bounds are pinned to the neighbours.
lanelet::Points3d arc_points(
  const lanelet::Id first_point_id, const lanelet::BasicPoint2d & center, const double radius,
  const double phi0, const double sweep, const size_t samples)
{
  lanelet::Points3d points;
  for (size_t i = 0; i <= samples; ++i) {
    const double phi = phi0 + sweep * static_cast<double>(i) / static_cast<double>(samples);
    points.emplace_back(
      first_point_id + i, center.x() + radius * std::cos(phi),
      center.y() + radius * std::sin(phi), 0.0);
  }
  return points;
}

//! The entry (eastbound, `kEntryLength` long) and the exit (westbound) which starts `kGap` to the
//! north of the entry end edge. The exit is only reachable by a U-turn unless the map provides
//! another way, see the long detour added below.
struct MiniMap
{
  lanelet::LaneletMapPtr map{std::make_shared<lanelet::LaneletMap>()};
  lanelet::ConstPoint3d entry_end_left;
  lanelet::ConstPoint3d entry_end_right;
  lanelet::ConstPoint3d exit_start_left;
  lanelet::ConstPoint3d exit_start_right;

  explicit MiniMap(const bool add_long_detour)
  {
    // entry: heading east
    const auto entry_left = lanelet::Points3d{
      new_point(1011, 0.0, 2.0), new_point(1012, kEntryLength, 2.0)};
    const auto entry_right = lanelet::Points3d{
      new_point(1021, 0.0, -2.0), new_point(1022, kEntryLength, -2.0)};
    const auto entry_centerline = lanelet::Points3d{
      new_point(1031, 0.0, 0.0), new_point(1032, kEntryLength, 0.0)};
    map->add(make_lanelet(kEntryId, entry_left, entry_right, entry_centerline));

    // exit: heading west, its start edge is the entry end edge shifted `kGap` to the north
    const auto exit_left =
      lanelet::Points3d{new_point(10211, kEntryLength, kGap - 2.0), new_point(10212, 0.0, kGap - 2.0)};
    const auto exit_right =
      lanelet::Points3d{new_point(10221, kEntryLength, kGap + 2.0), new_point(10222, 0.0, kGap + 2.0)};
    const auto exit_centerline =
      lanelet::Points3d{new_point(10231, kEntryLength, kGap), new_point(10232, 0.0, kGap)};
    map->add(make_lanelet(kExitId, exit_left, exit_right, exit_centerline));

    const auto entry = map->laneletLayer.get(kEntryId);
    const auto exit = map->laneletLayer.get(kExitId);
    entry_end_left = entry.leftBound().back();
    entry_end_right = entry.rightBound().back();
    exit_start_left = exit.leftBound().front();
    exit_start_right = exit.rightBound().front();

    if (add_long_detour) {
      add_detour();
    }
  }

  //! A long way around: east on `kLong1Id`, a map lanelet that turns around at
  //! x = kEntryLength + 48, and back west on `kLong2Id` into the exit. This is a legal forward
  //! route, but much longer than the U-turn.
  void add_detour()
  {
    const double x_far = kEntryLength + 48.0;

    const auto long1_left =
      lanelet::Points3d{lanelet::Point3d(entry_end_left), new_point(10411, x_far, 2.0)};
    const auto long1_right =
      lanelet::Points3d{lanelet::Point3d(entry_end_right), new_point(10421, x_far, -2.0)};
    const auto long1_centerline =
      lanelet::Points3d{new_point(10431, kEntryLength, 0.0), new_point(10432, x_far, 0.0)};
    map->add(make_lanelet(kLong1Id, long1_left, long1_right, long1_centerline));

    const auto long1 = map->laneletLayer.get(kLong1Id);
    // the turnaround connects the end edge of the eastbound lane with the start edge of the
    // westbound lane, i.e. a heading change of 180 degrees with a radius of 2.06 m
    const auto turnaround_left = lanelet::Points3d{
      lanelet::Point3d(long1.leftBound().back()), new_point(10511, x_far + 0.3, kGap / 2.0),
      new_point(10512, x_far, kGap - 2.0)};
    const auto turnaround_right = lanelet::Points3d{
      lanelet::Point3d(long1.rightBound().back()), new_point(10521, x_far + 3.0, 0.0),
      new_point(10522, x_far + 3.0, kGap), new_point(10523, x_far, kGap + 2.0)};
    const auto turnaround_centerline =
      arc_points(10531, {x_far, kGap / 2.0}, kGap / 2.0, -M_PI / 2.0, M_PI, 16);
    map->add(make_lanelet(kTurnaroundId, turnaround_left, turnaround_right, turnaround_centerline));

    const auto turnaround = map->laneletLayer.get(kTurnaroundId);
    const auto long2_left = lanelet::Points3d{
      lanelet::Point3d(turnaround.leftBound().back()), lanelet::Point3d(exit_start_left)};
    const auto long2_right = lanelet::Points3d{
      lanelet::Point3d(turnaround.rightBound().back()), lanelet::Point3d(exit_start_right)};
    const auto long2_centerline =
      lanelet::Points3d{new_point(10631, x_far, kGap), new_point(10632, kEntryLength, kGap)};
    map->add(make_lanelet(kLong2Id, long2_left, long2_right, long2_centerline));
  }

  void add_uturn()
  {
    const auto uturn_left = lanelet::Points3d{
      lanelet::Point3d(entry_end_left), lanelet::Point3d(exit_start_left)};
    const auto uturn_right = lanelet::Points3d{
      lanelet::Point3d(entry_end_right), lanelet::Point3d(exit_start_right)};
    const auto uturn_centerline =
      arc_points(10331, {kEntryLength, kGap / 2.0}, kGap / 2.0, -M_PI / 2.0, M_PI, 32);
    auto uturn = make_lanelet(kUturnId, uturn_left, uturn_right, uturn_centerline);
    uturn.setAttribute("turn_direction", "left");
    uturn.setAttribute("autoware_uturn", "yes");
    map->add(uturn);
  }
};

LaneletMapBin to_map_bin(const lanelet::LaneletMapPtr & map)
{
  LaneletMapBin map_bin;
  map_bin.header.frame_id = "map";
  lanelet::utils::conversion::toBinMsg(map, &map_bin);
  return map_bin;
}

Pose make_pose(const double x, const double y, const double yaw)
{
  Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0.0;
  pose.orientation.z = std::sin(yaw * 0.5);
  pose.orientation.w = std::cos(yaw * 0.5);
  return pose;
}

// access the protected route handler to set the map under test
struct DefaultPlanner : public autoware::mission_planner_universe::lanelet2::DefaultPlanner
{
  void set_map(const LaneletMapBin & map_bin) { route_handler_.setMap(map_bin); }
};

//! The goal is inside the exit lanelet, which is 35 m west of the U-turn spot and far east of the
//! map: a U-turn takes about 15 m, the detour around the turnaround about 120 m.
RoutePoints uturn_goal_points()
{
  return {make_pose(1.0, 0.0, 0.0), make_pose(kEntryLength / 2.0, kGap, M_PI)};
}

bool route_contains(const LaneletRoute & route, const lanelet::Id id)
{
  return std::any_of(
    route.segments.begin(), route.segments.end(),
    [id](const auto & segment) { return segment.preferred_primitive.id == id; });
}

class UturnFallbackTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }

  void TearDown() override
  {
    planner_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  void reset_planner(const double max_forward_detour_ratio)
  {
    planner_.reset();
    node_.reset();

    rclcpp::NodeOptions options;
    // The parameters are set here instead of using autoware_test_utils so that this test only
    // depends on the packages of this repository. The vehicle geometry is the one of
    // autoware_test_utils/config/test_vehicle_info.param.yaml.
    options.parameter_overrides(
      {{"wheel_radius", 0.383},
       {"wheel_width", 0.235},
       {"wheel_base", 2.79},
       {"wheel_tread", 1.64},
       {"front_overhang", 1.0},
       {"rear_overhang", 1.1},
       {"left_overhang", 0.128},
       {"right_overhang", 0.128},
       {"vehicle_height", 2.5},
       {"max_steer_angle", 0.70},
       {"goal_angle_threshold_deg", 45.0},
       {"enable_correct_goal_pose", false},
       {"consider_no_drivable_lanes", false},
       {"max_forward_detour_ratio", max_forward_detour_ratio},
       // the test map is smaller than the footprint of the test vehicle
       {"check_footprint_inside_lanes", false}});

    node_ = std::make_shared<rclcpp::Node>("test_uturn_fallback_node", options);
    planner_ = std::make_unique<DefaultPlanner>();
    planner_->initialize(node_.get());
  }

  void set_map(const lanelet::LaneletMapPtr & map) { planner_->set_map(to_map_bin(map)); }

  LaneletRoute plan(const RoutePoints & points) { return planner_->plan(points); }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<DefaultPlanner> planner_;
};

// the map has no connection from the entry to the exit, so a U-turn is the only way
TEST_F(UturnFallbackTest, plansRouteThroughUturnWhenNoForwardRouteExists)
{
  reset_planner(0.0);
  MiniMap mini_map(false);
  mini_map.add_uturn();
  set_map(mini_map.map);

  const auto route = plan(uturn_goal_points());

  ASSERT_FALSE(route.segments.empty()) << "a route through the U-turn lanelet must be planned";
  EXPECT_TRUE(route_contains(route, kUturnId));
}

// without a U-turn lanelet the very same request has to fail, otherwise the test above would not
// prove anything about the fallback
TEST_F(UturnFallbackTest, failsWhenTheMapHasNoUturnLanelet)
{
  reset_planner(0.0);
  MiniMap mini_map(false);
  set_map(mini_map.map);

  EXPECT_TRUE(plan(uturn_goal_points()).segments.empty());
}

// a goal that can be reached by driving forward must not be routed through a U-turn
TEST_F(UturnFallbackTest, prefersForwardRoute)
{
  reset_planner(0.0);
  MiniMap mini_map(false);
  mini_map.add_uturn();
  set_map(mini_map.map);

  const RoutePoints points{make_pose(1.0, 0.0, 0.0), make_pose(9.0, 0.0, 0.0)};
  const auto route = plan(points);

  ASSERT_FALSE(route.segments.empty());
  EXPECT_FALSE(route_contains(route, kUturnId))
    << "a forward route exists, so the U-turn lanelet must not be used";
}

// the map has both a forward route (a long detour around a turnaround) and a U-turn. With the
// default detour ratio the forward route is kept even though it is much longer.
TEST_F(UturnFallbackTest, keepsForwardRouteByDefault)
{
  reset_planner(0.0);
  MiniMap mini_map(true);
  mini_map.add_uturn();
  set_map(mini_map.map);

  const auto route = plan(uturn_goal_points());

  ASSERT_FALSE(route.segments.empty()) << "the detour must be found";
  EXPECT_TRUE(route_contains(route, kTurnaroundId)) << "the detour around the turnaround is used";
  EXPECT_FALSE(route_contains(route, kUturnId));
}

// with a detour ratio the much shorter U-turn wins over the forward route
TEST_F(UturnFallbackTest, prefersUturnWhenTheForwardDetourIsTooLong)
{
  reset_planner(3.0);
  MiniMap mini_map(true);
  mini_map.add_uturn();
  set_map(mini_map.map);

  const auto route = plan(uturn_goal_points());

  ASSERT_FALSE(route.segments.empty());
  EXPECT_TRUE(route_contains(route, kUturnId));
  EXPECT_FALSE(route_contains(route, kTurnaroundId));
}
}  // namespace
