#!/usr/bin/env python3
"""Verify the speed-bump slow-down profile from a recorded rosbag.

For every speed bump of the map it reports the planned velocity around the bump and checks that
  - the ego cruises at `--slow-speed` (default 2.3 kph = 0.6389 m/s) through the bump and its
    slow_start_margin / slow_end_margin extension,
  - it decelerates with a constant deceleration down to that speed before the zone,
  - it accelerates with a constant acceleration back up after it,
plus the speed the vehicle actually drove (from /localization/kinematic_state).

Usage:
  ./verify_speed_bump.py --bag /tmp/bump_bag \\
      --map /home/jm/autoware_map/sample-map-planning-0921/lanelet2_map.osm
"""

import argparse
import math
import xml.etree.ElementTree as ET
from collections import defaultdict

import rclpy.serialization
from autoware_planning_msgs.msg import Path, Trajectory
from nav_msgs.msg import Odometry
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

TRAJ_TOPICS = [
    "/planning/scenario_planning/lane_driving/behavior_planning/path",
    "/planning/scenario_planning/trajectory",
]
ODOM_TOPIC = "/localization/kinematic_state"
MSG_TYPES = {
    "autoware_planning_msgs/msg/Trajectory": Trajectory,
    "autoware_planning_msgs/msg/Path": Path,
    "nav_msgs/msg/Odometry": Odometry,
}

PLATEAU_TOL = 1e-3  # [m/s] velocity equality tolerance for the plateau
RAMP_TOL = 0.05  # [m/s^2] tolerance on the implied constant acceleration
FRAME_TARGET_DIST = 12.0  # [m] pick the frame where the ego is this far before the bump


# ---------------------------------------------------------------- map geometry
def load_bumps(osm_path):
    root = ET.parse(osm_path).getroot()
    nodes = {}
    for nd in root.findall("node"):
        tags = {t.get("k"): t.get("v") for t in nd.findall("tag")}
        if "local_x" in tags and "local_y" in tags:
            nodes[nd.get("id")] = (float(tags["local_x"]), float(tags["local_y"]))

    bumps = []
    for way in root.findall("way"):
        tags = {t.get("k"): t.get("v") for t in way.findall("tag")}
        if tags.get("type") == "speed_bump":
            poly = [nodes[nd.get("ref")] for nd in way.findall("nd") if nd.get("ref") in nodes]
            if len(poly) >= 3:
                bumps.append(poly)
    return bumps


def _dist_point_segment(p, a, b):
    (px, py), (ax, ay), (bx, by) = p, a, b
    dx, dy = bx - ax, by - ay
    denom = dx * dx + dy * dy
    t = 0.0 if denom == 0.0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _inside(p, poly):
    x, y = p
    inside = False
    for i in range(len(poly)):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % len(poly)]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


def distance_to_polygon(p, poly):
    """Distance to the polygon boundary, negative when the point is inside."""
    d = min(_dist_point_segment(p, poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly)))
    return -d if _inside(p, poly) else d


# ------------------------------------------------------------------ bag reading
def read_bag(bag_path):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_path, storage_id="sqlite3"), ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = {t for t in TRAJ_TOPICS + [ODOM_TOPIC] if t in types}

    traj, odom = defaultdict(list), []
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        if topic not in wanted:
            continue
        msg = rclpy.serialization.deserialize_message(raw, MSG_TYPES[types[topic]])
        if topic == ODOM_TOPIC:
            odom.append((stamp * 1e-9, msg.pose.pose.position.x, msg.pose.pose.position.y,
                         float(msg.twist.twist.linear.x)))
        else:
            traj[topic].append((stamp, msg))
    return traj, odom


def path_points(msg):
    return [(pt.pose.position.x, pt.pose.position.y, float(pt.longitudinal_velocity_mps))
            for pt in msg.points]


def arc_lengths(pts):
    s = [0.0]
    for (x0, y0, _), (x1, y1, _) in zip(pts, pts[1:]):
        s.append(s[-1] + math.hypot(x1 - x0, y1 - y0))
    return s


# -------------------------------------------------------------------- checking
def analyse_frame(bump, pts, s, slow_speed, decel, accel):
    """Print the profile around one bump and check the ramp law. Returns True when OK."""
    dists = [distance_to_polygon((x, y), bump) for x, y, _ in pts]
    vels = [v for _, _, v in pts]

    plateau = [i for i, v in enumerate(vels) if abs(v - slow_speed) < PLATEAU_TOL]
    zone = f"[-{dists[0]:.2f} .. {dists[-1]:.2f}] m rel. bump, ego {dists[0]:.2f} m before it"
    print(f"  frame: {zone}")

    if not plateau:
        print(f"  FAIL: no path point cruises at {slow_speed:.4f} m/s ({slow_speed * 3.6:.2f} kph)")
        print("        (is the bump regulatory element on the route / the module loaded?)")
        return False
    lo, hi = plateau[0], plateau[-1]
    print(f"  plateau: {len(plateau)} points at {min(vels[lo:hi + 1]) * 3.6:.2f}"
          f"..{max(vels[lo:hi + 1]) * 3.6:.2f} kph,"
          f" from {dists[lo]:+.2f} m to {dists[hi]:+.2f} m rel. the bump boundary")

    ok = True
    for side, rng, want, label in (
        ("approach", range(lo), decel, "deceleration"),
        ("exit", range(hi + 1, len(vels)), accel, "acceleration"),
    ):
        implied = []
        for i in rng:
            ds = (s[lo] - s[i]) if side == "approach" else (s[i] - s[hi])
            if ds > 0.5:
                implied.append((vels[i] ** 2 - slow_speed ** 2) / (2.0 * ds))
        if not implied:
            print(f"  {side}: not visible in this frame")
            continue
        spread = max(implied) - min(implied)
        v_end = vels[0] if side == "approach" else vels[-1]
        print(f"  {side}: implied {label} {min(implied):.3f}..{max(implied):.3f} m/s^2 "
              f"(configured {want:.2f}), reaching {v_end * 3.6:.2f} kph at the "
              f"{'ramp start' if side == 'approach' else 'path end'}")
        if spread > RAMP_TOL or abs(sum(implied) / len(implied) - want) > RAMP_TOL:
            ok = False
    return ok


def pick_frame(msgs, bump):
    """Frame in which the ego is ~FRAME_TARGET_DIST before the bump and still moving."""
    best = None
    for _, msg in msgs:
        pts = path_points(msg)
        if len(pts) < 3:
            continue
        d_ego = distance_to_polygon(pts[0][:2], bump)
        if d_ego < 0.0 or pts[0][2] < 0.2:  # already on the bump, or standing still
            continue
        if best is None or abs(d_ego - FRAME_TARGET_DIST) < abs(best[0] - FRAME_TARGET_DIST):
            best = (d_ego, arc_lengths(pts), pts)
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--map", required=True)
    ap.add_argument("--slow-speed", type=float, default=0.6389, help="[m/s] expected cruise speed")
    ap.add_argument("--deceleration", type=float, default=0.5, help="[m/s^2] configured approach a")
    ap.add_argument("--acceleration", type=float, default=0.5, help="[m/s^2] configured exit a")
    ap.add_argument("--topic", default=None)
    args = ap.parse_args()

    bumps = load_bumps(args.map)
    if not bumps:
        raise SystemExit(f"no speed_bump way found in {args.map}")
    print(f"map: {len(bumps)} speed bump(s), expecting {args.slow_speed * 3.6:.2f} kph over them")

    traj, odom = read_bag(args.bag)
    if odom:
        print(f"odometry: {len(odom)} samples, ego drove "
              f"{min(o[3] for o in odom) * 3.6:.2f}..{max(o[3] for o in odom) * 3.6:.2f} kph")

    overall = True
    for topic in ([args.topic] if args.topic else [t for t in TRAJ_TOPICS if t in traj]):
        print(f"\n########## {topic} ({len(traj[topic])} messages) ##########")
        for i, bump in enumerate(bumps):
            cx = sum(p[0] for p in bump) / len(bump)
            cy = sum(p[1] for p in bump) / len(bump)
            print(f"\n=== speed bump #{i} at ({cx:.1f}, {cy:.1f}) ===")
            picked = pick_frame(traj[topic], bump)
            if picked is None:
                print("  no frame with the ego approaching the bump (passed before the record?)")
                overall = False
                continue
            _, s, pts = picked
            overall &= analyse_frame(bump, pts, s, args.slow_speed,
                                     args.deceleration, args.acceleration)

    # the executed speed, as a sanity check on top of the planned profile
    if odom and bumps:
        bump = bumps[0]
        near = [(math.hypot(o[1] - sum(p[0] for p in bump) / len(bump),
                            o[2] - sum(p[1] for p in bump) / len(bump)), o[3]) for o in odom]
        near = [v for d, v in near if d < 12.0]
        if near:
            print(f"\n=== vehicle actually drove {min(near) * 3.6:.2f}..{max(near) * 3.6:.2f} kph "
                  f"within 12 m of speed bump #0 (min {min(near):.4f} m/s) ===")

    print("\nRESULT:", "OK" if overall else "CHECK FAILED")


if __name__ == "__main__":
    main()
