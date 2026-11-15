""" Office Assistance Robot scenarios: fixed start / goal configurations.

This module is intentionally free of simulator imports (only numpy, Pillow,
PyYAML) so that scenario configuration and coordinate validation can be
checked on machines without the Pepper 2D simulator installed.

Coordinates are in the map frame of `asl_office_j` (metres), i.e. the same
frame used by PepperRLEnv for `virtual_peppers[i].pos` and `agent_goals[i]`.

NOTE: the occupancy map has no semantic room labels. The office roles below
(reception, pantry, ...) are *designated* to distinct open areas of the map.
Every coordinate was chosen by inspecting the map and is re-checked by
`validate_scenario()` (free cell, clearance, connectivity to the map origin).
See docs/member3_start_goal_navigation.md.

Python 3.6 compatible.
"""
from __future__ import print_function
import os
from collections import deque

import numpy as np

OFFICE_MAP_NAME = "asl_office_j"

# Robot radius used by PepperRLEnv (kRobotRadius = 0.3) plus the 0.2 m margin
# PepperRLEnv uses when computing its own reachable space.
ROBOT_RADIUS = 0.3
DEFAULT_MIN_CLEARANCE = ROBOT_RADIUS + 0.2  # [m]

# Designated office locations in the asl_office_j map frame [m].
OFFICE_LOCATIONS = {
    # large open area at the west end of the floor (entrance lobby)
    "reception": (-23.3, -0.7),
    # east wing, vertical corridor next to the east room block
    "meeting_room": (13.9, 2.4),
    # north corridor, in front of the north-west row of offices
    "employee_cabin": (-11.0, 5.1),
    # south corridor, east of the central hall
    "pantry": (5.0, -1.0),
    # central hall, next to its west wall
    "charging_station": (-1.5, 2.5),
}

# start_yaw: initial robot heading [rad], 0 = facing +x (east)
OFFICE_SCENARIOS = {
    "document_delivery": {
        "description": "Scenario 1: deliver documents from Reception to the Meeting Room",
        "map_name": OFFICE_MAP_NAME,
        "start": "reception",
        "goal": "meeting_room",
        "start_yaw": 0.0,
    },
    "pantry_assistance": {
        "description": "Scenario 2: go from an Employee Cabin to the Pantry",
        "map_name": OFFICE_MAP_NAME,
        "start": "employee_cabin",
        "goal": "pantry",
        "start_yaw": 0.0,
    },
    "meeting_room_assistance": {
        "description": "Scenario 3: leave the Charging Station and go to the Meeting Room",
        "map_name": OFFICE_MAP_NAME,
        "start": "charging_station",
        "goal": "meeting_room",
        "start_yaw": 0.0,
    },
}

SCENARIO_NAMES = sorted(OFFICE_SCENARIOS.keys())


def get_office_scenario(name):
    """ returns a dict with map_name, start_pose (x, y, th), goal_pose (x, y, th) """
    if name not in OFFICE_SCENARIOS:
        raise KeyError("Unknown office scenario '{}'. Available: {}".format(name, SCENARIO_NAMES))
    sc = OFFICE_SCENARIOS[name]
    sx, sy = OFFICE_LOCATIONS[sc["start"]]
    gx, gy = OFFICE_LOCATIONS[sc["goal"]]
    return {
        "name": name,
        "description": sc["description"],
        "map_name": sc["map_name"],
        "start_name": sc["start"],
        "goal_name": sc["goal"],
        # PepperRLEnv poses are [x, y, theta]; goal theta is ignored by the
        # simulator's goal-reached check (it only compares xy).
        "start_pose": np.array([sx, sy, float(sc["start_yaw"])]),
        "goal_pose": np.array([gx, gy, 0.0]),
    }


def default_map_folder():
    """ the maps/ folder shipped with this repository """
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "maps"))


class OccupancyMap(object):
    """ minimal ROS map_server style map (yaml + pgm) reader, used for validation only """
    def __init__(self, map_folder, map_name):
        import yaml
        from PIL import Image
        yaml_path = os.path.join(map_folder, map_name + ".yaml")
        with open(yaml_path, "r") as f:
            meta = yaml.safe_load(f)
        img = np.array(Image.open(os.path.join(map_folder, meta["image"]))).astype(np.float64)
        if img.ndim == 3:
            img = img[:, :, 0]
        occupancy = img / 255. if meta.get("negate", 0) else 1. - img / 255.
        self.resolution = float(meta["resolution"])
        self.origin = np.array(meta["origin"][:2], dtype=np.float64)
        self.free_thresh = float(meta["free_thresh"])
        self.occupied_thresh = float(meta["occupied_thresh"])
        # image row 0 is the top of the map (max y); flip so that row index grows with y
        self.occupancy = occupancy[::-1, :]
        # conservative: occupied AND unknown cells both count as not free
        self.free = self.occupancy < self.free_thresh

    def xy_to_rc(self, x, y):
        """ map frame [m] -> (row, col) in self.free, rows growing with y """
        c = int(np.floor((x - self.origin[0]) / self.resolution))
        r = int(np.floor((y - self.origin[1]) / self.resolution))
        return r, c

    def is_inside(self, x, y):
        r, c = self.xy_to_rc(x, y)
        return 0 <= r < self.free.shape[0] and 0 <= c < self.free.shape[1]

    def clearance_ok(self, x, y, min_clearance):
        """ True if every cell within min_clearance of (x, y) is free and inside the map """
        k = int(np.ceil(min_clearance / self.resolution))
        r, c = self.xy_to_rc(x, y)
        if r - k < 0 or c - k < 0 or r + k >= self.free.shape[0] or c + k >= self.free.shape[1]:
            return False
        window = self.free[r - k:r + k + 1, c - k:c + k + 1]
        dr, dc = np.mgrid[-k:k + 1, -k:k + 1]
        disk = (dr * dr + dc * dc) <= k * k
        return bool(np.all(window[disk]))

    def inflated_coarse_free(self, min_clearance, coarse_res=0.1):
        """ coarse grid (cell = coarse_res m) of cells where the robot center may be.
        A coarse cell is blocked if any fine cell inside it is not free, then blocked
        cells are inflated by min_clearance. Conservative by construction. """
        f = int(round(coarse_res / self.resolution))
        H = (self.free.shape[0] // f) * f
        W = (self.free.shape[1] // f) * f
        blocked = ~self.free[:H, :W].reshape(H // f, f, W // f, f).all(axis=(1, 3))
        k = int(np.ceil(min_clearance / (f * self.resolution)))
        inflated = blocked.copy()
        for dr in range(-k, k + 1):
            for dc in range(-k, k + 1):
                if dr * dr + dc * dc > k * k:
                    continue
                shifted = np.zeros_like(blocked)
                rs = slice(max(dr, 0), blocked.shape[0] + min(dr, 0))
                rd = slice(max(-dr, 0), blocked.shape[0] + min(-dr, 0))
                cs = slice(max(dc, 0), blocked.shape[1] + min(dc, 0))
                cd = slice(max(-dc, 0), blocked.shape[1] + min(-dc, 0))
                shifted[rd, cd] = blocked[rs, cs]
                inflated |= shifted
        return ~inflated, f * self.resolution

    def path_length(self, start_xy, goal_xy, min_clearance, coarse_res=0.1):
        """ approximate shortest collision-free path length [m] between two points
        (8-connected grid search on the inflated coarse grid). None if unreachable. """
        import heapq
        ok, cres = self.inflated_coarse_free(min_clearance, coarse_res)

        def to_cell(p):
            return (int(np.floor((p[1] - self.origin[1]) / cres)),
                    int(np.floor((p[0] - self.origin[0]) / cres)))
        s, t = to_cell(start_xy), to_cell(goal_xy)
        for cell in (s, t):
            if not (0 <= cell[0] < ok.shape[0] and 0 <= cell[1] < ok.shape[1]) or not ok[cell]:
                return None
        nbs = [(1, 0, 1.), (-1, 0, 1.), (0, 1, 1.), (0, -1, 1.),
               (1, 1, 2 ** .5), (1, -1, 2 ** .5), (-1, 1, 2 ** .5), (-1, -1, 2 ** .5)]
        dist = {s: 0.}
        pq = [(0., s)]
        while pq:
            d, u = heapq.heappop(pq)
            if u == t:
                return d * cres
            if d > dist.get(u, np.inf):
                continue
            for dr, dc, cost in nbs:
                v = (u[0] + dr, u[1] + dc)
                if 0 <= v[0] < ok.shape[0] and 0 <= v[1] < ok.shape[1] and ok[v]:
                    nd = d + cost
                    if nd < dist.get(v, np.inf):
                        dist[v] = nd
                        heapq.heappush(pq, (nd, v))
        return None


def validate_pose(occmap, x, y, min_clearance=DEFAULT_MIN_CLEARANCE):
    """ returns a list of error strings (empty if the pose is valid) """
    errors = []
    if not occmap.is_inside(x, y):
        errors.append("({:.2f}, {:.2f}) is outside the map".format(x, y))
        return errors
    r, c = occmap.xy_to_rc(x, y)
    if not occmap.free[r, c]:
        errors.append("({:.2f}, {:.2f}) is not in free space (occupied or unknown)".format(x, y))
    elif not occmap.clearance_ok(x, y, min_clearance):
        errors.append("({:.2f}, {:.2f}) is closer than {:.2f} m to an obstacle/unknown area".format(
            x, y, min_clearance))
    return errors


def validate_scenario(name, map_folder=None, min_clearance=DEFAULT_MIN_CLEARANCE, occmap=None):
    """ checks start and goal of an office scenario against the occupancy map.

    returns (errors, info) where errors is a list of strings (empty = valid) and
    info contains the computed approximate path length.
    """
    sc = get_office_scenario(name)
    if occmap is None:
        occmap = OccupancyMap(map_folder or default_map_folder(), sc["map_name"])
    errors = []
    for label, pose in (("start", sc["start_pose"]), ("goal", sc["goal_pose"])):
        errors += ["{} ({}): {}".format(label, sc[label + "_name"], e)
                   for e in validate_pose(occmap, pose[0], pose[1], min_clearance)]
    info = {"path_length": None}
    if not errors:
        info["path_length"] = occmap.path_length(sc["start_pose"][:2], sc["goal_pose"][:2], min_clearance)
        if info["path_length"] is None:
            errors.append("goal is not reachable from start with {:.2f} m clearance".format(min_clearance))
        # PepperRLEnv computes its reachable space starting from the map origin (0, 0).
        elif occmap.path_length((0., 0.), sc["start_pose"][:2], min_clearance) is None:
            errors.append("start is not connected to the map origin (0, 0), which PepperRLEnv "
                          "uses as the seed of its reachable space")
    return errors, info


def validate_all(map_folder=None, min_clearance=DEFAULT_MIN_CLEARANCE):
    """ validates every scenario, returns {name: (errors, info)} """
    occmaps = {}
    results = {}
    for name in SCENARIO_NAMES:
        map_name = OFFICE_SCENARIOS[name]["map_name"]
        if map_name not in occmaps:
            occmaps[map_name] = OccupancyMap(map_folder or default_map_folder(), map_name)
        results[name] = validate_scenario(name, min_clearance=min_clearance, occmap=occmaps[map_name])
    return results
