""" Office Assistance Robot: configurable static obstacles per office scenario.

The Pepper 2D simulator has NO API for adding obstacles. Static obstacles exist
only as occupied cells of the occupancy map, which PepperRLEnv reads once at
construction (raytracer, TSDF collision grid, reachable space).

This module therefore:
  1. defines obstacles per scenario (circles and axis-aligned rectangles),
  2. validates them against the original map (bounds, free space, start/goal
     distance, separation, route still feasible),
  3. writes an obstacle-augmented COPY of the map (.pgm + .yaml) into a
     separate folder. The original map in maps/ is never modified.

OfficeScenarioEnv (navrep/envs/officeenv.py) loads that copy instead of the
original map when obstacles are enabled.

Dependencies: numpy, Pillow, PyYAML only (no simulator). Python 3.6 compatible.
"""
from __future__ import print_function
import copy
import os
import tempfile

import numpy as np

from navrep.envs.office_scenarios import (OccupancyMap, default_map_folder, get_office_scenario,
                                          validate_pose, SCENARIO_NAMES, DEFAULT_MIN_CLEARANCE)

# prefix of the temporary folders holding obstacle-augmented map copies
TEMP_MAP_PREFIX = "navrep_office_obstacles_"

NO_OBSTACLES = "none"
SCENARIO_OBSTACLES = "scenario"
OBSTACLE_MODES = [SCENARIO_OBSTACLES, NO_OBSTACLES]

OBSTACLE_TYPES = ["chair", "table", "cabinet", "box", "trolley", "bin", "plant"]
SHAPES = ["circle", "rect"]

# minimum distance [m] between an obstacle's edge and the start / goal position.
# robot radius 0.3 + goal-reached padding 0.2 + 0.5 safety margin
MIN_START_GOAL_DISTANCE = 1.0
# minimum distance [m] between the edges of two obstacles
MIN_OBSTACLE_SEPARATION = 0.1

# Obstacles per scenario, in the asl_office_j map frame [m].
# circle: radius [m]; rect: width (x extent) and height (y extent) [m], axis-aligned, centred on (x, y)
OFFICE_OBSTACLES = {
    "document_delivery": [
        {"name": "reception_waiting_chair", "type": "chair", "shape": "circle",
         "x": -21.3, "y": -1.6, "radius": 0.3},
        {"name": "delivery_boxes", "type": "box", "shape": "rect",
         "x": 1.7, "y": -1.1, "width": 0.8, "height": 0.5},
    ],
    "pantry_assistance": [
        {"name": "corridor_filing_cabinet", "type": "cabinet", "shape": "rect",
         "x": -1.0, "y": 5.4, "width": 0.8, "height": 0.4},
        {"name": "cleaning_trolley", "type": "trolley", "shape": "rect",
         "x": 2.2, "y": 2.0, "width": 0.5, "height": 0.8},
    ],
    "meeting_room_assistance": [
        {"name": "hall_printer_table", "type": "table", "shape": "rect",
         "x": 0.0, "y": 1.0, "width": 0.8, "height": 0.6},
        {"name": "meeting_room_spare_chair", "type": "chair", "shape": "circle",
         "x": 12.4, "y": 0.4, "radius": 0.3},
    ],
}


def get_obstacles(scenario, mode=SCENARIO_OBSTACLES):
    """ list of obstacle dicts for a scenario; [] in 'none' mode """
    if mode not in OBSTACLE_MODES:
        raise ValueError("Unknown obstacle mode '{}'. Available: {}".format(mode, OBSTACLE_MODES))
    get_office_scenario(scenario)  # raises KeyError for unknown scenarios
    if mode == NO_OBSTACLES:
        return []
    return copy.deepcopy(OFFICE_OBSTACLES.get(scenario, []))


# ---------------------------------------------------------------------------
# geometry
# ---------------------------------------------------------------------------

def check_obstacle_definition(obst):
    """ structural checks, returns list of error strings """
    errors = []
    name = obst.get("name", "<unnamed>")
    if not obst.get("name"):
        errors.append("obstacle has no name")
    if obst.get("type") not in OBSTACLE_TYPES:
        errors.append("{}: type must be one of {}".format(name, OBSTACLE_TYPES))
    shape = obst.get("shape")
    if shape not in SHAPES:
        errors.append("{}: shape must be one of {}".format(name, SHAPES))
        return errors
    keys = ["x", "y"] + (["radius"] if shape == "circle" else ["width", "height"])
    for k in keys:
        v = obst.get(k)
        if not isinstance(v, (int, float)) or isinstance(v, bool) or not np.isfinite(v):
            errors.append("{}: '{}' must be a finite number".format(name, k))
    if errors:
        return errors
    for k in keys[2:]:
        if obst[k] <= 0:
            errors.append("{}: '{}' must be positive".format(name, k))
    return errors


def distance_to_obstacle(obst, x, y):
    """ distance [m] from point (x, y) to the obstacle's edge (<= 0 means inside) """
    if obst["shape"] == "circle":
        return float(np.hypot(x - obst["x"], y - obst["y"]) - obst["radius"])
    dx = abs(x - obst["x"]) - obst["width"] / 2.
    dy = abs(y - obst["y"]) - obst["height"] / 2.
    outside = np.hypot(max(dx, 0.), max(dy, 0.))
    return float(outside + min(max(dx, dy), 0.))


def obstacle_bounds(obst):
    """ (xmin, xmax, ymin, ymax) """
    if obst["shape"] == "circle":
        r = obst["radius"]
        return obst["x"] - r, obst["x"] + r, obst["y"] - r, obst["y"] + r
    w, h = obst["width"] / 2., obst["height"] / 2.
    return obst["x"] - w, obst["x"] + w, obst["y"] - h, obst["y"] + h


def obstacle_mask(occmap, obst):
    """ boolean mask over occmap.free (rows grow with y) of the cells covered by the obstacle """
    res = occmap.resolution
    mask = np.zeros(occmap.free.shape, dtype=bool)
    xmin, xmax, ymin, ymax = obstacle_bounds(obst)
    c0 = max(int(np.floor((xmin - occmap.origin[0]) / res)), 0)
    c1 = min(int(np.ceil((xmax - occmap.origin[0]) / res)), mask.shape[1])
    r0 = max(int(np.floor((ymin - occmap.origin[1]) / res)), 0)
    r1 = min(int(np.ceil((ymax - occmap.origin[1]) / res)), mask.shape[0])
    if c1 <= c0 or r1 <= r0:
        return mask
    cx = occmap.origin[0] + (np.arange(c0, c1) + 0.5) * res
    cy = occmap.origin[1] + (np.arange(r0, r1) + 0.5) * res
    X, Y = np.meshgrid(cx, cy)
    if obst["shape"] == "circle":
        inside = (X - obst["x"]) ** 2 + (Y - obst["y"]) ** 2 <= obst["radius"] ** 2
    else:
        inside = (np.abs(X - obst["x"]) <= obst["width"] / 2.) & (np.abs(Y - obst["y"]) <= obst["height"] / 2.)
    mask[r0:r1, c0:c1] = inside
    return mask


def apply_obstacles(occmap, obstacles):
    """ returns a copy of occmap with the obstacles marked as fully occupied """
    new = copy.copy(occmap)
    new.occupancy = occmap.occupancy.copy()
    for obst in obstacles:
        new.occupancy[obstacle_mask(occmap, obst)] = 1.
    new.free = new.occupancy < new.free_thresh
    return new


# ---------------------------------------------------------------------------
# validation
# ---------------------------------------------------------------------------

def validate_obstacles(obstacles, start_pose=None, goal_pose=None, occmap=None,
                       map_folder=None, map_name=None, min_clearance=DEFAULT_MIN_CLEARANCE,
                       check_route=True):
    """ validates a list of obstacles. returns (errors, info).

    info["path_length_without"], info["path_length_with"]: approx. route lengths [m]
    info["on_route"]: names of obstacles that block the obstacle-free shortest route
    """
    info = {"path_length_without": None, "path_length_with": None, "on_route": []}
    errors = []
    names = [o.get("name") for o in obstacles]
    for n in set(names):
        if names.count(n) > 1:
            errors.append("duplicate obstacle name '{}'".format(n))
    for obst in obstacles:
        errors += check_obstacle_definition(obst)
    if errors or not obstacles:
        if not errors and check_route and occmap is not None and start_pose is not None \
                and goal_pose is not None:
            info["path_length_without"] = info["path_length_with"] = occmap.path_length(
                start_pose[:2], goal_pose[:2], min_clearance)
        return errors, info
    if occmap is None:
        occmap = OccupancyMap(map_folder or default_map_folder(), map_name)
    for obst in obstacles:
        name = obst["name"]
        xmin, xmax, ymin, ymax = obstacle_bounds(obst)
        if not (occmap.is_inside(xmin, ymin) and occmap.is_inside(xmax, ymax)):
            errors.append("{}: footprint is (partly) outside the map".format(name))
            continue
        mask = obstacle_mask(occmap, obst)
        if not np.all(occmap.free[mask]):
            errors.append("{}: footprint overlaps walls / existing obstacles / unknown space".format(name))
        for label, pose in (("start", start_pose), ("goal", goal_pose)):
            if pose is not None:
                d = distance_to_obstacle(obst, pose[0], pose[1])
                if d < MIN_START_GOAL_DISTANCE:
                    errors.append("{}: only {:.2f} m from the {} position (min {:.2f} m)".format(
                        name, d, label, MIN_START_GOAL_DISTANCE))
    for i in range(len(obstacles)):
        for j in range(i + 1, len(obstacles)):
            a, b = obstacles[i], obstacles[j]
            mask_a = obstacle_mask(occmap, a)
            # inflate b by the min separation and check overlap with a
            b_infl = dict(b)
            if b["shape"] == "circle":
                b_infl["radius"] = b["radius"] + MIN_OBSTACLE_SEPARATION
            else:
                b_infl["width"] = b["width"] + 2 * MIN_OBSTACLE_SEPARATION
                b_infl["height"] = b["height"] + 2 * MIN_OBSTACLE_SEPARATION
            if np.any(mask_a & obstacle_mask(occmap, b_infl)):
                errors.append("{} and {} are closer than {:.2f} m".format(
                    a["name"], b["name"], MIN_OBSTACLE_SEPARATION))
    if errors or not check_route or start_pose is None or goal_pose is None:
        return errors, info
    augmented = apply_obstacles(occmap, obstacles)
    for label, pose in (("start", start_pose), ("goal", goal_pose)):
        errors += ["{} after adding obstacles: {}".format(label, e)
                   for e in validate_pose(augmented, pose[0], pose[1], min_clearance)]
    info["path_length_without"] = occmap.path_length(start_pose[:2], goal_pose[:2], min_clearance)
    info["path_length_with"] = augmented.path_length(start_pose[:2], goal_pose[:2], min_clearance)
    if info["path_length_with"] is None:
        errors.append("obstacles block every route from start to goal ({:.2f} m clearance)".format(
            min_clearance))
    elif augmented.path_length((0., 0.), start_pose[:2], min_clearance) is None:
        errors.append("obstacles disconnect the start from the map origin (0, 0), the seed of "
                      "PepperRLEnv's reachable space")
    # which obstacles sit on the obstacle-free route (removing each alone changes nothing otherwise)
    for obst in obstacles:
        single = apply_obstacles(occmap, [obst])
        length = single.path_length(start_pose[:2], goal_pose[:2], min_clearance)
        if length is None or length > info["path_length_without"] + 0.05:
            info["on_route"].append(obst["name"])
    return errors, info


def validate_scenario_obstacles(scenario, mode=SCENARIO_OBSTACLES, map_folder=None, occmap=None):
    sc = get_office_scenario(scenario)
    if occmap is None:
        occmap = OccupancyMap(map_folder or default_map_folder(), sc["map_name"])
    return validate_obstacles(get_obstacles(scenario, mode), sc["start_pose"], sc["goal_pose"],
                              occmap=occmap)


# ---------------------------------------------------------------------------
# augmented map file
# ---------------------------------------------------------------------------

def write_augmented_map(map_folder, map_name, obstacles, out_folder=None, out_name=None):
    """ writes <out_folder>/<out_name>.yaml + .pgm: the original map with obstacles painted as
    occupied (pixel value 0). The original files are only read. Returns (out_folder, out_name). """
    import yaml
    from PIL import Image
    if out_folder is None:
        out_folder = tempfile.mkdtemp(prefix=TEMP_MAP_PREFIX)
    if out_name is None:
        out_name = map_name + "_with_obstacles"
    if os.path.abspath(out_folder) == os.path.abspath(map_folder) and out_name == map_name:
        raise ValueError("refusing to overwrite the original map")
    with open(os.path.join(map_folder, map_name + ".yaml"), "r") as f:
        meta = yaml.safe_load(f)
    img = np.array(Image.open(os.path.join(map_folder, meta["image"])))
    if img.ndim != 2 or img.dtype != np.uint8:
        raise ValueError("only 8-bit grayscale maps are supported")
    occmap = OccupancyMap(map_folder, map_name)
    for obst in obstacles:
        mask = obstacle_mask(occmap, obst)[::-1, :]  # back to image row order (row 0 = top)
        img[mask] = 255 if meta.get("negate", 0) else 0
    if not os.path.isdir(out_folder):
        os.makedirs(out_folder)
    meta["image"] = out_name + ".pgm"
    Image.fromarray(img).save(os.path.join(out_folder, out_name + ".pgm"))
    with open(os.path.join(out_folder, out_name + ".yaml"), "w") as f:
        yaml.safe_dump(meta, f, default_flow_style=None)
    return out_folder, out_name
