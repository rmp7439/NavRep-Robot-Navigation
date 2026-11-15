""" Tests for office obstacle configuration (Member 4).

Lightweight tests (numpy, Pillow, PyYAML only):
    python -m pytest tests/test_office_obstacles.py -v

The simulator tests at the bottom need pepper_2d_simulator (Docker container)
and are skipped automatically when it is not importable.
"""
import hashlib
import os

import numpy as np
import pytest

from navrep.envs.office_scenarios import (OccupancyMap, default_map_folder, get_office_scenario,
                                          SCENARIO_NAMES, DEFAULT_MIN_CLEARANCE)
from navrep.envs.office_obstacles import (OFFICE_OBSTACLES, NO_OBSTACLES, SCENARIO_OBSTACLES,
                                          MIN_START_GOAL_DISTANCE, get_obstacles, validate_obstacles,
                                          validate_scenario_obstacles, check_obstacle_definition,
                                          distance_to_obstacle, obstacle_mask, apply_obstacles,
                                          write_augmented_map)

VALID_CHAIR = {"name": "c", "type": "chair", "shape": "circle", "x": -21.3, "y": -1.6, "radius": 0.3}


@pytest.fixture(scope="module")
def occmap():
    return OccupancyMap(default_map_folder(), "asl_office_j")


def _validate(occmap, obstacles, scenario="document_delivery", check_route=False):
    sc = get_office_scenario(scenario)
    return validate_obstacles(obstacles, sc["start_pose"], sc["goal_pose"], occmap=occmap,
                              check_route=check_route)


# --- configuration -----------------------------------------------------------

def test_every_scenario_has_obstacles():
    assert sorted(OFFICE_OBSTACLES.keys()) == SCENARIO_NAMES
    for name in SCENARIO_NAMES:
        assert len(get_obstacles(name)) >= 1


@pytest.mark.parametrize("name", SCENARIO_NAMES)
def test_scenario_obstacles_are_valid(occmap, name):
    errors, info = validate_scenario_obstacles(name, occmap=occmap)
    assert errors == []
    # obstacles make the route longer (they are on it) but never block it
    assert info["path_length_with"] is not None
    assert info["path_length_with"] >= info["path_length_without"]
    assert len(info["on_route"]) >= 1


@pytest.mark.parametrize("name", SCENARIO_NAMES)
def test_obstacle_definitions_and_dimensions(name):
    for o in get_obstacles(name):
        assert check_obstacle_definition(o) == []
        dims = [o["radius"]] if o["shape"] == "circle" else [o["width"], o["height"]]
        assert all(d > 0 for d in dims)


@pytest.mark.parametrize("name", SCENARIO_NAMES)
def test_obstacles_keep_away_from_start_and_goal(name):
    sc = get_office_scenario(name)
    for o in get_obstacles(name):
        assert distance_to_obstacle(o, *sc["start_pose"][:2]) >= MIN_START_GOAL_DISTANCE
        assert distance_to_obstacle(o, *sc["goal_pose"][:2]) >= MIN_START_GOAL_DISTANCE


@pytest.mark.parametrize("name", SCENARIO_NAMES)
def test_obstacles_inside_map(occmap, name):
    for o in get_obstacles(name):
        assert obstacle_mask(occmap, o).any()
        assert occmap.is_inside(o["x"], o["y"])


def test_start_goal_unchanged():
    # Member 3 coordinates must be preserved
    expected = {"document_delivery": ((-23.3, -0.7), (13.9, 2.4)),
                "pantry_assistance": ((-11.0, 5.1), (5.0, -1.0)),
                "meeting_room_assistance": ((-1.5, 2.5), (13.9, 2.4))}
    for name, (s, g) in expected.items():
        sc = get_office_scenario(name)
        assert np.allclose(sc["start_pose"][:2], s) and np.allclose(sc["goal_pose"][:2], g)


def test_no_obstacles_mode(occmap):
    for name in SCENARIO_NAMES:
        assert get_obstacles(name, NO_OBSTACLES) == []
        errors, info = validate_scenario_obstacles(name, mode=NO_OBSTACLES, occmap=occmap)
        assert errors == [] and info["path_length_with"] == info["path_length_without"]


def test_get_obstacles_returns_copies():
    obs = get_obstacles("document_delivery")
    obs[0]["x"] = 999.
    assert get_obstacles("document_delivery")[0]["x"] != 999.


def test_unknown_mode_and_scenario_rejected():
    with pytest.raises(ValueError):
        get_obstacles("document_delivery", "lots")
    with pytest.raises(KeyError):
        get_obstacles("not_a_scenario", SCENARIO_OBSTACLES)


# --- invalid configurations ----------------------------------------------------

@pytest.mark.parametrize("change", [
    {"radius": 0.}, {"radius": -0.2}, {"radius": "big"}, {"x": float("nan")},
    {"type": "elephant"}, {"shape": "triangle"}, {"name": ""},
])
def test_invalid_definitions_rejected(occmap, change):
    o = dict(VALID_CHAIR, **change)
    errors, _ = _validate(occmap, [o])
    assert errors != []


def test_invalid_rect_dimensions_rejected(occmap):
    o = {"name": "t", "type": "table", "shape": "rect", "x": 1.7, "y": -1.1, "width": 0.8, "height": 0.}
    assert _validate(occmap, [o])[0] != []


def test_valid_single_obstacle_accepted(occmap):
    assert _validate(occmap, [VALID_CHAIR])[0] == []


def test_obstacle_outside_map_rejected(occmap):
    assert _validate(occmap, [dict(VALID_CHAIR, x=100., y=100.)])[0] != []


def test_obstacle_on_wall_or_unknown_rejected(occmap):
    assert _validate(occmap, [dict(VALID_CHAIR, x=-31.5, y=13.5)])[0] != []  # unknown corner


def test_obstacle_on_start_or_goal_rejected(occmap):
    sc = get_office_scenario("document_delivery")
    for pose in (sc["start_pose"], sc["goal_pose"]):
        errors, _ = _validate(occmap, [dict(VALID_CHAIR, x=pose[0], y=pose[1])])
        assert any("from the" in e for e in errors)


def test_overlapping_obstacles_rejected(occmap):
    a = dict(VALID_CHAIR, name="a")
    b = dict(VALID_CHAIR, name="b", x=VALID_CHAIR["x"] + 0.4)
    errors, _ = _validate(occmap, [a, b])
    assert any("closer than" in e for e in errors)


def test_duplicate_names_rejected(occmap):
    errors, _ = _validate(occmap, [VALID_CHAIR, dict(VALID_CHAIR, x=-20.5)])
    assert any("duplicate" in e for e in errors)


def test_blocking_obstacle_rejected(occmap):
    # two cabinets closing both the south and the north corridor: no route left
    wall_s = {"name": "s", "type": "cabinet", "shape": "rect", "x": -8.3, "y": -2.1, "width": 0.3, "height": 4.0}
    wall_n = {"name": "n", "type": "cabinet", "shape": "rect", "x": -8.5, "y": 5.4, "width": 0.3, "height": 3.0}
    sc = get_office_scenario("document_delivery")
    blocked = apply_obstacles(occmap, [wall_s, wall_n])
    assert blocked.path_length(sc["start_pose"][:2], sc["goal_pose"][:2], DEFAULT_MIN_CLEARANCE) is None
    errors, _ = _validate(occmap, [wall_s, wall_n], check_route=True)
    assert errors != []


# --- augmented map file -------------------------------------------------------

def _md5(path):
    with open(path, "rb") as f:
        return hashlib.md5(f.read()).hexdigest()


def test_write_augmented_map(tmp_path, occmap):
    folder = default_map_folder()
    orig_pgm, orig_yaml = os.path.join(folder, "asl_office_j.pgm"), os.path.join(folder, "asl_office_j.yaml")
    before = (_md5(orig_pgm), _md5(orig_yaml))
    obstacles = get_obstacles("document_delivery")
    out_folder, out_name = write_augmented_map(folder, "asl_office_j", obstacles, out_folder=str(tmp_path))
    assert (_md5(orig_pgm), _md5(orig_yaml)) == before  # original untouched
    aug = OccupancyMap(out_folder, out_name)
    assert aug.free.shape == occmap.free.shape and np.allclose(aug.origin, occmap.origin)
    total = np.zeros_like(occmap.free)
    for o in obstacles:
        m = obstacle_mask(occmap, o)
        total |= m
        assert np.all(aug.occupancy[m] >= aug.occupied_thresh)
    # everything else identical
    assert np.array_equal(aug.occupancy[~total], occmap.occupancy[~total])
    # CMap2D's loader uses occupancy = 1 - pixel / 254 and occupied if >= occupied_thresh
    from PIL import Image
    img = np.array(Image.open(os.path.join(out_folder, out_name + ".pgm")))[::-1, :]
    assert np.all((1. - img[total] / 254.) >= aug.occupied_thresh)


def test_write_refuses_to_overwrite_original():
    folder = default_map_folder()
    with pytest.raises(ValueError):
        write_augmented_map(folder, "asl_office_j", [], out_folder=folder, out_name="asl_office_j")


def test_cli_check_with_and_without_obstacles(capsys):
    from navrep.scripts.play_office_scenario import main
    for name in SCENARIO_NAMES:
        assert main(["--scenario", name, "--check"]) == 0
        assert main(["--scenario", name, "--check", "--obstacles", "none"]) == 0
    out = capsys.readouterr().out
    assert out.count("obstacles VALID") == 3
    assert out.count("obstacles: none") == 3


# --- simulator (Docker container only) --------------------------------------

@pytest.mark.parametrize("name", SCENARIO_NAMES)
def test_env_applies_obstacles_to_simulator(name):
    pytest.importorskip("pepper_2d_simulator")
    from navrep.envs.officeenv import OfficeScenarioEnv
    sc = get_office_scenario(name)
    env = OfficeScenarioEnv(scenario=name, obstacles=SCENARIO_OBSTACLES, silent=True)
    try:
        assert env.rlenv.args.map_name == "asl_office_j_with_obstacles"
        map2d = env.rlenv.map2d
        for o in get_obstacles(name):
            i, j = map2d.xy_to_ij([[o["x"], o["y"]]])[0]
            assert map2d.occupancy()[i, j] >= map2d.thresh_occupied()
        env.reset()
        assert np.allclose(env.rlenv.virtual_peppers[0].pos, sc["start_pose"])
        assert np.allclose(env.rlenv.agent_goals[0][:2], sc["goal_pose"][:2])
    finally:
        env.close()


def test_env_without_obstacles_uses_original_map():
    pytest.importorskip("pepper_2d_simulator")
    from navrep.envs.officeenv import OfficeScenarioEnv
    env = OfficeScenarioEnv(scenario="document_delivery", silent=True)  # default: no obstacles
    try:
        assert env.rlenv.args.map_name == "asl_office_j"
    finally:
        env.close()

