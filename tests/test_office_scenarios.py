""" Tests for office start/goal scenarios.

Lightweight tests (numpy, Pillow, PyYAML only) run anywhere:
    python -m pytest tests/test_office_scenarios.py -v

The simulator tests at the bottom need pepper_2d_simulator (Docker container)
and are skipped automatically when it is not importable.
"""
import numpy as np
import pytest

from navrep.envs.office_scenarios import (OFFICE_SCENARIOS, OFFICE_LOCATIONS, SCENARIO_NAMES,
                                          OccupancyMap, default_map_folder, get_office_scenario,
                                          validate_pose, validate_scenario, DEFAULT_MIN_CLEARANCE)

EXPECTED = {
    "document_delivery": ("reception", "meeting_room"),
    "pantry_assistance": ("employee_cabin", "pantry"),
    "meeting_room_assistance": ("charging_station", "meeting_room"),
}


@pytest.fixture(scope="module")
def occmap():
    return OccupancyMap(default_map_folder(), "asl_office_j")


def test_three_scenarios_defined():
    assert sorted(EXPECTED.keys()) == SCENARIO_NAMES


@pytest.mark.parametrize("name", sorted(EXPECTED.keys()))
def test_scenario_start_goal_roles(name):
    sc = get_office_scenario(name)
    assert (sc["start_name"], sc["goal_name"]) == EXPECTED[name]
    assert sc["start_pose"].shape == (3,) and sc["goal_pose"].shape == (3,)
    assert -np.pi <= sc["start_pose"][2] <= 2 * np.pi
    # start and goal must differ by more than the goal-reached radius (0.3 + 0.2 m)
    assert np.linalg.norm(sc["start_pose"][:2] - sc["goal_pose"][:2]) > 3.0


def test_unknown_scenario_raises():
    with pytest.raises(KeyError):
        get_office_scenario("not_a_scenario")


def test_map_metadata(occmap):
    # values from maps/asl_office_j.yaml
    assert occmap.resolution == pytest.approx(0.02)
    assert np.allclose(occmap.origin, [-32.16, -11.52])
    assert occmap.free.shape == (1280, 2736)


@pytest.mark.parametrize("loc", sorted(OFFICE_LOCATIONS.keys()))
def test_every_location_is_free_with_clearance(occmap, loc):
    x, y = OFFICE_LOCATIONS[loc]
    assert validate_pose(occmap, x, y, DEFAULT_MIN_CLEARANCE) == []


@pytest.mark.parametrize("name", sorted(EXPECTED.keys()))
def test_scenario_valid_and_reachable(occmap, name):
    errors, info = validate_scenario(name, occmap=occmap)
    assert errors == []
    # path must be at least the straight-line distance
    sc = get_office_scenario(name)
    assert info["path_length"] >= np.linalg.norm(sc["start_pose"][:2] - sc["goal_pose"][:2]) - 0.2


def test_validator_rejects_outside_map(occmap):
    assert validate_pose(occmap, 100., 100.) != []


def test_validator_rejects_unknown_space(occmap):
    # top-left corner of the map image is unknown (grey, value 205)
    assert validate_pose(occmap, -31.5, 13.5) != []


def test_validator_rejects_occupied_cell(occmap):
    rows, cols = np.where(occmap.occupancy > occmap.occupied_thresh)
    r, c = rows[len(rows) // 2], cols[len(cols) // 2]
    x = occmap.origin[0] + (c + 0.5) * occmap.resolution
    y = occmap.origin[1] + (r + 0.5) * occmap.resolution
    assert occmap.xy_to_rc(x, y) == (r, c)
    assert validate_pose(occmap, x, y) != []


def test_validator_rejects_point_too_close_to_wall(occmap):
    rows, cols = np.where(occmap.occupancy > occmap.occupied_thresh)
    r, c = rows[len(rows) // 2], cols[len(cols) // 2]
    # find a free cell within 0.2 m of that wall cell
    for dr in range(-10, 11):
        for dc in range(-10, 11):
            if occmap.free[r + dr, c + dc]:
                x = occmap.origin[0] + (c + dc + 0.5) * occmap.resolution
                y = occmap.origin[1] + (r + dr + 0.5) * occmap.resolution
                assert validate_pose(occmap, x, y) != []
                return
    pytest.skip("no free cell next to the chosen wall cell")


def test_script_list_and_check_run_without_simulator(capsys):
    from navrep.scripts.play_office_scenario import main
    assert main(["--list"]) == 0
    for name in SCENARIO_NAMES:
        assert main(["--scenario", name, "--check"]) == 0
    out = capsys.readouterr().out
    assert out.count("VALID (") == 3


# ---------------------------------------------------------------------------
# simulator tests (Docker container only)
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("name", sorted(EXPECTED.keys()))
def test_env_places_robot_and_goal(name):
    pytest.importorskip("pepper_2d_simulator")
    from navrep.envs.officeenv import OfficeScenarioEnv
    sc = get_office_scenario(name)
    env = OfficeScenarioEnv(scenario=name, silent=True)
    try:
        for _ in range(2):  # the pose must be restored on every reset
            obs = env.reset()
            assert np.allclose(env.rlenv.virtual_peppers[0].pos, sc["start_pose"])
            assert np.allclose(env.rlenv.agent_goals[0][:2], sc["goal_pose"][:2])
            assert obs[0].shape == (1080,) and obs[1].shape == (5,)
            # robotstate[:2] is the goal in the robot frame: its norm = start-goal distance
            assert np.linalg.norm(obs[1][:2]) == pytest.approx(
                np.linalg.norm(sc["start_pose"][:2] - sc["goal_pose"][:2]), abs=1e-3)
            obs, rew, done, info = env.step(np.zeros(3))
            assert not done
            assert env.rlenv.episode_damage[0] == 0
    finally:
        env.close()


def test_random_mode_preserved():
    pytest.importorskip("pepper_2d_simulator")
    from navrep.envs.officeenv import OfficeScenarioEnv
    env = OfficeScenarioEnv(map_name="asl_office_j", silent=True)  # no start/goal -> MarkEnv behaviour
    try:
        starts = []
        for _ in range(3):
            env.reset()
            starts.append(env.rlenv.virtual_peppers[0].pos[:2].copy())
        assert not np.allclose(starts[0], starts[1]) or not np.allclose(starts[1], starts[2])
    finally:
        env.close()
