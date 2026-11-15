""" Integration tests for the three office scenarios using a SIMULATOR STAND-IN.

Test category 2 of docs/custom_office_scenarios.md:
the real OfficeScenarioEnv / MarkEnv code runs, but `pepper_2d_simulator` is
replaced by a small stub that mimics the PepperRLEnv methods used by NavRep
(_reset_agents, _set_agents_pos, _set_agents_goals, step, reset, close) and
loads the map file named in args at construction time, like the real one.
These tests therefore do NOT prove anything about LiDAR, collisions or the real
simulator. Real-simulator tests live in test_office_scenarios.py /
test_office_obstacles.py and skip when pepper_2d_simulator is missing.

    python -m pytest tests/test_office_integration.py -v
"""
import hashlib
import importlib
import os
import sys
import tempfile
import types
from argparse import Namespace

import numpy as np
import pytest

from navrep.envs.office_scenarios import (OccupancyMap, default_map_folder, get_office_scenario,
                                          SCENARIO_NAMES)
from navrep.envs.office_obstacles import get_obstacles, obstacle_mask, TEMP_MAP_PREFIX

EXPECTED_OBSTACLES = {
    "document_delivery": ["reception_waiting_chair", "delivery_boxes"],
    "pantry_assistance": ["corridor_filing_cabinet", "cleaning_trolley"],
    "meeting_room_assistance": ["hall_printer_table", "meeting_room_spare_chair"],
}
EXPECTED_ROLES = {
    "document_delivery": ("reception", "meeting_room"),
    "pantry_assistance": ("employee_cabin", "pantry"),
    "meeting_room_assistance": ("charging_station", "meeting_room"),
}


# ---------------------------------------------------------------------------
# simulator stand-in
# ---------------------------------------------------------------------------

class _StubPepper(object):
    def __init__(self):
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)

    def reset(self):
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)


class _StubPepperRLEnv(object):
    instances = []

    def __init__(self, args, silent=False):
        self.args = args
        self.DT = args.dt
        # like the real PepperRLEnv: the map is read ONCE, here, before anything else is built
        self.loaded_map = OccupancyMap(args.map_folder, args.map_name)
        self.reachable_xy = None
        self.virtual_peppers = [_StubPepper()]
        self.calls = []
        self.closed = False
        _StubPepperRLEnv.instances.append(self)
        self._reset_agents()

    def _reset_agents(self, agents_mask=None):
        self.calls.append("_reset_agents")
        self.agent_goals = np.zeros((1, 3))
        self.virtual_peppers[0].pos = np.array([np.random.uniform(-5, 5), np.random.uniform(-1, 1),
                                                np.random.rand() * 2 * np.pi])
        self.agent_goals[0] = [np.random.uniform(-5, 5), 4., 0.]
        self.episode_step = np.zeros(1, dtype=int)
        self.episode_damage = np.zeros(1)

    def _set_agents_pos(self, agents_pos):
        for i, pos in enumerate(agents_pos):
            if pos is not None:
                self.virtual_peppers[i].reset()
                self.virtual_peppers[i].pos = pos

    def _set_agents_goals(self, agents_goals):
        for i, goal in enumerate(agents_goals):
            if goal is not None:
                self.agent_goals[i] = goal

    def step(self, actions, ONLY_FOR_AGENT_0=False):
        self.calls.append("step")
        vp, goal = self.virtual_peppers[0], self.agent_goals[0]
        c, s = np.cos(-vp.pos[2]), np.sin(-vp.pos[2])
        d = goal[:2] - vp.pos[:2]
        state = np.array([c * d[0] - s * d[1], s * d[0] + c * d[1], 0., 0., 0.])
        if actions is not None:
            self.episode_step += 1
        return (np.zeros((4, 1080, 1)), state), 0., False, {"goal_reached": False}

    def reset(self, agents_mask=None, ONLY_FOR_AGENT_0=False):
        self._reset_agents()
        return self.step(None, ONLY_FOR_AGENT_0=ONLY_FOR_AGENT_0)[0]

    def close(self):
        self.closed = True


def _stub_simulator_module():
    mod = types.ModuleType("pepper_2d_simulator")
    mod.PepperRLEnv = _StubPepperRLEnv
    mod.parse_PepperRLEnv_args = lambda args=None: (Namespace(
        map_folder=None, map_name=None, n_agents=1, mode=None, continuous=None,
        no_ros=None, dt=None, deterministic=False), [])
    mod.check_PepperRLEnv_args = lambda args: None
    return mod


def _stub_gym_modules():
    gym = types.ModuleType("gym")
    spaces = types.ModuleType("gym.spaces")

    class _Space(object):
        def __init__(self, *args, **kwargs):
            pass
    spaces.Box = spaces.Tuple = _Space
    gym.Env = object
    gym.spaces = spaces
    return gym, spaces


@pytest.fixture
def officeenv_module():
    """ imports navrep.envs.officeenv against the stand-in; restores sys.modules afterwards """
    replaced = {}

    def _set(name, module):
        replaced.setdefault(name, sys.modules.get(name))
        sys.modules[name] = module

    _set("pepper_2d_simulator", _stub_simulator_module())
    try:
        import gym  # noqa: F401  (real gym in the Docker container)
    except ImportError:
        gym, spaces = _stub_gym_modules()
        _set("gym", gym)
        _set("gym.spaces", spaces)
    try:
        import pkg_resources
    except ImportError:
        pkg_resources = types.ModuleType("pkg_resources")
        _set("pkg_resources", pkg_resources)
    orig_resource_filename = getattr(pkg_resources, "resource_filename", None)
    # the simulator's map package is replaced by this repository's maps/ (byte-identical asl_office_j)
    pkg_resources.resource_filename = lambda pkg, sub: default_map_folder()
    for name in ("navrep.envs.markenv", "navrep.envs.officeenv"):
        replaced.setdefault(name, sys.modules.get(name))
        sys.modules.pop(name, None)
    _StubPepperRLEnv.instances = []
    try:
        yield importlib.import_module("navrep.envs.officeenv")
    finally:
        if orig_resource_filename is not None:
            pkg_resources.resource_filename = orig_resource_filename
        for name, module in replaced.items():
            if module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = module


def _md5(path):
    with open(path, "rb") as f:
        return hashlib.md5(f.read()).hexdigest()


def _original_map_hashes():
    folder = default_map_folder()
    return _md5(os.path.join(folder, "asl_office_j.pgm")), _md5(os.path.join(folder, "asl_office_j.yaml"))


# ---------------------------------------------------------------------------
# tests
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("name", SCENARIO_NAMES)
def test_scenario_with_own_obstacles(officeenv_module, name):
    before = _original_map_hashes()
    sc = get_office_scenario(name)
    assert (sc["start_name"], sc["goal_name"]) == EXPECTED_ROLES[name]
    assert sc["map_name"] == "asl_office_j"
    env = officeenv_module.OfficeScenarioEnv(scenario=name, obstacles="scenario", silent=True)
    try:
        assert [o["name"] for o in env.obstacles] == EXPECTED_OBSTACLES[name]
        rlenv = env.rlenv
        assert rlenv.args.map_name == "asl_office_j_with_obstacles"
        assert os.path.basename(rlenv.args.map_folder).startswith(TEMP_MAP_PREFIX)
        # the map loaded at simulator construction contains this scenario's obstacles ...
        loaded = rlenv.loaded_map
        for o in get_obstacles(name):
            assert not loaded.free[obstacle_mask(loaded, o)].any()
        # ... and only those: other scenarios' obstacle footprints are unchanged
        original = OccupancyMap(default_map_folder(), "asl_office_j")
        for other in SCENARIO_NAMES:
            if other == name:
                continue
            for o in get_obstacles(other):
                m = obstacle_mask(original, o)
                assert np.array_equal(loaded.occupancy[m], original.occupancy[m])
        # repeated resets restore the configured start and goal
        for _ in range(3):
            obs = env.reset()
            assert np.allclose(rlenv.virtual_peppers[0].pos, sc["start_pose"])
            assert np.allclose(rlenv.agent_goals[0][:2], sc["goal_pose"][:2])
            assert np.allclose(env.current_episode_goal[:2], sc["goal_pose"][:2])
            assert obs[0].shape == (1080,) and obs[1].shape == (5,)
            assert np.linalg.norm(obs[1][:2]) == pytest.approx(
                np.linalg.norm(sc["start_pose"][:2] - sc["goal_pose"][:2]))
            assert rlenv.calls[-2:] == ["_reset_agents", "step"]
            env.step(np.zeros(3))
        # the simulator was built exactly once (no rebuild per reset)
        assert len(_StubPepperRLEnv.instances) == 1
    finally:
        env.close()
    assert _original_map_hashes() == before


@pytest.mark.parametrize("name", SCENARIO_NAMES)
def test_scenario_without_obstacles_uses_original_map(officeenv_module, name):
    sc = get_office_scenario(name)
    env = officeenv_module.OfficeScenarioEnv(scenario=name, obstacles="none", silent=True)
    try:
        assert env.obstacles == [] and env.obstacle_map is None
        assert env.rlenv.args.map_name == "asl_office_j"
        assert env.rlenv.args.map_folder == default_map_folder()
        env.reset()
        assert np.allclose(env.rlenv.virtual_peppers[0].pos, sc["start_pose"])
    finally:
        env.close()


def test_default_is_no_obstacles(officeenv_module):
    env = officeenv_module.OfficeScenarioEnv(scenario="document_delivery", silent=True)
    try:
        assert env.obstacles == [] and env.rlenv.args.map_name == "asl_office_j"
    finally:
        env.close()


def test_random_mode_still_random(officeenv_module):
    env = officeenv_module.OfficeScenarioEnv(map_name="asl_office_j", silent=True)
    try:
        positions = []
        for _ in range(3):
            env.reset()
            positions.append(env.rlenv.virtual_peppers[0].pos.copy())
        # the stand-in's own random reset was used (MarkEnv path), not a fixed pose
        assert not np.allclose(positions[0], positions[1])
        assert env.rlenv.calls[-2:] == ["_reset_agents", "step"]
    finally:
        env.close()


def test_random_mode_with_custom_obstacles(officeenv_module):
    obstacles = get_obstacles("document_delivery")
    env = officeenv_module.OfficeScenarioEnv(map_name="asl_office_j", obstacles=obstacles, silent=True)
    try:
        assert env.rlenv.args.map_name == "asl_office_j_with_obstacles"
        env.reset()
    finally:
        env.close()


def test_scenario_obstacles_need_a_scenario(officeenv_module):
    with pytest.raises(ValueError):
        officeenv_module.OfficeScenarioEnv(map_name="asl_office_j", obstacles="scenario", silent=True)


def test_invalid_start_rejected(officeenv_module):
    with pytest.raises(ValueError):
        officeenv_module.OfficeScenarioEnv(map_name="asl_office_j", start_pose=[-31.5, 13.5, 0.],
                                           silent=True)


def test_invalid_obstacle_rejected_before_simulator_is_built(officeenv_module):
    sc = get_office_scenario("document_delivery")
    on_start = [{"name": "box_on_start", "type": "box", "shape": "circle",
                 "x": sc["start_pose"][0], "y": sc["start_pose"][1], "radius": 0.3}]
    tmp_before = set(os.listdir(tempfile.gettempdir()))
    with pytest.raises(ValueError):
        officeenv_module.OfficeScenarioEnv(scenario="document_delivery", obstacles=on_start, silent=True)
    assert _StubPepperRLEnv.instances == []
    leaked = [d for d in set(os.listdir(tempfile.gettempdir())) - tmp_before if d.startswith(TEMP_MAP_PREFIX)]
    assert leaked == []


def test_temporary_map_removed_on_close(officeenv_module):
    env = officeenv_module.OfficeScenarioEnv(scenario="pantry_assistance", obstacles="scenario", silent=True)
    folder = env.obstacle_map[0]
    assert os.path.isfile(os.path.join(folder, "asl_office_j_with_obstacles.pgm"))
    env.close()
    assert not os.path.exists(folder)
    assert env.obstacle_map is None
    assert os.path.isfile(os.path.join(default_map_folder(), "asl_office_j.pgm"))
    env.close()  # closing twice is harmless


def test_each_env_gets_its_own_temporary_map(officeenv_module):
    a = officeenv_module.OfficeScenarioEnv(scenario="document_delivery", obstacles="scenario", silent=True)
    b = officeenv_module.OfficeScenarioEnv(scenario="meeting_room_assistance", obstacles="scenario",
                                           silent=True)
    try:
        assert a.obstacle_map[0] != b.obstacle_map[0]
    finally:
        a.close()
        b.close()


def test_set_scenario_conflict_rejected(officeenv_module):
    env = officeenv_module.OfficeScenarioEnv(scenario="document_delivery", obstacles="scenario", silent=True)
    try:
        with pytest.raises(ValueError):
            env.reset(set_scenario="markmaze")
    finally:
        env.close()


def test_headless_cli_with_stub(officeenv_module, capsys):
    from navrep.scripts.play_office_scenario import main
    for name in SCENARIO_NAMES:
        assert main(["--scenario", name, "--headless", "--steps", "3", "--silent"]) == 0
        assert main(["--scenario", name, "--headless", "--steps", "0", "--silent",
                     "--obstacles", "none"]) == 0
    out = capsys.readouterr().out
    assert out.count("simulator map: asl_office_j_with_obstacles") == 3
    for names in EXPECTED_OBSTACLES.values():
        assert "active obstacles: " + ", ".join(names) in out
    assert out.count("active obstacles: none") == 3
