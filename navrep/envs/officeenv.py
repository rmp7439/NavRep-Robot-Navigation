""" MarkEnv with configurable (fixed) robot start pose and goal pose.

MarkEnv itself is NOT modified. This subclass only overrides reset():

    PepperRLEnv.reset()  ==  _reset_agents()  +  step(None)

and we insert the existing PepperRLEnv helpers _set_agents_pos() /
_set_agents_goals() between those two calls, so the first observation already
reflects the fixed start and goal.

If neither start_pose nor goal_pose is given, the behaviour is identical to
MarkEnv (random reachable start and random goal >= 3 m away).

Obstacles (Member 4): PepperRLEnv has no API to add obstacles; it reads static
obstacles from the occupancy map once, at construction. With obstacles enabled,
_make_env() validates them, writes an obstacle-augmented COPY of the map
(navrep.envs.office_obstacles.write_augmented_map) and builds PepperRLEnv from
that copy. Default is no additional obstacles (unchanged behaviour).

Python 3.6 compatible.
"""
from __future__ import print_function
import numpy as np

from navrep.envs.markenv import MarkEnv, parse_PepperRLEnv_args, PepperRLEnv, check_PepperRLEnv_args
from navrep.envs.office_scenarios import (get_office_scenario, validate_scenario, validate_pose,
                                          OccupancyMap, DEFAULT_MIN_CLEARANCE)
from navrep.envs.office_obstacles import (get_obstacles, validate_obstacles, write_augmented_map,
                                          NO_OBSTACLES)

REQUIRED_RLENV_METHODS = ["_reset_agents", "_set_agents_pos", "_set_agents_goals", "step"]


class OfficeScenarioEnv(MarkEnv):
    def __init__(self, scenario=None, start_pose=None, goal_pose=None, map_name=None,
                 silent=False, max_episode_length=1000, collect_trajectories=True,
                 validate=True, obstacles=NO_OBSTACLES):
        """
        scenario: name from navrep.envs.office_scenarios.SCENARIO_NAMES (sets map, start and goal)
        start_pose: optional [x, y, theta] override (map frame, metres / radians)
        goal_pose: optional [x, y] or [x, y, theta] override
        map_name: map to use when no scenario is given (e.g. "asl_office_j")
        validate: check start/goal against the map file before running
        obstacles: "none" (default), "scenario" (obstacles of the given scenario, see
                   navrep.envs.office_obstacles.OFFICE_OBSTACLES) or a list of obstacle dicts
        """
        self.scenario = None
        if scenario is not None:
            self.scenario = get_office_scenario(scenario)
            map_name = self.scenario["map_name"]
            if start_pose is None:
                start_pose = self.scenario["start_pose"]
            if goal_pose is None:
                goal_pose = self.scenario["goal_pose"]
        if map_name is None:
            raise ValueError("Either scenario or map_name must be given")
        self.start_pose = None if start_pose is None else self._as_pose(start_pose)
        self.goal_pose = None if goal_pose is None else self._as_pose(goal_pose)
        self.map_name = map_name
        if isinstance(obstacles, str):
            if obstacles != NO_OBSTACLES and scenario is None:
                raise ValueError("obstacles='{}' requires a scenario".format(obstacles))
            obstacles = [] if obstacles == NO_OBSTACLES else get_obstacles(scenario, obstacles)
        self.obstacles = list(obstacles)
        self.obstacle_map = None  # (folder, name) of the augmented map once written
        self.validate = validate
        self._validated = False
        # MarkEnv.__init__ builds the simulator and calls self.reset()
        super(OfficeScenarioEnv, self).__init__(silent=silent, maps=[map_name],
                                                max_episode_length=max_episode_length,
                                                collect_trajectories=collect_trajectories)

    @staticmethod
    def _as_pose(pose):
        pose = np.array(pose, dtype=np.float64).flatten()
        if pose.shape[0] == 2:
            pose = np.array([pose[0], pose[1], 0.])
        if pose.shape[0] != 3:
            raise ValueError("pose must be [x, y] or [x, y, theta], got {}".format(pose))
        return pose

    def _make_env(self, set_scenario=None):
        if not self.obstacles:
            return super(OfficeScenarioEnv, self)._make_env(set_scenario=set_scenario)
        # same as MarkEnv._make_env, except map_folder / map_name point to the augmented map copy
        try:
            self.rlenv.close()
        except AttributeError:
            pass
        if self.obstacle_map is None:
            from pkg_resources import resource_filename
            original_folder = resource_filename('asl_pepper_2d_sim_maps', 'maps')
            if self.validate:
                errors, _ = validate_obstacles(self.obstacles, self.start_pose, self.goal_pose,
                                               map_folder=original_folder, map_name=self.map_name)
                if errors:
                    raise ValueError("Invalid obstacle configuration:\n  " + "\n  ".join(errors))
            self.obstacle_map = write_augmented_map(original_folder, self.map_name, self.obstacles)
            if not self.silent:
                print("Obstacle map written to {}".format(self.obstacle_map[0]))
        args, _ = parse_PepperRLEnv_args(args=[])
        args.n_agents = 1
        args.map_folder, args.map_name = self.obstacle_map
        args.mode = "BOUNCE"
        args.continuous = True
        args.no_ros = True
        args.dt = self.DT
        check_PepperRLEnv_args(args)
        self.rlenv = PepperRLEnv(args, silent=self.silent)
        self.rlenv.virtual_peppers[0].NO_INERTIA = True

    def _validate_poses(self):
        """ checks against the map file actually loaded by the simulator """
        occmap = OccupancyMap(self.rlenv.args.map_folder, self.rlenv.args.map_name)
        if self.scenario is not None and \
                np.allclose(self.start_pose, self.scenario["start_pose"]) and \
                np.allclose(self.goal_pose, self.scenario["goal_pose"]):
            errors, _ = validate_scenario(self.scenario["name"], occmap=occmap)
        else:
            errors = []
            for label, pose in (("start", self.start_pose), ("goal", self.goal_pose)):
                if pose is not None:
                    errors += [label + ": " + e for e in
                               validate_pose(occmap, pose[0], pose[1], DEFAULT_MIN_CLEARANCE)]
        if errors:
            raise ValueError("Invalid start/goal configuration:\n  " + "\n  ".join(errors))
        # cross-check with the simulator's own reachable space (coarse grid, seeded at origin)
        reachable_xy = getattr(self.rlenv, "reachable_xy", None)
        if reachable_xy is not None and not self.silent:
            for label, pose in (("start", self.start_pose), ("goal", self.goal_pose)):
                if pose is not None:
                    d = np.min(np.linalg.norm(reachable_xy - pose[:2], axis=-1))
                    if d > 0.3:
                        print("WARNING: {} {} is {:.2f} m from the simulator's reachable space".format(
                            label, pose[:2], d))

    def reset(self, set_scenario=None):
        if self.start_pose is None and self.goal_pose is None:
            return super(OfficeScenarioEnv, self).reset(set_scenario=set_scenario)
        # --- same as MarkEnv.reset, with PepperRLEnv.reset inlined ---
        if len(self.maps) > 1 or set_scenario is not None:
            self._make_env(set_scenario=set_scenario)
        for method in REQUIRED_RLENV_METHODS:
            if not hasattr(self.rlenv, method):
                raise AttributeError("PepperRLEnv has no method '{}'; the installed pepper_2d_simulator "
                                     "version is not supported by OfficeScenarioEnv".format(method))
        if self.validate and not self._validated:
            self._validate_poses()
            self._validated = True
        self.rlenv._reset_agents()  # random spawn + random goal + buffer/episode counters reset
        if self.start_pose is not None:
            self.rlenv._set_agents_pos([self.start_pose.copy()])
        if self.goal_pose is not None:
            self.rlenv._set_agents_goals([self.goal_pose.copy()])
        obs, _, _, _ = self.rlenv.step(None, ONLY_FOR_AGENT_0=True)  # observation only, no motion
        # --- end of PepperRLEnv.reset ---
        self.steps_since_reset = 0
        obs = (obs[0][-1, :, 0], obs[1])  # latest scan only (buffer, ray, channel)
        self.current_episode_trajectory = []
        self.current_episode_goal = self.rlenv.agent_goals[0] * 1.
        return obs
