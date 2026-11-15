""" Launch an Office Assistance Robot scenario in the Pepper 2D simulator (MarkEnv).

Examples:
    python -m navrep.scripts.play_office_scenario --list
    python -m navrep.scripts.play_office_scenario --scenario document_delivery --check
    python -m navrep.scripts.play_office_scenario --scenario document_delivery
    python -m navrep.scripts.play_office_scenario --scenario pantry_assistance --headless --steps 20
    python -m navrep.scripts.play_office_scenario --scenario pantry_assistance --obstacles none

--list and --check do not need the simulator (map files only).
Default mode opens the interactive viewer, like play_markenv.py
(WASD move, Q/E rotate, R reset, ESC exit).
"""
from __future__ import print_function
import argparse
import os
import sys

import numpy as np

from navrep.envs.office_scenarios import (SCENARIO_NAMES, get_office_scenario, validate_scenario,
                                          default_map_folder, OccupancyMap)
from navrep.envs.office_obstacles import (OBSTACLE_MODES, SCENARIO_OBSTACLES, get_obstacles,
                                          validate_obstacles)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Office Assistance Robot start/goal scenarios")
    parser.add_argument("--scenario", choices=SCENARIO_NAMES, help="office scenario to run")
    parser.add_argument("--list", action="store_true", help="list scenarios and exit")
    parser.add_argument("--check", action="store_true",
                        help="validate start/goal against the map file and exit (no simulator)")
    parser.add_argument("--map-folder", type=str, default=None,
                        help="map folder for --check (default: this repository's maps/)")
    parser.add_argument("--obstacles", choices=OBSTACLE_MODES, default=SCENARIO_OBSTACLES,
                        help="'scenario': add the scenario's office obstacles (default), "
                             "'none': original map only")
    parser.add_argument("--headless", action="store_true",
                        help="no viewer: reset, print poses, step with zero action and exit")
    parser.add_argument("--steps", type=int, default=10, help="number of steps in --headless mode")
    parser.add_argument("--max-episode-length", type=int, default=1000)
    parser.add_argument("--silent", action="store_true")
    return parser.parse_args(argv)


def describe(name, obstacle_mode=SCENARIO_OBSTACLES):
    sc = get_office_scenario(name)
    print("{}: {}".format(name, sc["description"]))
    print("  map:   {}".format(sc["map_name"]))
    print("  start: {:<17} x={:7.2f} y={:7.2f} theta={:5.2f} rad".format(sc["start_name"], *sc["start_pose"]))
    print("  goal:  {:<17} x={:7.2f} y={:7.2f}".format(sc["goal_name"], *sc["goal_pose"][:2]))
    obstacles = get_obstacles(name, obstacle_mode)
    if not obstacles:
        print("  obstacles: none (original map only)")
    for o in obstacles:
        size = "r={:.2f}".format(o["radius"]) if o["shape"] == "circle" else \
            "{:.2f} x {:.2f}".format(o["width"], o["height"])
        print("  obstacle: {:<25} {:<8} x={:7.2f} y={:7.2f} {}".format(o["name"], o["type"], o["x"], o["y"], size))


def check(name, map_folder, obstacle_mode=SCENARIO_OBSTACLES):
    sc = get_office_scenario(name)
    occmap = OccupancyMap(map_folder, sc["map_name"])
    errors, info = validate_scenario(name, occmap=occmap)
    if not errors:
        obst_errors, obst_info = validate_obstacles(get_obstacles(name, obstacle_mode), sc["start_pose"],
                                                    sc["goal_pose"], occmap=occmap)
        errors += obst_errors
    if errors:
        print("  INVALID:")
        for e in errors:
            print("    - " + e)
        return False
    print("  VALID (approx. collision-free path length {:.1f} m)".format(info["path_length"]))
    if obstacle_mode != "none":
        print("  obstacles VALID: path length with obstacles {:.1f} m, obstacles on the direct route: {}".format(
            obst_info["path_length_with"], ", ".join(obst_info["on_route"]) or "none"))
    return True


def main(argv=None):
    args = parse_args(argv)
    if args.list:
        for name in SCENARIO_NAMES:
            describe(name, args.obstacles)
        return 0
    if args.scenario is None:
        print("--scenario is required (one of: {})".format(", ".join(SCENARIO_NAMES)))
        return 2
    describe(args.scenario, args.obstacles)
    if args.check:
        return 0 if check(args.scenario, args.map_folder or default_map_folder(), args.obstacles) else 1

    os.environ["CUDA_VISIBLE_DEVICES"] = "-1"  # disable GPU, as in play_markenv.py
    from navrep.envs.officeenv import OfficeScenarioEnv  # imports the simulator

    env = OfficeScenarioEnv(scenario=args.scenario, silent=args.silent,
                            max_episode_length=args.max_episode_length,
                            obstacles=args.obstacles)
    if args.headless:
        vp = env.rlenv.virtual_peppers[0]
        print("after reset: robot pos {}  goal {}".format(np.round(vp.pos, 3),
                                                         np.round(env.rlenv.agent_goals[0], 3)))
        for i in range(args.steps):
            obs, rew, done, info = env.step(np.zeros(3))
            if done:
                print("episode ended at step {} (info: {})".format(i + 1, info))
                break
        print("after {} zero-action steps: robot pos {}  damage {}".format(
            i + 1, np.round(vp.pos, 3), env.rlenv.episode_damage[0]))
        print("observation shapes: scan {}  robotstate {}".format(obs[0].shape, obs[1].shape))
        env.close()
        return 0

    from navrep.tools.envplayer import EnvPlayer
    EnvPlayer(env)  # runs until ESC
    print(env.episode_statistics[["scenario", "steps", "goal_reached", "timed_out",
                                  "collisioned_out", "goal"]].to_string())
    return 0


if __name__ == "__main__":
    sys.exit(main())
