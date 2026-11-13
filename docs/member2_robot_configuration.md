# Member 2 – Robot Configuration

## 1. Overview

The project uses a simulated Pepper 2D robot through the NavRep MarkEnv environment. The robot operates in a 2D environment and uses continuous velocity commands for movement and LiDAR observations for navigation.

No physical robot hardware is required.

## 2. Robot Configuration

| Parameter | Configuration |
|---|---|
| Robot model | Virtual Pepper 2D |
| Number of robots | 1 |
| Collision radius | 0.3 m |
| Movement mode | Direct velocity (NO_INERTIA enabled) |
| Collision mode | BOUNCE |
| Action type | Continuous |
| Action dimensions | 3 (X, Y, angular) |
| Action range | -1 to +1 |
| ROS integration | Disabled |

## 3. Movement Parameters

The active robot uses the aggressive locomotion configuration.

| Parameter | X | Y | Angular |
|---|---:|---:|---:|
| Maximum velocity | 0.55 m/s | 0.55 m/s | 2 rad/s |
| Preferred velocity | 0.35 m/s | 0.35 m/s | 1 rad/s |
| Maximum acceleration | 0.55 m/s² | 0.55 m/s² | 3 rad/s² |
| Maximum jerk | 5 m/s³ | 5 m/s³ | 50 rad/s³ |

Normalized actions are converted into velocity commands using the maximum velocity limits. The X and Y components are jointly normalized to prevent diagonal translational commands from exceeding the configured maximum speed.

**Note:** MarkEnv enables NO_INERTIA. Therefore, the normal acceleration and jerk-based update is bypassed. Maximum velocity clipping remains active.

## 4. LiDAR Configuration

| Parameter | Value |
|---|---:|
| Scan type | Merged LiDAR |
| Number of readings | 1080 |
| Field of view | Approximately 360 degrees |
| Minimum range | 0.05 m |
| Configured maximum range | 100 m |
| Angular increment | Approximately 0.333 degrees |
| Scan period | 0.05 seconds |
| Scan frequency | 20 Hz |

The effective sensing distance may be limited by the simulated map and ray-tracing environment.

## 5. Simulation Environment

The default MarkEnv map configuration uses:
- markclutter
- markmaze

The environment selects a map randomly when no specific scenario is supplied.

| Parameter | Value |
|---|---|
| Simulation timestep | 0.5 seconds |
| Maximum episode length | 1000 steps |
| Action mode | Continuous |
| Number of agents | 1 |
| Collision mode | BOUNCE |
| Deterministic spawn | Disabled by default |
| Circular scenario | Disabled by default |

The robot's spawn position and goal are selected randomly by default, subject to reachability and distance constraints.

## 6. Observation Space

The MarkEnv observation consists of two components:

| Observation | Shape | Description |
|---|---|---|
| LiDAR observation | (1080,) | Merged LiDAR readings |
| Robot-state observation | (5,) | Goal-relative position and robot velocity information |

The robot-state observation contains goal-relative X and Y values, linear velocity X and Y, and angular velocity.

## 7. Important Source Files

| File | Responsibility |
|---|---|
| navrep/envs/markenv.py | MarkEnv configuration, action space, timestep and map selection |
| pepper_2d_simulator.py | Pepper movement, velocity limits, collision radius and LiDAR configuration |
| pepper_2d_iarlenv.py | Additional simulator wrapper and ROS-related functionality |

The active MarkEnv environment directly uses PepperRLEnv. Parameters found in other wrappers do not necessarily apply to the active configuration.

## 8. Verification Performed

The following tests were successfully completed:

1. Created the MarkEnv environment.
2. Reset the environment successfully.
3. Verified observation shapes: (1080,) and (5,).
4. Executed one random action successfully.
5. Verified that the environment returned a reward and a non-terminal status.

These tests confirm basic environment initialization, reset, observation generation and one simulation step.

## 9. Configuration Notes

- Robot configuration was inspected from the simulator source code.
- Existing movement and sensor parameters were retained rather than changed arbitrarily.
- NO_INERTIA is important when interpreting movement behavior.
- The active MarkEnv configuration uses one robot and merged LiDAR observations.
- Physical hardware is not required because the project operates in simulation.

## 10. Member 2 Responsibilities

Member 2 is responsible for inspecting and documenting the robot model, movement limits, collision radius, sensor configuration, action interface and initial environment behavior.

This documentation provides the robot-related information needed by the navigation and obstacle-configuration team members.

## Runtime Movement Verification

The robot configuration was tested using the MarkEnv environment.

| Test | Command action | Observed result | Status |
|---|---|---|---|
| Forward movement | [0.5, 0.0, 0.0] for 3 steps | 0.4125 m accumulated forward movement | Passed |
| Rotation | [0.0, 0.0, 0.5] for 3 steps | Angular velocity 1.0 rad/s; accumulated angular movement 1.5 rad | Passed |
| Stop | [0.0, 0.0, 0.0] | Linear and angular velocities became zero | Passed |

All three tests completed without terminating the episode.

These tests verify that the configured robot can move forward, rotate in place, and stop using the normalized MarkEnv action interface.
