# NavRep Environment Setup Report

## 1. Operating System
- Host: Windows 10 Home, 64-bit
- Linux environment: WSL2
- Distribution: Ubuntu 24.04.4 LTS
- Container platform: Docker Desktop 29.6.2
- Container Python: 3.6.15 (Debian Buster)

## 2. Repository
- Project: NavRep – Robot Navigation and Simulation
- Upstream: https://github.com/ethz-asl/navrep
- Repository path inside container: /workspace/navrep

## 3. Main Installed Dependencies
- Python 3.6.15
- pip 21.2.4
- setuptools 59.6.0
- wheel 0.37.1
- NumPy 1.19.5
- Cython 0.29.28
- Gym 0.15.7
- TensorFlow 1.13.2
- Stable-Baselines 2.10.0
- PyTorch 1.6.0
- Keras 2.3.1
- SciPy 1.5.4
- Matplotlib 3.3.4
- Pandas 1.1.0
- Pillow 8.4.0
- OpenCV 4.5.5.64
- PyYAML 6.0
- numba 0.53.1
- imageio 2.15.0

## 4. Installation Summary
1. Installed Docker Desktop and enabled WSL2 integration.
2. Built a Docker image based on Python 3.6.15 and Debian Buster.
3. Cloned the NavRep repository.
4. Installed compatible Python dependencies.
5. Installed supporting navigation and simulator dependencies.
6. Built required native extensions.
7. Configured the simulator map-resource package.
8. Applied NumPy dtype compatibility changes to the Pepper simulator.

## 5. Supporting Components
Supporting components include pymap2d, pyniel, pyIA, CMap2D, RangeLibc, rvo2, and asl_pepper_2d_simulator.

The Pepper simulator source was obtained from:
https://github.com/ethz-asl/interaction_actions_for_navigation

Some dependencies were installed from source repositories because the required packages were not available through the attempted PyPI installation.

## 6. Compatibility Changes
The Pepper simulator required double-precision NumPy arrays for certain Cython functions. The following conversions were applied in pepper_2d_simulator.py:
- Input to fast_2f_norm converted to numpy.float64.
- Velocity and velocity limits passed to fast_3f_clip converted to numpy.float64.
- Command-velocity array created with numpy.float64.

These changes resolved the observed dtype mismatch errors.

## 7. Errors and Solutions
### Legacy Python dependencies
The project uses older dependencies.
Solution: Python 3.6.15 was used inside Docker.

### Missing pymap2d
Solution: Cloned its source repository and installed locally.

### Missing simulator map-resource package
Solution: Created and installed a local asl_pepper_2d_sim_maps resource package pointing to the simulator map directory.

### Cython dtype mismatch
Solution: Converted affected NumPy arrays to float64.

### ROS unavailable
The simulator reported that ROS was not found and disabled ROS integration. Non-ROS mode was used.

### Torchvision CUDA dependency
The installed legacy torchvision package expected a CUDA 9 runtime library that was unavailable. CPU PyTorch import worked; torchvision functionality was not verified.

### CrowdNav dependency issue
The NavRep training-environment import failed with a CrowdNav package incompatibility: ModuleNotFoundError: No module named 'a'. The available public mirror also did not expose the CrowdSim class expected by NavRep.
Status: unresolved.

## 8. Verification
The following checks succeeded:
- Python and pip available inside the container.
- import navrep succeeded.
- Pepper simulator imported in non-ROS mode.
- MarkEnv was created successfully.
- MarkEnv reset returned observation data.
- Several environment steps completed.
- Simulator RGB rendering was tested.

These checks do not establish that every NavRep scenario or the complete training pipeline works.

## 9. MarkEnv Test Commands
Run inside the container:

    cd /workspace/navrep
    python -c "from navrep.envs.markenv import MarkEnv; env=MarkEnv(silent=True); print('Environment created'); env.close()"

To inspect observation shapes:

    python -c "from navrep.envs.markenv import MarkEnv; env=MarkEnv(silent=True); obs=env.reset(); print('Observation shapes:', obs[0].shape, obs[1].shape); env.close()"

## 10. Limitations
- Python 3.6 and Debian Buster are end-of-life.
- ROS integration was not enabled.
- Full NavRep training-environment import remains blocked by the CrowdNav dependency issue.
- torchvision CUDA-dependent functionality was not verified.
- Only the MarkEnv setup and limited tests described above are claimed as verified.

## 11. Handoff Notes
The repository is mounted at /workspace/navrep. Some manually installed source dependencies and simulator modifications currently reside under /root/Code inside the running container. They are not all stored in the repository. Preserve the existing container until these changes have been captured for reproducibility.

The environment setup and documentation work should be reviewed alongside the known limitations before other team members rely on it.
