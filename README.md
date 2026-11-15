# SARDAR VALLABHBHAI NATIONAL INSTITUTE OF TECHNOLOGY, SURAT
## DEPARTMENT OF COMPUTER SCIENCE & ENGINEERING
### CS359: Artificial Intelligence for Robotics | B.Tech. (CSE) IIIrd Year
**Assignment:** NavRep – Robot Navigation and Simulation  
**Project:** Office Assistance Robot Simulation  
**Date:** 15 November 2026  
**Upstream Base:** https://github.com/ethz-asl/navrep  

---

## 1. Group Members

| Sr. No. | Student Name | Roll Number | Role / Task Contribution |
|---|---|---|---|
| 1 | Rudra Manish Patni | U24CS074 | **Task 1:** Environment setup, Docker container, dependency installation & build documentation |
| 2 | Ridham Jain | U24CS078 | **Task 2:** Testing existing NavRep simulation scenarios & recording baseline videos |
| 3 | Ayushman Singh | U24CS103 | **Task 3:** Custom scenario start/goal design & Dijkstra path planning (`office_scenarios.py`) |
| 4 | Mori Aryan Ashokbhai | U24CS080 | **Task 4:** Obstacle design, geometric modeling & dynamic map augmentation (`office_obstacles.py`) |
| 5 | Sivahari Prakash Babu | U24CS005 | **Task 5:** Environment integration (`OfficeScenarioEnv`) & CLI scripts (`play_office_scenario.py`) |
| 6 | Janav Rana | U24CS039 | **Task 6:** Closed-loop proportional controller, autonomous execution & video production (`record_office_scenarios.py`) |
| 7 | Patel Vinay Atulkumar | U24CS077 | **Task 7:** Comprehensive test suite verification (78/78 tests passing) & submission preparation |

---

## 2. Operating System & Environment

- **Host OS:** Windows 10 / 11 (64-bit)
- **Container Platform:** Docker Desktop with WSL2 backend
- **Container OS:** Debian GNU/Linux 10 (Buster)
- **Python Version:** Python 3.6.15 (64-bit)
- **Compiler:** GCC 8.3.0 (`build-essential`)

---

## 3. Dependencies & Installation

NavRep requires legacy Python 3.6 due to dependencies like Gym 0.20, Pyglet 1.5, and older Cython extensions.

### Required Packages:
- `numpy==1.19.5`
- `PyYAML==6.0`
- `Pillow==8.4.0`
- `matplotlib==3.3.4`
- `scipy==1.5.4`
- `pandas==1.1.0`
- `Cython==0.29.28`
- `gym==0.20.0`
- `pyglet==1.5.22`
- `numba==0.53.1`
- `pytest==7.0.1`
- System packages: `libglu1-mesa`, `ffmpeg`, `libgl1-mesa-glx`

### Setup Commands (inside Docker):
```bash
# 1. Install system utilities
apt-get update && apt-get install -y build-essential git ffmpeg libglu1-mesa libgl1-mesa-glx

# 2. Install Python dependencies
pip install numpy==1.19.5 PyYAML==6.0 Pillow==8.4.0 matplotlib==3.3.4 scipy==1.5.4 pandas==1.1.0 Cython==0.29.28 gym==0.20.0 pyglet==1.5.22 numba==0.53.1 pytest==7.0.1

# 3. Build native C/Cython extensions
pip install --no-build-isolation pyrangelibc-danieldugas==0.0.2

# 4. Install Pepper simulator and map package
cd /root/Code/interaction_actions_for_navigation/external/asl_pepper/asl_pepper_2d_simulator && pip install -e .
cd /root/Code/asl_pepper_2d_sim_maps && pip install -e .
cd /workspace/navrep && pip install -e .

# 5. Apply simulator float64 patch
cd / && patch -p0 < /workspace/navrep/setup/patches/pepper_2d_simulator_dtype.patch
```

---

## 4. Code Modifications

1. **`navrep/envs/officeenv.py`:**
   - Created `OfficeScenarioEnv` as a clean subclass of `MarkEnv`.
   - Allows setting fixed start poses `[x, y, theta]` and goal poses `[x, y]`.
   - The original `MarkEnv` and `PepperRLEnv` remain unmodified.

2. **`navrep/envs/office_obstacles.py`:**
   - SoftBank Pepper simulator reads obstacles only from static occupancy maps at startup.
   - Built an obstacle generator that takes obstacle shapes (rectangles, circles) and draws them into a temporary copy of `asl_office_j.pgm` in `/tmp/`.
   - The copy is deleted automatically when `env.close()` is called.

3. **`navrep/envs/office_scenarios.py`:**
   - Defined 3 office assistance scenarios with verified start and goal coordinates.
   - Added Dijkstra search on inflated grid to compute shortest path lengths and check 0.50m robot clearance.

4. **`navrep/scripts/record_office_scenarios.py`:**
   - Implemented a closed-loop waypoint tracking controller:
     ```python
     omega = float(np.clip(0.85 * angle_err, -1.0, 1.0))
     speed_factor = max(0.2, math.cos(angle_err))
     vx = 0.75 * speed_factor
     ```
   - Automatically drives the robot, records frames, and encodes them to MP4 using ffmpeg.

5. **`navrep/envs/markenv.py`:**
   - Fixed `TypeError: render() got an unexpected keyword argument 'save_to_file'` in `render()` so standard rendering calls work without crashing.

6. **Map YAML relative paths (`maps/cla_glass_hall.yaml`):**
   - Replaced hardcoded `/home/daniel/maps/` paths with relative map paths so they load portably across different systems.

---

## 5. Challenges Encountered & Fixes

1. **Cython build error on modern pip:**
   - *Error:* `pyrangelibc` failed to build because Python 2 print syntax was rejected under build isolation.
   - *Fix:* Passed `--no-build-isolation` to pip so it uses Cython 0.29 with language level 2 compatibility.

2. **Simulator array dtype crash:**
   - *Error:* Cython extensions (`fast_2f_norm`, `fast_3f_clip`) expected double-precision `float64` pointers, while NumPy passed `float32`.
   - *Fix:* Applied `pepper_2d_simulator_dtype.patch` to explicitly cast vectors to `float64`.

3. **Angular oscillation bug:**
   - *Error:* The initial controller used a bang-bang switch ($\pm 1.0$ rad/s). Because $DT=0.5$s and $kMaxVel=2.0$ rad/s, each step rotated the robot by exactly $1.0$ rad ($57^\circ$), causing it to oscillate back and forth forever.
   - *Fix:* Replaced bang-bang steering with proportional error scaling ($0.85 \times \Delta\theta$), giving smooth exponential convergence to waypoints.

4. **Visual map transposition mismatch:**
   - *Error:* In `CMap2D`, the grid is indexed as `(2736, 1280)` with Dimension 0 as $X$ and Dimension 1 as $Y$. Drawing directly to image coordinates without transposing caused the background map to be squished sideways while coordinates were plotted upright.
   - *Fix:* Transposed and vertically flipped the occupancy grid with `np.flipud(m.occupancy().T)` so image axes match world coordinates.

---

## 6. How to Run

Run these commands inside the Docker container:

### Run Unit Tests
```bash
python -m pytest tests/test_office_scenarios.py tests/test_office_obstacles.py tests/test_office_integration.py -v
```
*(All 78 tests should pass)*

### Run Scenario Validation (No physics needed)
```bash
python -m navrep.scripts.play_office_scenario --scenario document_delivery --check
python -m navrep.scripts.play_office_scenario --scenario pantry_assistance --check
python -m navrep.scripts.play_office_scenario --scenario meeting_room_assistance --check
```

### Run Headless Simulation
```bash
python -m navrep.scripts.play_office_scenario --scenario document_delivery --headless --steps 100
```

### Record All Custom Scenarios Automatically
```bash
python -m navrep.scripts.record_office_scenarios
```
Videos will be saved directly into `videos/`.

---

## 7. Results Summary

### Task 2: Existing Scenarios
- `videos/existing_scenario_asl_office_j.mp4`: Tested standard NavRep `MarkEnv` in office floor.
- `videos/existing_scenario_hg_main_hall.mp4`: Tested standard NavRep in open hall.

### Task 3: Custom Office Assistance Scenarios
| Scenario | Start Position | Goal Position | Obstacles | Path Length | Steps | Collision Damage | Status | Output Video |
|---|---|---|---|---|---|---|---|---|
| **Document Delivery** | Reception `(-23.3, -0.7)` | Meeting Room `(13.9, 2.4)` | Chair, delivery boxes | 41.0 m | 198 | **0.00** | **Goal Reached** | `videos/document_delivery.mp4` |
| **Pantry Assistance** | Cabin `(-11.0, 5.1)` | Pantry `(5.0, -1.0)` | Cabinet, cleaning trolley | 23.3 m | 112 | **0.00** | **Goal Reached** | `videos/pantry_assistance.mp4` |
| **Meeting Room Assistance** | Charging Stn `(-1.5, 2.5)` | Meeting Room `(13.9, 2.4)` | Printer table, spare chair | 19.5 m | 94 | **0.00** | **Goal Reached** | `videos/meeting_room_assistance.mp4` |
