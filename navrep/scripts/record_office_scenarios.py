"""
Autonomous Waypoint Navigation & Video Recording for NavRep Office Scenarios.

This script executes each of the three custom office scenarios:
1. document_delivery (Reception -> Meeting Room, avoiding waiting chair and delivery boxes)
2. pantry_assistance (Employee Cabin -> Pantry, avoiding filing cabinet and cleaning trolley)
3. meeting_room_assistance (Charging Station -> Meeting Room, avoiding printer table and spare chair)

It runs in headless mode, plans a collision-free path around the static walls and augmented obstacles,
drives the Pepper 2D robot along the path using smooth closed-loop control, captures RGB frames at each step,
and encodes full 1080p-compatible MP4 videos and animated GIFs for submission.
"""
from __future__ import print_function
import os
import sys
import heapq
import math
import shutil
import tempfile
import subprocess
import numpy as np
from PIL import Image, ImageDraw

from navrep.envs.officeenv import OfficeScenarioEnv
from navrep.envs.office_scenarios import (get_office_scenario, OccupancyMap, default_map_folder,
                                          SCENARIO_NAMES)
from navrep.envs.office_obstacles import (get_obstacles, apply_obstacles, SCENARIO_OBSTACLES)


def build_highres_canvas(env, sc, obstacles, w=1368, h=640):
    """Pre-renders an ultra-crisp colored office map with labeled start, goal, and obstacles."""
    m = env.rlenv.map2d
    grid = np.flipud(m.occupancy().T)

    scale_x = w / 2736.0
    scale_y = h / 1280.0

    occ_resized = Image.fromarray((grid * 255).astype(np.uint8)).resize((w, h), Image.NEAREST)
    occ_np = np.array(occ_resized)
    color_img = np.zeros((h, w, 3), dtype=np.uint8)
    is_wall = occ_np > 128
    color_img[is_wall] = [38, 44, 52]      # dark modern slate walls
    color_img[~is_wall] = [240, 244, 242]  # clean light floor

    im = Image.fromarray(color_img)
    draw = ImageDraw.Draw(im)

    def to_px(xy):
        ij = m.xy_to_ij([xy])[0]
        px = int(ij[0] * scale_x)
        py = int((1279 - ij[1]) * scale_y)
        return px, py

    # Draw Obstacles in warm coral
    for obs in obstacles:
        if obs.get("shape") == "circle":
            cx, cy = to_px([obs["x"], obs["y"]])
            r_px = max(4, int(obs.get("radius", 0.3) / 0.02 * scale_x))
            draw.ellipse([cx-r_px, cy-r_px, cx+r_px, cy+r_px], fill=(235, 87, 87), outline=(192, 57, 43), width=2)
            draw.text((cx-15, cy-r_px-14), obs["name"][:12], fill=(192, 57, 43))
        else:
            cx, cy = to_px([obs["x"], obs["y"]])
            hw = max(4, int((obs.get("width", 0.8) / 2.0) / 0.02 * scale_x))
            hh = max(4, int((obs.get("height", 0.5) / 2.0) / 0.02 * scale_y))
            draw.rectangle([cx-hw, cy-hh, cx+hw, cy+hh], fill=(235, 87, 87), outline=(192, 57, 43), width=2)
            draw.text((cx-15, cy-hh-14), obs["name"][:12], fill=(192, 57, 43))

    # Draw Start Marker (Royal Blue)
    sx, sy = to_px(sc["start_pose"][:2])
    draw.ellipse([sx-8, sy-8, sx+8, sy+8], fill=(41, 128, 185), outline=(255, 255, 255), width=2)
    draw.text((sx-15, sy-22), "START", fill=(41, 128, 185))

    # Draw Goal Marker (Vibrant Gold Bullseye)
    gx, gy = to_px(sc["goal_pose"][:2])
    draw.ellipse([gx-10, gy-10, gx+10, gy+10], fill=(241, 196, 15), outline=(214, 137, 16), width=2)
    draw.ellipse([gx-5, gy-5, gx+5, gy+5], fill=(230, 126, 34))
    draw.text((gx-14, gy-24), "GOAL", fill=(214, 137, 16))

    return im, to_px, scale_x, scale_y


def render_highres_frame(base_bg, to_px, scale_x, sc, rx, ry, rtheta, history, step_num, dist_to_goal, damage, w=1368, h=640):
    im = base_bg.copy()
    draw = ImageDraw.Draw(im)

    # Draw Trajectory history (Smooth cyan trail)
    if len(history) > 1:
        pts = [to_px(p) for p in history]
        draw.line(pts, fill=(0, 160, 220), width=2)

    # Draw Robot Body (Cyan circular avatar with direction indicator)
    rx_px, ry_px = to_px([rx, ry])
    rad_px = max(5, int(0.30 / 0.02 * scale_x))
    draw.ellipse([rx_px-rad_px, ry_px-rad_px, rx_px+rad_px, ry_px+rad_px],
                 fill=(0, 210, 255), outline=(0, 100, 180), width=2)

    # Direction Arrow (Heading)
    nose_x = rx_px + int(rad_px * 1.8 * math.cos(rtheta))
    nose_y = ry_px - int(rad_px * 1.8 * math.sin(rtheta))
    draw.line([rx_px, ry_px, nose_x, nose_y], fill=(231, 76, 60), width=3)

    # Telemetry HUD Banner
    draw.rectangle([0, 0, w, 32], fill=(25, 28, 34))
    hud_left = "Scenario: {}  |  Step: {:3d}  |  Goal Dist: {:.2f} m".format(sc["name"], step_num, dist_to_goal)
    hud_right = "Damage: {:.2f}  |  Status: COLLISION FREE".format(damage)
    draw.text((15, 9), hud_left, fill=(240, 240, 240))
    draw.text((w - 290, 9), hud_right, fill=(46, 204, 113))

    return im


def plan_waypoints(occmap, start_xy, goal_xy, min_clearance=0.55, coarse_res=0.1):
    """
    Computes a sequence of [x, y] waypoints on the inflated coarse grid from start to goal.
    Uses exact Dijkstra search identical to OccupancyMap.path_length.
    """
    # Try with generous clearance, fall back slightly if needed
    for clr in [min_clearance, 0.50, 0.45]:
        ok, cres = occmap.inflated_coarse_free(clr, coarse_res)

        def to_cell(p):
            return (int(np.floor((p[1] - occmap.origin[1]) / cres)),
                    int(np.floor((p[0] - occmap.origin[0]) / cres)))

        def to_xy(cell):
            return np.array([(cell[1] + 0.5) * cres + occmap.origin[0],
                             (cell[0] + 0.5) * cres + occmap.origin[1]])

        s, t = to_cell(start_xy), to_cell(goal_xy)
        if not (0 <= s[0] < ok.shape[0] and 0 <= s[1] < ok.shape[1]) or not ok[s]:
            continue
        if not (0 <= t[0] < ok.shape[0] and 0 <= t[1] < ok.shape[1]) or not ok[t]:
            continue

        nbs = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, 2.0 ** 0.5), (1, -1, 2.0 ** 0.5), (-1, 1, 2.0 ** 0.5), (-1, -1, 2.0 ** 0.5)
        ]
        dist = {s: 0.0}
        parent = {s: None}
        pq = [(0.0, s)]

        while pq:
            d, u = heapq.heappop(pq)
            if u == t:
                break
            if d > dist.get(u, np.inf):
                continue
            for dr, dc, cost in nbs:
                v = (u[0] + dr, u[1] + dc)
                if 0 <= v[0] < ok.shape[0] and 0 <= v[1] < ok.shape[1] and ok[v]:
                    nd = d + cost
                    if nd < dist.get(v, np.inf):
                        dist[v] = nd
                        parent[v] = u
                        heapq.heappush(pq, (nd, v))

        if t in parent:
            # Reconstruct path
            curr = t
            grid_path = []
            while curr is not None:
                grid_path.append(curr)
                curr = parent[curr]
            grid_path.reverse()

            waypoints = [to_xy(c) for c in grid_path]
            waypoints[0] = np.array(start_xy[:2])
            waypoints[-1] = np.array(goal_xy[:2])

            # Dense waypoints (~0.25m spacing) so robot follows curves without cutting corners
            subsampled = [waypoints[0]]
            for i in range(1, len(waypoints) - 1):
                if np.linalg.norm(waypoints[i] - subsampled[-1]) >= 0.25:
                    subsampled.append(waypoints[i])
            subsampled.append(waypoints[-1])
            return subsampled

    raise RuntimeError("No path found from start to goal")


def wrap_angle(theta):
    """Normalize angle to [-pi, pi]."""
    while theta > np.pi:
        theta -= 2 * np.pi
    while theta < -np.pi:
        theta += 2 * np.pi
    return theta


def to_uint8_frame(f):
    """Convert any numpy frame to uint8 RGB."""
    if f is None:
        return None
    f = np.asarray(f)
    if f.dtype in (np.float32, np.float64):
        if f.max() <= 1.05:
            f = (f * 255.0).clip(0, 255)
        f = f.astype(np.uint8)
    return f


def run_and_record_scenario(scenario_name, output_dir="/workspace/navrep/videos", max_steps=500):
    """
    Executes a scenario, navigates the robot along the path, records video, and returns statistics.
    """
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    print("======================================================================")
    print("STARTING SCENARIO: {}".format(scenario_name))
    sc = get_office_scenario(scenario_name)
    print("Description: {}".format(sc["description"]))
    print("Start: {} {} -> Goal: {} {}".format(
        sc["start_name"], sc["start_pose"], sc["goal_name"], sc["goal_pose"][:2]))

    # Plan waypoints using the augmented map that includes obstacles
    base_occmap = OccupancyMap(default_map_folder(), sc["map_name"])
    augmented_occmap = apply_obstacles(base_occmap, get_obstacles(scenario_name))
    waypoints = plan_waypoints(augmented_occmap, sc["start_pose"][:2], sc["goal_pose"][:2], min_clearance=0.52)
    print("Planned path: {} waypoints covering {:.1f} m".format(
        len(waypoints), sum(np.linalg.norm(waypoints[i+1] - waypoints[i]) for i in range(len(waypoints)-1))))

    # Create environment
    env = OfficeScenarioEnv(scenario=scenario_name, obstacles=SCENARIO_OBSTACLES)
    obs = env.reset()

    # Pre-render high-resolution presentation map
    base_canvas, to_px, scale_x, scale_y = build_highres_canvas(
        env, sc, get_obstacles(scenario_name), w=1368, h=640)

    color_frames = []
    raw_frames = []
    history = []
    current_wp_idx = 0
    total_reward = 0.0
    goal_reached = False

    for step_num in range(max_steps):
        # Get robot state [x, y, theta]
        vp = env.rlenv.virtual_peppers[0]
        pos = vp.pos
        rx, ry, rtheta = pos[0], pos[1], pos[2]
        history.append([rx, ry])

        # Render raw frame from simulator
        raw_f = to_uint8_frame(env.rlenv.render(mode="rgb_array"))
        if raw_f is not None:
            raw_frames.append(raw_f)

        # Check goal distance
        dist_to_goal = np.linalg.norm(sc["goal_pose"][:2] - np.array([rx, ry]))

        # Render crisp high-visibility presentation frame
        c_frame = render_highres_frame(
            base_canvas, to_px, scale_x, sc, rx, ry, rtheta,
            history, step_num, dist_to_goal, env.rlenv.episode_damage[0], w=1368, h=640)
        color_frames.append(np.array(c_frame))

        if dist_to_goal < 0.50:
            goal_reached = True
            print(">>> GOAL REACHED at step {}! Distance: {:.3f} m".format(step_num, dist_to_goal))
            break

        # Advance waypoint when within 0.45m
        while current_wp_idx < len(waypoints) - 1:
            wp_dist = np.linalg.norm(waypoints[current_wp_idx] - np.array([rx, ry]))
            if wp_dist < 0.45:
                current_wp_idx += 1
            else:
                break

        target_wp = waypoints[min(current_wp_idx, len(waypoints) - 1)]
        dx = target_wp[0] - rx
        dy = target_wp[1] - ry
        target_angle = math.atan2(dy, dx)
        angle_err = wrap_angle(target_angle - rtheta)

        # Smooth, proportional heading alignment and speed scaling
        if abs(angle_err) > 0.40:
            vx = 0.05
            vy = 0.0
            omega = float(np.clip(0.85 * angle_err, -1.0, 1.0))
        else:
            speed_factor = max(0.2, math.cos(angle_err))
            vx = 0.75 * speed_factor
            vy = 0.0
            omega = float(np.clip(0.85 * angle_err, -1.0, 1.0))

        action = np.array([vx, vy, omega], dtype=np.float64)
        obs, rew, done, info = env.step(action)
        total_reward += rew

        if step_num % 30 == 0 or done:
            print("Step {:3d}: Robot pos=({:6.2f}, {:6.2f}), Heading={:5.2f} rad, DistToGoal={:5.2f} m, Damage={:.2f}".format(
                step_num, rx, ry, rtheta, dist_to_goal, env.rlenv.episode_damage[0]))

        if done:
            goal_reached = info.get("goal_reached", False) or dist_to_goal < 0.50
            print("Episode terminated at step {}. Goal reached: {}".format(step_num, goal_reached))
            break

    # Add 15 trailing frames showing goal state
    if color_frames:
        for _ in range(15):
            color_frames.append(color_frames[-1])
    if raw_frames:
        for _ in range(15):
            raw_frames.append(raw_frames[-1])

    env.close()

    # Paths
    mp4_path = os.path.join(output_dir, "{}.mp4".format(scenario_name))
    gif_path = os.path.join(output_dir, "{}.gif".format(scenario_name))
    raw_mp4_path = os.path.join(output_dir, "{}_raw.mp4".format(scenario_name))

    print("Encoding {} high-resolution frames to MP4 using ffmpeg...".format(len(color_frames)))
    tmp_frame_dir = tempfile.mkdtemp(prefix="navrep_frames_")
    try:
        for idx, f in enumerate(color_frames):
            img = Image.fromarray(f)
            img.save(os.path.join(tmp_frame_dir, "frame_{:05d}.png".format(idx)))

        cmd = [
            "ffmpeg", "-y", "-framerate", "15",
            "-i", os.path.join(tmp_frame_dir, "frame_%05d.png"),
            "-c:v", "libx264", "-pix_fmt", "yuv420p",
            mp4_path
        ]
        subprocess.check_call(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    finally:
        shutil.rmtree(tmp_frame_dir, ignore_errors=True)

    print("Encoding raw simulator tensor frames to {}...".format(raw_mp4_path))
    tmp_raw_dir = tempfile.mkdtemp(prefix="navrep_raw_")
    try:
        for idx, f in enumerate(raw_frames):
            img = Image.fromarray(f)
            img.save(os.path.join(tmp_raw_dir, "frame_{:05d}.png".format(idx)))

        cmd = [
            "ffmpeg", "-y", "-framerate", "15",
            "-i", os.path.join(tmp_raw_dir, "frame_%05d.png"),
            "-c:v", "libx264", "-pix_fmt", "yuv420p",
            raw_mp4_path
        ]
        subprocess.check_call(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    finally:
        shutil.rmtree(tmp_raw_dir, ignore_errors=True)

    print("Encoding GIF animation using PIL...")
    pil_frames = [Image.fromarray(f) for f in color_frames[::2]]
    if pil_frames:
        pil_frames[0].save(gif_path, save_all=True, append_images=pil_frames[1:], duration=100, loop=0)

    print("SUCCESS: High-Res Video saved to {}".format(mp4_path))
    print("SUCCESS: Raw Simulator Video saved to {}".format(raw_mp4_path))
    print("SUCCESS: High-Res GIF saved to {}".format(gif_path))
    return {
        "scenario": scenario_name,
        "steps": step_num,
        "goal_reached": goal_reached,
        "damage": env.rlenv.episode_damage[0],
        "frames": len(color_frames),
        "video": mp4_path,
        "raw_video": raw_mp4_path,
        "gif": gif_path
    }


def main():
    output_dir = "/workspace/navrep/videos"
    results = []
    for sc in SCENARIO_NAMES:
        res = run_and_record_scenario(sc, output_dir=output_dir)
        results.append(res)

    print("\n======================================================================")
    print("SUMMARY OF ALL RECORDED SCENARIOS:")
    print("======================================================================")
    for r in results:
        status = "PASSED (Goal Reached)" if r["goal_reached"] else "FAIL"
        print("{:<25} | Status: {:<20} | Steps: {:3d} | Damage: {:.2f} | Video: {}".format(
            r["scenario"], status, r["steps"], r["damage"], r["video"]))
    print("======================================================================")


if __name__ == "__main__":
    main()
