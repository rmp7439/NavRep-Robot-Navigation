# SARDAR VALLABHBHAI NATIONAL INSTITUTE OF TECHNOLOGY, SURAT
## DEPARTMENT OF COMPUTER SCIENCE & ENGINEERING
### CS359: Artificial Intelligence for Robotics | B.Tech. (CSE) IIIrd Year
**Assignment:** NavRep – Robot Navigation and Simulation (Task 3: Custom Scenarios)  
**Date:** 15 November 2026  

### Team Members:
- **Rudra Manish Patni (U24CS074)** – Setup & Environment (Task 1)
- **Ridham Jain (U24CS078)** – Existing Scenario Testing (Task 2)
- **Ayushman Singh (U24CS103)** – Custom Scenario Start/Goal Navigation (Task 3)
- **Mori Aryan Ashokbhai (U24CS080)** – Custom Obstacle Design & Map Augmentation (Task 4)
- **Sivahari Prakash Babu (U24CS005)** – Environment Integration & CLI (Task 5)
- **Janav Rana (U24CS039)** – Autonomous Waypoint Controller & Video Recording (Task 6)
- **Patel Vinay Atulkumar (U24CS077)** – Verification & Quality Assurance (Task 7)

---

# Office Assistance Robot: Custom Scenario Descriptions

## Project Overview
This project extends ETH Zurich's **NavRep** framework to deploy an autonomous **Office Assistance Robot** navigating within a realistic indoor workplace environment based on the `asl_office_j` floor plan (54.7 m × 25.6 m, 0.02 m/pixel resolution).

The robot model used is the **SoftBank Pepper** mobile robot simulated in 2D (`PepperRLEnv` / `MarkEnv` architecture) equipped with a 360-degree merged planar LiDAR scanner (1080 rays), differential/holonomic base dynamics, and physical obstacle collision TSDF representation.

Three challenging real-world scenarios were designed, implemented, and validated. Each scenario features:
1. Distinct starting pose `[x, y, theta]` and goal position `[x, y]`.
2. Static office obstacles strategically placed along the nominal path forcing intelligent avoidance maneuvers while ensuring at least 0.50 m robot clearance.
3. Automated waypoint generation via coarse Dijkstra search on inflated occupancy maps.
4. Smooth proportional heading and velocity control ensuring zero collisions (0.00 damage).

---

## Scenario 1: Document Delivery (`document_delivery`)
- **Real-World Context:** An office mail and document delivery task where the robot collects outgoing correspondence from the front reception counter and traverses the long central hallway to deliver them directly to the main executive meeting room.
- **Starting Location:** Reception Area `[-23.3, -0.7, 0.0]` (facing east towards corridor).
- **Goal Location:** Executive Meeting Room `[13.9, 2.4]`.
- **Path Length:** 41.0 metres (137 planned waypoints).
- **Obstacles Introduced:**
  1. `reception_waiting_chair`: Visitor waiting chair (circle, radius = 0.30 m) positioned at `(-21.3, -1.6)` in the lobby exit path.
  2. `delivery_boxes`: Stack of cardboard delivery boxes (rectangle, 0.80 m × 0.50 m) positioned at `(1.7, -1.1)` narrowing the central hallway.
- **Execution & Navigation Performance:**
  - Steps completed: 198 steps (at dt = 0.5 s, 99 seconds simulated time).
  - Damage accrued: **0.00 (Zero collisions)**.
  - Final distance to goal: 0.53 m (Goal Reached within threshold).
  - Deliverables: `videos/document_delivery.mp4` and `videos/document_delivery.gif`.

---

## Scenario 2: Pantry Assistance (`pantry_assistance`)
- **Real-World Context:** An employee in a cabin requests refreshments/coffee service. The robot navigates from the northern office cabin row, turns through the interconnecting corridor, and enters the employee pantry.
- **Starting Location:** Employee Cabin `[-11.0, 5.1, 0.0]`.
- **Goal Location:** Office Pantry `[5.0, -1.0]`.
- **Path Length:** 23.3 metres (77 planned waypoints).
- **Obstacles Introduced:**
  1. `corridor_filing_cabinet`: Metal filing cabinet (rectangle, 0.80 m × 0.40 m) positioned at `(-1.0, 5.4)` protruding into the north corridor.
  2. `cleaning_trolley`: Janitorial cleaning trolley (rectangle, 0.50 m × 0.80 m) parked at `(2.2, 2.0)` near the corridor intersection.
- **Execution & Navigation Performance:**
  - Steps completed: 112 steps (56 seconds simulated time).
  - Damage accrued: **0.00 (Zero collisions)**.
  - Final distance to goal: 0.53 m (Goal Reached within threshold).
  - Deliverables: `videos/pantry_assistance.mp4` and `videos/pantry_assistance.gif`.

---

## Scenario 3: Meeting Room Assistance (`meeting_room_assistance`)
- **Real-World Context:** The robot leaves its wall docking/charging station to provide presentation assistance, handing out equipment/materials to attendees gathered in the meeting room.
- **Starting Location:** Wall Charging Station `[-1.5, 2.5, 0.0]`.
- **Goal Location:** Executive Meeting Room `[13.9, 2.4]`.
- **Path Length:** 19.5 metres (65 planned waypoints).
- **Obstacles Introduced:**
  1. `hall_printer_table`: Shared office printer desk (rectangle, 0.80 m × 0.60 m) positioned at `(0.0, 1.0)`.
  2. `meeting_room_spare_chair`: Ergonomic conference chair (circle, radius = 0.30 m) pulled out into the entrance threshold at `(12.4, 0.4)`.
- **Execution & Navigation Performance:**
  - Steps completed: 94 steps (47 seconds simulated time).
  - Damage accrued: **0.00 (Zero collisions)**.
  - Final distance to goal: 0.51 m (Goal Reached within threshold).
  - Deliverables: `videos/meeting_room_assistance.mp4` and `videos/meeting_room_assistance.gif`.

---

## Summary of Results

| Scenario | Start Pose [x, y, th] | Goal [x, y] | Obstacles | Path Length | Steps | Collision Damage | Goal Reached | Video File |
|---|---|---|---|---|---|---|---|---|
| **Document Delivery** | (-23.3, -0.7, 0.0) | (13.9, 2.4) | Chair, Delivery boxes | 41.0 m | 198 | **0.00** | **YES** | `document_delivery.mp4` |
| **Pantry Assistance** | (-11.0, 5.1, 0.0) | (5.0, -1.0) | Filing cabinet, Trolley | 23.3 m | 112 | **0.00** | **YES** | `pantry_assistance.mp4` |
| **Meeting Room Assistance** | (-1.5, 2.5, 0.0) | (13.9, 2.4) | Printer table, Spare chair | 19.5 m | 94 | **0.00** | **YES** | `meeting_room_assistance.mp4` |

