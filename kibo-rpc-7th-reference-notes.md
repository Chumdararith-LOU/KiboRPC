# 7th Kibo-RPC — Reference Notes

Distilled from four source PDFs (uploaded 2026-11-14):
- **Guidebook** v1.0 (JAXA, 2026-07-06)
- **Rulebook** v1.0 (JAXA, 2026-10-05)
- **Programming Manual** v1.0 (JAXA, 2026-10-05) — contact z-krpc7@ml.jaxa.jp
- **Quaternions and Rotations** — Yan-Bin Jia, Com S 477/577 lecture notes (Iowa State, 2013)

Team repo: https://github.com/Chumdararith-LOU/KiboRPC (full krpc7th-simulator workspace + team code; see §9). **From 2026-11-22 the working branch is `sim-v1.2`** (JAXA simulator v1.2; see §11).
Beginner course (14 modules): https://claude.ai/artifact/CEAxdF8ycwKFFW1yf7pJDy

---

## 1. Key dates

| Item | Date |
|---|---|
| Application deadline | 2026-09-30 23:59 JST |
| Simulator / Rulebook / Programming Manual released | Early Oct 2026 |
| Simulator v1.2 released | Nov 2026 (team branch `sim-v1.2`, 2026-11-22) |
| **Question deadline** | **2027-01-20 12:00 JST** |
| **Program submission deadline** | **2027-01-20 23:59 JST** |
| Preliminary round period | 2027-01-21 → 2027-02-17 |
| Finalist program modification | Early Mar → mid-Apr 2027 (ISS-final simulator mid-Mar) |
| ISS Final Round run | Early May 2027 |
| Final Round Event (results announced) | June 2027 |

Cambodia is a participating country. Teams: 3–8 members (guardians not counted). Beginner Course (side event, junior-high age or younger, fewer checkpoints, orientation not scored, no ISS run).

---

## 2. The game (Preliminary Round)

**Scenario:** slight pressure drop in Kibo → Int-Ball2 (IB2) inspects suspected leak locations.

**Flow:**
1. Timer starts when IB2 undocks from the Dock Station.
2. Get checkpoint positions/attitudes (provided automatically via API).
3. Visit all CheckPoints on an efficient route.
4. At each CheckPoint, read the AR marker at the required position + attitude.
5. Reach the GoalPoint → send the mission-completion report.

**Preconditions**
- 6–12 CheckPoints depending on difficulty (Easy / Normal / Hard / Very Hard).
- A CheckPoint is a sphere positioned randomly relative to an AR marker (angle + distance params).
- Disturbance: **microgravity module** (simulator only); its pattern is not disclosed.
- **Time limit: 5 minutes** from undock, completion report must be inside the limit.
- Known in advance: start point, rounding-completion report pose, total CP count, CP range per difficulty, AR marker poses + IDs, goal point.
- Random per run: each CP's offset from its marker. Not disclosed: CP pose pattern, gravity pattern.

**Runs:** each submitted program is auto-run **10 times** with different random/disturbance conditions; **final score = average of 10**.

**Submission:** run the program on the "Preliminary" simulator → must reach **FINISHED** status (FAILED can't be submitted) → RESULTS → SUBMIT. CANCEL to swap before deadline; buttons lock after deadline.

---

## 3. Scoring

A CheckPoint pass needs **all three conditions held simultaneously for 1 second**:
1. **Position** — distance to CP centre within `score_bands` radius.
2. **Camera centering** — angle between camera line-of-sight and direction to the AR marker within `centering_bands`.
3. **Roll about line of sight** — rotation about the camera axis vs. target `attitude.quaternion` within `roll_bands`.

**Level = lowest of the three** (e.g. Lv3/Lv2/Lv3 → Lv2). Any condition out of range → no points.

Example bands from the API (per checkpoint, read from the response, don't hard-code):

| Level | Position radius | Centering angle | Roll angle |
|---|---|---|---|
| Lv3 | 0.1 m | 10° | 10° |
| Lv2 | 0.2 m | 20° | 20° |
| Lv1 | 0.3 m | 30° | 30° |

**Formula terms:** `Pt_check` (per CP, by level) + `Pt_goal` (all CPs passed and goal reached in time) = `Pt_all`; `Score = Pt_all × Mag_clear`. `Mag_clear` is a time-remaining multiplier, **max ≈ 2×**. Actions after Mission Finish don't count. Simulator v1.2 now enforces the "all CPs passed" condition for the goal (§11).

Manual tip: first secure CheckPoints Passed + Goal Reached, then maximise Time Bonus.

**Web scorer, measured 2026-11-22** (dashboard of the three v1.6 runs, simulator 1.2, course "Regular"):

| Run | Passed | Goal | Mission Time (= judge `duration_sec`, Mission Start → Mission Finish) | Time Bonus | Final Score |
|---|---|---|---|---|---|
| Normal | 8/8 Lv3 | Reached (+30) | 234.35 s | ×1.22 | 134.10 |
| Hard | 8/10 Lv3 | Failed | 250.57 s | ×1.17 | 93.20 |
| Very hard | 9/12 Lv3 | Failed | 260.25 s | ×1.13 | 102.00 |

- **Lv3 checkpoint = 10 pt** (the goal checkpoint CP12 also scores its 10 pt). Lv2 / Lv1 values not yet seen.
- **Goal Score = 30 pt**, only when every checkpoint passed and the goal is reached.
- **Time Bonus ≈ 1 + (300 − T) / 300**, with T measured from **Mission Start** (not the start signal). It fits all three runs within 0.1 pt; at T = 0 it is 2.0, matching "max ≈ 2×".
- **Final Score = (Σ checkpoint points + goal points) × Time Bonus.**
- Consequences: a second saved is worth S/300 pt (0.37 pt/s on a perfect Normal run). On Hard/Very hard (no goal), an extra checkpoint pays only if it costs less than about 3000·M/(S+10) s, about 35–39 s. A skipped checkpoint on Normal costs 10 + 30 pt.
- The dashboard marks checkpoints not in the course as N/A, and in-course checkpoints not passed as FAIL.

---

## 4. Coordinates (origin = docking station)

**IB2 start:** DockingStation pos (0.200, 0.000, 0.000), quat (x,y,z,w) = (0, 0, 0, 1)

| ID | Target | x | y | z | qx | qy | qz | qw |
|---|---|---|---|---|---|---|---|---|
| 1 | SAIBO | 1.451 | -1.001 | 0.681 | -0.707 | 0 | 0 | 0.707 |
| 2 | RYUTAI | 2.973 | -1.001 | 0.718 | -0.707 | 0 | 0 | 0.707 |
| 3 | MSPR_1 | 3.313 | -1.001 | 0.523 | -0.707 | 0 | 0 | 0.707 |
| 4 | MSPR_2 | 4.181 | -1.001 | 0.523 | -0.707 | 0 | 0 | 0.707 |
| 5 | RMS_Console | 5.754 | -1.001 | 0.838 | -0.707 | 0 | 0 | 0.707 |
| 6 | EPS1 | 1.895 | -0.282 | -0.208 | 0 | 0 | 0.707 | -0.707 |
| 7 | MELFI1 | 1.451 | -1.001 | 0.372 | -0.707 | 0 | 0 | 0.707 |
| 8 | EPS2_2 | 4.779 | 0.112 | 1.884 | -0.707 | 0.707 | 0 | 0 |
| 9 | EPS2_1 | 2.066 | 1.046 | -0.014 | 0 | 0.707 | -0.707 | 0 |
| 10 | ECLSS_TCS2 | 5.851 | 0.010 | 1.884 | -0.707 | 0.707 | 0 | 0 |
| 11 | Airlock | 7.195 | 0.363 | 1.230 | 0.5 | 0.5 | -0.5 | -0.5 |
| 12 | Bay0_Closeout_Panel | 0.000 | -0.059 | -0.102 | -0.5 | 0.5 | -0.5 | 0.5 |

Note: the API example uses `goalpoint_id: 12`. (The simulator's `area_config.json` stores marker quaternions in a different convention, e.g. marker 1 = [-0.5, 0.5, 0.5, 0.5]; positions match.)

**AR markers:** ArUco **4x4 dictionary**, IDs **1–12**, side length **0.02 m** (OpenCV 3.3.1 ArUco).
**Camera:** front-mounted, optical axis = body +X; offset **x = 0.14 m**, y = 0, z = 0 (body frame: +X front, +Y left, +Z up). OpenCV camera frame is X right, Y down, Z forward, so a frame conversion is needed to get to `body` / `iss_body`.

---

## 5. Dev environment

- ROS 1 **Melodic**, GCC 7.5.0 / **C++14**, Python **3.6.9** (Manual). ⚠ Guidebook says Python 2.7.17, so trust the Manual / template. No dataclasses, no walrus operator.
- Ubuntu needed (WSL2 on Windows) even for Web-sim-only users (to unpack resources and run `packing.sh`).
- Local sim: Docker; min 4 cores / 8 GB RAM / 20 GB (rec. 6 cores / 16 GB / 40 GB SSD). The manual says Mac is unsupported, but the team runs Gazebo in Docker on a Mac. On Apple Silicon, `--arm64` matches the web sim.
- **Web sim runs ARM64** (like the real IB2 Jetson TX2); local sim is x86_64 unless built with `bash ./build.sh --arm64` (QEMU).

```bash
mkdir -p ~/krpc && tar -xvf krpc7th-simulator.tar.gz -C ~/krpc
cd ~/krpc/krpc7th-simulator
bash ./build.sh            # --no-cache | --arm64 | --platform=linux/amd64
bash ./simulation.sh       # view at http://localhost ; Ctrl+C to stop
bash ./replay.sh           # replay rosbag
cd users && ./packing.sh YourSourceCode   # → YourSourceCode.tar.gz for the Web sim
```

Local sim env knobs: `LEVELS=easy|normal|hard|veryhard`, `playTime=5m` (default is 10m; official limit is 5 min), `SIM_MODE=regular|beginners`, `ENABLE_DEBUG_MODE=true`. v1.2 adds `USERS_DIR_OVERRIDE` (mount another users folder).

**Only edit** `users/catkin_ws/src/krpc_user/`. Templates: `scripts/user_template.py`, `src/user_template_node.cpp`. Don't touch `krpc_api_server`, `krpc_ib2_msgs`, `docker/`, `rootfs/`, `rviz/`, `bootstrap/`.
Local outputs: `share/logs/<timestamp>/` → `cheat_sheet.json` (CP layout for that run), `simulation.bag`, `judge_module_logs/` (`result.json` has per-CP distance / centering / roll numbers; `localization_history.txt` has the pose every 0.1 s), `user_programs_logs/`.
Web sim downloads: result JSON (`Illegal`, `JudgeReport.mission/checkpoint_detail/goal/events`) + user program log only. Use rosbag for sensor data.

---

## 6. Game API (ROS)

| Name | Type | Definition | Purpose |
|---|---|---|---|
| `/mission/get_checkpoints` | Service | krpc_ib2_msgs/CheckPoints.srv | Empty req → `success`, `message` (JSON string) |
| `/event/mission_start` | Service | krpc_ib2_msgs/MissionStart.srv | Notify start |
| `/event/checkpoint_passed` | Action | krpc_ib2_msgs/CheckpointPassed.action | Goal `int32 checkpoint_id` → Result `success`, `message` (v1.2: the real verdict, see §11) |
| `/event/mission_finished` | Service | krpc_ib2_msgs/MissionFinished.srv | Completion report (at GoalPoint) |
| `/sensor_fusion/navigation` | Topic | ib2_msgs/Navigation | pose, twist, accel `a`, status (0 OFF, 1 FUSION, 2 INERTIAL, 3 SLAM) |
| `/ctl/command` | Action | ib2_msgs/CtlCommand.action | Goal: `PoseStamped target`, `CtlStatusType type` |
| `/ctl/wrench` | Topic | geometry_msgs/WrenchStamped | Direct force [N] / torque [Nm] in `body` (harder; don't mix with /ctl/command carelessly) |
| `/ctl/status` | Topic | ib2_msgs/CtlStatus | pose, twist, a, current `type` |
| `/camera_main/image_raw` | Topic | sensor_msgs/Image | Camera image (cv_bridge → OpenCV) |
| `/camera_main/camera_info` | Topic | sensor_msgs/CameraInfo | `K` (3x3), `D` distortion; None until first msg |
| `/ib2_user/start` | Topic | platform_msgs/UserLogic | **Mandatory** subscription; mission entry point |
| `/ib2_user/stop` | Service | platform_msgs/StopProcessingUserNode | **Mandatory**; cancel running actions, then respond SUCCESS(0)/ERROR(255) |
| `/ib2_user/ready` | Param | bool | **Mandatory** `True`; start isn't published until set |

**get_checkpoints JSON:** `checkpoint_count`, `checkpoints[]` each with `checkpoint_id`, `position[3]`, `marker_pose{position, quaternion}`, `score_bands[]`, `attitude{quaternion, centering_bands[], roll_bands[]}`; plus `goalpoint_id`. Quaternions are **[x, y, z, w]**.

**CtlCommand types:** `MOVE_TO_ABSOLUTE_TARGET` (40, frame `iss_body`), `MOVE_TO_RELATIVE_TARGET` (30, frame `body`), `STOP_MOVING` (20, target zero → then KEEP_POSE).
Result codes: TERMINATE_SUCCESS 0, ABORTED 1, TIME_OUT 2, INVALID_NAV 3, INVALID_CMD 4.
Other CtlStatusType values: STAND_BY 0, CAPTURED 1, DISTURBED 2, KEEP_POSE 10, KEEPING_POSE_BY_COLLISION 11, RELEASE 50, DOCK 60, …, DOCK_WITHOUT_CORRECTION 69, SCAN 70.
Poll `/ctl/status.type` going from MOVE_* → KEEP_POSE/STOP_MOVING to detect completion. Cancel with `cancel_goal()` then `wait_for_result(timeout)`. On cancel the controller runs a stopping profile, then holds pose (KEEP_POSE), so it does not drift away.

**Important:** a successful move ≠ a pass. Only call `checkpoint_passed` after verifying position + centering + roll are met **and** the vehicle is stable (hold ≥1 s). The template's default origin target earns nothing.

**Template flow:** `__callback_start` (double-start guard) → `__run_user_flow`: `mission_start` → USER EDIT AREA (route + per-CP move/verify/pass) → `mission_finished`.

**Robustness rules (Manual §5, §6.5):** finite timeouts on every `wait_for_service`/`wait_for_server`; retry limits; don't resend `checkpoint_passed` endlessly; skip camera logic if no fresh frames; track remaining mission time; log target vs current pose, retries, timings.

---

## 7. Quaternion cheat-sheet (from Jia notes)

- q = q0 + q1·i + q2·j + q3·k (scalar first in the notes; **ROS/API order is [x, y, z, w]**, so w = q0). Eigen's constructor is `Quaterniond(w, x, y, z)`.
- Product: **pq = p0q0 − p·q + p0·q + q0·p + p×q** (non-commutative).
- Conjugate q* = q0 − q; norm |q|² = q0² + |q|²; inverse q⁻¹ = q*/|q|² (= q* for unit q).
- Unit quaternion for rotation θ about unit axis u: **q = cos(θ/2) + u·sin(θ/2)**.
- Rotate vector v: **v' = q v q*** (treat v as a pure quaternion).
- Composition: q2 ⊗ q1 = q1 then q2 in the world frame; q1 ⊗ q2 = q1 then q2 in the body frame.
- q and −q represent the same rotation. Angle between attitudes = 2·acos(|a·b|).
- Twist about unit axis a: 2·atan2(q_xyz·a, w) (flip q if w < 0). This is the judge's roll measure.

---

## 8. Known doc quirks
- Python version mismatch (Guidebook 2.7 vs Manual 3.6.9).
- Rulebook TOC page numbers are wrong; the appendix itself notes `Pt_all` wording "needs to be unified" and "GSE" undefined, so scoring formula details may be revised.
- Final Round details "to be announced later"; final runs are ordered in tiers by preliminary rank, and a rerun is allowed only if the failure wasn't caused by the team's code.

---

## 9. How the local simulator v1.0 judged (from the repo, reviewed 2026-11-14)

**Partly superseded by simulator v1.2 (§11):** the single-instant evaluation, "first report locks", the always-"accepted" reply and the lax goal rule below are v1.0 behaviour only.

Source: `rootfs/krpc/judge_module/judge.py`, `rootfs/krpc/random_env_generator/generate_objects.py`, `users/catkin_ws/src/krpc_api_server/`. The Web simulator may differ, so treat this as strong evidence, not a guarantee.

**How checkpoints are generated** (unchanged in v1.2)
- Each CP is **0.5 m from its marker**, along the marker's normal with random yaw ±20° and pitch ±15°, kept inside the SafeArea `min [0.2, -0.8, 0.0]`, `max [7.0, 0.85, 1.68]`.
- The target quaternion makes body +X point from the CP centre straight at the marker, then adds a **random roll of ±180°** about that line (roll_q ⊗ base_q, world frame).
- So flying exactly to `position` + `attitude.quaternion` gives centering ≈ 0° and roll ≈ 0°.
- CP count: easy 6, normal 8, hard 10, veryhard 12. The goal marker (12) is always included and is **appended last**; the other markers come in random order.

**How the judge scores** (per-pose formulas unchanged in v1.2)
- Pose source is **`/sensor_fusion/navigation`**, the same topic the user program can read. The program can compute exactly what the judge will see.
- Distance is measured from the **robot centre** to the CP `position`.
- Centering is the angle between body +X (rotated by the robot's quaternion) and the vector **robot centre → marker position**. The camera's 0.14 m offset is *not* used here.
- Roll is the difference between the robot's and the target's twist angle about that same robot→marker axis, wrapped to ±180°.
- Level = min of the three; a 0 in any of them = fail.
- v1.0 only: the pose is evaluated **once, at the instant `/event/checkpoint_passed` arrives**.
- v1.0 only: **only the first report per CP counts** ("checkpoint N already processed").
- v1.0 only: at `mission_finished`, the goal counts if its CP was already passed; otherwise the goal CP is re-evaluated at the current pose.
- v1.0 only: `krpc_api_node` always returns `success=True` ("accepted") for checkpoint_passed.
- No time multiplier is computed locally (only the Web scorer does it).

**Review of the team code (commit f3aabe3, `scripts/user_template.py`)**
- Current USER EDIT AREA: fetch CPs → loop in list order → MOVE_TO_ABSOLUTE_TARGET (30 s timeout) → immediately call checkpoint_passed → mission_finished.
- Issues found:
  1. No settle/verify step before reporting.
  2. No route optimisation.
  3. No 5-minute mission clock or goal fallback.
  4. `__action_active` isn't set in the custom loop, so `/ib2_user/stop` won't cancel an in-flight move.
  5. Unbounded `wait_for_service`, and no confirmation that the cancel finished after a timeout.
  6. `goalpoint_id` is read but never used.

---

## 10. Robot and controller facts, plus timing estimates (from source, 2026-11-14)

Sources: `docker/ros_overlay/src/platform_sim/simulation/ib2_gazebo/sim/sim.yaml` (repo) and JAXA `int-ball2_simulator` (melodic branch) `platform/guidance_control/ctl_only/ctl.yaml` + `src/`. Verify the estimates with Gazebo experiments E2–E5 (course Module 14).

**Robot**
- Mass 3.216 kg, radius 0.10 m, inertia 0.0136 kg·m² per axis.
- 8 fans, each 0.06 N max with ±0.012 N noise. Max push along body x is 0.181 N (4 fans); along y about 0.10 N, along z about 0.12 N.
- Control runs at 200 Hz with a 0.1 s nav delay.
- Simulated nav error: about 2 mm bias plus 0.85 mm noise in position, about 0.01° in attitude.
- Real robot: about 20 cm diameter, Jetson TX2, stereo VSLAM + Epson M-G370 IMU, about 3 h battery, launched June 2023. JAXA reports about 8% nav error on long, complex flights.

**Controller**
- Profile limits: v_max 0.20 m/s, w_max 0.436 rad/s (25°/s). Moves are planned with eta = 0.4 of capacity, so a ≈ 0.0225 m/s² and α ≈ 0.24 rad/s².
- PD gains:
  - Position: kp 0.6219, kd 1.1152, ki 0. That gives ωn 0.79 rad/s and ζ 0.707.
  - Attitude: kp 2.6181, kd 3.2361, giving ζ = 1.0.
- **Absolute moves translate first (holding the start attitude), then rotate.** The times add up.
- **TERMINATE_SUCCESS** needs 3 s (duration_goal) of staying within tolerance_pos 0.10 m and tolerance_att 0.5. The 0.5 is compared against an angle in radians, so it is about 29°. Success is therefore much looser than Lv3, and it costs about 3 s per move.
- Bang-coast-bang timing: t = d/v + v/a if d > v²/a (1.78 m), else 2√(d/a).
  - Estimated moves: 1 m ≈ 13.3 s, 3 m ≈ 23.9 s, 7 m ≈ 43.9 s.
  - Estimated turns: 90° ≈ 5.4 s, 180° ≈ 9.0 s.

**Mission-time estimates.** Each level was run on 20 random courses with the 3 s settle model.

| Level | Course order (mean) | Held–Karp best route (mean) |
|---|---|---|
| Easy | 196 s | 166 s |
| Normal | 248 s | 203 s |
| Hard | 300 s | 238 s |
| Very hard | 359 s | 277 s |

Route planning is mandatory for Hard and Very hard. Example (sample Normal course, seed 7): course order took 244 s and the best route 9, 8, 5, 3, 6, 1, 7, goal took 206 s. (These early estimates used ctl.yaml physics; the measured model in the experiment log is slower, see §11 for the current full-route figures.)

**Judge geometry insight.** Sideways error s at 0.5 m adds about atan(s/0.5) of centering error: 5.4 cm gives 6°, and 8.8 cm alone uses up the 10° Lv3 limit. Fix this by aiming from the actual position: point body x at the marker from the current pose, and keep the target twist about that line.

---

## 11. Simulator v1.2 changes (team branch `sim-v1.2`, commit 3473d0e, reviewed 2026-11-22)

Files changed: `judge.py` (+372/−117), `krpc_api_node.py`, `cheat_sheet_loader.py`, `run.sh`, new `krpc_msgs/srv/EvaluateCheckpoint.srv`, a new status indicator, and the ISS model (gravity off for `iss_body`). The checkpoint generator is unchanged.

**1. Checkpoints are judged over a hold window.**
- `checkpoint_passed` now calls the judge service `/krpc/judge/evaluate_checkpoint`, which **blocks** while it collects navigation poses for `checkpoint_hold_seconds` (default **1.0 s**, sampled at up to 30 Hz).
- Every sample must be inside the outermost bands, and the score is the **lowest level over all samples**. Drifting during the second after the report lowers or voids the checkpoint.
- The hold time can be set from the cheat sheet (`checkpoint_hold_seconds`) or a judge rosparam; the web sim might use another value.
- Only one evaluation runs at a time ("checkpoint evaluation in progress").

**2. The report now returns the real verdict.**
- `result.success` / `result.message`: "checkpoint accepted: band=N", or a reason such as "checkpoint left outermost margin", "mission not started", "checkpoint already passed: N".
- **A failed checkpoint can be reported again.** Only a successful one is locked (so a Lv1 pass still blocks a later Lv3 try).
- "mission not started" makes the E6 race detectable from our side. `mission_start` has no repeat guard in `krpc_api_node`, so it can be called again.

**3. The goal rule is now strict (matches the Rulebook's Pt_goal).**
- At `mission_finished`, the judge collects another 1 s hold at the current pose and evaluates it against the goal checkpoint.
- The goal succeeds **only if every checkpoint in the course, including the goal checkpoint itself, has passed**. Otherwise the reason is "checkpoints not passed: …".
- The old "goal checkpoint previously passed" shortcut is gone.
- Consequence: any skipped checkpoint forfeits Pt_goal. Our E3–E7 "goal reached" on Hard/Very hard would be failures under v1.2.

**4. Start-up.**
- `/ib2_user/start` is now **latched**, and `run.sh` confirms delivery to `/krpc_user` (aborts after 60 s if not delivered).
- The `/gs/data` publisher is still not latched and the judge is still launched with a fixed wait, so the E6 judge race can still happen in principle. Keep the v1.4 gate.
- Local bug: the abort flag `share/start_not_delivered` is written to the host share folder and never deleted. After one failed delivery, every later local run ends immediately. Fix: `rm -f share/start_not_delivered` before running.

**5. Full-route feasibility under the strict goal rule** (30 random courses per level; measured model × 0.96, plus 1 s hold per report and at the goal):

| Level | All CPs + goal, best route | Fits 300 s |
|---|---|---|
| Easy (6) | median 219 s (189–237) | 30/30 |
| Normal (8) | median 270 s (254–288) | 30/30 |
| Hard (10) | median 317 s (301–333) | 0/30 |
| Very hard (12) | median 368 s (357–378) | 0/30 |

(See §12 for the fan system, which our own controller in v2.0 uses to get past this.)

With the stock controller, Pt_goal is out of reach on Hard (needs about 5–10% faster) and Very hard (needs about 20% faster). Whether that matters depends on the unknown sizes of Pt_goal and Mag_clear (and whether Mag_clear needs the goal), which the web results should reveal.

---

## 12. The fan system and /ctl/wrench (from JAXA's source, 2026-11-23)

Sources: `int-ball2_simulator` `ctl_only/ctl.yaml` (`fan`), `fsm/src/fsm.cpp`, `fsm/src/thrust_allocator.cpp`, `ctl_only/src/ctl.cpp`, `pos_att_profiler.cpp`; team repo `sim.yaml` (`thr_parameter`, `nav_parameter`); `ib2.urdf`.

**Pipeline:** `/ctl/wrench` (body-frame force N, torque N·m) → `fsm` node: fan forces f = Wp·y⁺ + Wm·y⁻, then minus the smallest (fans cannot push backwards; a negative value becomes positive through |f|), pwm = 4.0825·√f → **saturation rule:** if 4 or more fans exceed PWM 1.0, the 4 largest go to full and **all others switch off**; else clip → `/ctl/duty` → `prop` → `/prop/status` → fan plugin: force = min((pwm/4.0825)², 0.06 N), 0.01 s lag, noise N(0, 0.2·f) → body wrench Σ f·vec, Σ f·(r × vec). The fans hold the last command until a new wrench arrives.

**Capability (no torque):** along the nose (±x) 0.181 N; y 0.0996 N; z 0.122 N; typical direction 0.082 N; weakest 0.071 N. Torque: x 3.03, y 4.55, z 8.19 mN·m. Force and torque share the same fans.

**Body:** mass 3.216 kg; inertia diag(0.01362, 0.01400, 0.01306) kg·m² (ib2.urdf); CG offset about 1.5 mm.

**Navigation (sim.yaml):** 0.1 s delay; position bias (−2.1, −1.8, −0.07) mm + 0.85 mm noise; velocity bias (−5.6, −4.3, 0.24) mm/s + 1.5 mm/s noise; attitude about 0.01°. `twist.linear` is world frame, `twist.angular` body frame (as JAXA's controller uses them).

**Handover:** a `/ctl/command` goal of type STAND_BY (0) puts JAXA's controller on standby: it publishes one zero wrench and then nothing, and clears its collision detector each cycle. KEEP_POSE (10) makes it hold the current pose again. `/ctl/status` (10 Hz) reports its state in `type.type`. The Programming Manual allows `/ctl/wrench` ("more seamless movement", the program must manage stopping itself).

**Web-only nodes (from a v1.7 rosbag):** `keep_pose_release` sends its own `/ctl/command` goal 8 ms after our first command following Mission Start (this preempts our first move); a gravity/disturbance module of unknown strength. Neither exists in the local package.
