# Kibo-RPC 7th Simulator — Deep-Dive Analysis Report

| Item | Content |
|---|---|
| **Subject** | `krpc7th-simulator` workspace (JAXA Kibo-RPC 7th) |
| **Audience** | Beginner developer preparing a competition program; supervisor review |
| **Prepared** | 2026-10-29 |
| **Scope** | Full directory inspection, architecture, scoring, API, run guide, workflow |

> **One-paragraph summary.** This workspace is a self-contained, Docker-based simulator for the
> JAXA **Int-Ball2** free-flying robot inside the ISS **Kibo** module. It runs a ROS 1 (Melodic) +
> Gazebo 9 physics simulation of a 3.2 kg spherical drone in microgravity, with 12 ArUco AR markers
> as checkpoints. A participant writes a single ROS node (Python or C++) in `users/catkin_ws/src/krpc_user/`
> that flies the robot to randomized checkpoints and reports each pass to a built-in "judge" node,
> which scores the run by position, camera-centering, and attitude. The whole thing is launched with
> three shell scripts (`build.sh`, `simulation.sh`, `replay.sh`) and packaged for the official Web
> simulator with `users/packing.sh`.

---

## Table of Contents

1. [Directory Structure & File Inventory](#section-1-directory-structure--file-inventory)
2. [Simulation Architecture Overview](#section-2-simulation-architecture-overview)
3. [Key Files Deep Dive](#section-3-key-files-deep-dive)
4. [How to Run the Simulation (Step-by-Step)](#section-4-how-to-run-the-simulation-step-by-step)
5. [API & Interface Reference](#section-5-api--interface-reference)
6. [Simulation Environment Details](#section-6-simulation-environment-details)
7. [Development Workflow & Best Practices](#section-7-development-workflow--best-practices)
8. [Key Findings & Recommendations for Supervisor](#section-8-key-findings--recommendations-for-supervisor)
9. [Appendix](#section-9-appendix)

---

# SECTION 1: Directory Structure & File Inventory

## 1.1 Top-level layout

```
krpc7th-simulator/
├── build.sh                       # Build the Docker simulator + replay images
├── simulation.sh                  # Launch one full simulation run
├── replay.sh                      # Replay a recorded rosbag
├── docker-compose.yaml            # Simulator + user-program containers (live run)
├── docker-compose.replay.yaml     # Replay container
├── opencode.json                  # AI-assistant (OpenCode) config — NOT part of the sim
├── README.md                      # Quick-start pointer to the Programming Manual
├── .gitignore                     # Ignores build/, devel/, share/, __pycache__/, etc.
├── .vscode/                       # VSCode C/C++ runner + debug config (dev convenience)
├── docker/                        # ★ Docker build definitions (do not edit)
├── rootfs/                        # ★ Simulator runtime scripts (do not edit)
├── rviz/                          # ★ RViz display config (do not edit)
├── share/                         # ★ Output: per-run logs, results, rosbag (generated)
├── users/                         # ★★ Participant working area (you edit here)
└── docs/                          # Programming Manual (EN + JA) + screenshots
```

### Purpose of each top-level directory

| Directory | Editable? | Purpose (plain English) |
|---|---|---|
| `docker/` | No | The "recipe" for building the simulator Docker image. Contains the base Dockerfile, the replay Dockerfile, and ROS packages (`ib2_msgs`, `krpc_msgs`, `platform_sim`) that get baked into the image. |
| `rootfs/` | No | The "brain" that runs *inside* the simulator container: it generates the random course, launches Gazebo, runs the judge/scorer, the flight recorder, the status overlay, and records the rosbag. |
| `rviz/` | No | `urdf.rviz` — the 3D visualization layout (robot model, camera images, markers, TF) shown in the browser. |
| `share/` | No (read-only output) | Where every run writes its results: `share/logs/<timestamp>/` and a `latest/` snapshot. This is what you inspect after a run. |
| `users/` | **Yes** | The only place you write code. Holds your `catkin_ws` with the `krpc_user` package, plus the provided API server and message definitions. |
| `docs/` | No | The authoritative `ProgrammingManual.md` (English) and `ProgrammingManual_ja.md` (Japanese), plus UI screenshots. |

> **Analogy.** Think of `docker/` + `rootfs/` + `rviz/` as the *airplane* (built and maintained by
> JAXA), `users/` as the *pilot's cockpit* (the only part you touch), and `share/` as the *black box*
> (the flight data written after each flight).

## 1.2 Root-level scripts

### `build.sh` (38 lines)
Builds **two** Docker images. It is run once (or when the image is updated).

```bash
BASE_TAG=${BASE_TAG:-bionic}
PLATFORM=${PLATFORM:-}
# ... parses --no-cache / --arm64 / --platform=...
docker build ... -t krpc/intball-simulator:latest -f docker/Dockerfile.krpc-base .
docker build ... -t krpc/krpc-replay:latest -f docker/Dockerfile.krpc-replay .
```

- **What it does:** builds `krpc/intball-simulator:latest` (the full simulator) and
  `krpc/krpc-replay:latest` (a thin image for replay). If the base build fails but a local
  `krpc/intball-simulator:latest` already exists, it reuses the cached image instead of aborting.
- **Options:** `--no-cache` (ignore build cache), `--arm64` (build for ARM64 via QEMU,
  `BASE_TAG=bionic-arm64`), `--platform=<p>` (explicit target platform).
- **Why it matters:** you must run this before the first `simulation.sh`. On an Apple-silicon / ARM
  machine, use `--arm64`.

### `simulation.sh` (4 lines)
```sh
docker compose -f docker-compose.yaml up --abort-on-container-exit --exit-code-from krpc
docker compose -f docker-compose.yaml down
```
- **What it does:** starts the two-container stack, waits until the `krpc` (simulator) container exits,
  then tears everything down. `--exit-code-from krpc` propagates the simulator's exit code.
- **Why it matters:** this is the single command to run a full simulation. Your program is built and
  started automatically inside the `krpc-user` container.

### `replay.sh` (6 lines)
```sh
export PLAY_RATE=${1:-1}
export START_TIME=${2:-0}
docker compose -f docker-compose.replay.yaml up --abort-on-container-exit --exit-code-from krpc
docker compose -f docker-compose.replay.yaml down
```
- **What it does:** replays the most recent `simulation.bag` in a 3D viewer. Args: playback speed
  (default `1`) and start time in seconds (default `0`).
- **Usage:** `bash ./replay.sh 2 30` → play at 2× speed starting at t=30 s.

### `docker-compose.yaml` (55 lines)
Defines **two** services on a shared bridge network `krpc`:

| Service | Image | Role |
|---|---|---|
| `krpc` | `krpc/intball-simulator:latest` | The simulator body: Gazebo, ROS master, RViz, VNC, web server (port 80). Runs `startup.sh` (desktop/VNC) + `run.sh` (the game). |
| `krpc-user` | `krpc/intball-simulator:latest` | Your program. Builds your `catkin_ws` (`build-user.sh`) then launches it (`boot-ib2.sh`). |

Key mounts and env (see §3.6 for the full table). The `krpc` container exposes port `80:80` so you can
open `http://localhost` in a browser.

### `docker-compose.replay.yaml` (15 lines)
A single `krpc` service using `krpc/krpc-replay:latest`, running `startup.sh` + `play.sh`. It mounts
`./share:/tmp/share` so it can read the recorded bag and textures.

### `opencode.json` (34 lines)
**Not part of the simulator.** It is configuration for the *OpenCode* AI coding assistant (tool
permissions, MCP servers `context7` and `searxng`, agent temperature). It exists so an AI agent can
help develop inside this repo. Ignore it for competition purposes.

## 1.3 The `users/` directory in detail

```
users/
├── README.md                      # How to submit source code
├── packing.sh                     # Creates the submission .tar.gz
├── bootstrap/
│   ├── boot-ib2.sh                # Waits for sim, then roslaunches your program
│   └── tail_log.sh                # Streams your program's console log
└── catkin_ws/
    ├── .catkin_workspace
    ├── bootstrap/build-user.sh    # catkin_make for your workspace
    └── src/
        ├── krpc_user/             # ★★ YOUR package (edit here)
        ├── krpc_api_server/       # Provided API server (do not edit)
        └── krpc_ib2_msgs/         # Provided message/action/service defs (do not edit)
```

### `users/README.md`
A short submission guide: (1) modify code in the user workspace, (2) do **not** modify
`krpc_api_server` / `krpc_ib2_msgs`, package names, or launch file names, (3) run
`bash packing.sh my_submission` from `users/`, (4) log in to `https://jaxa.krpc.jp/user-auth`,
(5) upload the archive to a Slot and click **START SIMULATION**.

### `users/bootstrap/boot-ib2.sh` (34 lines)
The launcher for your program, run inside the `krpc-user` container:
1. Sources ROS Melodic + your built workspace.
2. **Waits** until the simulator sets the ROS param `/krpc/ready == true` (polls every 2 s).
3. Verifies `cheat_sheet.json` exists at `/tmp/share/logs/latest/cheat_sheet.json`.
4. Sets `ROSCONSOLE_FORMAT` so every log line is tagged with the node name (lets `run.sh` later
   extract just your node's output into `user_program_output.log`).
5. Runs `roslaunch krpc_user krpc_user.launch checkpoint_source_path:=...` (via `stdbuf` for
   line-buffered output) and streams the console through `tail_log.sh`.

### `users/bootstrap/tail_log.sh` (5 lines)
`tail -n +1 -F <log>` — keeps your program's console output visible in real time.

### `users/catkin_ws/src/krpc_user/` — the main editable package
Your mission logic lives here. Contains the Python template (`scripts/user_template.py`), the C++
template (`src/user_template_node.cpp`), the launch file (`launch/krpc_user.launch`), and build files.
**This is the only package you should edit.**

### `users/catkin_ws/src/krpc_api_server/` — provided API server (do not edit)
`krpc_api_node.py` is the "referee's front desk." It exposes the mission-event services/actions
(`/mission/get_checkpoints`, `/event/mission_start`, `/event/mission_finished`,
`/event/checkpoint_passed`) and forwards each event as JSON to the `/gs/data` topic, which the judge
consumes. It loads the course from `cheat_sheet.json` via `cheat_sheet_loader.py`.

### `users/catkin_ws/src/krpc_ib2_msgs/` — message/action/service definitions (do not edit)
Defines the competition's ROS interfaces: `CheckpointPassed.action`, `GsData.msg`,
`CheckPoints.srv`, `MissionStart.srv`, `MissionFinished.srv`. These generate the Python/C++ types your
code imports.

### `users/packing.sh` (1 line)
```bash
tar -zcvf $1.tar.gz catkin_ws/src
```
- **What it does:** tars the entire `catkin_ws/src` (your package + the two provided packages) into
  `<name>.tar.gz`.
- **How it's used:** `cd users && bash packing.sh my_submission` → `users/my_submission.tar.gz`, which
  you upload to the Web simulator. The Web simulator builds and runs exactly this archive.

---

# SECTION 2: Simulation Architecture Overview

## 2.1 How the Docker setup works

`docker-compose.yaml` creates a **bridge network** named `krpc` with two containers that talk to each
other over ROS:

```
┌───────────────────────────────  network: krpc  ───────────────────────────────┐
│                                                                              │
│  ┌──────────────────────────────┐        ┌──────────────────────────────┐    │
│  │  container: krpc             │        │  container: krpc-user        │    │
│  │  (simulator body)            │        │  (your program)              │    │
│  │  ─────────────────────────── │        │  ─────────────────────────── │    │
│  │  • ROS Master (rosmaster)    │◄───────┤  • krpc_api_node (API)       │    │
│  │  • Gazebo 9 (physics)        │  ROS   │  • krpc_user (YOUR node)     │    │
│  │  • RViz (3D view)            │  topics│                              │    │
│  │  • VNC + nginx (web, :80)    │  /     │  built by build-user.sh,     │    │
│  │  • judge, flight recorder,   │  svc/  │  launched by boot-ib2.sh     │    │
│  │    status indicator, rosbag  │  action│                              │    │
│  └──────────────────────────────┘        └──────────────────────────────┘    │
│        ▲  port 80:80 → http://localhost (browser)                            │
└──────────────────────────────────────────────────────────────────────────────┘
```

- **What runs in the `krpc` (simulator) container:** the ROS master, Gazebo with the ISS + Int-Ball2
  models, RViz, a VNC desktop + nginx web server (so you can watch in a browser), and the KRPC
  "modules" — judge (scorer), flight recorder, status indicator, and the rosbag recorder. All of this
  is orchestrated by `rootfs/run.sh`.
- **What runs in the `krpc-user` container:** *only* your code. It first builds your `catkin_ws`
  (`build-user.sh` → `catkin_make`), then launches `krpc_user.launch`, which starts **two** nodes:
  `krpc_api_node` (the provided API server) and `krpc_user` (your node).
- **ROS Master's role:** it is the "phone switchboard" of ROS. It runs inside the `krpc` container and
  both containers point at it via `ROS_MASTER_URI=http://krpc:11311`. Every node registers with it and
  uses it to discover topics/services/actions. `ROS_HOSTNAME` distinguishes the two machines
  (`krpc` vs `krpc-user`).

> **Analogy.** The ROS master is a switchboard operator. Your node and the simulator's nodes each call
> the operator to say "I'm publishing/subscribing to X," and the operator connects them. The operator
> itself doesn't carry the data — it only knows who's who.

## 2.2 Web Simulator vs. Local Simulator

| Aspect | Web Simulator | Local Simulator |
|---|---|---|
| **Where it runs** | JAXA-managed servers | Your PC (Docker) |
| **CPU architecture** | **ARM64 (aarch64)** — matches the real Int-Ball2 (Jetson TX2) | **x86_64** (typical dev PC) |
| **Access** | Browser, with an account issued by your country's POC | `http://localhost` |
| **Purpose** | Final validation / official evaluation in a common environment | Iterative development & debugging |
| **Logs** | Downloadable from the web dashboard | Written to local `share/logs/` |
| **Same image?** | Yes — both use `krpc/intball-simulator` | Yes |

**Why the architecture difference matters:** the real robot's computer is ARM64. The Web simulator runs
on ARM64 so timing, library behavior, and performance are closer to the real hardware. Your local
x86_64 simulator is faster to iterate on but may behave slightly differently. **Rule of thumb:**
develop locally, then do a final check on the Web simulator before submitting.

## 2.3 Data flow

### The full run lifecycle (driven by `rootfs/run.sh`)

```
startup.sh (VNC/desktop)  ──┐
                            ├─► run.sh:
run.sh:                     │
  1. set_timestamp          # create share/logs/<timestamp>/
  2. set_random_env         # generate_objects.py → cheat_sheet.json (random course)
  3. launch_simulation      # Gazebo + ISS model + RViz + ctl/fsm + camera republish
  4. launch_judge_module    # judge.py (scorer)
  5. launch_status_indicator# RViz overlay of progress
  6. launch_flight_recorder # records pose + events → data.json
  7. launch_record_rosbag   # rosbag record -a → simulation.bag
  8. ready                  # wait /krpc/judge_ready, then set /krpc/ready=true
  9. go                     # wait /ib2_user/ready, then publish /ib2_user/start
 10. watch_dog              # sleep until playTime (default 10 min) or finish_report
 11. kill_user / force_exit # stop your node, kill roslaunch
 12. save_logs              # copy everything into share/logs/<timestamp>/ + latest/
```

### How your program talks to the simulator

Your node (`krpc_user`) and the simulator exchange data over **ROS topics, services, and actions**
(see §5 for the full reference). The mission flow is:

```
[run.sh] publishes /ib2_user/start
        │
        ▼
[Your node]  ──service──►  /event/mission_start        (notify "I've started")
        │
        │  ──service──►  /mission/get_checkpoints      (fetch the course JSON)
        │
        │  loop over each checkpoint:
        │     ──action──►  /ctl/command  (MOVE_TO_ABSOLUTE_TARGET → fly there)
        │     ──action──►  /event/checkpoint_passed    (report "I passed CP n")
        │
        └──service──►  /event/mission_finished         (notify "I'm done")
```

- **Where sensor data comes from:** Gazebo plugins (`nav`, `thr`, `airflow`, `hill`, `issdyn`, `mag`,
  `ib2_imu`) simulate the robot's sensors. The fused self-position/velocity/attitude is published on
  `/sensor_fusion/navigation` (type `ib2_msgs/Navigation`). Camera images come from Gazebo camera
  sensors on `/camera_main/image_raw` (and left/right), with compressed variants republished by
  `image_transport`.
- **Where the "ground truth" comes from:** the judge reads the *true* robot pose from
  `/sensor_fusion/navigation` (and TF `iss_body`→`body`) to score you. **You must not try to read the
  ground-truth URDF or cheat** — `run.sh` deletes `/iss_description` from the param server after RViz
  reads it, and the judge flags `/krpc/comm/illegal` as a violation.

### How results are logged and scored

- The **judge** (`rootfs/krpc/judge_module/judge.py`) subscribes to `/gs/data` (your events, forwarded
  by `krpc_api_node`), `/sensor_fusion/navigation` (true pose), and `/krpc/comm/illegal`. It evaluates
  each checkpoint and the goal, and writes `result.json`, `finish_report`, `data.log`, and
  `localization_history.txt`.
- The **flight recorder** (`krpc_flight_recorder.py`) records pose + scene events into `data.json` and
  a plain-text `ib2.log`.
- The **rosbag recorder** captures *all* topics into `simulation.bag` (this is what `replay.sh` plays).
- After the run, `run.sh save_logs` copies everything into `share/logs/<timestamp>/` and refreshes
  `share/logs/latest/`.

## 2.4 Scoring / judging mechanism

**Where the logic lives:** `rootfs/krpc/judge_module/judge.py` (class `Referee`).

**How a checkpoint is judged** (`evaluate_checkpoint_navigation`): when you send
`/event/checkpoint_passed` for checkpoint *n*, the judge measures three things against the course
defined in `cheat_sheet.json`:

1. **Position** — 3D distance between the robot and the checkpoint `position`. Compared to
   `score_bands` (e.g. radius 0.1 m → Lv3, 0.2 m → Lv2, 0.3 m → Lv1).
2. **Camera centering** — the angle between the camera's line of sight and the direction to the AR
   marker. Compared to `attitude.centering_bands` (e.g. 10° → Lv3, 20° → Lv2, 30° → Lv1).
3. **Attitude around the line of sight (roll)** — rotation about the camera→marker axis relative to the
   target `attitude.quaternion`. Compared to `attitude.roll_bands`.

**The level is the *minimum* of the three.** If *any* condition is outside its margin (level 0), the
checkpoint **fails**. Otherwise you earn the score for that level.

> **Analogy.** It's like a free-throw contest with three judges: distance to the spot, how centered the
> ball is, and how level your wrist is. You get the *lowest* of the three scores — one bad judge sinks
> the whole attempt.

**Goal:** the goal is a specific checkpoint (by default marker **12**, the "Bay0 Closeout Panel"). It is
evaluated at **Mission Finish** (`judge_finish`): if you already passed it, it counts; otherwise the
judge re-evaluates your current pose against the goal checkpoint.

**Files generated after a run** (in `share/logs/<timestamp>/`):

| File | What it is |
|---|---|
| `cheat_sheet.json` | The randomized course for that run (checkpoints, bands, goal). |
| `simulation.bag` | Full rosbag recording (replayable with `replay.sh`). |
| `iss.urdf` + `textures/` | The ISS model + the 12 AR-marker textures used that run. |
| `judge_module_logs/result.json` | **The score** — mission timing, per-checkpoint detail, goal, events. |
| `judge_module_logs/judge.log`, `game.log`, `system.log`, `rosbag.log`, `status_indicator.log` | Judge + launcher logs. |
| `judge_module_logs/data.log` | Raw event stream (each `/gs/data` message). |
| `judge_module_logs/localization_history.txt` | Robot pose sampled every 0.1 s during the mission. |
| `flight_recorder_logs/data.json`, `ib2.log`, `flight_recorder.log` | Flight-recorder output. |
| `user_programs_logs/user_program_output.log` | **Your node's** stdout/ROS log (extracted). |
| `user_programs_logs/ros/<uuid>/*.log` | Per-node ROS logs (`krpc_api_node`, `krpc_user`, `roslaunch`). |

A real `result.json` from a sample run (`share/logs/latest/`):

```json
{
  "Illegal": false,
  "JudgeReport": {
    "mission": { "started": true, "finished": true, "duration_sec": 199.17 },
    "checkpoint_detail": [
      { "checkpoint_id": 3, "success": true,  "level": 3, "reason": "checkpoint validated" },
      { "checkpoint_id": 4, "success": true,  "level": 3, "reason": "checkpoint validated" },
      { "checkpoint_id": 5, "success": true,  "level": 3, "reason": "checkpoint validated" },
      { "checkpoint_id": 1, "success": false, "level": 0, "reason": "not attempted" }
    ],
    "goal": { "success": false, "reason": "checkpoint outside threshold" },
    "events": [ { "status": "Mission Start" }, { "status": "CheckPointPassed", "checkpoint_id": 3 }, ... ]
  }
}
```

---

# SECTION 3: Key Files Deep Dive

## 3.1 `users/catkin_ws/src/krpc_user/scripts/user_template.py` (284 lines)

**What it is:** the reference Python node. A single class `UserTemplate` that wires up all the
competition interfaces and runs a minimal "fly to each checkpoint" mission.

**Main callback / methods:**

| Method | Role |
|---|---|
| `__init__` | Creates action clients (`/ctl/command`, `/event/checkpoint_passed`), service proxies (`/event/mission_start`, `/event/mission_finished`, `/mission/get_checkpoints`), the **mandatory** subscriber on `/ib2_user/start`, the **mandatory** service on `/ib2_user/stop`, and sets the **mandatory** param `/ib2_user/ready = True`. |
| `__callback_start` | Triggered by `/ib2_user/start`. Guards against double-start, then calls `__run_user_flow`. |
| `__run_user_flow` | The mission sequence: `mission_start` → **USER EDIT AREA** → `mission_finished`. |
| `__notify_mission_start` / `__notify_mission_finished` | Thin wrappers: `wait_for_service()` then call, log, return `success`. |
| `__notify_checkpoint_passed(id)` | Sends a `CheckpointPassedGoal`, polls `wait_for_result(0.2)`, cancels on shutdown. |
| `__get_checkpoint_id(cp)` | Reads `checkpoint_id`/`checkpointid`/`id` from a checkpoint dict. |
| `__resolve_first_checkpoint_target` | Calls `/mission/get_checkpoints`, parses the JSON, returns the first checkpoint's position + attitude quaternion (falls back to origin if it fails). |
| `__move_to_checkpoint(target)` | Builds a `PoseStamped`, sends a `CtlCommandGoal` with `MOVE_TO_ABSOLUTE_TARGET`, waits for result, returns `TERMINATE_SUCCESS`. |
| `__stop_processing` | On `/ib2_user/stop`: cancels any active move action, returns `SUCCESS`. |

**How it handles the mission:**
- **Start:** waits for `/ib2_user/start`, then calls `/event/mission_start`.
- **Movement:** for each checkpoint, builds a target pose and sends `/ctl/command`
  (`MOVE_TO_ABSOLUTE_TARGET`), waiting up to 30 s (`wait_for_result(rospy.Duration(30.0))`); on timeout
  it cancels and moves to the next checkpoint.
- **Checkpoint passing:** after a successful move, calls `/event/checkpoint_passed` with the checkpoint id.
- **Finish:** after the loop, calls `/event/mission_finished` (unless ROS is shutting down).

**The "USER EDIT AREA":** the block between the `# USER EDIT AREA` and `# END USER EDIT AREA` comments
inside `__run_user_flow`. This is where you put your real route planning, camera-based aiming, retries,
and game logic. The template's default just flies to each checkpoint's stored position/attitude.

> **Why it matters:** this is your starting point. Keep the three "DO NOT REMOVE THIS LINE" items
> (`/ib2_user/start` subscriber, `/ib2_user/stop` service, `/ib2_user/ready` param) — they are mandatory
> for the harness to start and stop your program.

## 3.2 `users/catkin_ws/src/krpc_user/src/user_template_node.cpp` (175 lines)

**What it is:** the equivalent C++14 node. Same mission flow, compiled to the `user_template_node`
executable.

**Key class / functions:**
- `struct CheckpointTarget { int checkpoint_id; geometry_msgs::PoseStamped pose; }`
- `class UserTemplate` with:
  - Constructor: subscribes `/ib2_user/start`, advertises `/ib2_user/stop`, creates service clients and
    action clients, sets `/ib2_user/ready`.
  - `run()`: uses a `ros::AsyncSpinner(2)` + `waitForShutdown()`.
  - `notifyMissionStart()` / `notifyMissionFinished()`: `waitForExistence()` + `call()`.
  - `resolveFirstCheckpoint(target)`: calls `/mission/get_checkpoints` and parses the JSON **with regular
    expressions** (no JSON library) to extract the first checkpoint's id, position, and attitude
    quaternion.
  - `moveToCheckpoint(target)`: sends a `CtlCommandGoal` (`MOVE_TO_ABSOLUTE_TARGET`), waits for result.
  - `notifyCheckpointPassed(id)`: sends a `CheckpointPassedGoal`, waits for result.
  - `startCallback(msg)`: the mission sequence (start → resolve → move → pass → finish).
  - `stopProcessing(...)`: cancels active actions, returns `SUCCESS`.

**How it differs from Python:**
- Compiled (faster, no GIL) but more verbose; you must manage the `AsyncSpinner` for callbacks.
- JSON parsing is done by hand with `std::regex` (fragile if the JSON format changes) rather than
  `json.loads`.
- The C++ template only visits the **first** checkpoint in its default flow (the Python template loops
  over all of them) — you'll extend both.

> **Why it matters:** choose Python for speed of development, C++ for performance. Both are fully
> supported; you select which one runs via the `team_user_script` launch argument (§3.5).

## 3.3 `users/catkin_ws/src/krpc_api_server/scripts/krpc_api_node.py` (211 lines)

**What it is:** the provided "referee's front desk" node (class `KrpcApiNode`). It is **not** the
scorer — it just receives your events and forwards them to the judge.

**Services / actions it provides:**

| Interface | Type | Handler | What it does |
|---|---|---|---|
| `/mission/get_checkpoints` | Service (`CheckPoints`) | `get_checkpoints` | Returns the course as a JSON string in `message`. |
| `/event/mission_start` | Service (`MissionStart`) | `mission_start` | Forwards `{"status":"Mission Start"}` to `/gs/data`. |
| `/event/mission_finished` | Service (`MissionFinished`) | `mission_finished` | Forwards `{"status":"Mission Finish"}` to `/gs/data`. |
| `/event/checkpoint_passed` | Action (`CheckpointPassedAction`) | `checkpoint_passed` | Forwards `{"status":"CheckPointPassed","checkpoint_id":N}` to `/gs/data`; returns success. |

**How it loads checkpoint data:** in `__init__` it resolves the cheat-sheet path from the ROS param
`~checkpoint_source_path` (set by the launch file), falling back to `../cheat_sheet.json` next to the
script. It loads it once with `CheatSheetLoader.from_path(...)` (see §3.4).

**How it "judges" checkpoint passing:** it **doesn't** judge. It publishes the event to `/gs/data` and
returns `success=True, message="accepted"`. The *actual* pass/fail decision is made by the judge
(`judge.py`) using the true pose. (Only a ROS-shutdown error is reported as aborted.)

> **Why it matters:** understand the split — your node *reports* events, `krpc_api_node` *relays* them,
> and `judge.py` *scores* them. You never talk to the judge directly.

## 3.4 `users/catkin_ws/src/krpc_api_server/scripts/cheat_sheet_loader.py` (71 lines)

**What it is:** a small loader (class `CheatSheetLoader`) that reads `cheat_sheet.json` and normalizes
it. It accepts several key spellings (`Checkpoints`/`checkpoints`, `checkpointid`/`checkpoint_id`/`id`,
`Goal`/`goal`) so the API is robust to format variations. `checkpoint_payload()` builds the JSON that
`/mission/get_checkpoints` returns: `checkpoint_count`, a `checkpoints` list (id, position,
marker_pose, score_bands, attitude), and `goalpoint_id`.

## 3.5 `users/catkin_ws/src/krpc_ib2_msgs/` — all definitions

| File | Kind | Fields | Purpose |
|---|---|---|---|
| `action/CheckpointPassed.action` | Action | Goal: `int32 checkpoint_id` · Result: `bool success`, `string message` · Feedback: `string status` | Report that you passed a checkpoint. |
| `msg/GsData.msg` | Message | `string data` | (Internal) JSON event payload carried on `/gs/data`. |
| `srv/CheckPoints.srv` | Service | Req: *(empty)* · Res: `bool success`, `string message` | Fetch the course (JSON in `message`). |
| `srv/MissionStart.srv` | Service | Req: *(empty)* · Res: `bool success`, `string message` | Notify mission start. |
| `srv/MissionFinished.srv` | Service | Req: *(empty)* · Res: `bool success`, `string message` | Notify mission finish. |

> The `.action` file generates `CheckpointPassedAction` (for the client) plus `CheckpointPassedGoal`,
> `CheckpointPassedResult`, `CheckpointPassedFeedback` and the protocol types. In Python you import
> `CheckpointPassedAction` and `CheckpointPassedGoal`.

## 3.6 `users/catkin_ws/src/krpc_user/launch/krpc_user.launch` (34 lines)

**What nodes it launches:**
1. `krpc_api_node` (from `krpc_api_server`) — the provided API server, with param
   `checkpoint_source_path` (default `/tmp/share/logs/latest/cheat_sheet.json`).
2. `krpc_user` (from `krpc_user`) — **your** node. Its executable is the launch argument
   `team_user_script` (default `user_template.py`; set it to `user_template_node` to run the C++ build).

**Parameters set** (in the `platform_launch` group — these tell the platform which subsystems to start):

| Param | Default | Meaning |
|---|---|---|
| `sensor_fusion` | `true` | Start the sensor-fusion node (provides `/sensor_fusion/navigation`). |
| `slam_wrapper` | `true` | Start the SLAM wrapper. |
| `ctl_only` | `true` | Start the control node (serves `/ctl/command`). |
| `fsm` | `true` | Start the finite-state-machine node. |
| `camera_left` / `camera_right` | `false` | Side cameras (set `slam_wrapper=false` to enable). |

## 3.7 `docker-compose.yaml` and `docker-compose.replay.yaml`

### `docker-compose.yaml`

| Item | `krpc` (simulator) | `krpc-user` (your program) |
|---|---|---|
| **Image** | `krpc/intball-simulator:latest` | `krpc/intball-simulator:latest` |
| **Ports** | `80:80` (browser) | — |
| **Volumes** | `/dev/shm`; `./share:/tmp/share`; `./rviz:/…/description/rviz`; `./rootfs/startup.sh`; `./rootfs/run.sh`; `./rootfs/krpc:/krpc`; AR-marker textures | `/dev/shm`; `./share:/tmp/share`; `./users:/users` |
| **Env** | `USER=ubuntu`, `SIM_MODE`, `ROUND`, `SIM_PATTERN`, `RESOLUTION=1680x900`, `playTime` (default `10m`), `ROS_MASTER_URI=http://krpc:11311`, `ROS_HOSTNAME=krpc`, `LEVELS` (default `normal`), `ENABLE_GRAVITY_MODULE`, `ENABLE_TOPIC_MONITOR`, `ENABLE_DEBUG_MODE`, `VNC_PASSWORD`, `VNC_ARGS` | `DEBUG=debug`, `SIM_MODE`, `ROUND`, `SIM_PATTERN`, `ROS_MASTER_URI=http://krpc:11311`, `ROS_HOSTNAME=krpc-user`, `ROS_LOG_DIR=/tmp/share/logs/latest/user_programs_logs/ros` |
| **Working dir** | — | `/users/catkin_ws/bootstrap` |
| **Command** | `bash -c "/startup.sh &> /tmp/vnc.log & /run.sh"` | `bash -lc "./build-user.sh && ../../bootstrap/boot-ib2.sh"` |

Notable env knobs you can set before `simulation.sh`:
- `LEVELS=easy|normal|hard|veryhard` — controls how many checkpoints are generated.
- `playTime=10m` — the watchdog time limit (format `<N>m`).
- `SIM_MODE=regular|beginners` — `beginners` disables the attitude (centering/roll) scoring.
- `ENABLE_DEBUG_MODE=true` — shows your robot's self-position overlay in RViz.

### `docker-compose.replay.yaml`
Single `krpc` service, image `krpc/krpc-replay:latest`, port `80:80`, volume `./share:/tmp/share`, env
`RESOLUTION`, `PLAY_RATE`, `START_TIME`, command `bash -c "/startup.sh &> /tmp/vnc.log & /play.sh"`.

## 3.8 `build.sh` and `simulation.sh` — exact behavior

**`build.sh`**
- **Commands:** `docker build … -t krpc/intball-simulator:latest -f docker/Dockerfile.krpc-base .` then
  `docker build … -t krpc/krpc-replay:latest -f docker/Dockerfile.krpc-replay .`
- **Options:** `--no-cache`, `--arm64`, `--platform=<p>`.
- **Prerequisites:** Docker installed and running; network access (the base image pulls ROS Melodic,
  Gazebo 9, and clones the upstream Int-Ball2 simulator from GitHub, and compiles OpenCV 3.3.1 from
  source — the first build is long, ~30–60 min).
- **Expected output:** two images listed by `docker images` (`krpc/intball-simulator:latest`,
  `krpc/krpc-replay:latest`).

**`simulation.sh`**
- **Commands:** `docker compose -f docker-compose.yaml up --abort-on-container-exit --exit-code-from krpc`
  then `docker compose -f docker-compose.yaml down`.
- **Options:** none (configure via env vars, see §3.7).
- **Prerequisites:** `build.sh` already run; port 80 free.
- **Expected output:** console logs from both containers; open `http://localhost` to watch; results in
  `share/logs/`.

---

# SECTION 4: How to Run the Simulation (Step-by-Step)

## 4.1 Prerequisites

| Item | Requirement |
|---|---|
| **OS** | Ubuntu 20.04+ (24.04 recommended). On Windows 11: **Ubuntu on WSL2** (mandatory — native Windows is not supported). |
| **CPU** | ≥ 4 cores (6+ recommended). |
| **RAM** | ≥ 8 GB (16 GB recommended; WSL2 users: check the WSL memory cap). |
| **Storage** | ≥ 20 GB free (40 GB recommended, SSD). |
| **Software** | **Docker** (required for the local simulator), **Git** + **VSCode** (recommended). |

Install Docker (Ubuntu):
```bash
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo usermod -aG docker $USER   # then restart the terminal
docker run hello-world          # verify
```

## 4.2 Building the simulator

```bash
cd ~/krpc/krpc7th-simulator
bash ./build.sh                 # add --arm64 on Apple-silicon / ARM machines
docker images                   # confirm krpc/intball-simulator:latest exists
```
Internally this builds the base image (ROS Melodic + Gazebo 9 + the Int-Ball2 platform + OpenCV 3.3.1
with ArUco) and the replay image. **First build is slow** (it compiles OpenCV from source).

## 4.3 Running the simulator

```bash
cd ~/krpc/krpc7th-simulator
bash ./simulation.sh
```
- **Access:** open **`http://localhost`** in a browser to watch the 3D simulation (RViz over VNC).
- **What happens:** your program is built and started automatically; the course is randomized; the
  robot flies; the judge scores.
- **Stop:** press **`Ctrl+C`** in the terminal, or let it finish (it stops automatically on
  `mission_finished` or when `playTime` elapses).
- **Results:** `share/logs/latest/` (and `share/logs/<timestamp>/`).

## 4.4 Developing your program

1. **Edit** code in `users/catkin_ws/src/krpc_user/` (Python: `scripts/user_template.py`; C++:
   `src/user_template_node.cpp`). Do **not** change the package name, `CMakeLists.txt`, `package.xml`,
   or launch file names.
2. **Build:** you don't build manually — `simulation.sh` runs `build-user.sh` (`catkin_make`) inside the
   container on every start. Just re-run `simulation.sh` to pick up changes.
3. **Package for submission:**
   ```bash
   cd users
   bash packing.sh my_submission      # → users/my_submission.tar.gz
   ```
4. **Test on the Web simulator:** log in at `https://jaxa.krpc.jp/user-auth`, go to **SIMULATION**,
   upload the `.tar.gz` to a Slot, choose level/version, click **START SIMULATION**, then **VIEW
   RESULT** / **DOWNLOAD LOG FILES**.

## 4.5 Replaying results

```bash
cd ~/krpc/krpc7th-simulator
bash ./replay.sh                 # 1× speed from t=0
bash ./replay.sh 2 30            # 2× speed starting at t=30 s
```
- **What's available:** the full 3D replay of the run (robot, ISS, markers) from `simulation.bag`, plus
  all recorded topics.
- **Analyze logs:** open `share/logs/latest/judge_module_logs/result.json` (the score),
  `user_programs_logs/user_program_output.log` (your node's log), and `judge_module_logs/*.log`
  (judge/launcher detail). Use `rosbag info`/`rosbag play` on `simulation.bag` for raw topic data.

---

# SECTION 5: API & Interface Reference

> Direction: **In** = your node subscribes/calls it; **Out** = your node provides it.
> "Mandatory" items are already in the template and must not be removed.

## 5.1 Topics

| Topic | Message type | Direction | Purpose / when to use |
|---|---|---|---|
| `/ib2_user/start` | `platform_msgs/UserLogic` | In (subscribe) | **Mandatory.** The "GO" signal. Your mission starts when this arrives. |
| `/sensor_fusion/navigation` | `ib2_msgs/Navigation` | In (subscribe) | Self position/attitude/velocity/acceleration. Use for feedback control and to know where you are. |
| `/camera_main/image_raw` | `sensor_msgs/Image` | In (subscribe) | Main camera image. Use for ArUco marker detection / visual aiming. |
| `/camera_main/camera_info` | `sensor_msgs/CameraInfo` | In (subscribe) | Camera intrinsics (`K`) + distortion (`D`). Needed for pose estimation. |
| `/ctl/status` | `ib2_msgs/CtlStatus` | In (subscribe) | Current control state (pose, twist, `CtlStatusType`). Poll to detect move completion. |
| `/gs/data` | `std_msgs/String` | (internal) | JSON event bus from `krpc_api_node` → judge. You don't publish this directly. |
| `/krpc/judge/checkpoint_result`, `/krpc/judge/goal_result` | `std_msgs/String` | (internal) | Judge → status-indicator overlay. Not for your program. |

## 5.2 Services

| Service | Type | Direction | Request → Response | When to use |
|---|---|---|---|---|
| `/mission/get_checkpoints` | `krpc_ib2_msgs/CheckPoints` | In (call) | *(empty)* → `bool success`, `string message` (JSON course) | At mission start, to fetch the randomized course. |
| `/event/mission_start` | `krpc_ib2_msgs/MissionStart` | In (call) | *(empty)* → `bool success`, `string message` | Immediately after `/ib2_user/start`. |
| `/event/mission_finished` | `krpc_ib2_msgs/MissionFinished` | In (call) | *(empty)* → `bool success`, `string message` | At the end of your mission. |
| `/ib2_user/stop` | `platform_msgs/StopProcessingUserNode` | **Out** (provide) | *(empty)* → `uint8 result` (`SUCCESS=0`, `ERROR=255`) | **Mandatory.** Cancel active actions and return `SUCCESS`. |

## 5.3 Actions

| Action | Type | Direction | Goal / Result / Feedback | When to use |
|---|---|---|---|---|
| `/ctl/command` | `ib2_msgs/CtlCommandAction` | In (send goal) | Goal: `PoseStamped target`, `CtlStatusType type` · Result: `time stamp`, `uint8 type` (`TERMINATE_SUCCESS=0`, `ABORTED=1`, `TIME_OUT=2`, `INVALID_NAV=3`, `INVALID_CMD=4`) · Feedback: `duration time_to_go`, `Pose pose_to_go` | Fly to a target. Use `MOVE_TO_ABSOLUTE_TARGET` (frame `iss_body`), `MOVE_TO_RELATIVE_TARGET` (frame `body`), or `STOP_MOVING`. |
| `/event/checkpoint_passed` | `krpc_ib2_msgs/CheckpointPassedAction` | In (send goal) | Goal: `int32 checkpoint_id` · Result: `bool success`, `string message` · Feedback: `string status` | After you've reached and stabilized at a checkpoint, to report the pass. |

**`CtlStatusType` values you'll use:** `MOVE_TO_ABSOLUTE_TARGET=40`, `MOVE_TO_RELATIVE_TARGET=30`,
`STOP_MOVING=20` (others like `STAND_BY`, `KEEP_POSE`, `DOCK*` are internal states).

## 5.4 Parameters

| Parameter | Type | Direction | Purpose |
|---|---|---|---|
| `/ib2_user/ready` | `bool` | **Out** (set) | **Mandatory.** Set `True` when your node is initialized. `/ib2_user/start` is not published until this is `true`. |
| `/krpc/ready` | `bool` | (set by `run.sh`) | Simulator-ready flag; `boot-ib2.sh` waits on it. |
| `/krpc/judge_ready` | `bool` | (set by judge) | Judge-ready flag; `run.sh` waits on it before setting `/krpc/ready`. |
| `~checkpoint_source_path` | `string` | (launch arg) | Path to `cheat_sheet.json` for `krpc_api_node`. |

> **Rule for every Action:** always design a timeout, a `cancel_goal()`, and a behavior for ROS
> shutdown. Never block forever on `wait_for_result`.

---

# SECTION 6: Simulation Environment Details

## 6.1 The simulated world (Kibo module)

- **World file:** `docker/ros_overlay/src/platform_sim/simulation/ib2_gazebo/worlds/empty.world`.
  Gravity is **off** (`<gravity>0 0 0</gravity>`) — true microgravity. It loads Gazebo plugins
  (`airflow`, `nav`, `hill`, `mag`, `custom_pose_spawn_plugin`, `ib2_route_display_plugin`) and a sun +
  three JPM point lights.
- **ISS model:** `docker/ros_overlay/src/platform_sim/description/urdf/iss.urdf`. It contains the JPM
  (Japanese Pressurized Module) mesh, a `dock_body` (docking station) link, and **12 AR marker visuals**
  (`ARmarker1`…`ARmarker12`) placed on the module walls/panels.
- **AR marker placement** (from `rootfs/krpc/random_env_generator/files/input/area_config.json`, in the
  Dock-Station coordinate frame):

| ID | Name (comment) | Position [x,y,z] (m) |
|---|---|---|
| 1 | SAIBO | [1.451, −1.001, 0.6806] |
| 2 | RYUTAI | [2.973, −1.001, 0.7176] |
| 3 | MSPR_1 | [3.313, −1.001, 0.5226] |
| 4 | MSPR_2 | [4.181, −1.001, 0.5226] |
| 5 | RMS_Console | [5.754, −1.001, 0.8376] |
| 6 | EPS1 | [1.895, −0.282, −0.2084] |
| 7 | MELFI1 | [1.451, −1.001, 0.3716] |
| 8 | EPS2_2 | [4.779, 0.112, 1.8836] |
| 9 | EPS2_1 | [2.066, 1.046, −0.0144] |
| 10 | ECLSS_TCS2 | [5.851, 0.01, 1.8836] |
| 11 | Airlock | [7.195, 0.363, 1.2296] |
| 12 | **Bay0_Closeout_Panel (GOAL)** | [0.0, −0.059, −0.1024] |

- **Safe (flyable) area:** `min [0.2, −0.8, 0.0]`, `max [7.0, 0.85, 1.68]` (m, DS frame). Checkpoints
  are always placed inside this box.
- **Docking Station:** `ds_pose` in `sim.yaml` = `[1.14009, 0.0, −0.79263]` (m, JPM reference frame).
- **Goal Point:** checkpoint **12** (Bay0 Closeout Panel) at `[0.0, −0.059, −0.1024]`.

> **How the course is randomized** (`generate_objects.py`): for each selected marker, a checkpoint is
> placed **0.5 m** in front of it, with random yaw (±20°), pitch (±15°), and roll (±180°), clamped to
> the safe area. The target attitude is "face the marker" plus a random roll. Difficulty presets set the
> checkpoint count: `easy=6`, `normal=8`, `hard=10`, `veryhard=12` (the goal marker is always included).

## 6.2 The robot model (Int-Ball2)

- **Model:** the `ib2` Gazebo model (from the upstream Int-Ball2 platform simulator), spawned by
  `custom_pose_spawn_plugin` at the pose in `sim.yaml` (`ib2:` block).
- **Mass properties** (`sim.yaml`): mass **3.216 kg**, center of gravity ≈ origin, radius **0.1 m**.
- **Sensors simulated:**
  - **Main camera** (front, optical axis = +X), mounted at `x=0.14 m` from center (`CAM_F_X=0.14`);
    left/right cameras available.
  - **IMU** (`ib2_imu` plugin) with configurable noise.
  - **Navigation** (`nav` plugin) — fused position/velocity/attitude with realistic sensor error.
- **Thrusters / movement dynamics** (`thr_parameter`): **8 fans**, each with nominal force **0.06 N**,
  force noise stddev **0.012 N**, arranged in a cube; a 2nd-order filter (τ ≈ 0.01 s) smooths the
  commands. Control frequency ~200 Hz with 0.1 s delay.

## 6.3 The disturbance model ("microgravity module")

The "microgravity" feel comes from several Gazebo plugins that continuously perturb the robot:

| Plugin | What it does | Key params (`sim.yaml`) |
|---|---|---|
| `airflow` | Applies a spatially-varying **airflow/wind** force field (from a PCD cloud) + drag. | `force_stddev=0.005`, `torque_stddev=0.005`, `kappa=0.06` |
| `hill` | Simulates the **ISS orbital rotation** (the module slowly rotates). | `w=0.0667 deg/s` (ISS orbit rate) |
| `issdyn` | Slow **ISS attitude drift** (sine + slope) so the whole frame moves. | `slope=0.001 deg/s`, `gain=0.1 deg/s`, `freq=0.02 Hz` |
| `mag` | **Magnetic** docking force near the dock (far/prox coefficients). | `threshold=0.058 m`, `stddev=0.05` |
| `nav` | Adds **sensor error** to the navigation output (so your position estimate is imperfect). | pos stddev ≈ 0.85 mm, vel ≈ 1.8 mm/s, att ≈ 0.01° |
| `ib2_imu` | Adds **IMU noise** (velocity/acceleration). | mean/stddev per axis |

> **Why it matters:** these disturbances are why a "fly to point" command doesn't land you perfectly —
> you need feedback control and a settle/wait time before reporting a checkpoint. The `nav` error means
> your *measured* position is slightly off from the *true* position the judge uses.

---

# SECTION 7: Development Workflow & Best Practices

## 7.1 Recommended development workflow

1. **Self-learning phase** — read `docs/ProgrammingManual.md` (Ch. 1–2), learn ROS 1 basics (nodes,
   topics, services, actions, params, launch files), and get comfortable with Python + `rospy`.
2. **Local development** — run `build.sh` once, then iterate: edit `krpc_user` → `simulation.sh` →
   watch in browser → read `share/logs/latest/`. Use `ENABLE_DEBUG_MODE=true` to see your robot's
   position overlay.
3. **Web simulator testing** — package with `packing.sh`, upload, and run on the Web simulator (ARM64)
   for final validation. This is the environment closest to the real hardware and to official scoring.
4. **Submission** — upload your final `.tar.gz` to the competition slot and run the official round.

## 7.2 Best practices for robust code (from the manual, Ch. 5)

- **Error handling:** null checks, check return values, exception handling, timeouts, handle
  disconnection / missing sensor data / abnormal values, limit retries, and always have a safe
  termination path.
- **Logging:** log start/end, version, plan, each API call + result, target & current pose, sensor
  values, error codes, retry counts, and processing time. Use levels: `ERROR` (can't continue), `WARN`
  (attention), `INFO` (progress), `DEBUG` (dev detail).
- **Movement control:** confirm the target before moving, **always check the move result**, limit
  retries, respect the flyable area, and **wait/settle after positioning** before reporting a
  checkpoint. To stop precisely, send `STOP_MOVING` or `cancel_goal()` and confirm the state reached
  `KEEP_POSE`.
- **Camera / ArUco detection:** use OpenCV 3.3.1's ArUco module with a **4×4 dictionary**, marker IDs
  **1–12**, marker side length **0.02 m (2 cm)**, and the camera matrix/distortion from
  `/camera_main/camera_info`. Convert `sensor_msgs/Image` with `cv_bridge`. Remember the camera is
  offset `x=0.14 m` forward.

## 7.3 Common pitfalls

| Symptom | Likely cause | Fix |
|---|---|---|
| **"FAILED" status** | The simulator itself failed to start (a `failed` event is written to `failed_to_exec_simulation`). | Check `system.log`/`game.log`; ensure `build.sh` succeeded and resources are sufficient. |
| **Missed checkpoints** | Move timed out (robot didn't reach the point in time), or you reported the pass while still outside the position/centering/roll bands. | Add settle time, use feedback control, verify you're inside all three bands before calling `checkpoint_passed`. |
| **Low score** | You passed checkpoints but at a low level (e.g. position OK but camera not centered / wrong roll), or the mission was slow (low time bonus). | Aim the camera at the marker center and match the target attitude; optimize route order and speed. |
| **`checkpoint_passed` returns failure** | You were outside a band when you reported. | Re-check pose/attitude; don't spam the same request. |
| **Program hangs** | Blocking forever on `wait_for_result` / `wait_for_service`. | Add finite timeouts and multiple exit conditions. |
| **Illegal flag set** | Attempted to read ground truth (e.g. the ISS URDF) or otherwise cheat. | Don't — it's detected and disqualifying. |

---

# SECTION 8: Key Findings & Recommendations for Supervisor

**Executive summary**

- **What it is:** A Dockerized, ROS 1 Melodic + Gazebo 9 simulation of JAXA's Int-Ball2 free-flying
  robot inside the ISS Kibo module. It randomizes a course of ArUco-marker checkpoints, lets a
  participant's ROS node fly the robot, and scores the run on position, camera-centering, and attitude.
- **Technical stack:** ROS 1 (Melodic), Gazebo 9, Python 3.6.9 / C++14 (GCC 7.5), OpenCV 3.3.1 + ArUco,
  Docker (Compose), RViz, rosbag. Runs on x86_64 locally and ARM64 on the official Web simulator.
- **Development workflow:** edit `users/catkin_ws/src/krpc_user/` → `build.sh` (once) → `simulation.sh`
  (auto-builds + runs) → inspect `share/logs/latest/` → `packing.sh` → upload to Web simulator →
  submit.
- **Learning curve / time:** A developer with basic Python needs roughly **1–2 weeks** to (a) set up the
  environment, (b) learn ROS 1 topics/services/actions, and (c) get a working checkpoint-passing
  program. Reaching a *competitive* score (high levels on all checkpoints + fast mission) is a further
  several weeks of control-tuning and visual-aiming work.
- **Key risks / challenges:**
  1. **Environment setup** — Docker + (on Windows) WSL2; the first `build.sh` is long (compiles OpenCV).
  2. **ROS learning curve** — actions, services, and the mandatory start/stop/ready handshake are new
     concepts.
  3. **Scoring precision** — the attitude (centering + roll) conditions are the hard part; a naive
     "fly to point" program passes position but scores low or fails.
  4. **Architecture mismatch** — local x86_64 vs. real/Web ARM64 can change timing/performance.
  5. **Disturbances** — airflow/orbital drift mean open-loop commands won't land precisely.
- **Recommendations to get started:**
  1. **Learn ROS 1 first** (nodes, topics, services, actions, params, launch) — it's the biggest
     prerequisite.
  2. **Get the template running** end-to-end locally before changing anything (verify you can watch it
     in the browser and see a `result.json`).
  3. **Understand the scoring** (§2.4) — position, centering, roll, min-of-three. This drives all
     design decisions.
  4. **Prioritize a robust "reach + settle + report" loop** with timeouts and retries, then add
     camera-based aiming to raise the level.
  5. **Validate on the Web simulator** (ARM64) before the official round.

---

# SECTION 9: Appendix

## 9.1 Glossary

| Term | Meaning |
|---|---|
| **ROS** | Robot Operating System — a middleware for robots. Provides nodes, topics, services, actions, params. |
| **ROS 1 / Melodic** | The first ROS generation; "Melodic" is the release for Ubuntu 18.04 (used here). |
| **Node** | A single executable process that does one job (e.g. your program, the judge). |
| **Topic** | A named, continuous data stream (publish/subscribe). |
| **Service** | A request/response call (one-shot). |
| **Action** | A long-running task with goal/result/feedback and cancellation (e.g. "move to X"). |
| **Parameter** | A named config value stored in the ROS param server. |
| **ROS Master** | The registry/switchboard nodes use to find each other. |
| **catkin** | ROS 1's build system (like CMake + package manager). `catkin_make` builds a workspace. |
| **catkin_ws** | A ROS workspace: `src/` (packages), `build/`, `devel/`. |
| **Gazebo** | A 3D physics simulator for robots; runs the world and sensors. |
| **URDF** | Unified Robot Description Format — an XML description of a robot/world's links, joints, meshes. |
| **ArUco** | A type of fiducial (QR-like) marker for pose estimation; detected with OpenCV. |
| **rviz / RViz** | ROS's 3D visualization tool (shows the robot, markers, camera, TF). |
| **TF** | ROS's coordinate-transform system (relates frames like `iss_body`, `body`, `world`). |
| **rosbag** | Records/plays back ROS topics to a `.bag` file. |
| **Docker** | Container runtime; packages the whole simulator into a portable image. |
| **Docker Compose** | Defines multi-container apps (here: simulator + user program) in YAML. |
| **VNC** | Virtual networking computing — remote desktop; how RViz is shown in the browser. |
| **WSL2** | Windows Subsystem for Linux 2 — runs Ubuntu on Windows. |
| **Int-Ball2** | JAXA's ~3.2 kg spherical free-flying robot for the ISS. |
| **Kibo** | The Japanese Experiment Module on the ISS. |
| **JPM** | Japanese Pressurized Module (part of Kibo). |
| **cheat_sheet.json** | The per-run randomized course definition (checkpoints, bands, goal). |
| **Ground truth** | The simulator's true robot state (used by the judge; off-limits to your program). |

## 9.2 Annotated file tree

```
krpc7th-simulator/
├── build.sh                     # Build simulator + replay Docker images
├── simulation.sh                # Run one simulation (compose up/down)
├── replay.sh                    # Replay the latest rosbag
├── docker-compose.yaml          # krpc (sim) + krpc-user (your program) services
├── docker-compose.replay.yaml   # Replay service
├── opencode.json                # AI-assistant config (not part of sim)
├── README.md                    # Quick start
├── .gitignore                   # Ignores build/, devel/, share/, caches
├── .vscode/                     # VSCode C/C++ runner + lldb debug config
│
├── docker/                      # ★ Build definitions (do not edit)
│   ├── Dockerfile.krpc-base     #   Full simulator image (ROS+Gazebo+OpenCV+platform)
│   ├── Dockerfile.krpc-replay   #   Thin replay image (FROM base)
│   ├── ros_overlay/src/
│   │   ├── ib2_msgs/            #   Robot message/service/action defs (Navigation, CtlCommand, …)
│   │   ├── krpc_msgs/           #   QrReport.msg
│   │   └── platform_sim/        #   ISS URDF, AR-marker meshes/textures, Gazebo world, sim.yaml
│   └── ros_python3_overlay/src/ #   Python3 ports (tf2_py, tf2_ros, vision_opencv/cv_bridge)
│
├── rootfs/                      # ★ Simulator runtime (do not edit)
│   ├── startup.sh               #   VNC/desktop/nginx setup
│   ├── run.sh                   #   THE orchestrator: course→Gazebo→judge→record→score→save
│   ├── play.sh                  #   Replay: restore textures/URDF + rosbag play
│   └── krpc/
│       ├── judge_module/judge.py            # Scorer (Referee)
│       ├── flight_recorder_module/krpc_flight_recorder.py  # Records pose+events
│       ├── status_indicator_module/krpc_status_indicator.py # RViz progress overlay
│       └── random_env_generator/            # generate_objects.py + area_config.json (course)
│
├── rviz/urdf.rviz               # ★ RViz layout (RobotModel, Image, Marker, TF, jsk overlays)
│
├── share/logs/                  # ★ Output (generated per run)
│   ├── latest/                  #   Snapshot of the most recent run
│   └── <timestamp>/             #   Full run: cheat_sheet.json, simulation.bag, iss.urdf,
│                                #   textures/, judge_module_logs/, flight_recorder_logs/,
│                                #   user_programs_logs/
│
├── users/                       # ★★ YOUR working area
│   ├── README.md                #   Submission guide
│   ├── packing.sh               #   tar czf <name>.tar.gz catkin_ws/src
│   ├── bootstrap/
│   │   ├── boot-ib2.sh          #   Wait for sim, roslaunch your program
│   │   └── tail_log.sh          #   Stream your console log
│   └── catkin_ws/
│       ├── bootstrap/build-user.sh   # catkin_make
│       └── src/
│           ├── krpc_user/       # ★★ EDIT HERE
│           │   ├── scripts/user_template.py      # Python template
│           │   ├── src/user_template_node.cpp    # C++ template
│           │   ├── launch/krpc_user.launch       # Launches api node + your node
│           │   ├── CMakeLists.txt / package.xml
│           ├── krpc_api_server/ # Provided API server (do not edit)
│           │   ├── scripts/krpc_api_node.py
│           │   ├── scripts/cheat_sheet_loader.py
│           │   └── launch/krpc_api.launch
│           └── krpc_ib2_msgs/   # Provided interface defs (do not edit)
│               ├── action/CheckpointPassed.action
│               ├── msg/GsData.msg
│               └── srv/{CheckPoints,MissionStart,MissionFinished}.srv
│
└── docs/
    ├── ProgrammingManual.md     # ★ English manual (authoritative)
    ├── ProgrammingManual_ja.md  # Japanese manual
    └── images/                  # UI screenshots
```

## 9.3 All scripts and their purposes

| Script | Location | Purpose |
|---|---|---|
| `build.sh` | root | Build `krpc/intball-simulator` + `krpc/krpc-replay` images. |
| `simulation.sh` | root | Run one full simulation (compose up/down). |
| `replay.sh` | root | Replay the latest `simulation.bag`. |
| `startup.sh` | `rootfs/` | Set up VNC/desktop/nginx inside the container. |
| `run.sh` | `rootfs/` | Orchestrate the whole run (course, Gazebo, judge, record, score, save). |
| `play.sh` | `rootfs/` | Replay mode: restore assets + `rosbag play`. |
| `boot-ib2.sh` | `users/bootstrap/` | Wait for sim ready, then roslaunch your program. |
| `tail_log.sh` | `users/bootstrap/` | Stream your program's console log. |
| `build-user.sh` | `users/catkin_ws/bootstrap/` | `catkin_make` your workspace. |
| `packing.sh` | `users/` | Create the submission `.tar.gz`. |
| `krpc_api_node.py` | `users/…/krpc_api_server/` | API server (relays your events to the judge). |
| `cheat_sheet_loader.py` | `users/…/krpc_api_server/` | Load/normalize `cheat_sheet.json`. |
| `user_template.py` | `users/…/krpc_user/` | **Your** Python node (template). |
| `user_template_node.cpp` | `users/…/krpc_user/` | **Your** C++ node (template). |
| `judge.py` | `rootfs/krpc/judge_module/` | The scorer. |
| `krpc_flight_recorder.py` | `rootfs/krpc/flight_recorder_module/` | Records pose + events. |
| `krpc_status_indicator.py` | `rootfs/krpc/status_indicator_module/` | RViz progress overlay. |
| `generate_objects.py` | `rootfs/krpc/random_env_generator/` | Randomize the course → `cheat_sheet.json`. |

## 9.4 All launch files and what they launch

| Launch file | Location | Launches |
|---|---|---|
| `krpc_user.launch` | `users/…/krpc_user/launch/` | `krpc_api_node` + your `krpc_user` node; sets `platform_launch` params (sensor_fusion, slam_wrapper, ctl_only, fsm, cameras). |
| `krpc_api.launch` | `users/…/krpc_api_server/launch/` | `krpc_api_node` only (with `checkpoint_source_path`). |
| `sim.launch` | (upstream `ib2_gazebo`) | Gazebo + ISS/IB2 models (called by `run.sh`). |
| `description.launch` | (upstream `description`) | ISS state publisher / RViz inputs (called by `run.sh`). |
| `bringup.launch` (ctl_only, fsm) | (upstream) | Control + FSM nodes (called by `run.sh`). |

## 9.5 TODO / FIXME / IMPORTANT comments found

- **In the core KRPC code (`users/`, `rootfs/`, `docker/ros_overlay`):** **none.** The competition
  code is clean of TODO/FIXME markers.
- **In vendored third-party ROS packages** (not part of the competition, baked into the image):
  - `docker/ros_python3_overlay/src/tf2_py/src/tf2_py.cpp` — two `//TODO: Create a converter…`
  - `docker/ros_python3_overlay/src/vision_opencv/cv_bridge/test/utest2.cpp` — `// TODO for Groovy…`
  - `docker/ros_python3_overlay/src/vision_opencv/opencv_tests/nodes/rosfacedetect.py` — `# TODO add this file…`
- **Notable "IMPORTANT"-style guidance** lives in comments, not as `IMPORTANT` tags: the three
  `# DO NOT REMOVE THIS LINE` markers in `user_template.py` / `user_template_node.cpp` (the mandatory
  `/ib2_user/start`, `/ib2_user/stop`, `/ib2_user/ready` interfaces), and the anti-cheat comment in
  `run.sh` ("delete urdf from rosparam to prevent hacking after RViz has read it").

---

*End of report.*
