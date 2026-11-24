# Kibo-RPC Experiment Log

Measured results from Gazebo and Web simulator runs. These numbers replace the estimates from ctl.yaml wherever they differ. Newest first.

**Method rule (agreed 2026-11-21):** one change per version, and each version gets a baseline run before the next change. `run.sh` has no seed option, so every Gazebo run draws a new random course. Versions are compared by (a) replaying the exact logged course offline and (b) several runs per version.

## Scoreboard (Web simulator, sim v1.2 rules)

| Run | Program | Level | Scored (all Lv3) | Goal | Judge mission time |
|---|---|---|---|---|---|
| W0 | KongVong_v01 (another team's program) | Normal | **8 / 8** | **yes** | 262.3 s |
| W1-N | v1.4.1 | Normal | 5 / 8 | no (aborted on the goal leg) | 217.9 s |
| W1-H | v1.4.1 | Hard | 7 / 10 | no (CPs 7, 9, 11 missing) | 245.7 s |
| W1-V | v1.4.1 | Very hard | 8 / 12 | no (CPs 1, 5, 10, 11 missing) | 262.7 s |
| W2-N | v1.5 | Normal | **8 / 8** | **yes** | 256.0 s |
| W2-H | v1.5 | Hard | 9 / 10 | no (CP 9 planned skip) | 283.5 s |
| W2-V | v1.5 | Very hard | 8 / 12 | no (CPs 5, 8, 10, 11 planned skips) | 250.3 s |
| W6-N | v2.0 | Normal | **8 / 8** | **yes** | 279.1 s |
| W6-H | v2.0 | Hard | 8 / 10 | no | 278.6 s |
| W6-V | v2.0 | Very hard | **12 / 12** | **yes** | 281.2 s |
| W7-N | v2.1 | Normal | **8 / 8** | **yes** | 201.4 s (146.2 pt) |
| W7-H | v2.1 | Hard | **10 / 10** | **yes** | 198.7 s (173.9 pt) |
| W7-V | v2.1 | Very hard | **12 / 12** | **yes** | 241.2 s (179.4 pt) |

Dashboard "Final Score" values: not yet collected (needed to fit points per level, Pt_goal and the time multiplier).

## W7 — Web simulator, v2.1 on Normal / Hard / Very hard, 2026-11-23 (branch `sim-v1.2`, commit 721c57b)

**First perfect web sweep: 30/30 checkpoints at Lv3, 3/3 goals, average ≈ 166.5 pt** (v2.0 ≈ 121, v1.8 ≈ 90).

| Level | Plan (expected finish / score) | Actual (judge) | Score | Seconds lost to pulses (legs > 2 s over plan) |
|---|---|---|---|---|
| Normal | 178 s / 154.6 | 201.4 s | 146.2 | ≈ 28 s: CP6 +8.8 (max error 0.15 m), CP8 +10.3 (0.42 m), CP5 +8.8 (0.56 m) |
| Hard | 200 s / 173.3 | 198.7 s | 173.9 | ≈ 2 s: CP9 +2.7, CP1 +2.4 |
| Very hard | 233 s / 183.6 | 241.2 s | 179.4 | ≈ 20 s: CP11 +4.7, CP8 +3.6, CP4 +12.5 (0.24 m) |

- Standby confirmed at the first attempt in all runs; no hand-backs; `/ctl_only` silent during flight; peak duty 0.975; JAXA's saturation rule never triggered (team probe).
- Undisturbed legs ran about 0.7 s over plan (the 1 s judge hold is inside that).
- Disturbed legs show large max errors (0.15–0.56 m) with almost no "paused" time, so the loss is the slow return after a pulse (correction capped at 6 cm/s) rather than waiting on the path.
- The offline simulator predicted this size of loss (with pulses: Normal 193 s vs 173 s without), so further speed work can be done offline.
- Next lever: faster, still stable recovery after a pulse. Worth about 0.5 pt per second saved; Normal lost ≈ 13 pt here.

## S3 — v2.2 offline evaluation, 2026-11-24

**Changes in v2.2:**
- Dynamic braking return speed (`RET_ETA = 0.4`, `RET_VMAX = 0.20 m/s`, `RET_MARGIN_M = 0.03 m`). When pushed off course by web gravity pulses (up to 0.14 N), the robot returns toward the reference line at up to 20 cm/s (governed by a parabolic physical braking curve) rather than crawling at a fixed 6 cm/s cap. Near the target, `VC_MAX` remains the floor.
- `TRACK_POS_FAIL_M = 2.0 m` (was 1.0 m): provides ample buffer during severe transient displacements from web gravity pulses.

**Offline results (IB2Sim with web gravity module pulses & KPR enabled):**
- **Normal:** 8/8 Lv3, Goal True, 0 aborts, duration **178.8 s**, score **154.5 pt**.
- **Hard:** 10/10 Lv3, Goal True, 0 aborts, duration **212.4 s**, score **167.9 pt**.
- **Very Hard:** 12/12 Lv3, Goal True, 0 aborts, duration **255.1 s**, score **172.5 pt**.
- 100% Lv3 pass rate across all tiers, 0 hand-backs, 0 fan saturations. Faster recovery reclaims seconds lost to web pulses.

## P1 + S2 — Probe of the W6 bags, and v2.1 offline, 2026-11-24

**Probe (team's tool, `web_results/...__probe/`):**
- Only `/krpc_user` and `/ctl_only` publish `/ctl/wrench`, so the "second publisher" hypothesis is disproven. Our duty peaked at 0.975 with no saturation events while in fan mode.
- `krpc_gravity_module` re-sets Gazebo gravity x every second. Baseline is about +0.0022 m/s². Pulses: −0.0395 for 5 s, +0.0054 for 3 s, +0.0441 for 5 s, −0.0069 for 1 s, −0.0335 for 2 s, with gaps of 9 / 19 / 23 s. On 3.216 kg this is up to 0.14 N along ISS ±y, more than the fans can push in weak directions (0.07–0.10 N).
- The hand-back to JAXA while moving made the saturation shove (STOP_MOVING assumes rest).

**v2.0 failure causes:** (1) pulses beyond fan capacity; (2) plans with no headroom (load 0.95 during turn + acceleration); (3) hand-back at speed.

**v2.1 changes** (bundled, because there are few web tickets; each was checked separately offline):
- Plans keep 20 % headroom: the push and twist together use at most 80 % of any fan (`PLAN_LOAD`).
- Path following: the reference waits when the robot falls more than 10 cm behind, instead of a fixed timeline. The turn keeps its own clock.
- Correction speed caps (6 cm/s, 0.2 rad/s) stop the orbiting limit cycle; the integral only works within 5 cm (no windup).
- Settle timeout 60 s past plan + waits (was 15 s).
- On any fan abort: brake to rest first, then hand back (no shove).

**Offline result** (IB2Sim physics + web pulses at a random phase, 10 courses per level, `--kpr`):

| Level | v2.0 with pulses | v2.1 with pulses | v2.1 no pulses |
|---|---|---|---|
| Easy | 124.1 pt, 10/10 perfect, 9 hand-backs | 131.2, 10/10, 0 | 137.5 |
| Normal | 138.4, 10/10, 7 | 149.2, 10/10, 0 | 156.6 |
| Hard | 132.2, 7/10, 9 | 163.2, 10/10, 0 | 173.6 |
| Very hard | 122.4, 3/10, 7 | **174.3, 10/10, 0** | 184.9 |

- The cost of the headroom with no pulses is under 1 pt per run.
- Non-fan fallback, aborted-first-move and slow-web cases still pass. Offline tests and the Python 3.6 parse check pass.

## W6 — Web simulator, v2.0 on Normal / Hard / Very hard, 2026-11-23 (branch `sim-v1.2`, commit 3bb00a7)

| Level | Passed (all Lv3) | Goal | Mission time | Score (formula) | Fan mode |
|---|---|---|---|---|---|
| Normal | 8/8 | yes | 279.1 s | ≈ 117.7 | handed back after 15.8 s (leg 1) |
| Hard | 8/10 | no | 278.6 s | ≈ 85.7 | handed back after 37.5 s (leg 2) |
| Very hard | **12/12** | **yes** | 281.2 s | **≈ 159.4** | 8 legs perfect, handed back in leg 9 |

- Average ≈ 121 vs v1.8's ≈ 90, even with the hand-backs. Very hard is the best web Very hard so far (v1.8: ≈ 96).
- Standby confirmed at the first attempt in all three runs; `/ctl/status` never left standby (no "woke up" warnings). Not caused by `keep_pose_release`.
- Fan legs before the failure matched the plan as in local Gazebo (Very hard: 6.6 m leg plan 40.3 s, actual 40.3 s; 2–6 mm accuracy).
- **Failure pattern:** in one leg per run the robot stopped following our commands. Normal leg 1: tracking 2–30 mm for 7 s, then error grew about 6 cm/s until the 0.6 m hand-back. Very hard leg 9: error grew from the start, with about 174 mN extra push along body +x in the first second (the same as "fans 5–8 full, others off" under JAXA's saturation rule).
- Under JAXA's controller (v1.8 web runs) the robot never accelerated much above JAXA's own limit (max about 0.034 m/s²), so the web disturbance is not a frequent large push.
- Hypotheses: (1) a web-only node (the gravity module?) also publishes `/ctl/wrench`; at our 50 Hz its messages win far more often than against JAXA's 200 Hz; (2) the fan saturation rule triggers on the web. Position data alone cannot decide; `tools/probe_bag.py` extracts publishers, every `/ctl/wrench` and `/ctl/duty` message and `/ctl/status` from the three bags (no web runs needed).

## L1 — v2.0 in local Docker Gazebo, 2026-11-23 (branch `sim-v1.2`, commit 9793f05, `share/logs/20261123144344`, `…145442`)

| Run | Level | Passed | Goal | Mission time (judge) | Score (formula) |
|---|---|---|---|---|---|
| 20261123144344 | Normal | **8/8 Lv3** | **yes** | 172.2 s | **156.9** |
| 20261123145442 | Very hard | **12/12 Lv3** | **yes** | 222.3 s | **188.9** |

- **First real proof of the speed project.** Zero warnings or errors in both logs; standby confirmed at the first attempt; no hand-back.
- **Gazebo matches the offline simulator:** every leg settled within −0.2 to +0.1 s of its planned time (6.6 m leg: plan 40.1 s, actual 40.2 s). Planner estimates 175 s / 228 s vs actual 172.2 / 225.4 s (our clock).
- **Accuracy:** every report 2–4 mm from the checkpoint centre, centering ≤ 0.5°, roll ≤ 0.4°, all band 3 at the first report.
- Learned nav velocity bias (−7.6, −4.6, 1.1) and (−5.5, −3.9, −1.4) mm/s, close to sim.yaml (−5.6, −4.3, 0.2).
- **The first report each run took 3.7 s** (later ones 1.0 s), as on the web. It is the judge/API connecting on first use, not ours; about 2.7 s per run.
- Mission Start can take up to 3 s to return (Very hard: our clock read 225.4 s vs the judge's 222.3 s); harmless, since the judge's clock starts later.
- Still unproven: the web's `keep_pose_release` node and disturbance program (not in the local package).

## v2.0 (delivered 2026-11-23): our own controller through /ctl/wrench (the speed project)

**Why:** W5 (v1.8 web, 2026-11-22): every report Lv3, but Very hard 9/12 at 280.4 s (planner estimate 281 s, very accurate) and even a Normal course skipped one checkpoint. With JAXA's controller, all 12 Very hard checkpoints need about 370–390 s. Web runs are nearly used up for November, so development moves to the offline simulator and local Docker.

**How the simulator turns a push into motion** (reference notes §12): `/ctl/wrench` → `fsm` allocates to 8 fans → `/ctl/duty` → `prop` → fan plugin (0.06 N per fan, 0.01 s lag, 20 % noise). Max push: 0.181 N along the nose, 0.082 N typical, 0.071 N weakest; JAXA's controller plans with 40 % of the push in whatever direction the robot faces (= the measured 0.010 m/s²) and flies first, then turns.

**Design** (`fanctl.py`, `fanmode.py`, `fans.py`):
- Each leg: straight line and single-axis turn at the same time; acceleration = 60 % of the weakest push along the line over the attitudes passed through; turn uses 35 % of the torque.
- PD + feed-forward tracking on a state predicted 0.1 s ahead (navigation delay); nav velocity bias learned while docked; every command scaled so no fan exceeds 95 % (JAXA's saturation rule never triggers).
- Handover: STAND_BY goal (JAXA's controller goes quiet), `/ctl/status` watched, standby re-sent if anything wakes it; end: zero wrench + KEEP_POSE goal. Report sent while still holding; settle check 5 cm / 6° / 1 cm/s for 0.3 s.
- Safety net: stale nav, tracking error > 0.6 m or 60°, a leg not settled 15 s after plan, or standby overridden > 8 times → hand back to JAXA's controller for the rest (v1.8 behaviour).
- Planner uses our leg times (+1.6 s per stop, ×1.05).

**Offline results** (6-DOF simulator with the real fan pipeline, `sim/`):
- Single legs land within 4 mm and 0.2° on plan time; 3 m + 167° turn: 29.7 s (JAXA about 48 s).
- Full courses through the real program (dry run with physics, web interference copied, 6 mN push), 5 courses per level: every checkpoint Lv3 + goal on all 20. Easy 139–147 s (≈ 137 pt), Normal 167–179 s (≈ 157), Hard 193–209 s (≈ 173), Very hard 222–232 s (≈ 187).
- Robust to: 6× and 15× the default disturbance (12 and 30 mN), 0.2 s nav delay, 20 Hz control, mass +8 %. Breaks at 50 mN (then hands back as designed).
- Fan mode off (`FAN_MODE = False`) reproduces v1.8 exactly.

**Not yet proven:** real Gazebo (local Docker first), the web's unknown disturbance program, and the web `keep_pose_release` node (copied in the dry run from the rosbag description).

## W5 — Web simulator, v1.8 on Normal / Hard / Very hard, 2026-11-22 (branch `sim-v1.2`, commit 56414d5)

| Level | Passed (all Lv3) | Goal | Mission time | Score (formula) |
|---|---|---|---|---|
| Normal | 7 / 8 (CP 9 planned skip) | no | 254.1 s | ≈ 80.7 |
| Hard | 8 / 10 | no | 244.8 s | ≈ 94.7 |
| Very hard | 9 / 12 | no | 280.4 s | ≈ 95.9 |

- The official clock works: start-up took 2–3 s and no longer counts. Planner estimates were close (Very hard 281 s planned, 280.4 s actual).
- The Normal course genuinely did not fit all 8 with JAXA's controller (CP9 too far for the budget), the limit of the stock controller.

## W4 — Web simulator, v1.7 on Normal / Hard / Very hard, 2026-11-22 (branch `sim-v1.2`, commits 3de07ba logs, 6a28738 code; `*_13xxxx__full_logs/`, reconstructed from the rosbags)

| Level | Passed (all Lv3) | Goal | Mission time | Score (formula) |
|---|---|---|---|---|
| Normal | 7 / 8 | **no** (CP 11 planned skip) | 225.8 s | ≈ 87.3 |
| Hard | 8 / 10 | no (CPs 6, 11 planned skips) | 251.4 s | ≈ 92.9 |
| Very hard | 9 / 12 | no (CPs 8, 9, 11 planned skips) | 271.4 s | ≈ 98.6 |

**Finding 1 — the Lv3-edge targeting works.** All 24 reports accepted at band 3. Reported distances 2–5 cm (the planned 4 cm offset plus arrival error), centering ≤ 4.2°, roll ≤ 2.6°. No aim fixes, no fallbacks.

**Finding 2 — the Normal run lost its goal to over-cautious planning, a latent bug since v1.2.**
- The web start-up took 12.0 s this time (servers ready at +10.3 s). Our clock started at the start signal, so the plan began at 12.0 s.
- The all-8 route was estimated at 274 s → 286 s on our clock; padded ×1.05 + 15 s reserve = 314.7 > 300, so CP11 was skipped.
- Reality: the 7-checkpoint route ran at 0.973 × the estimate and finished 62 s early. All 8 would have finished at about 266 s official time (≈ 122 pt instead of 87).
- The planner maximised the number of checkpoints, so it never saw that the last one was worth 40 pt (10 + goal bonus).

**Finding 3 — whole-mission accuracy on the web** (actual / estimated, from planning to finish): v1.5 runs 0.95, 1.03, 1.03; v1.7 runs 0.973, 0.991, 0.974. Goal legs stay slow (1.10–1.20).

**v1.8 (delivered 2026-11-22), one change: plan with the official rules.**
- **Official clock:** the 300 s budget, the hard stops and the go-home rule now count from Mission Start (the web scorer's clock), not from the start signal. Start-up delays no longer cost budget.
- **Official score:** new `planner.plan_route_score` picks the subset and order with the highest expected (10 × checkpoints + 30 if all) × (1 + (300 − T)/300), with the same go-home feasibility rule; `planner.mission_score` holds the formula. Equals brute force on 60 random courses (new test).
- **Margins from 6 web runs:** goal leg costed ×1.15 (`GOAL_LEG_FACTOR`), other padding 1.05 → 1.03, end reserve 15 → 6 s. The 294 s goal-leg stop stays.
- Replay of the 9 logged web courses: the W4 Normal course now plans all 8 (expected ≈ 118 pt vs 86); the other 8 courses keep the same number of checkpoints.
- Dry run, 160 runs (10 courses × 4 levels × start-up 2 s and 12 s, `--err --goal-slow=1.15`; new dry-run options `--goal-slow` and an official score in each RESULT line): Normal goals 8/10 → 10/10 (2 s start) and 1/10 → 10/10 (12 s start), mean score +7.8 and +32.7. Easy, Hard, Very hard within ±2.6 pt. In this harsher-than-web dry run one Hard and one Very hard course hit the 294 s goal stop after an aim fix (fake roll errors up to 12°; the web goal reports never exceeded 2.8°).

## v1.7 (delivered 2026-11-22), one change: stop at the edge of the Lv3 zone

- For every move (and every retry), the commanded pose is computed from the robot's current pose: up to **4 cm short** of the checkpoint centre on the approach side, re-aimed at the marker from there (`aim_from_here`), and the turn ended up to **4° early** (new `geometry.q_slerp`).
- Safety check before sending: the commanded pose itself must score Lv3 with distance ≤ 0.5 × Lv3 radius and centering, roll ≤ 4.5°. Otherwise it tries half the offset, then the exact centre. With the web arrival error (≤ 2.4 cm, ≤ 3.5°) this stays inside our report margins (8 cm, 8°).
- Offsets are smaller than in study S1 (5 cm, 6°) because the controller's own arrival error must fit in the same budget.
- The planner and go-home rule are unchanged (they now slightly overestimate leg times; recalibrate after a web baseline).
- New dry-run option `--err`: the fake controller misses its target like the web one (about 1 cm per axis, up to 3.5°).
- Dry run, 20 courses (5 per level) with `--err`: every checkpoint Lv3, no fallbacks or aim fixes, same goals and checkpoints as v1.6; mean mission time saved 6.0 s (Easy), 1.9 s (Normal), 4.8 s (Hard), 2.2 s (Very hard). Abort retries and the late-goal stop still work.
- Expected value: about +1–2 pt on a perfect Normal run; on Hard and Very hard a few seconds may occasionally fit one more checkpoint.

## Study S1 — Would lower levels let us pass every checkpoint? (2026-11-22, offline, `analysis/tolerance_study.py`)

Question from the team: on Hard / Very hard, aim for Lv2 or Lv1 so that all checkpoints fit and the 30 pt goal bonus is earned?

Method: best all-checkpoint route on 30 random courses per level, with the arrival distance shortened by the position slack and every turn shortened by the angle slack ("stop at the near edge of the allowed zone"), web-calibrated (×1.03, plus 1 s judge hold per report). Relative savings are more reliable than the absolute times.

| Stop at | Normal median | Hard median | Very hard median | Hard courses fitting 290 s |
|---|---|---|---|---|
| exact centre (today) | 289 s | 339 s | 394 s | 0/30 |
| Lv3 edge (5 cm, 6°) | 282 s | 331 s | 384 s | 0/30 |
| Lv2 edge (15 cm, 16°) | 269 s | 315 s | 364 s | 0/30 |
| Lv1 edge (25 cm, 26°) | 255 s | 298 s | 339 s | 6/30 |

Findings:
- Most of each leg is flying and turning, so looser levels save only about 1–4 s per stop: Lv3 edge saves about 8–10 s per run, Lv2 edge 20–30 s, Lv1 edge 35–55 s.
- Very hard never fits, even at Lv1. Hard fits only sometimes at Lv1, and then finishes near 300 s (time bonus about ×1.0, and the program is shut down if it overruns).
- Hard all-Lv1 + goal beats today's 9 × Lv3 (≈ 94 pt) only if Lv1 is worth more than about 6 pt. Lv1 and Lv2 point values are still unknown.
- **"Lv3 edge" is free:** same points, about 8–10 s faster on every level (≈ +3 pt on a perfect Normal run).
- To earn the goal bonus on Hard at Lv3, about 40–50 s per run must be saved, which needs turning while flying (custom `/ctl/wrench` control).

## W2 — Web simulator, v1.5 on Normal / Hard / Very hard, 2026-11-22 (repo branch `sim-v1.2`, commit e19f4fe, `web_results/{Normal,Hard,Very Hard}/`)

**Results:** see the Web scoreboard. Every reported checkpoint was accepted at band 3. Normal now matches W0 (the other team's perfect Normal run), and is 6.3 s faster.

**Finding 1 — the retry works.** The first move was again aborted at once in all three runs (now 6 of 6 web runs). Each retry 0.5 s later succeeded, so the abort costs about 0.5 s instead of a checkpoint. No mid-flight aborts this time.

**Finding 2 — the goal leg runs long, and the Hard run nearly lost its goal checkpoint to our own hard stop.**
- Hard: route [1, 7, 6, 2, 3, 5, 10, 8, 12], planned skip [9], estimated finish 278 s. The goal leg was estimated at 49.2 s and took 58.8 s to stable Lv3 (×1.20).
- CP12 was "Lv3 held" at 284.6 s on our clock and reported at 285.4 s. The hard stop is checked at the top of the loop at 285.0 s, so half a second more and the program would have stopped 4 cm from the goal without reporting CP12.
- Goal legs on the web, actual/estimated flight time: 1.11, 1.10, 1.20, 1.13, 1.13 (W1-H, W1-V, W2-H, W2-N, W2-V).

**Finding 3 — model accuracy on the web.** Over 42 legs without a retry, stable-Lv3 time / estimated flight time: median 1.06, mean 1.08 (excluding the aim-fix outlier, 0.88–1.20). Whole missions: estimated finish vs actual finish 271 → 258 (Normal), 278 → 285 (Hard), 246 → 252 (Very hard). Report latency after "Lv3 held": median 0.8 s, up to 3.2 s (6 of 45 over 2 s).

**Finding 4 — judge race:** judge connected 7.5, 0.8 and 0.5 s before the start signal. The gate passed at +1.9–2.0 s in all three runs (waiting for the course, not the judge).

**Rulebook check (§2.2.3, §2.3.1, §4.2.5):** the timer starts when Undock is completed; the completion report must be inside 5 minutes or no completion points; if the limit is exceeded the program is shut down automatically. The remaining-time multiplier is applied to the total score. The difficulty level used for the official 10 preliminary runs is not stated in the Rulebook, Guidebook or Manual ("multiple patterns with the same route"). Add to the JAXA questions.

**v1.6 (delivered 2026-11-22), one change: a later hard stop on the goal leg only.**
- `GOAL_HARD_STOP_S = 294` while flying to the goal (our clock starts about 2 s before Mission Start, so this is about 292 s judge time). Other checkpoints keep the 285 s stop. The planner and go-home rule are unchanged.
- Dry run with the goal leg slowed (`--slow=<move>:<factor>`, new in `tests/dry_run.py`): Hard seed 6 ×1.5 and seed 3 ×1.35 crossed 285 s; v1.5 abandoned CP12 (level 0), v1.6 scored it Lv3 and sent mission_finished at 291.4 s and 287.8 s. Cases that finish before 285 s are identical. Without slowdowns v1.6 is identical to v1.5 on 12 courses.
- Next candidates (one at a time): goal-leg time factor about 1.15 in the planner and go-home check; decide Hard/Very hard skip policy once the scores are known.

## Scoreboard (local Gazebo, sim v1.0 rules)

| Run | Version | Level | Scored (all Lv3) | Goal | Judge mission time |
|---|---|---|---|---|---|
| E1 | team baseline | Normal | 2 / 8 | no | 216.5 s |
| E2 | v1.0 | Normal | 8 / 8 | yes | 262.9 s |
| E3 | v1.0 | Hard | 8 / 10 | yes* | 265.3 s |
| E4 | v1.1 | Hard | 7 / 10 | yes* | 244.3 s |
| E5 | v1.2 | Hard | **9 / 10** | yes* | 254.9 s |
| E6 | v1.3 | Very hard | 0 (judge missed Mission Start; the robot flew 9 / 12) | — | — |
| E7 | v1.4 | Very hard | **9 / 12** | yes* | 253.9 s |

\* Counted by the old v1.0 judge. Under sim v1.2's strict goal rule (every checkpoint must pass) these would be goal failures.

## W1 — Web simulator, v1.4.1 on Normal / Hard / Very hard, 2026-11-21 (repo branch `sim-v1.2`, commit ae8765d, folder `web_results/`)

**Results:** see the Web scoreboard. Every reported checkpoint was accepted at band 3. No checkpoint was lost to the 1 s hold rule. All losses came from controller aborts and planned skips.

**Finding 1 — the controller aborts moves on the web sim, and v1.4.1 never retried.**
- The **first move of every run** ended at once with result 1 (TERMINATE_ABORTED, "after 0.0 s"). v1.4.1 waited 6 s, saw the robot still at the dock and moved on, so the first checkpoint of the route was lost in all three runs (Normal CP2, Hard CP7, Very hard CP1).
- On Normal, the **goal leg** was aborted mid-flight after 23 s, 0.59 m short of the goal. v1.4.1 did not retry and sent `mission_finished` there with 79 s left, so CP12 was never reported and the goal failed ("checkpoint left outermost margin").
- Likely cause: the controller's collision detector (`dtc.cpp`: a burst of acceleration/angular-rate noise, then calm, is read as a collision and aborts the active move). The web sim has disturbances. Why the very first move aborts is not proven (no simulator logs on the web); the other team's first move did not abort.
- The other team's program retries every move once (MAX_ATTEMPTS 2).

**Finding 2 — the planner skipped checkpoints that fit, which now forfeits the goal.**
- Normal planned skip [11] (estimated finish 248 s with 7 of 8). Under the strict goal rule that alone loses the goal.
- Hard finished with 49 s unused and Very hard with 35 s unused, partly because the aborted first checkpoint was never flown.

**Finding 3 — the time model holds on the web sim.**
- The other team's Normal course replayed through our model: estimate 264 s (×0.96 plus 1 s holds) vs 262.3 s actual. Our planner picks exactly their route [1, 2, 3, 5, 10, 8, 9, 12].
- Report latency after "Lv3 held": about 0.8 s sim normally, 2.7–3.2 s for some reports.
- The web sim runs at about 0.36× real time (sim seconds per wall second), slower than the Mac (about 0.5×).

**Finding 4 — the judge race exists on the web too.** Probe timelines (judge connected, relative to the start signal): Normal −0.8 s; Very hard: registered −2.4 s, and the gate only confirmed the connection at +1.8 s; Hard: the sim clock had not started at node launch, so the times are compressed, but the judge was confirmed at +5.6 s. The margin is thin, so the v1.4 gate stays. It cost 0–1.8 s per run.

**Why W0 (the other team) scored the most:** it was a Normal run with all 8 checkpoints and the goal (which also unlocks Pt_goal), and no aborts. Nothing in its motion is faster than ours: same route, same speed model.

**Open:** the dashboard scores for W0 and W1 are needed to work out the points per level, Pt_goal and the time multiplier.

**v1.5 (delivered 2026-11-22), one change: retry a move that ends without success.**
- If `/ctl/command` ends with any result other than TERMINATE_SUCCESS, wait 0.5 s and send the same target again, up to 2 retries per checkpoint.
- A regular checkpoint is retried only if there is still time to fly it and get home (same go-home rule). The goal is always retried until the hard stop.
- Logging only: the full course JSON is now logged, so web runs can be replayed offline.
- `tests/dry_run.py` updated to sim v1.2 rules (1 s hold with the lowest level counting, real verdicts, retry after a failed report, strict goal rule) and to inject aborts (`--abort=1:0,9:15`).
- Dry run: with the W1 pattern (first move aborted at once, a later move aborted after 20 s), v1.4.1 lost a checkpoint and the goal; v1.5 scored 8/8 plus the goal on both Normal seeds. A goal-leg abort after 15 s was also recovered. Without aborts v1.5 is identical to v1.4.1 on 12 courses.
- Next candidate (one change): goal-aware planning, i.e. plan all checkpoints whenever they fit (Normal seed 3 still skips one in the dry run, as W1-N did).

## E7 — v1.4 on Very hard, 2026-11-21 (run 20261121163426, repo commit 0ea6349)

**Result: 9 of 12 scored, all Lv3 (8 checkpoints + goal). Goal reached ("previously passed", old rule). Judge mission time 253.9 s. First counted Very hard run.**
- Route: [9, 8, 4, 3, 2, 6, 7, 1, 12], with planned skips [5, 10, 11]. No go-home cut was needed.

**The readiness gate worked.**
- The judge connected to `/gs/data` at 16:35:42.705.
- v1.4 logged "judge connected" and sent Mission Start at 16:35:48.044, giving 5.3 s of margin.
- The judge logged "status arrived: Mission Start" and "Mission Finish". The run ended normally at mission finish, not by the 600 s watchdog.

**Pre-start cost**
- Start signal → judge confirmed: 8.9 s sim time, mostly waiting for the action servers and the course.
- Plus about 3 s inside the `mission_start` service call itself.
- In total, Mission Start went out 12.0 s after the start signal.
- v1.4 counts its clock from the start signal, so our log says 265.9 s at finish while the judge says 253.9 s.
- This matters: planned at 9.0 s, the route fitted 8 checkpoints. Planned at 12 s, it would fit 7. If the limit counted from Mission Start, the same course allows 8 with an estimated finish of 272 s.
- **Open question for JAXA:** does the 5-minute limit (and the time multiplier) count from the start signal / undock or from the Mission Start event? A draft question was given to the team on 2026-11-21. The question deadline is 2027-01-20 12:00 JST. (Web results report `duration_sec` from Mission Start to Mission Finish.)

**Other observations**
- CP3: move success at 134.4 s, report at 139.4 s. This is the same small-final-turn pattern as E5 CP5 (attitude within the controller's 29° tolerance, then a short turn), and is normal.
- Every reported checkpoint: distance 1.1–1.4 cm, centering 0.8–2.9°, roll 0.2–4.8°.

## E6 — v1.3 on Very hard, 2026-11-21 (run 20261121161010, repo commit ad0c010)

**Result: the judge ignored the whole run.** No result.json, empty data.log, and the run ran out the full 600 s watchdog.

**Root cause (found by the team, confirmed in the logs):** a start-up race inside the simulator.
- `krpc_api_node` published "Mission Start" on `/gs/data` at 16:11:18.996. The judge (`/kibo_rpc_judge`) only connected at 16:11:22.656, 3.7 s later.
- `/gs/data` keeps no old messages, so the start was lost. The judge then rejected all 9 reports and the finish ("… received before mission start").
- It happened because the start signal came about 1 s after our node was ready, while connections were still being set up. In E5 the judge had connected about 6 s before Mission Start.
- The robot itself flew 9 of 12 at Lv3 with the goal at 256.3 s (route [7, 1, 2, 3, 4, 8, 9, 6, 12], planned skips [5, 10, 11]).

**v1.4 (delivered 2026-11-21), one change: a readiness gate before Mission Start** (`judge_link.py`).
- Order:
  1. Wait for the `/ctl/command` and `/event/checkpoint_passed` servers.
  2. Fetch the course.
  3. Confirm the judge is connected to `/gs/data`.
  4. Plan the route.
  5. Then call `mission_start`.
- The judge check uses the ROS master's subscriber list plus `krpc_api_node`'s standard `getBusInfo` (ROS bookkeeping only).
- Limits: wait up to 45 s if a judge is registered but not yet connected; only 5 s if no judge is visible or the check fails.
- Dry run with the race simulated: delays of 0, 3.7, 10 and 30 s all scored; 60 s was lost; a failed check proceeds after 5 s.
- v1.4.1 (2026-11-22) added a logging-only start-up timing probe (`startup_probe.py`); results in W1.

## E5 — v1.2 on Hard, 2026-11-21 (run 20261121153626, repo commit a2a758f)

**Result: 9 of 10 scored, all Lv3 (8 checkpoints + goal). Goal reached (old rule). Mission time 254.9 s (estimated 278 s).**
- Route: [3, 5, 10, 8, 4, 2, 1, 7, 12]. The only skip, CP11 (the far airlock), was planned from the start.

**Findings**
1. The time-budgeted route works as designed.
2. CP5's 5 s gap between move success and report is normal: the arrival attitude was within the controller's 29° tolerance, and our check waited for the final small turn.
3. CP2 was reported while the robot was still gliding the last 2 cm at about 2 cm/s. It was valid Lv3, but centering was rising. Candidate later change: tighten stillness to about 0.015 m/s, or require the move to have ended.
4. Model accuracy over all v1.x runs (39 legs): real/estimate is 0.96 overall, range 0.75–1.05.

**v1.3, one change: go-home and plan padding 1.15 → 1.05.** It gains a checkpoint on the E3 course and is unchanged on the others.

## E4 — v1.1 on Hard, 2026-11-21 (run 20261121144256, repo commit 67ff38b)

**Result: 7 of 10, all Lv3, goal reached, 244.3 s, with 56 s unused.**
- The report rule works: stable Lv3 → report fell from about 3.4 s to 2.0 s.
- The ×1.15 padding cost a checkpoint.
- The route planned all 9, flew to the far end early, then dropped the middle checkpoints.
- This led to **v1.2: a time-budgeted route** (orienteering on the Held–Karp table; equals brute force on 60 random cases).

## E3 — v1.0 on Hard, 2026-11-21 (run 20261121135852, repo commit 47f71c1)

**Result: 8 of 10, all Lv3, goal reached, 265.3 s.** The goal leg (6.39 m with a 179° turn) hit v1.0's 60 s timeout and only scored Lv3 by luck (roll 8.9°).

| CP | dist (m) | turn (°) | actual | stable Lv3 | v1.0 est | v1.1 est |
|---|---|---|---|---|---|---|
| 1 | 1.67 | 104 | 34.4 | 30.9 | 33.7 | 35.4 |
| 7 | 0.37 | 127 | 22.3 | 19.3 | 23.2 | 22.6 |
| 6 | 0.50 | 108 | 23.2 | 20.2 | 24.1 | 23.9 |
| 2 | 1.05 | 158 | 31.6 | 28.2 | 31.4 | 32.2 |
| 3 | 0.36 | 97 | 21.5 | 18.4 | 21.9 | 21.3 |
| 5 | 2.57 | 75 | 42.0 | 38.8 | 37.8 | 40.5 |
| 11 | 1.45 | 76 | 30.2 | 26.8 | 31.2 | 32.6 |
| 12 (goal) | 6.39 | 179 | 60.0 (timeout) | 58.8 | 61.1 | 64.6 |

**Where the time goes:** translation, plus a ~1.7 s gap, plus a 75–180° turn (6–11 s), plus the report delay. The stock controller always translates first and turns second; overlapping them needs custom `/ctl/wrench` control.

## E2 — v1.0 on Normal, 2026-11-21 (run 20261121134643, repo commit 9433652)

**Result: 8 of 8 at Lv3, goal reached, 262.9 s.** Our predicted level matched the judge on every checkpoint.
- The goal leg nearly timed out.
- The refitted model is accel ≈ 0.010 m/s², angular accel ≈ 0.18 rad/s², overhead ≈ 3 s per stop.
- Controller source: a new goal preempts the old one (`isPreemptRequested`); navigation older than 10 s triggers INVALID_NAV.

## E1 — Baseline run, 2026-11-21 (run 20261121093747)

**Result:** 2 of 8 CPs, goal failed.

**Lessons**
- The 30 s timeout was too short.
- Course order zig-zags.
- After a cancel, wait for the action to end before the next goal.
- The local sim runs at about 0.5× real time on the Mac (scores use sim time).
- The airflow "Out of Area" warnings are harmless.

**Measured physics** (from localization_history.txt, 10 Hz, sim time)

| Quantity | Estimate (ctl.yaml) | Measured |
|---|---|---|
| Translation acceleration | 0.0225 m/s² | ≈ 0.014 m/s² (E2 refit: ≈ 0.010) |
| Speed limit | 0.20 m/s | 0.197 m/s peak |
| Position after arrival | — | 1.0–1.5 cm from target |
| Turn rate peak | 25°/s | ~25–28°/s |
| Angular acceleration | 0.24 rad/s² | ≈ 0.15 rad/s² (E2 refit: ≈ 0.18) |
| Gap between translation end and turn start | — | ≈ 1.5–2 s |
| Move-then-turn order | from source | confirmed |
