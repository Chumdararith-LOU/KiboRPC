# Kibo-RPC Experiment Log

Measured results from Gazebo runs. These numbers replace the estimates from ctl.yaml wherever they differ. Newest first.

**Method rule (agreed 2026-11-21):** one change per version, and each version gets a baseline run before the next change. `run.sh` has no seed option, so every Gazebo run draws a new random course. Versions are compared by (a) replaying the exact logged course offline and (b) several runs per version.

## E4 — v1.1 on Hard, 2026-11-21 (run 20261121144256, repo commit 67ff38b)

**Result: 7 of 10 scored, all Lv3 (6 checkpoints + goal); 3 skipped by the go-home rule. Goal reached ("previously passed"). Mission time 244.3 s, so 56 s of the 300 s went unused.**
- Route: [1, 6, 9, 10, 5, 8, 4, 3, 7, 12], planned for all 9 and estimated at 328 s.
- At 191.2 s, after CP8, go-home skipped CP4, CP3 and CP7: "cp 4 needs 83 s more".

**Findings**
1. **The report rule works.** Stable Lv3 to report now takes 1.6–2.6 s (mean 2.0 s), down from about 3.4 s with v1.0. Given the 1 s hold, the floor is about 1 s.
2. **The v1.1 model is accurate and slightly conservative.**
   - Legs, as actual / model: CP1 31.0/33.9, CP6 23.9/24.6, CP9 28.7/28.3, CP10 49.3/51.9, CP5 27.3/27.9, CP8 30.8/33.4, goal 53.1/53.0 s.
   - Across E3 and E4, the model over-estimates on most legs.
3. **The ×1.15 go-home padding cost a checkpoint.** CP4 + goal needed an estimated 83.4 s, and the rule allowed only 81.6 s. The real finish would have been about 275 s. Tuning the padding (towards about 1.05) is a candidate *separate* change, after v1.2.
4. **The route problem again.** The plan was for all 9; it flew to the far end (CP10) early, then had to drop the middle checkpoints.

**v1.2 (delivered 2026-11-21), one change: a time-budgeted route** (`planner.plan_route_budget`, orienteering on the Held–Karp table).
- The budget rule is identical to the live go-home rule (×1.15 padding, 15 s reserve), so the plan and the rule never disagree.
- It equals brute force on 60 random cases.
- Replays on the logged courses (model timing, same rules):

| Course | v1.1 | v1.2 |
|---|---|---|
| E3 Hard | 6/9, 233 s | **7/9**, 254 s |
| E4 Hard | 6/9, 253 s | **7/9**, 262 s |
| E2 Normal | 7/7, 265 s | 7/7, 265 s (same route) |

- Dry run with the fake robot, 5 seeds per level:
  - Hard: v1.1 scored 7–8 of 10; v1.2 scored **8 of 10 every time**.
  - Very hard: v1.1 scored 7–8 of 12; v1.2 scored **8–9 of 12**.
  - Easy and Normal were unchanged.
  - The goal was always reached, and every report was Lv3.

## E3 — v1.0 on Hard, 2026-11-21 (run 20261121135852, repo commit 47f71c1)

The team ran v1.0 on purpose as a Hard baseline before v1.1 (the log header says v1.0).

**Result: 8 of 10 scored, all Lv3 (7 checkpoints + goal); 2 skipped by the go-home rule. Goal reached ("previously passed"). Mission time 265.3 s.**
- Route: [1, 7, 6, 2, 3, 5, 11, 10, 8, 12], estimated 307 s.
- At 205.3 s, after CP11, go-home skipped CP10 and CP8: "cp 10 needs 84 s more".
- **The goal leg hit v1.0's 60 s timeout.** It was 6.39 m with a 179° turn.
  - Four seconds before the timeout, centering was still 50°. The turn finished just in time, and the fallback report scored Lv3 with roll 8.9°.
  - That was luck. This confirms the 60 s cap flaw that v1.1 fixes (its timeout for this leg would be 100 s).

**Route finding: plan for the time budget, not for all checkpoints.**
- v1.0 planned all 9 checkpoints (estimated 317 s with the v1.1 model). It flew out to the airlock (CP11, x ≈ 7.2) and then had to skip CP10 and CP8 on the way home.
- The same Held–Karp table can pick the best *subset* that fits the budget (the "orienteering" variant), at no extra cost. This became v1.2.

**Model check (seconds from leg start to report).** v1.1's model tracks the real legs within about 2 s and is slightly conservative, which is what the go-home rule needs. v1.0 under-estimated the long legs.

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

**Where the time goes on Hard.** Each leg is translation, plus a ~1.7 s gap, plus a 75–180° turn (6–11 s), plus the report delay.
- v1.0's report delay was about 3.4 s from stable Lv3; v1.1's is about 2.0 s.
- Getting 10/10 on Hard needs either the preemption test (E5), or overlapping the turn with the translation (custom `/ctl/wrench` control). Overlapping could save roughly 6–11 s per leg.

## E2 — v1.0 on Normal, 2026-11-21 (run 20261121134643, repo commit 9433652)

**Result: 8 of 8 checkpoints at Lv3, goal reached ("previously passed"), mission time 262.9 s.**
- Route: [9, 6, 1, 2, 3, 8, 5, 12]. v1 estimated 256 s.
- Our predicted level matched the judge on every checkpoint.
- Every move ended within 1.1–1.4 cm of the target.
- Aim error at report was 0.2–2.4° and roll error 0.0–3.6°.

**Findings**
1. **The goal leg nearly timed out.** The 5.31 m leg with a 95° turn took 57.1 s, against v1's 60 s cap. v1.1 raises the cap to 100 s.
2. **Reporting was later than necessary.** The judge saw a stable Lv3 about 1.5–2.5 s before the controller's success, and v1 reported about 1.0–1.2 s after success.
   - The cause was v1's stillness rule (0.01 m/s, 2°/s), which waits for the end of the slow final turn.
   - We replayed the flight log through other rules. Lv3 with a margin (distance ≤ 0.8 × the Lv3 radius, angles ≤ the Lv3 limit − 2°) plus stillness < 0.03 m/s and < 8°/s fires 1.5–1.9 s earlier per checkpoint.
   - The judge level stayed 3 for the next 2 s every time.
3. **Refitted time model** (least squares on the 8 legs, start → report):
   - Accel ≈ 0.010 m/s², angular accel ≈ 0.18 rad/s², overhead ≈ 3 s per stop.
   - Residuals are −2.8 to +4.8 s per leg.
   - The old model under-predicted the long goal leg by 6.2 s, so the go-home rule now pads estimates by 15%.
4. **Phase timings** (translation done → stable Lv3 = gap + turn):
   - 122° took 9.4 s, 115° 8.8 s, 100° 8.4 s, 68° 8.2 s and 15° 4.4 s.
   - A turn-model profile plus a 1.7 s gap fits these well.

**Controller source check (preemption).** In `ctl.cpp` `guidance()`, a new goal raises isPreemptRequested. The old goal is aborted, and status goes to STAND_BY while the new goal starts. So sending the next goal without cancelling first should work. Not yet tested in Gazebo (planned E5). In the same function, navigation older than 10 s (`wait_cancel`) triggers INVALID_NAV, which matches the E1 abort.

**v1.1 changes (delivered 2026-11-21).** These were bundled; in hindsight they should have been split, per the method rule above.
- Move timeout max 60 → 100 s, with timeout = 1.5 × estimate + 15.
- Margin-based Lv3 report rule with looser stillness.
- Model is now a 0.010, α 0.18, settle 3.0.
- Go-home uses ×1.15 padding and a 15 s reserve.

## E1 — Baseline run, 2026-11-21 (run 20261121093747)

**Setup:** team code at commit f3aabe3 (course order, 30 s move timeout, report straight after success). Level Normal, 8 CPs, local Gazebo on the team Mac, playTime 10m (default). Logs are in the repo at `share/logs/latest/` (commit 99e809c).

**Result:**
- Mission time 216.5 s.
- 2 of 8 CPs passed (CP3 Lv3, CP4 Lv3). 5 moves timed out, and the move to CP12 aborted with INVALID_NAV.
- The goal failed ("checkpoint outside threshold"), because mission_finished was sent far from the goal.

**What went wrong**
1. **The 30 s move timeout is too short.** CP2 (2.79 m away) took 26.4 s just to translate, and the rotation had only just begun when the timeout hit.
2. **Course order zig-zags.** It went 2 → 3 → 4 → 5 (x 5.9) → 6 (x 1.9) → 7 → 10 (x 5.8) → 12 (x 0.5).
3. **Cancel followed immediately by a new goal did not take effect.** After the CP10 timeout and cancel, the robot kept flying towards CP10 during the "CP12" leg, then the controller aborted with INVALID_NAV. Rule: after cancel, wait until the action has really ended before sending the next goal.
4. **The local sim runs at about 0.50× real time on the Mac.** That was 216 s of sim time over 429 s of wall time. Timing in ROS and the judge uses sim time, so scores are unaffected.
5. The airflow plugin logged about 116k "Point … is Out of Area prepared by PCD file" warnings near the dock. They are harmless spam.

**Measured physics** (from localization_history.txt, 10 Hz, sim time)

| Quantity | Estimate (ctl.yaml) | Measured | Notes |
|---|---|---|---|
| Translation acceleration | 0.0225 m/s² | **≈ 0.014 m/s²** (E2 refit: ≈ 0.010) | 2.79 m leg: triangle profile, peak 0.197 m/s at 14 s |
| Speed limit | 0.20 m/s | 0.197 m/s peak | confirmed |
| Position after arrival | — | 1.0–1.5 cm from target | comfortably Lv3 |
| Turn rate peak | 25°/s | ~25–28°/s | confirmed |
| Angular acceleration | 0.24 rad/s² | **≈ 0.15 rad/s²** (E2 refit: ≈ 0.18) | turns of 77° ≈ 5.5 s, 136° ≈ 9 s, 174° ≈ 10.5 s |
| Gap between translation end and turn start | — | ≈ 1.5–2 s | plus the 3 s success dwell after the turn |
| Move-then-turn order | from source | confirmed | attitude is unchanged until the translation ends |
