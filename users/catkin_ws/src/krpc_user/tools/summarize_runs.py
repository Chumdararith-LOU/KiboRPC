#!/usr/bin/env python3
"""One line per run: level, checkpoints at Lv3, goal, mission time, official score, program version.

Usage (from the simulator folder):  python3 summarize_runs.py share/logs
Reads every <run>/judge_module_logs/result.json (and the user log for the version, level and
any fan-mode hand-back) below the folder given. Score = (10 x Lv3 + 30 goal) x (1 + (300 - T)/300).
"""
import json
import os
import re
import sys


def runs(root):
    for dirpath, dirnames, filenames in os.walk(root):
        if "result.json" in filenames and dirpath.endswith("judge_module_logs"):
            yield os.path.dirname(dirpath)


def summarize(run):
    with open(os.path.join(run, "judge_module_logs", "result.json")) as f:
        rep = json.load(f).get("JudgeReport", {})
    cps = rep.get("checkpoint_detail", [])
    lv3 = sum(1 for c in cps if c.get("success") and c.get("level") == 3)
    other = sum(1 for c in cps if c.get("success") and c.get("level") != 3)
    goal = bool((rep.get("goal") or {}).get("success"))
    t = (rep.get("mission") or {}).get("duration_sec")
    score = (10 * lv3 + 30 * goal) * (1 + (300 - t) / 300.0) if t is not None else None
    version, handback = "?", ""
    log = os.path.join(run, "user_programs_logs", "user_program_output.log")
    if os.path.exists(log):
        txt = open(log, errors="replace").read()
        m = re.search(r"krpc_user (v[\d.]+)", txt)
        version = m.group(1) if m else "?"
        if "fan mode stopped" in txt or "fan mode could not take over" in txt:
            handback = "  HAND-BACK to JAXA controller"
    n = len(cps) or 0
    level = {6: "easy", 8: "normal", 10: "hard", 12: "veryhard"}.get(n, "%d CPs" % n)
    return "%-28s %-6s %-9s %2d/%-2d Lv3%s  goal %-5s  %6s s  score %6s%s" % (
        os.path.basename(run.rstrip("/")), version, level, lv3, n, (" +%d lower" % other) if other else "",
        goal, "%.1f" % t if t is not None else "-", "%.1f" % score if score is not None else "-", handback)


if __name__ == "__main__":
    root = sys.argv[1] if len(sys.argv) > 1 else "share/logs"
    found = sorted(set(runs(root)))
    if not found:
        print("no runs with judge_module_logs/result.json under", root)
    for r in found:
        try:
            print(summarize(r))
        except Exception as exc:
            print("%s: could not read (%s)" % (r, exc))
