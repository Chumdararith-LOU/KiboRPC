#!/usr/bin/env python3
"""
Reconstruct the share/logs/latest structure from Web Simulator downloads
(results.zip and optional simulation.bag.zip) in web_results/.
"""

import os
import sys
import json
import zipfile
import shutil
import subprocess
import glob

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

def reconstruct_single_run(res_zip_path, output_dir=None):
    res_zip_path = os.path.abspath(res_zip_path)
    parent_dir = os.path.dirname(res_zip_path)
    folder_name = os.path.basename(parent_dir)

    base_name = os.path.basename(res_zip_path).replace("__results.zip", "")
    bag_zip_path = os.path.join(parent_dir, f"{base_name}__simulation.bag.zip")

    if output_dir is None:
        output_dir = os.path.join(parent_dir, f"{base_name}__full_logs")

    print(f"\n=======================================================")
    print(f"Reconstructing run: {base_name}")
    print(f"Directory: {parent_dir}")
    print(f"Output: {output_dir}")
    print(f"Has bag.zip: {os.path.exists(bag_zip_path)}")
    print(f"=======================================================")

    os.makedirs(output_dir, exist_ok=True)
    judge_dir = os.path.join(output_dir, "judge_module_logs")
    user_dir = os.path.join(output_dir, "user_programs_logs")
    flight_dir = os.path.join(output_dir, "flight_recorder_logs")
    tex_dir = os.path.join(output_dir, "textures")
    os.makedirs(judge_dir, exist_ok=True)
    os.makedirs(user_dir, exist_ok=True)
    os.makedirs(flight_dir, exist_ok=True)
    os.makedirs(tex_dir, exist_ok=True)

    # 1. Extract results.zip
    print(f"Extracting {os.path.basename(res_zip_path)}...")
    with zipfile.ZipFile(res_zip_path, 'r') as zf:
        zf.extract("result.json", judge_dir)
        zf.extract("user_program_output.log", user_dir)

    # 2. Reconstruct cheat_sheet.json from user_program_output.log
    user_log_path = os.path.join(user_dir, "user_program_output.log")
    cheat_sheet_path = os.path.join(output_dir, "cheat_sheet.json")
    with open(user_log_path, 'r') as f:
        for line in f:
            if "course json:" in line:
                cj_str = line.split("course json:")[1].strip()
                course_data = json.loads(cj_str)
                cheat_sheet = {
                    "Level": folder_name.lower(),
                    "Checkpoints": course_data.get("checkpoints", []),
                    "goalpoint_id": course_data.get("goalpoint_id", 12)
                }
                with open(cheat_sheet_path, 'w') as out_cs:
                    json.dump(cheat_sheet, out_cs, indent=4)
                print(f"Generated {cheat_sheet_path}")
                break

    # 3. Generate data.log and flight recorder data.json from result.json
    res_path = os.path.join(judge_dir, "result.json")
    data_log_path = os.path.join(judge_dir, "data.log")
    fr_data_path = os.path.join(flight_dir, "data.json")
    with open(res_path, 'r') as f:
        res_data = json.load(f)
    events = res_data.get("JudgeReport", {}).get("events", [])
    with open(data_log_path, 'w') as f_dl:
        for ev in events:
            f_dl.write(f"{ev}\n")
    with open(fr_data_path, 'w') as f_fr:
        json.dump(events, f_fr, indent=4)
    print(f"Generated {data_log_path} and {fr_data_path}")

    # 4. Copy static assets (iss.urdf, textures)
    urdf_src = os.path.join(REPO_ROOT, "docker/ros_overlay/src/platform_sim/description/urdf/iss.urdf")
    if os.path.exists(urdf_src):
        shutil.copy2(urdf_src, os.path.join(output_dir, "iss.urdf"))
    
    tex_src = os.path.join(REPO_ROOT, "docker/ros_overlay/src/platform_sim/krpc_description/description/media/ib2_iss/media/materials/textures")
    if os.path.exists(tex_src):
        for img in glob.glob(os.path.join(tex_src, "ARmarker*.png")):
            shutil.copy2(img, tex_dir)

    # 5. Extract simulation.bag and telemetry if bag zip exists
    if os.path.exists(bag_zip_path):
        print(f"Extracting rosbag from {os.path.basename(bag_zip_path)}...")
        bag_path = os.path.join(output_dir, "simulation.bag")
        if not os.path.exists(bag_path):
            with zipfile.ZipFile(bag_zip_path, 'r') as bz:
                bz.extract("simulation.bag", output_dir)
        
        # Run Docker extraction for localization_history, judge.log, and ib2.log
        print("Extracting flight telemetry & ROS logs via docker...")
        docker_cmd = [
            "docker", "run", "--rm",
            "-v", f"{output_dir}:/work",
            "krpc/intball-simulator:latest",
            "bash", "-c",
            """
            source /opt/ros/melodic/setup.bash
            python -c "
import rosbag, json
bag = rosbag.Bag('/work/simulation.bag')

# 1. localization_history.txt (10 Hz nav) & ib2.log (1 Hz nav)
with open('/work/judge_module_logs/localization_history.txt', 'w') as f_loc, \\
     open('/work/flight_recorder_logs/ib2.log', 'w') as f_ib2:
    last_ib2_s = -1.0
    for topic, msg, t in bag.read_messages(topics=['/sensor_fusion/navigation']):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        f_loc.write(json.dumps({'position': [p.x, p.y, p.z], 'orientation': [o.x, o.y, o.z, o.w]}) + '\\n')
        sec = t.to_sec()
        if sec - last_ib2_s >= 1.0:
            last_ib2_s = sec
            f_ib2.write('t=%.1f x:%.4f y:%.4f z:%.4f\\n' % (sec, p.x, p.y, p.z))

# 2. judge.log and status_indicator.log
with open('/work/judge_module_logs/judge.log', 'w') as f_judge, \\
     open('/work/judge_module_logs/status_indicator.log', 'w') as f_si:
    for topic, msg, t in bag.read_messages(topics=['/rosout']):
        line = '[%s] [%.3f]: %s\\n' % (msg.name, msg.header.stamp.to_sec(), msg.msg)
        if 'judge' in msg.name:
            f_judge.write(line)
        elif 'status_indicator' in msg.name:
            f_si.write(line)
bag.close()
"
            """
        ]
        res = subprocess.run(docker_cmd, capture_output=True, text=True)
        if res.returncode == 0:
            print("Successfully extracted telemetry (localization_history.txt, judge.log, status_indicator.log, ib2.log)!")
        else:
            print(f"Warning: Telemetry extraction error: {res.stderr}")

    print(f"Reconstructed: {output_dir}")
    return True

if __name__ == "__main__":
    if len(sys.argv) > 1:
        for arg in sys.argv[1:]:
            if arg.endswith("__results.zip"):
                reconstruct_single_run(arg)
            elif os.path.isdir(arg):
                for rz in sorted(glob.glob(os.path.join(arg, "*__results.zip"))):
                    reconstruct_single_run(rz)
    else:
        print("Usage: reconstruct_web_logs.py <path_to_results.zip or directory>")
