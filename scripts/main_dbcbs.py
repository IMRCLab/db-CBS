import subprocess
import shutil
import tempfile
from pathlib import Path
import sys
import os
import yaml

sys.path.append(os.getcwd())


def run_dbcbs(filename_env, folder, timelimit, cfg):
    with tempfile.TemporaryDirectory() as tmpdirname:
        p = Path(tmpdirname)
        filename_cfg = p / "cfg.yaml"
        with open(filename_cfg, 'w') as f:
            yaml.dump(cfg, f, Dumper=yaml.CSafeDumper)

        print(filename_env)
        filename_stats = "{}/stats.yaml".format(folder)
        filename_result_dbcbs = Path(folder) / "result_dbcbs.yaml"
        filename_result_dbcbs_opt = Path(folder) / "result_dbcbs_opt.yaml"

        cmd = ["./db_cbs", 
            "-i", filename_env,
            "-o", filename_result_dbcbs,
            "--opt", filename_result_dbcbs_opt,
            "--stats", filename_stats,
            "--cfg", str(filename_cfg),
            "-t", str(1e6)]
        print(subprocess.list2cmdline(cmd))
        try:
            with open("{}/log.txt".format(folder), 'w') as logfile:
                result = subprocess.run(cmd, timeout=timelimit, stdout=logfile, stderr=logfile)
        except subprocess.TimeoutExpired as e:
            print(f"Command timed out after {timelimit} seconds.")
            cmd_str = " ".join(str(arg) for arg in cmd)
            subprocess.run(["pkill", "-f", " ".join(cmd_str)])

            filename_stats = f"{folder}/stats.yaml"
            try:
                with open(filename_stats, 'r') as file:
                    stats = yaml.safe_load(file)
                    if stats and "stats" in stats and len(stats["stats"]):
                        print("db-cbs succeeded!")
                    else:
                        print("db-cbs failed!")
            except FileNotFoundError:
                print(f"Stats file {filename_stats} not found.")

        except Exception as e:
            print(f"An unexpected error occurred: {e}")




