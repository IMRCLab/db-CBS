import shutil
import subprocess
import time
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
        duration_dbcbs = 0
            
        filename_result_dbcbs = Path(folder) / "result_dbcbs.yaml"
        filename_result_dbcbs_joint = Path(folder) / "dbcbs_joint.yaml"
        filename_result_dbcbs_opt = Path(folder) / "result_dbcbs_opt.yaml"
        filename_stats = Path(folder) / "stats.yaml"
        t_dbcbs_start = time.time()

        cmd = ["./db_cbs", 
            "-i", filename_env,
            "-o", filename_result_dbcbs,
            "--joint", filename_result_dbcbs_joint,
            "--opt", filename_result_dbcbs_opt,
            "--stats", filename_stats,
            "-c", str(filename_cfg)]
        print(subprocess.list2cmdline(cmd))
        try:
            with open("{}/log.txt".format(folder), 'w') as logfile:
                result = subprocess.run(cmd, timeout=timelimit, stdout=logfile, stderr=logfile)
        except:
            if Path(filename_result_dbcbs_opt).exists():
                t_dbcbs_stop = time.time()
                duration_dbcbs += t_dbcbs_stop - t_dbcbs_start
                cost = 0
                with open(filename_result_dbcbs_opt) as f:
                    result = yaml.safe_load(f)
                    for r in result["result"]:
                        cost += len(r["actions"]) * 0.1
                print("Success!")
            else:
                print("Failure!")



