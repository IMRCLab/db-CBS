import shutil
import subprocess
import time
import shutil
import tempfile
from pathlib import Path
import sys
import os
import yaml
# import msgpack


sys.path.append(os.getcwd())


def run_dbcbs(filename_env, folder, timelimit, cfg):
    with tempfile.TemporaryDirectory() as tmpdirname:
        p = Path("../results/")
        filename_motions = p / "dbg/motions.msgpack"
        filename_stats = "{}/stats.yaml".format(folder)
        start = time.time()
        dt = 0.1
        duration_dbcbs = 0
        with open(filename_stats, 'w') as stats:
            stats.write("stats:\n")
            
            filename_result_dbcbs = Path(folder) / "result_dbcbs.yaml"
            filename_result_dbcbs_joint = Path(folder) / "result_dbcbs_joint.yaml"
            filename_result_dbcbs_opt = Path(folder) / "result_dbcbs_opt.yaml"
            t_dbcbs_start = time.time()
            result = subprocess.run(["./db_cbs", 
                "-i", filename_env,
                "-m", filename_motions,
                "-o", filename_result_dbcbs,
                "--joint", filename_result_dbcbs_joint,
                "--opt", filename_result_dbcbs_opt])
            t_dbcbs_stop = time.time()
            duration_dbcbs += t_dbcbs_stop - t_dbcbs_start
            if result.returncode != 0:
                print("db-cbs failed")
            else:
                # shutil.copyfile(filename_result_dbcbs_opt, "{}/result_dbcbs_opt.yaml".format(folder))
                with open(filename_result_dbcbs_opt) as f:
                    result = yaml.safe_load(f)
                    cost = result["cost"] # cost*2
                now = time.time()
                t = now - start
                print("success!", cost, t)
                stats.write("  - t: {}\n".format(t))
                stats.write("    cost: {}\n".format(cost))
                stats.write("    duration_dbcbs: {}\n".format(duration_dbcbs))
                stats.flush()



