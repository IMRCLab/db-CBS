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
        print(filename_env)
        filename_stats = "{}/stats.yaml".format(folder)
        start = time.time()
        duration_dbcbs = 0
        with open(filename_stats, 'w') as stats:
            stats.write("stats:\n")
            
            filename_result_dbcbs = Path(folder) / "result_dbcbs.yaml"
            filename_result_dbcbs_joint = Path(folder) / "dbcbs_joint.yaml"
            filename_result_dbcbs_opt = Path(folder) / "result_dbcbs_opt.yaml"
            t_dbcbs_start = time.time()

            cmd = ["./db_cbs", 
                "-i", filename_env,
                "-o", filename_result_dbcbs,
                "--joint", filename_result_dbcbs_joint,
                "--opt", filename_result_dbcbs_opt,
                "--delta", str(0.5)]
            print(subprocess.list2cmdline(cmd))
            result = subprocess.run(cmd)
            t_dbcbs_stop = time.time()
            duration_dbcbs += t_dbcbs_stop - t_dbcbs_start
            if result.returncode != 0:
                print("db-cbs failed")
            else:
                # shutil.copyfile(filename_result_dbcbs_opt, "{}/result_dbcbs_opt.yaml".format(folder))
                try:
                    cost = 0
                    with open(filename_result_dbcbs_opt) as f:
                        result = yaml.safe_load(f)
                        for r in result["result"]:
                            cost += len(r["actions"]) * 0.1

                    #     cost = result["cost"] # cost*2
                    now = time.time()
                    t = now - start
                    print("success!", cost, t)
                    stats.write("  - t: {}\n".format(t))
                    stats.write("    cost: {}\n".format(cost))
                    stats.write("    duration_dbcbs: {}\n".format(duration_dbcbs))
                    stats.flush()
                except:
                    print("Failure!")



