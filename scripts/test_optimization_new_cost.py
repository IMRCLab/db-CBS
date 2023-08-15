

import subprocess


build_cmd = [
    "make", "multirobot_optimization"]

run_cmd_new = [
    "./multirobot_optimization",
    "--env",
    "../example/straight.yaml",
    "--init",
    "../example/guess_indiv_straight.yaml",
    "--out",
    "buu.yaml",
    "--s",
    "1",
    ">",
    "/tmp/db_log.txt"]

run_cmd_old = [
    "./multirobot_optimization",
    "--env",
    "../example/straight.yaml",
    "--init",
    "../example/guess_indiv_straight.yaml",
    "--out",
    "buu.yaml",
    "--s",
    "0",
    ">",
    "/tmp/db_log.txt"]


visualize_cmd = [
    "python3",
    "../scripts/visualize.py",
    "../example/straight.yaml",
    "--result",
    "/tmp/check5.yaml",
    "--video",
    "straight_solution_optimized.mp4"]

print("new optimization")
for i in [build_cmd, run_cmd_new, visualize_cmd]:
    print("running cmd")
    print(' '.join(i))

    out = subprocess.run(i)
    assert out.returncode == 0


print("old optimization")
for i in [build_cmd, run_cmd_old, visualize_cmd]:
    print("running cmd")
    print(' '.join(i))

    out = subprocess.run(i)
    assert out.returncode == 0
