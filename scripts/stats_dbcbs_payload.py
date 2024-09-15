import yaml
from pathlib import Path
import shutil
from dataclasses import dataclass
import subprocess
import multiprocessing as mp
import tqdm
import psutil
import tempfile
import time


@dataclass
class ExecutionTask:
    """Class for keeping track of an item in inventory."""
    # env: Path
    # cfg: Path
    # result_folder: Path
    instance: str
    db: list
    trial: int
    timelimit: float


def run_optimization(result_folder, filename_init, filename_env , result, timelimit):
    try: 
        with open("{}/log_opt.txt".format(str(result_folder)), 'w') as logfile:
            subprocess.run(["./dynoplan/main_optimization",
                "--init_file", str(filename_init),
                "--env_file", str(filename_env),
                "--models_base_path", "../dynoplan/dynobench/models/",
                "--results_file", str(result),
                "--weight_goal", "800"],
            stdout=logfile, stderr=logfile, timeout=timelimit, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error: {e}")

def visualize_dintCables(filename_env, output):
    print(output.with_suffix(".trajopt.yaml"))
    subprocess.run(["python3",
        "../scripts/visualize_dintCables.py",
        "--env", str(filename_env),
        "--robot", "DintegratorCables",
        "--result", output.with_suffix(".trajopt.yaml"),
        "--output", output.with_suffix(".html")],
        check=True)

def generate_init_guess(script, path_to_env, path_to_dbcbs, path_to_result,  path_to_payload):
	subprocess.run(["python3",
				script,
				"--env", path_to_env,
				"--dbcbs", path_to_dbcbs,
				"--output", path_to_result.with_suffix(".html"),
                "--result", path_to_result.with_suffix(".yaml"),
                "--payload", path_to_payload])


def run_visualize(script, filename_env, filename_result):
	subprocess.run(["python3",
				script,
				filename_env,
				"--result", filename_result,
				"--video", filename_result.with_suffix(".mp4")])


def run_dbcbs(filename_env, folder, timelimit, cfg):
    with tempfile.TemporaryDirectory() as tmpdirname:
        p = Path(tmpdirname)
        filename_cfg = p / "cfg.yaml"
        with open(filename_cfg, 'w') as f:
            yaml.dump(cfg, f, Dumper=yaml.CSafeDumper)
        filename_stats = "{}/stats.yaml".format(folder)
        start = time.time()
        duration_dbcbs = 0	
        delta = cfg["delta_0"]	
        delta_rate = cfg["delta_rate"]	

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
                "-c", str(filename_cfg)]
            print(subprocess.list2cmdline(cmd))
            try:
                with open("{}/log.txt".format(folder), 'w') as logfile:
                    result = subprocess.run(cmd, timeout=timelimit, stdout=logfile, stderr=logfile)
                t_dbcbs_stop = time.time()
                duration_dbcbs += t_dbcbs_stop - t_dbcbs_start
                if result.returncode != 0:
                    print("db-cbs failed ", result.returncode)
                else:
                    with open(filename_result_dbcbs, "r") as f:
                        results_dbcbs = yaml.load(f,Loader=yaml.CSafeLoader)
                    
                    cost = results_dbcbs["cost"]
                    expansions = results_dbcbs["expansions"]
                    now = time.time()
                    t = now - start
                    print("success!", t)                    
                    stats.write("  - duration_dbcbs: {}\n".format(t))
                    stats.write("    delta_0: {}\n".format(delta))
                    stats.write("    delta_rate: {}\n".format(delta_rate)) 
                    stats.write("    cost: {}\n".format(cost))
                    stats.write("    expansions: {}\n".format(expansions))
                    stats.flush()
            except:
                print("Failure!")


def execute_task(task: ExecutionTask):
    scripts_path = Path("../scripts")
    results_path = Path("../stats_db")
    env_path = Path().resolve() / "../example"
    env = (env_path / "payload_examples" / task.instance).with_suffix(".yaml") 
    assert(env.is_file())

    cfg = env_path / "algorithms.yaml" # using single alg.yaml
    assert(cfg.is_file())

    with open(cfg) as f:
        cfg = yaml.safe_load(f)

    result_folder = results_path / task.instance / str(task.db[0]) / "{:03d}".format(task.trial)
    if result_folder.exists():
            print("Warning! {} exists already. Deleting...".format(result_folder))
            shutil.rmtree(result_folder)
    result_folder.mkdir(parents=True, exist_ok=False)

    # find cfg
    mycfg = cfg["db-cbs"]["default"]
    mycfg["delta_0"] = task.db[0]
    mycfg["delta_rate"] = task.db[1]
    # wildcard matching
    import fnmatch
    for k, v in mycfg.items():
        if fnmatch.fnmatch(Path(task.instance).name, k):
            mycfg = {**mycfg, **v} # merge two dictionaries

    if Path(task.instance).name in mycfg:
        mycfg_instance = cfg[task.alg][Path(task.instance).name]
        mycfg = {**mycfg, **mycfg_instance} # merge two dictionaries
    print("Using configurations ", mycfg)
    print("---------------------------------------")
    print("Running db-CBS......")
    run_dbcbs(str(env), str(result_folder), task.timelimit, mycfg)
 
 
    print("Visualizing db-CBS solution......")
    vis_script = scripts_path / "visualize.py"
    run_visualize(vis_script, env, result_folder / "result_dbcbs.yaml")
    
    init_guess_script = scripts_path / "init_guess_cables.py"

    with open(env) as f:
        env_dict = yaml.safe_load(f)

    env_joint_robot = {"environment": env_dict["environment"], "robots": list()}
    env_joint_robot["robots"].append(env_dict["joint_robot"][0])

    env_joint_robot_path = result_folder / "env.yaml"

    with open(env_joint_robot_path, "w") as f:
        yaml.dump(env_joint_robot, f, default_flow_style=None)
    path_to_payload = result_folder / "result_dbcbs_payload.yaml"
    path_to_dbcbs = result_folder / "result_dbcbs.yaml"
    path_to_result = result_folder / "init_guess_cables"
    
    print("Generating initial guess from db-CBS solution and visualizing it......")
    generate_init_guess(init_guess_script, str(env_joint_robot_path), str(path_to_dbcbs), path_to_result, str(path_to_payload))
    

    print("Running optimization......")
    run_optimization(result_folder ,path_to_result.with_suffix(".yaml"), str(env_joint_robot_path), result_folder / "output", task.timelimit)

    print("Visualizing optimization solution......")
    visualize_dintCables(str(env_joint_robot_path), result_folder / "output")


def main():
    parallel = True
    instances = [
        "empty", 
        "window",
        "window_small"
    ]
    db_params = [
    # delta_0, delta_rate
        [0.9, 0.5],
        [0.8, 0.5],
        [0.7, 0.5],
        [0.6, 0.5],
        [0.5, 0.5],
        [0.4, 0.5],
        [0.35, 0.5],
    ] 

    trials = 1
    timelimit = 1000

    tasks = []
    for instance in instances:
        for db in db_params:
            for trial in range(trials):
                tasks.append(ExecutionTask(instance, db, trial, timelimit))

    if parallel and len(tasks) > 1:
        use_cpus = psutil.cpu_count(logical=False)-1
        print("Using {} CPUs".format(use_cpus))
        with mp.Pool(use_cpus) as p:
            for _ in tqdm.tqdm(p.imap_unordered(execute_task, tasks)):
                pass
    else:
        for task in tasks:
            execute_task(task)



if __name__ == '__main__':
    main()
