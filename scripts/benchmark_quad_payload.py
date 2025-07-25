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
    instance: str
    db_param: list
    trial: int
    timelimit: float

def run_controller(folder, reftrajectory, output, model_path, computeAcc=True, nocableTrack=False):
    
    subprocess.run(["python3",
				"../dynoplan/dynobench/example/test_quad3dpayload_n.py",
					"-cff", "-w",
					"--inp", folder / reftrajectory,
					"--out", folder / output,
					"--model_path", model_path,
				], env={"PYTHONPATH": "dynoplan/dynobench/:/home/khaledwahba94/imrc/payload-uavs-planner/coltrans-planning/deps/crazyflie-firmware"}, check=True)
    

def run_checker(filename_env, filename_result, filename_log):
	with open(filename_log, 'w') as f:
		cmd = ["./dynoplan/dynobench/check_trajectory",
					"--result_file", filename_result,
					"--env_file", filename_env,
					"--models_base_path" , "../dynoplan/dynobench/models/",
					"--goal_tol" , "999",
					"--u_bound_tol", "0.101",
					"--col_tol", "0.01"]
		print(subprocess.list2cmdline(cmd))
		out = subprocess.run(cmd,
					stdout=f, stderr=f)
	return out.returncode == 0



def run_optimization(result_folder, filename_init, filename_env , result, timelimit):
    try: 
        with open("{}/log_opt.txt".format(str(result_folder)), 'w') as logfile:
            print("init_file: ", filename_init)
            print("env_file: ", filename_env)
            subprocess.run(["./dynoplan/main_optimization",
                "--init_file", str(filename_init),
                "--env_file", str(filename_env),
                "--models_base_path", "../dynoplan/dynobench/models/",
                "--solver_id", "1",
                "--weight_goal", "200",
                # "--time_weight", "-0.01",
                # "--time_ref", "1.5",
                "--max_iter", "50",
                "--results_file", str(result),
                "--collision_weight", "500."],
            stdout=logfile, stderr=logfile, timeout=15*60, check=True)
            return True
    except subprocess.CalledProcessError as e:
        print(f"Error: {e}")
        return False

def visualize_payload(filename_env, output, opt_success=True, reference_traj=None, visualize_controller_out=False):
    output_html = output.with_suffix(".html")

    if visualize_controller_out:
        subprocess.run(["python3",
            "../scripts/visualize_payload.py",
            "--env", str(filename_env),
            "--robot", "point",
            "--result", output,
            "--output", output_html,
			"--ref", str(reference_traj)],
            check=True)
    else:
        subprocess.run(["python3",
            "../scripts/visualize_payload.py",    
			"--env", str(filename_env),
		 	"--robot", "point",
			"--result", output,
			 "--output", output_html,
		 ], check=True)

def visualize_unicycles(filename_env, output,  reference_traj=None, visualize_controller_out=False):
    output_html = output.with_suffix(".html")
    if visualize_controller_out:
        subprocess.run(["python3",
            "../scripts/visualize_unicycles.py",
            "--env", str(filename_env),
            "--robot", "unicycle",
            "--ref", reference_traj,
            "--result", str(output),
            "--output", output_html],
            check=True)
    else:
        subprocess.run(["python3",
            "../scripts/visualize_unicycles.py",    
			"--env", str(filename_env),
		 	"--robot", "unicycle",
			"--result", output,
			 "--output", output_html,
		 ], check=True)


def generate_init_guess(script, path_to_env, path_to_dbcbs, path_to_result,  path_to_payload, num_robots):
	subprocess.run(["python3",
        script,
        "--joint_robot_env", path_to_env,
        "--dbcbs", path_to_dbcbs,
        "--output", path_to_result.with_suffix(".html"),
        "--result", path_to_result.with_suffix(".yaml"),
        "--payload", path_to_payload,
        "--num_robots", str(num_robots)])


def run_unicycles_controller(folder, reftrajectory, output, model_path):
	try:
		subprocess.run(["python3",
					"../dynoplan/dynobench/example/unicycle_sim.py",
					"-w",
					"--inp", folder / reftrajectory,
					"--out", folder / output,
					"--model_path", model_path,
					], env={"PYTHONPATH": "dynoplan/dynobench"}, check=True)
	except Exception as e:
		print(e)


def run_visualize(script, filename_env, filename_result, path_to_payload=None):
    if path_to_payload is not None:
        subprocess.run(["python3",
            script,
            "--env", filename_env,
            "--result", filename_result,
            "--payload", path_to_payload,
            "--video", filename_result.with_suffix(".html")])
    else: 
        subprocess.run(["python3",
            script,
            "--env", filename_env,
            "--result", filename_result,
            "--video", filename_result.with_suffix(".html")])

def run_dbcbs(filename_env, folder, task, cfg):
    timelimit = task.timelimit
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
        payload_cfg = cfg["payload"]
        with open(filename_stats, 'w') as stats:
            stats.write("stats:\n")
            
            filename_result_dbcbs = Path(folder) / f"result_dbcbs.yaml"
            filename_result_dbcbs_joint = Path(folder) / "dbcbs_joint.yaml"
            filename_result_dbcbs_opt = Path(folder) / "result_dbcbs_opt.yaml"
            t_dbcbs_start = time.time()
            cmd = ["./db_cbs", 
                "-i", filename_env,
                "-o", filename_result_dbcbs,
                "--optimization", filename_result_dbcbs_opt,
                "--cfg", str(filename_cfg),
                "-t", str(timelimit*1000)] # -t is in milliseconds [ms]
            print(subprocess.list2cmdline(cmd))
            try:
                with open("{}/log_dbcbs.txt".format(folder), 'w') as logfile:
                    result = subprocess.run(cmd, timeout=timelimit, stdout=logfile, stderr=logfile)
                t_dbcbs_stop = time.time()
                duration_dbcbs += t_dbcbs_stop - t_dbcbs_start
                with open(filename_result_dbcbs, "r") as f:
                    results_dbcbs = yaml.load(f,Loader=yaml.CSafeLoader)

                if result.returncode != 0 and results_dbcbs["result"][0]["states"] is None:
                    print("db-cbs failed ", result.returncode)
                
                else:
                    
                    cost = results_dbcbs["cost"]
                    expansions = results_dbcbs["expansions"]
                    now = time.time()
                    t = now - start
                    print("success!", t, ", instance:", task.instance["name"], " trial: ", task.trial)                    
                    stats.write("  - duration_dbcbs: {}\n".format(t))
                    stats.write("    delta_0: {}\n".format(delta))
                    stats.write("    delta_rate: {}\n".format(delta_rate))
                    stats.write("    payload_cfg: {}\n".format(payload_cfg)) 
                    stats.write("    cost: {}\n".format(cost))
                    stats.write("    expansions: {}\n".format(expansions))
                    stats.flush()
                    return True
            except:
                with open(filename_result_dbcbs, "r") as f:
                    results_dbcbs = yaml.load(f,Loader=yaml.CSafeLoader)

                if results_dbcbs["result"][0]["states"] is None:
                    print("db-cbs failed ", result.returncode)
                    print("Failure!")
                    return False
                else:
                    cost = results_dbcbs["cost"]
                    expansions = results_dbcbs["expansions"]
                    now = time.time()
                    t = now - start
                    print("success!", t, ", instance:", task.instance["name"], " trial: ", task.trial)                    
                    stats.write("  - duration_dbcbs: {}\n".format(t))
                    stats.write("    delta_0: {}\n".format(delta))
                    stats.write("    delta_rate: {}\n".format(delta_rate))
                    stats.write("    payload_cfg: {}\n".format(payload_cfg)) 
                    stats.write("    cost: {}\n".format(cost))
                    stats.write("    expansions: {}\n".format(expansions))
                    stats.flush()
                    return True
                    
def execute_task(task: ExecutionTask):
    scripts_path = Path("../scripts")
    results_path = Path("../stats_db")
    cfg_path = Path().resolve() / "../example"
    example_path = Path("../dynoplan/dynobench/envs")

    env_path = (example_path / "benchmark_planners/dbcbs" / task.instance["name"]).with_suffix(".yaml") 
    assert(env_path.is_file())

    cfg = cfg_path / "algorithms.yaml" # using single alg.yaml
    assert(cfg.is_file())

    with open(cfg) as f:
        cfg = yaml.safe_load(f)

    result_folder = results_path / task.instance["name"] / "{:03d}".format(task.trial)
    if result_folder.exists():
            print("Warning! {} exists already. Deleting...".format(result_folder))
            shutil.rmtree(result_folder)
    result_folder.mkdir(parents=True, exist_ok=False)

    # find cfg
    mycfg = cfg["db-cbs"]["default"]
    mycfg["delta_0"] = task.db_param["delta_0"]
    mycfg["delta_rate"] = task.db_param["delta_rate"]
    mycfg["num_primitives_0"] = task.db_param["num_primitives_0"]
    mycfg["num_primitives_rate"] = task.db_param["num_primitives_rate"]
    mycfg["heuristic1"] = task.db_param["heuristic1"]
    mycfg["payload"] = task.db_param["payload"]
    # wildcard matching
    import fnmatch
    for k, v in mycfg.items():
        if fnmatch.fnmatch(Path(task.instance["name"]).name, k):
            mycfg = {**mycfg, **v} # merge two dictionaries

    if Path(task.instance["name"]).name in mycfg:
        mycfg_instance = cfg[task.alg][Path(task.instance["name"]).name]
        mycfg = {**mycfg, **mycfg_instance} # merge two dictionaries
    print("Using configurations ", mycfg)
    print("---------------------------------------")
    print("Running db-CBS......")
    if(run_dbcbs(str(env_path), str(result_folder), task, mycfg)):
        print("Visualizing db-CBS solution......")
        vis_script = scripts_path / "mesh_visualizer.py"
        path_to_dbcbs_result =  result_folder / "result_dbcbs.yaml"
        path_to_payload = result_folder / "result_dbcbs_payload.yaml"
        path_to_unicycles = result_folder / "result_dbcbs_unicycles_dummy.yaml"
        if (path_to_payload.exists() or path_to_unicycles.exists()):

            with open(env_path) as f:
                env_dict = yaml.safe_load(f)

            env_joint_robot_path = result_folder / "env.yaml"

            robot_type = env_dict["joint_robot"][0]["type"]

        

            print("Visualize initial guess...")
            if "point" in robot_type:
                run_visualize(vis_script, env_path, path_to_dbcbs_result, path_to_payload)
                visualize_payload(str(env_joint_robot_path), result_folder / "init_guess_payload.yaml", opt_success=False)
                print("Running the controller......\n")
                run_controller(result_folder, "result_dbcbs_opt.yaml", "trajectory_opt.yaml", "../dynoplan/dynobench/models/" + task.instance["model"])
                print("Visualizing the controller output......")
                visualize_payload(str(env_joint_robot_path), result_folder / "trajectory_opt.yaml", reference_traj=result_folder / "result_dbcbs_opt.yaml", visualize_controller_out=True)   
            
            if "unicycle" in robot_type:
                
                run_visualize(vis_script, env_path, path_to_dbcbs_result, path_to_payload=None)
                visualize_unicycles(str(env_joint_robot_path), result_folder / "init_guess_unicycles.yaml", visualize_controller_out=False)
                print("Running the controller......\n")
                run_unicycles_controller(result_folder, "result_dbcbs_opt.yaml", "trajectory_opt.yaml", "../dynoplan/dynobench/models/" + task.instance["model"])
                print("Visualizing the controller output......")
                
                visualize_unicycles(str(env_joint_robot_path), result_folder / "trajectory_opt.yaml", reference_traj=result_folder / "result_dbcbs_opt.yaml", visualize_controller_out=True)
            run_checker(str(env_joint_robot_path), result_folder / "trajectory_opt.yaml", (result_folder / "trajectory_opt.yaml").with_suffix(".check.txt"))
        else: 
            run_visualize(vis_script, env_path, path_to_dbcbs_result)
    else: 
        print(f"db-cbs failed in {task.instance['name']}, trial {task.trial}")

def main():
    parallel = True
    instances = [
        {"name": "window_2robots", "model": "point_2.yaml"},
        {"name": "window_3robots", "model": "point_3.yaml"},
        {"name": "window_4robots", "model": "point_4.yaml"},
        {"name": "window_5robots", "model": "point_5.yaml"},
        {"name": "window_6robots", "model": "point_6.yaml"},
       
        {"name": "forest_2robots", "model": "point_2.yaml"},
        {"name": "forest_3robots", "model": "point_3.yaml"},
        {"name": "forest_4robots", "model": "point_4.yaml"},
        {"name": "forest_5robots", "model": "point_5.yaml"},
        {"name": "forest_6robots", "model": "point_6.yaml"},
 
        {"name": "window_2robots_unicycle", "model": "unicyclesWithRods_2.yaml"},
        {"name": "window_3robots_unicycle", "model": "unicyclesWithRods_3.yaml"},
        {"name": "window_4robots_unicycle", "model": "unicyclesWithRods_4.yaml"},
        {"name": "window_5robots_unicycle", "model": "unicyclesWithRods_5.yaml"},
        {"name": "window_6robots_unicycle", "model": "unicyclesWithRods_6.yaml"},

        {"name": "forest_2robots_unicycle", "model": "unicyclesWithRods_2.yaml"},
        {"name": "forest_3robots_unicycle", "model": "unicyclesWithRods_3.yaml"},
        {"name": "forest_4robots_unicycle", "model": "unicyclesWithRods_4.yaml"},
        {"name": "forest_5robots_unicycle", "model": "unicyclesWithRods_5.yaml"},
        {"name": "forest_6robots_unicycle", "model": "unicyclesWithRods_6.yaml"},

        # {"name": "lego_2robots_unicycle", "model": "unicyclesWithRods_2_big.yaml"},
        # {"name": "lego_3robots_unicycle", "model": "unicyclesWithRods_3.yaml"},
    ]

    db_params = [    
        {"delta_0": 0.9, "delta_rate": 0.9, "num_primitives_0": 1000, "num_primitives_rate": 2., "heuristic1": "no-reverse-search", "payload": {"solve_p0": True, "anytime": False, "p0_init_guess": [-1.0,0,0],  "tol":0.9}}, # window_2robots
        {"delta_0": 0.9, "delta_rate": 0.9, "num_primitives_0": 1000, "num_primitives_rate": 1.2, "heuristic1": "no-reverse-search", "payload": {"solve_p0": True, "anytime": False, "p0_init_guess": [-1.0,0,0],  "tol":0.9}}, # window_3robots
        {"delta_0": 0.9, "delta_rate": 0.95, "num_primitives_0": 5000, "num_primitives_rate": 1.2, "heuristic1": "no-reverse-search", "payload": {"solve_p0": True, "anytime": False, "p0_init_guess": [-1.0,0,0],  "tol":0.9}}, # window_4robots
        {"delta_0": 0.9,  "delta_rate": 0.9, "num_primitives_0": 1000, "num_primitives_rate": 1.5, "heuristic1": "no-reverse-search", "payload": {"solve_p0": True, "anytime": False, "p0_init_guess": [-1.0,0,0],  "tol":0.9}}, # window_5robots
        {"delta_0": 0.9,  "delta_rate": 0.9, "num_primitives_0": 6000, "num_primitives_rate": 1.5, "heuristic1": "no-reverse-search", "payload": {"solve_p0": True, "anytime": False, "p0_init_guess": [-1.0,0,0],  "tol":0.9}}, # window_6robots
        
        {"delta_0": 0.9, "delta_rate": 0.9, "num_primitives_0": 1000, "num_primitives_rate": 1.5, "heuristic1": "no-reverse-search", "payload": {"solve_p0": True, "anytime": False, "p0_init_guess": [-0.5,0,0],  "tol":0.9}}, # forest_2robots
        {"delta_0": 0.9, "delta_rate": 0.9, "num_primitives_0": 1000, "num_primitives_rate": 1.5, "heuristic1": "no-reverse-search", "payload": {"solve_p0": True, "anytime": False, "p0_init_guess": [-0.5,0,0],  "tol":0.9}}, # forest_3robots
        {"delta_0": 0.9, "delta_rate": 0.9, "num_primitives_0": 6000, "num_primitives_rate": 1.5, "heuristic1": "no-reverse-search", "payload": {"solve_p0": True, "anytime": False, "p0_init_guess": [-0.5,0,0],  "tol":0.9}}, # forest_4robots
        {"delta_0": 0.9, "delta_rate": 0.9, "num_primitives_0": 6000, "num_primitives_rate": 1.5, "heuristic1": "no-reverse-search", "payload": {"solve_p0": True, "anytime": False, "p0_init_guess": [-0.5,0,0],  "tol":0.9}}, # forest_5robots
        {"delta_0": 0.9, "delta_rate": 0.9, "num_primitives_0": 1000, "num_primitives_rate": 1.5, "heuristic1": "no-reverse-search", "payload": {"solve_p0": True, "anytime": False, "p0_init_guess": [-0.5,0,0],  "tol":0.9}}, # forest_6robots
    
        {"delta_0": 0.3, "delta_rate": 0.9, "num_primitives_0": 100, "num_primitives_rate": 1.2, "heuristic1": "no-reverse-search", "payload": {"solve_p0": True, "anytime": True, "tol": 0.3}},  # window_2robots_unicycle
        {"delta_0": 0.3, "delta_rate": 0.9, "num_primitives_0": 100, "num_primitives_rate": 1.2, "heuristic1": "no-reverse-search", "payload": {"solve_p0": True, "anytime": False, "tol": 0.3}},  # window_2robots_unicycle
        {"delta_0": 0.3, "delta_rate": 0.9, "num_primitives_0": 100, "num_primitives_rate": 1.2, "heuristic1": "no-reverse-search", "payload": {"solve_p0": True, "anytime": False, "tol": 0.3}},  # window_3robots_unicycle
        {"delta_0": 0.3, "delta_rate": 0.9, "num_primitives_0": 100, "num_primitives_rate": 1.2, "heuristic1": "no-reverse-search", "payload": {"solve_p0": True, "anytime": False, "tol": 0.3}},  # window_4robots_unicycle
        {"delta_0": 0.3, "delta_rate": 0.9, "num_primitives_0": 100, "num_primitives_rate": 1.2, "heuristic1": "no-reverse-search", "payload": {"solve_p0": True, "anytime": False, "tol": 0.3}},  # window_5robots_unicycle
        {"delta_0": 0.3, "delta_rate": 0.9, "num_primitives_0": 100, "num_primitives_rate": 1.2, "heuristic1": "no-reverse-search", "payload": {"solve_p0": True, "anytime": False, "tol": 0.3}}, # window_6robots_unicycle
        
        {"delta_0": 0.3, "delta_rate": 0.9, "num_primitives_0": 100, "num_primitives_rate": 1.5, "heuristic1": "no-reverse-search", "payload": {"solve_p0": True, "anytime": False,  "tol": 0.3}}, # forest_2robots_unicycle
        {"delta_0": 0.3, "delta_rate": 0.9, "num_primitives_0": 100, "num_primitives_rate": 1.5, "heuristic1": "no-reverse-search", "payload": {"solve_p0": True, "anytime": False,  "tol": 0.3}},  # forest_3robots_unicycle
        {"delta_0": 0.3, "delta_rate": 0.9, "num_primitives_0": 100, "num_primitives_rate": 1.5, "heuristic1": "no-reverse-search", "payload": {"solve_p0": True, "anytime": False,  "tol": 0.3}},  # forest_4robots_unicycle
        {"delta_0": 0.3, "delta_rate": 0.9, "num_primitives_0": 100, "num_primitives_rate": 1.5, "heuristic1": "no-reverse-search", "payload": {"solve_p0": True, "anytime": False,  "tol": 0.3}},  # forest_5robots_unicycle
        {"delta_0": 0.3, "delta_rate": 0.9, "num_primitives_0": 100, "num_primitives_rate": 1.5, "heuristic1": "no-reverse-search", "payload": {"solve_p0": True, "anytime": False,  "tol": 0.3}},  # forest_6robots_unicycle
        
        {"delta_0": 0.3, "delta_rate": 0.99, "num_primitives_0": 500, "num_primitives_rate": 2.2, "heuristic1": "no-reverse-search", "payload": {"solve_p0": True, "anytime": False,  "tol": 0.3}},  # window_2robots_unicycle
        {"delta_0": 0.3, "delta_rate": 0.99, "num_primitives_0": 500, "num_primitives_rate": 2.2, "heuristic1": "no-reverse-search", "payload": {"solve_p0": True, "anytime": False,  "tol": 0.3}},  # window_2robots_unicycle


        ############################# Working Params ############################################
        # {"delta_0": 0.15, "delta_rate": 0.9, "num_primitives_0": 500, "num_primitives_rate": 1.5, "heuristic1": "no-reverse-search", "payload": {"solve_p0": True, "anytime": False,  "tol": 0.25}},  # window_2robots_unicycle
        # {"delta_0": 0.25, "delta_rate": 0.9, "num_primitives_0": 500, "num_primitives_rate": 1.5, "heuristic1": "no-reverse-search", "payload": {"solve_p0": True, "anytime": False,  "tol": 0.35}},  # window_3robots_unicycle
        # {"delta_0": 0.3, "delta_rate": 0.9, "num_primitives_0": 1000,  "num_primitives_rate": 1.5, "heuristic1": "no-reverse-search", "payload": {"solve_p0": True, "anytime": False,  "tol": 0.35}},  # window_4robots_unicycle
        # {"delta_0": 0.3, "delta_rate": 0.9, "num_primitives_0": 1000,  "num_primitives_rate": 1.5, "heuristic1": "no-reverse-search", "payload": {"solve_p0": True, "anytime": False,  "tol": 0.35}},  # window_5robots_unicycle
        # {"delta_0": 0.3, "delta_rate": 0.9, "num_primitives_0": 1000, "num_primitives_rate": 1.5, "heuristic1": "no-reverse-search", "payload": {"solve_p0": True, "anytime": False,  "tol": 0.35}}, # window_6robots_unicycle
        
        # {"delta_0": 0.25, "delta_rate": 0.9, "num_primitives_0": 5000, "num_primitives_rate": 1.5, "heuristic1": "no-reverse-search", "payload": {"solve_p0": True, "anytime": False,  "tol": 0.3}}, # forest_2robots_unicycle
        # {"delta_0": 0.25, "delta_rate": 0.9, "num_primitives_0": 5000, "num_primitives_rate": 1.5, "heuristic1": "no-reverse-search", "payload": {"solve_p0": True, "anytime": False,  "tol": 0.3}},  # forest_3robots_unicycle
        # {"delta_0": 0.25, "delta_rate": 0.9, "num_primitives_0": 5000, "num_primitives_rate": 1.5, "heuristic1": "no-reverse-search", "payload": {"solve_p0": True, "anytime": False,  "tol": 0.3}},  # forest_4robots_unicycle
        # {"delta_0": 0.25, "delta_rate": 0.9, "num_primitives_0": 5000, "num_primitives_rate": 1.5, "heuristic1": "no-reverse-search", "payload": {"solve_p0": True, "anytime": False,  "tol": 0.3}},  # forest_5robots_unicycle
        # {"delta_0": 0.25, "delta_rate": 0.9, "num_primitives_0": 5000, "num_primitives_rate": 1.5, "heuristic1": "no-reverse-search", "payload": {"solve_p0": True, "anytime": False,  "tol": 0.3}},  # forest_6robots_unicycle

        # {"delta_0": 0.15, "delta_rate": 0.9, "num_primitives_0": 600, "num_primitives_rate": 1.5, "heuristic1": "no-reverse-search", "payload": {"solve_p0": True, "anytime": False,  "tol": 0.25}}, # lego_3_robots_unicycle

    ] 


    trials = 1
    timelimit = 350 # [s]
    tasks = []
    for instance, db in zip(instances, db_params):
        for trial in range(trials):
            tasks.append(ExecutionTask(instance, db, trial, timelimit))

    if parallel and len(tasks) > 1:
        use_cpus = psutil.cpu_count(logical=False) - 1
        print("Using {} CPUs".format(use_cpus))
        with mp.Pool(use_cpus) as p:
            for _ in tqdm.tqdm(p.imap_unordered(execute_task, tasks)):
                pass
    else:
        for task in tasks:
            execute_task(task)



if __name__ == '__main__':
    main()