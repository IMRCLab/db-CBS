import yaml
from main_ompl import run_ompl
from main_dbcbs import run_dbcbs
from main_dbecbs import run_dbecbs
from pathlib import Path
import shutil
import subprocess
from dataclasses import dataclass
import multiprocessing as mp
import tqdm
import psutil
# import checker
from benchmark_stats import export_table_txt


@dataclass
class ExecutionTask:
	"""Class for keeping track of an item in inventory."""
	# env: Path
	# cfg: Path
	# result_folder: Path
	instance: str
	alg: str
	trial: int
	timelimit: float

def run_visualize(script, filename_env, filename_result):
	subprocess.run(["python3",
				script,
				filename_env,
				"--result", filename_result,
				"--video", filename_result.with_suffix(".mp4")])
	
def run_checker(filename_env, filename_result, filename_log):
	with open(filename_log, 'w') as f:
		cmd = ["./dynoplan/dynobench/check_trajectory_multirobot",
					"--result_file", filename_result,
					"--env_file", filename_env,
					"--models_base_path" , "../dynoplan/dynobench/models/",
					"--goal_tol" , "999999"]
		print(subprocess.list2cmdline(cmd))
		out = subprocess.run(cmd,
					stdout=f, stderr=f)
	return out.returncode == 0

def run_search_visualize(script, filename_env, filename_trajs, filename_result):
	subprocess.run(["python3",
				script,
				filename_env,
				"--trajs", filename_trajs,
				"--result", filename_result,
				"--video", filename_result.with_suffix(".mp4")])

def execute_task(task: ExecutionTask):
	scripts_path = Path("../scripts")
	results_path = Path("../results")
	env_path = Path().resolve() / "../example"
	env = (env_path / task.instance).with_suffix(".yaml") 
	assert(env.is_file())

	cfg = env_path / "algorithms.yaml" # using single alg.yaml
	assert(cfg.is_file())

	with open(cfg) as f:
		cfg = yaml.safe_load(f)

	result_folder = results_path / task.instance / task.alg / "{:03d}".format(task.trial)
	if result_folder.exists():
			print("Warning! {} exists already. Deleting...".format(result_folder))
			shutil.rmtree(result_folder)
	result_folder.mkdir(parents=True, exist_ok=False)

	# find cfg
	mycfg = cfg[task.alg]
	mycfg = mycfg['default']
	profile = True
	# wildcard matching
	import fnmatch
	for k, v in cfg[task.alg].items():
		if fnmatch.fnmatch(Path(task.instance).name, k):
			mycfg = {**mycfg, **v} # merge two dictionaries

	if Path(task.instance).name in cfg[task.alg]:
		mycfg_instance = cfg[task.alg][Path(task.instance).name]
		mycfg = {**mycfg, **mycfg_instance} # merge two dictionaries

	print("Using configurations ", mycfg)

	if task.alg == "sst":
		run_ompl(str(env), str(result_folder), task.timelimit, mycfg)
		visualize_files = [p.name for p in result_folder.glob('result_*')]
		check_files = [p.name for p in result_folder.glob('result_*')]
	elif task.alg == "db-cbs":
		run_dbcbs(str(env), str(result_folder), task.timelimit, mycfg)
		visualize_files = [p.name for p in result_folder.glob('result_*')]
		check_files = [p.name for p in result_folder.glob('result_dbcbs_opt*')]
		search_plot_files = [p.name for p in result_folder.glob('expanded_trajs*')]
	elif task.alg == "db-ecbs":
		run_dbecbs(str(env), str(result_folder), task.timelimit, mycfg, profile)
		visualize_files = [p.name for p in result_folder.glob('result_*')]
		check_files = [p.name for p in result_folder.glob('result_dbecbs_opt*')]
	
	for file in check_files:
		if not run_checker(env, result_folder / file, (result_folder / file).with_suffix(".check.txt")):
			print("WARNING: CHECKER FAILED -> DELETING stats!")
			(result_folder / "stats.yaml").unlink(missing_ok=True)

	# vis_script = scripts_path / "visualize.py"
	# for file in visualize_files:
	# 	run_visualize(vis_script, env, result_folder / file)


def main():
	parallel = True
	instances = [
		# "swap4_unicycle",
		# "swap4_unicycle_sphere",
		# "swap4_double_integrator",
		# "swap4_trailer",
		# "swap4_unicycle2",
		# "alcove_unicycle",
		# "drone1c",
		"drone2c",
		# "drone3c",

	]

	algs = [
		# "db-cbs",
		"db-ecbs",
	]
	trials = 1
	timelimit = 5*60

	tasks = []
	for instance in instances:
		for alg in algs:
			for trial in range(trials):
				tasks.append(ExecutionTask(instance, alg, trial, timelimit))

	if parallel and len(tasks) > 1:
		use_cpus = psutil.cpu_count(logical=False)-1
		print("Using {} CPUs".format(use_cpus))
		with mp.Pool(use_cpus) as p:
			for _ in tqdm.tqdm(p.imap_unordered(execute_task, tasks)):
				pass
	else:
		for task in tasks:
			execute_task(task)
	
	export_table_txt(instances, algs)

if __name__ == '__main__':
	main()
