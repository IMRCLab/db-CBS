import yaml
from main_ompl import run_ompl
from main_s2m2 import run_s2m2
from main_kcbs import run_kcbs
from main_dbcbs import run_dbcbs
from pathlib import Path
import shutil
import subprocess
from dataclasses import dataclass
import multiprocessing as mp
import tqdm
import psutil
# import checker
from benchmark_stats import run_benchmark_stats


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


def execute_task(task: ExecutionTask):
	scripts_path = Path("../scripts")
	results_path = Path("../results")
	# tuning_path = Path("../tuning")
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
	if Path(task.instance).name in cfg[task.alg]:
		mycfg_instance = cfg[task.alg][Path(task.instance).name]
		mycfg = {**mycfg, **mycfg_instance} # merge two dictionaries

	print("Using configurations ", mycfg)

	if task.alg == "sst":
		run_ompl(str(env), str(result_folder), task.timelimit, mycfg)
		visualize_files = [p.name for p in result_folder.glob('result_*')]
		check_files = [p.name for p in result_folder.glob('result_*')]
	elif task.alg == "s2m2":
		run_s2m2(str(env), str(result_folder), task.timelimit, mycfg)
		visualize_files = [p.name for p in result_folder.glob('result_*')]
		check_files = [p.name for p in result_folder.glob('result_*')]
	elif task.alg == "k-cbs":
		run_kcbs(str(env), str(result_folder), task.timelimit, mycfg)
		visualize_files = [p.name for p in result_folder.glob('result_*')]
		check_files = [p.name for p in result_folder.glob('result_*')]
	elif task.alg == "db-cbs":
		run_dbcbs(str(env), str(result_folder), task.timelimit, mycfg)
		visualize_files = [p.name for p in result_folder.glob('result_*')]
		check_files = [p.name for p in result_folder.glob('result_*')]

	# for visualization
	vis_script = scripts_path / "visualize.py"
	for file in visualize_files:
		run_visualize(vis_script, env, result_folder / file)


def main():
	parallel = True
	instances = [
		"swap2_unicycle",
		"swap2_trailer",
		"alcove_unicycle",
		"makespan_vs_soc_0",
		"makespan_vs_soc_1",
		"infeasible_0",
		# "parallelpark",
		# "bugtrap",
        # "wall",
		# "swap",
		# "classic",
		# "alcove",
		# "alcove_hard",
		# "straight",
		# "swap_cars",
		# "swap2_hetero",
		# "swap4_unicycle",
	]
	algs = [
		"sst",
		"s2m2",
		"k-cbs",
		"db-cbs",
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
	
	run_benchmark_stats(instances,algs,trials)
	

if __name__ == '__main__':
	main()
