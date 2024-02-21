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
from benchmark_table import write_table
import paper_tables


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
		check_files = [p.name for p in result_folder.glob('result_dbcbs_opt*')]
	
	for file in check_files:
		if not run_checker(env, result_folder / file, (result_folder / file).with_suffix(".check.txt")):
			print("WARNING: CHECKER FAILED -> DELETING stats!")
			(result_folder / "stats.yaml").unlink(missing_ok=True)

	vis_script = scripts_path / "visualize.py"
	for file in visualize_files:
		run_visualize(vis_script, env, result_folder / file)


def main():
	parallel = True
	instances = [
		# 1 robot cases
		"swap1_unicycle",
		"swap1_unicycle_sphere",
		"swap1_trailer",
		"swap1_unicycle2",
		"swap1_double_integrator",
		# 2 robot cases
		"swap2_unicycle",
		"swap2_unicycle_sphere",
		"swap2_double_integrator",
		"swap2_trailer",
		"swap2_unicycle2",
		"swap2_hetero",
		"makespan_vs_soc_1",
		"makespan_vs_soc_0",
		"alcove_unicycle",
		"alcove_unicycle_sphere",
		"at_goal_unicycle",
		"at_goal_unicycle_sphere",
		# 3 robot cases
		"swap3_unicycle",
		"swap3_unicycle_sphere",
		"swap3_double_integrator",
		"swap3_trailer",
		"swap3_unicycle2",
		# 4 robot cases
		"swap4_unicycle",
		"swap4_unicycle_sphere",
		"swap4_double_integrator",
		"swap4_trailer",
		"swap4_unicycle2",

		# special test cases
		"infeasible_0",

		# windows cases
		# "window2_unicycle",
		# "window3_unicycle",
		"window4_unicycle",
		"window4_unicycle2",
		"window4_double_integrator",
		"window4_trailer",
		"window4_unicycle_sphere",

		#demo
		# "swap2_demo",
		# "swap4_demo",
		# "window4_demo",
	]

	#add random cases
	for kind in ["unicycle_sphere", "hetero"]:
		for n in [2,4,8]:
			for k in range(10):
				instances.append("gen_p10_n{}_{}_{}".format(n,k, kind))

	# instances = ["window4_unicycle_sphere"]

	algs = [
		# "sst",
		# "s2m2",
		# "k-cbs",
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
	
	run_benchmark_stats(instances, algs, trials, timelimit)

	write_table(instances, algs, Path("../results"), "table.pdf", trials, timelimit)

	subprocess.run(
		['pdftk',
		 Path("../results") / 'table.pdf',
		 Path("../results") / 'stats.pdf',
		 'cat', 'output',
		 Path("../results") / 'results.pdf'
		]
	)
	# delete temp files
	(Path("../results") / 'table.pdf').unlink()
	(Path("../results") / 'stats.pdf').unlink()

	paper_tables.write_table1(trials, timelimit)
	paper_tables.write_table2(trials, timelimit)
	paper_tables.write_table3(trials, timelimit)
	paper_tables.write_table4(trials, timelimit)
	paper_tables.write_table5(trials, timelimit)

if __name__ == '__main__':
	main()
