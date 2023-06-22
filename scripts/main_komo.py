import numpy as np
import yaml
import argparse
import tempfile
from pathlib import Path
import subprocess
import shutil
import time
import robots

from utils_optimization import UtilsSolutionFile
import translate_g

def _run_komo(filename_g, filename_env, filename_initial_guess, filename_result, filename_cfg, robot_type, N=-1):

	if "unicycle_first_order" in robot_type:
		order = 1
	elif "unicycle_second_order" in robot_type:
		order = 2
	elif "car_first_order_with_1_trailers" in robot_type:
		order = 1
	elif "quadrotor" in robot_type:
		order = 2
	else:
		raise "No known robot_type!"

	while True:
		# Run KOMO
		result = subprocess.run(["./main_rai",
				"-model", "\""+str(filename_g)+"\"",
				"-waypoints", "\""+str(filename_initial_guess)+"\"",
				"-N", str(N),
				"-display", str(0),
				"-animate", str(0),
				"-order", str(order),
				"-robot", robot_type,
				"-cfg", "\""+str(filename_cfg)+"\"",
				"-env", "\"" + str(filename_env)+"\"",
				"-out", "\""+str(filename_result)+"\""])
		# a negative returncode indicates an internal error -> repeat
		if result.returncode >= 0:
			break
	if result.returncode != 0:
		print("KOMO failed")
		return False
	else:
		return True

def run_komo(filename_env, filename_initial_guess, filename_result, cfg = ""):

	with tempfile.TemporaryDirectory() as tmpdirname:
		p = Path(tmpdirname)
		# p = Path("../results/dbg")

		with open(filename_env) as f:
			env = yaml.safe_load(f)
		robot_type = env["robots"][0]["type"]

		# convert environment YAML -> g
		filename_g = p / "env.g"
		translate_g.write(filename_env, str(filename_g))

		# write config file
		filename_cfg = p / "rai.cfg"
		with open(filename_cfg, 'w') as f:
			f.write(cfg)

		return _run_komo(filename_g, filename_env, filename_initial_guess, filename_result, filename_cfg, robot_type)


def run_komo_with_T_scaling(filename_env, filename_initial_guess, filename_result, cfg = "", max_T = None):

	with tempfile.TemporaryDirectory() as tmpdirname:
		p = Path(tmpdirname)
		# p = Path("../results/dbg")

		with open(filename_env) as f:
			env = yaml.safe_load(f)
		robot_type = env["robots"][0]["type"]

		# convert environment YAML -> g
		filename_g = p / "env.g"
		translate_g.write(filename_env, str(filename_g))

		# write config file
		filename_cfg = p / "rai.cfg"
		with open(filename_cfg, 'w') as f:
			f.write(cfg)

		# hack
		utils_sol_file = UtilsSolutionFile(robot_type)
		utils_sol_file.load(filename_initial_guess)
		if utils_sol_file.T() == 0:
			return False
		filename_modified_guess = p / "guess.yaml"

		# for factor in [1.0]:
		for factor in [0.8, 1.0, 1.2]:
			T = int(utils_sol_file.T() * factor)
			if max_T is not None and T > max_T:
				return False
			print("Trying T ", T)
			# utils_sol_file.save_rescaled(filename_modified_guess, int(utils_sol_file.T() * 1.1))
			if factor == 1.0:
				result = _run_komo(filename_g, filename_env, filename_initial_guess, filename_result, filename_cfg, robot_type)
			else:
				utils_sol_file.save_rescaled(filename_modified_guess, T)
				result = _run_komo(filename_g, filename_env, filename_modified_guess, filename_result, filename_cfg, robot_type)
			# shutil.copyfile(filename_modified_guess, filename_result)
			# return True
			if result:
				return True
		return False


def run_komo_standalone(filename_env, folder, timelimit, cfg = "",
		search = "binarySearch",
		initialguess = "ompl",
		T_range_rel=None,
		T_range_abs=None,
		use_T=None):

	# search = "linear"
	# search = "binarySearch"

	with tempfile.TemporaryDirectory() as tmpdirname:
		p = Path(tmpdirname)
		# p = Path("../results/test")

		start = time.time()

		with open(filename_env) as f:
			env = yaml.safe_load(f)
		robot_type = env["robots"][0]["type"]
		robot = robots.create_robot(robot_type)

		if "unicycle" in robot_type:
			robot_type_guess = "unicycle_first_order_0"
		elif "trailer" in robot_type:
			robot_type_guess = "car_first_order_with_1_trailers_0"
		elif "quadrotor" in robot_type:
			robot_type_guess = "none" # define a generic SE(3) type
		else:
			raise "No known robot_type!"

		# convert environment YAML -> g
		filename_g = p / "env.g"
		translate_g.write(filename_env, str(filename_g))

		# write config file
		filename_cfg = p / "rai.cfg"
		with open(filename_cfg, 'w') as f:
			f.write(cfg)

		if initialguess == "ompl":
			# compute initial guess via OMPL
			filename_initial_guess = "{}/result_ompl.yaml".format(folder)
			result = subprocess.run(["./main_ompl_geometric", 
				"-i", filename_env,
				"-o", filename_initial_guess,
				"--timelimit", str(10),
				"-p", "rrt*",
				"--robottype", robot_type_guess,
				])
		else:
			filename_initial_guess = initialguess

		if filename_initial_guess != "none" and robot_type_guess != "none":
			utils_sol_file = UtilsSolutionFile(robot_type_guess)
			utils_sol_file.load(filename_initial_guess)

			if T_range_rel is None and T_range_abs is None:
				length = utils_sol_file.file['result'][0]['pathlength']
				max_speed = 0.5
				min_T = int(length / max_speed * 10)
				max_T = None
			
			if T_range_rel is not None:
				T_file = utils_sol_file.T()
				min_T = max(1, int(T_range_rel[0] * T_file))
				max_T = max(int(T_range_rel[1] * T_file), min_T+1)
		else:
			min_T = 1
			max_T = None

		if T_range_abs is not None:
			if min_T is not None:
				min_T = max(T_range_abs[0], min_T)
			else:
				min_T = T_range_abs[0]
			if max_T is not None:
				max_T = min(T_range_abs[1], max_T)
			else:
				max_T = T_range_abs[1]

		print("T range: ", min_T, max_T)

		# prepare stats
		filename_stats = "{}/stats.yaml".format(folder)
		filename_result = "{}/result_komo.yaml".format(folder)

		with open(filename_stats, 'w') as stats:
			stats.write("stats:\n")

			best_T = None
			if search == "linear":
				T = min_T - 1
			if search == "linearReverse":
				T = T_range_abs[1] + 1
			elif search == "none":
				T = use_T
			elif search == "binarySearch":
				# modified binary search
				T = None
			while time.time() - start < timelimit:
				if search == "linear":
					if max_T is not None:
						if min_T >= max_T:
							break
					T = T + 1
					print("TRYING ", T)
				elif search == "linearReverse":
					if T <= 1:
						break
					T = T - 1
					print("Trying ", T)
				elif search == "binarySearch":
					if max_T is not None:
						if min_T >= max_T:
							break
						T = int((min_T + max_T) / 2)
					else:
						if T is None:
							T = min_T
						else:
							T = T * 2

					print("TRYING ", T, min_T, max_T)

				if filename_initial_guess != "none" and robot_type_guess != "none":
					filename_modified_guess = p / "guess_{}.yaml".format(T)
					utils_sol_file.save_rescaled(filename_modified_guess, T)
				else:
					filename_modified_guess = "none"

				# Run KOMO
				filename_temp_result = p / "result_{}.yaml".format(T)
				success =  _run_komo(filename_g, filename_env, filename_modified_guess, filename_temp_result, filename_cfg, robot_type, T)
				if not success:
					print("KOMO failed with T", T)
					min_T = T + 1

					if search == "none":
						break
					if search == "linearReverse":
						break
				else:
					print("KOMO SUCCESS with T", T)
					now = time.time()
					t = now - start
					stats.write("  - t: {}\n    cost: {}\n".format(t, T * robot.dt))

					if search == "linear":
						best_T = T
						break
					elif search == "linearReverse":
						best_T = T
					elif search == "none":
						best_T = T
						break
					elif search == "binarySearch":
						max_T = T - 1
						if best_T is None or T < best_T:
							best_T = T
						# return True
			
		if best_T is not None:
			shutil.copyfile(p / "result_{}.yaml".format(best_T), filename_result)
			if filename_initial_guess != "none" and robot_type_guess != "none":
				shutil.copyfile(p / "guess_{}.yaml".format(best_T), filename_initial_guess)
			return True
		return False
	

def main():
	parser = argparse.ArgumentParser()
	parser.add_argument("env", help="file containing the environment (YAML)")
	parser.add_argument("initial_guess", help="file containing the initial_guess (e.g., from db-A*) (YAML)")
	parser.add_argument("result", help="file containing the optimization result (YAML)")
	args = parser.parse_args()

	run_komo(args.env, args.initial_guess, args.result)


if __name__ == '__main__':
	main()
