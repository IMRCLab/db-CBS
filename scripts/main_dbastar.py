import shutil
import numpy as np
from scp import SCP
import robots
import yaml
import argparse
import subprocess
import time
import random
import copy
import shutil
from collections import defaultdict
import tempfile
from pathlib import Path
import msgpack

import sys
import os
sys.path.append(os.getcwd())

# import main_scp
# import main_komo
# import gen_motion_primitive
from motionplanningutils import RobotHelper
import checker

# ./dbastar -i ../benchmark/dubins/kink_0.yaml -m motions.yaml -o output.yaml --delta 0.3

def find_smallest_delta(filename_env, filename_motions, filename_result_dbastar, max_delta, max_cost):
	low = 0
	high = max_delta
	delta = high
	best_delta = None
	eps = 0.01

	while low < high - eps:
		delta = (high + low) / 2
		print("ATTEMPT WITH DELTA ", delta, low, high)
		result = subprocess.run(["./dbastar", 
			"-i", filename_env,
			"-m", filename_motions,
			"-o", filename_result_dbastar,
			"--delta", str(delta),
			"--maxCost", str(max_cost)])
		if result.returncode != 0:
			# failure -> need higher delta
			low = delta
		else:
			# success -> try lower delta
			high = delta
			best_delta = delta
		print("NEW ", low, high)

	return best_delta


def compute_motion_importance(filename_env, filename_motions, filename_result_dbastar, delta, max_cost, motions_stats):
	# load the result
	with open(filename_result_dbastar) as f:
		result = yaml.safe_load(f)
		old_cost = len(result["result"][0]["actions"])

	# load the motions file
	with open(filename_motions) as f:
		all_motions = yaml.safe_load(f)

	for name, v in result["result"][0]["motion_stats"].items():
		# create temporary motions file with one motion removed
		motions = copy.copy(all_motions)
		for k, m in enumerate(motions):
			if name == m["name"]:
				break
		del motions[k]
		with open("motions_tmp.yaml", 'w') as file:
			yaml.dump(motions, file)


		result = subprocess.run(["./dbastar",
							"-i", filename_env,
							"-m", 'motions_tmp.yaml',
							"-o", 'result_dbastar_tmp.yaml',
							"--delta", str(delta),
							"--maxCost", str(max_cost)])
		if result.returncode != 0:
			# failure -> this was a very important edge
			print(k, "super important!")
			motions_stats[name] += 1
		else:
			# success -> compute numeric importance
			# load the result
			with open('result_dbastar_tmp.yaml') as f:
				result = yaml.safe_load(f)
				new_cost = len(result["result"][0]["actions"])
			print(k, old_cost, new_cost, 1 - old_cost / new_cost)
			motions_stats[name] += np.clip(1 - old_cost / new_cost, 0, 1)
	return motions_stats

def run_dbastar(filename_env, folder, timelimit, cfg, opt_alg="scp", motions_stats=None):

	add_prims = cfg["add_primitives_per_iteration"]
	desired_branching_factor = cfg["desired_branching_factor"]
	epsilon = cfg["suboptimality_bound"]
	alpha = cfg["alpha"]
	filter_duplicates = cfg["filter_duplicates"]

	with tempfile.TemporaryDirectory() as tmpdirname:
		# p = Path(tmpdirname)
		p = Path("../results/dbg")

		filename_motions = p / "motions.msgpack"

		sol = 0
		filename_stats = "{}/stats.yaml".format(folder)

		with open(filename_env) as f:
			env = yaml.safe_load(f)

		robot_node = env["robots"][0]
		robot_type = robot_node["type"]
		robot = robots.create_robot(robot_type)
		rh = RobotHelper(robot_type)
		# initialize delta
		x0 = np.array(robot_node["start"])
		xf = np.array(robot_node["goal"])

		# delta = rh.distance(x0, xf) * 0.9
		maxCost = 1e6


		# load existing motions
		with open('../cloud/motions/{}_sorted.msgpack'.format(robot_node["type"]), 'rb') as f:
			all_motions = msgpack.unpack(f)
		print("Have {} motions in total".format(len(all_motions)))
		motions = all_motions[0:add_prims]
		del all_motions[0:add_prims]
		with open(filename_motions, 'wb') as file:
			msgpack.pack(motions, file)


		start = time.time()
		duration_dbastar = 0
		duration_opt = 0

		with open(filename_stats, 'w') as stats:
			stats.write("stats:\n")
			while time.time() - start < timelimit:
				print("maxCost", maxCost)

				filename_result_dbastar = p / "result_dbastar.yaml"
				filename_result_opt = p / "result_opt.yaml"

				t_dbastar_start = time.time()
				result = subprocess.run(["./dbastar", 
					"-i", filename_env,
					"-m", filename_motions,
					"-o", filename_result_dbastar,
					"--delta", str(-desired_branching_factor),
					"--epsilon", str(epsilon),
					"--alpha", str(alpha),
					"--filterDuplicates", str(filter_duplicates),
					"--maxCost", str(maxCost)])
				t_dbastar_stop = time.time()
				duration_dbastar += t_dbastar_stop - t_dbastar_start
				if result.returncode != 0:
					# print("dbA* failed; Generating more primitives")


					print("dbA* failed; Using more primitives", len(motions))
					if len(all_motions) >= add_prims:
						motions.extend(all_motions[0:add_prims])
						del all_motions[0:add_prims]
					else:
						break

					with open(filename_motions, 'wb') as file:
						msgpack.pack(motions, file)

				else:
					delta_achieved = checker.compute_delta(filename_env, filename_result_dbastar)
					print("DELTA CHECK", delta_achieved)

					t_opt_start = time.time()
					t_opt_stop = time.time()
					duration_opt += t_opt_stop - t_opt_start

					shutil.copyfile(filename_result_dbastar, "{}/result_dbastar_sol{}.yaml".format(folder, sol))

					sol += 1

					# use more primitives in all cases
					if len(all_motions) >= add_prims:
						motions.extend(all_motions[0:add_prims])
						del all_motions[0:add_prims]
					else:
						break
						
					with open(filename_motions, 'wb') as file:
						msgpack.pack(motions, file)


def main():
	parser = argparse.ArgumentParser()
	parser.add_argument("env", help="file containing the environment (YAML)")
	args = parser.parse_args()

	motions_stats = defaultdict(float)
	for i in range(1):
		run_dbastar(args.env, i, motions_stats)
	print(sorted( ((v,k) for k,v in motions_stats.items()), reverse=True) )


if __name__ == '__main__':
	main()
