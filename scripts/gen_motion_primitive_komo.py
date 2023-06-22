import numpy as np
# from scp import SCP
from main_komo import run_komo_standalone
from utils_motion_primitives import sort_primitives, visualize_motion, plot_stats
import robots
import yaml
import msgpack
import multiprocessing as mp
import tqdm
import itertools
import argparse
import subprocess
import tempfile
from pathlib import Path
import psutil
import checker
import time


import sys, os
sys.path.append(os.getcwd())

def gen_motion(robot_type, start, goal, is2D, cfg):

	dbg = False
	with tempfile.TemporaryDirectory() as tmpdirname:
		if dbg:
			p = Path("../results/test")
		else:
			p = Path(tmpdirname)
		env = {
			"environment":{
				"min": [-10, -10],
				"max": [10, 10],
				"obstacles": []
			},
			"robots": [{
				"type": robot_type,
				"start": list(start),
				"goal": list(goal),
			}]
		}
		if not is2D:
			env["environment"]["min"].append(-10)
			env["environment"]["max"].append(10)

		filename_env = str(p / "env.yaml")
		with open(filename_env, 'w') as f:
			yaml.dump(env, f, Dumper=yaml.CSafeDumper)
		
		filename_result = p / "result_komo.yaml"

		success = run_komo_standalone(filename_env, str(p), cfg['timelimit'], cfg['rai_cfg'], cfg['search'], initialguess="none", T_range_abs=[0, 200])

		if not success:
			return []

		# checker.check(str(filename_env), str(filename_result))

		if dbg:
			subprocess.run(["python3",
						"../benchmark/{}/visualize.py".format(robot_type),
						str(filename_env),
						"--result", str(filename_result),
						"--video", str(filename_result.with_suffix(".mp4"))])

		# read the result
		with open(filename_result) as f:
			result = yaml.load(f, Loader=yaml.CSafeLoader)

		states = np.array(result["result"][0]["states"])
		actions = np.array(result["result"][0]["actions"])

		eucledian_distance = 0
		split = [0]
		for k in range(1, len(states)):
			if is2D:
				eucledian_distance += np.linalg.norm(states[k-1][0:2] - states[k][0:2])
			else:
				eucledian_distance += np.linalg.norm(states[k-1][0:3] - states[k][0:3])
			if eucledian_distance >= 0.5:
				split.append(k)
				eucledian_distance = 0
		
		# include last segment, if it not very short
		if len(states) - split[-1] > 5:
			split.append(len(states)-1)

		# create motions
		motions = []
		for idx in range(1, len(split)):
			start_k = split[idx-1]
			k = split[idx]
			# shift states
			if is2D:
				states[start_k:, 0:2] -= states[start_k, 0:2]
			else:
				states[start_k:, 0:3] -= states[start_k, 0:3]
			# create motion
			motion = dict()
			motion['x0'] = states[start_k].tolist()
			motion['xf'] = states[k].tolist()
			motion['states'] = states[start_k:k+1].tolist()
			motion['actions'] = actions[start_k:k].tolist()
			motion['T'] = k-start_k
			motions.append(motion)
			# use the following break to only create the first motion
			# this will create a nicer (uniform) distribution, but take
			# much longer
			# break
		return motions


def gen_random_motion(robot_type):
	# NOTE: It is *very* important to keep this as a local import, otherwise
	#       random numbers may repeat, when using multiprocessing
	from motionplanningutils import RobotHelper

	# load tuning settings for this case
	tuning_path = Path("../tuning")
	cfg = tuning_path / robot_type / "algorithms.yaml"
	print(cfg)
	assert(cfg.is_file())

	with open(cfg) as f:
		cfg = yaml.safe_load(f)

	# find cfg
	mycfg = cfg['gen-motion']

	rh = RobotHelper(robot_type, mycfg["env_limit"])
	start = rh.sampleUniform()
	# shift to center (at 0,0)
	start[0] = 0
	start[1] = 0
	if not rh.is2D():
		start[2] = 0
	goal = rh.sampleUniform()
	motions =  gen_motion(robot_type, start, goal, rh.is2D(), mycfg)
	for motion in motions:
		motion['distance'] = rh.distance(motion['x0'], motion['xf'])
	return motions


def main():
	parser = argparse.ArgumentParser()
	parser.add_argument("robot_type", help="name of robot type to generate motions for")
	parser.add_argument("--N", help="number of motions", default=100, type=int)
	args = parser.parse_args()

	# rh = RobotHelper(args.robot_type)

	motions = []
	tasks = itertools.repeat(args.robot_type, args.N)

	tmp_path = Path("../results/tmp/motions/{}".format(args.robot_type))
	tmp_path.mkdir(parents=True, exist_ok=True)

	def add_motions(additional_motions):
		if len(additional_motions) > 0:
			motions.extend(additional_motions)
			print("Generated {} motions".format(len(motions)), flush=True)
			# Store intermediate results, in case we need to interupt the generation
			i = 0
			while True:
				p = tmp_path / "{}.yaml".format(i)
				if not p.exists():
					with open(p, 'w') as f:
						yaml.dump(additional_motions, f)
					break
				i = i + 1

	# if args.N <= 10:
	if False:
		while len(motions) < args.N:
			multiple_motions = gen_random_motion(args.robot_type)
			motions.extend(multiple_motions)
	else:
		# mp.set_start_method('spawn')
		use_cpus = psutil.cpu_count(logical=False)
		async_results = []
		with mp.Pool(use_cpus) as p:
			while len(motions) < args.N:
				# clean up async_results
				async_results = [x for x in async_results if not x.ready()]
				# run some more workers
				while len(async_results) < use_cpus:
					ar = p.apply_async(gen_random_motion, (args.robot_type,), callback=add_motions)
					async_results.append(ar)
				time.sleep(1)
			p.terminate()

	for k, motion in enumerate(motions):
		motion['name'] = 'm{}'.format(k)

	out_path = Path("../cloud/motions")
	out_path.mkdir(parents=True, exist_ok=True)

	# now sort the primitives
	sorted_motions = sort_primitives(motions, args.robot_type)
	with open(out_path / "{}_sorted.msgpack".format(args.robot_type), 'wb') as file:
		msgpack.pack(sorted_motions, file)

	# visualize the top 100
	# for k, m in enumerate(sorted_motions[0:10]):
	# 	visualize_motion(m, args.robot_type, tmp_path / "top_{}.mp4".format(k))

	# # plot statistics
	# plot_stats(sorted_motions, args.robot_type, tmp_path / "stats.pdf")


if __name__ == '__main__':
	main()
