import argparse
import yaml
import sys, os
import numpy as np
import tempfile
from pathlib import Path
import subprocess
import robots
import random
import msgpack

sys.path.append(os.getcwd())
from motionplanningutils import RobotHelper

def sort_primitives(motions: list, robot_type: str, top_k=None) -> list:
	rh = RobotHelper(robot_type, pos_limit=100)

	if top_k is None:
		top_k = len(motions)

	x0s = [m["x0"] for m in motions]
	xfs = [m["xf"] for m in motions]
	idxs = rh.sortMotions(x0s, xfs, top_k)

	used_motions = [motions[idx] for idx in idxs]
	return used_motions

	# The code below is the equivalent, but slower, Python version

	print(idxs)
	# exit()

	for k, m in enumerate(motions):
		m['idx'] = k

	# use as first/seed motion the one that moves furthest
	best_motion = motions[0]
	largest_d = 0
	for m in motions:
		d = rh.distance(m["x0"], m["xf"])
		if d > largest_d:
			largest_d = d
			best_motion = m

	used_motions = [best_motion]
	unused_motions = list(motions)
	unused_motions.remove(best_motion)

	if top_k is None:
		top_k = len(motions) - 1

	for k in range(1, top_k):
		best_d = -1
		best_motion = None
		for m1 in unused_motions:
			# find smallest distance to existing neighbors
			smallest_d_x0 = np.inf
			for m2 in used_motions:
				d = rh.distance(m1["x0"], m2["x0"])
				if d < smallest_d_x0:
					smallest_d_x0 = d
			smallest_d_xf = np.inf
			for m2 in used_motions:
				d = rh.distance(m1["xf"], m2["xf"])
				if d < smallest_d_xf:
					smallest_d_xf = d
			# find largest among the smallest
			smallest_d = smallest_d_x0 + smallest_d_xf
			if smallest_d > best_d:
				best_motion = m1
				best_d = smallest_d
		assert(best_motion is not None)
		used_motions.append(best_motion)
		unused_motions.remove(best_motion)
		print("sorting ", k)

	for m in used_motions:
		print(m['idx'])
	exit()
	return used_motions

def visualize_motion(motion: dict, robot_type: str, output_file: str):
	with tempfile.TemporaryDirectory() as tmpdirname:
		p = Path(tmpdirname)


		# convert to result file format
		result = {
			"result": [{
					"states": motion["states"],
					"actions": motion["actions"],
			}]}
		filename_result = str(p / "result.yaml")
		with open(filename_result, 'w') as f:
			yaml.dump(result, f, Dumper=yaml.CSafeDumper)

		# generate environment file
		env = {
			"environment":{
				"min": [-2, -2],
				"max": [2, 2],
				"obstacles": []
			},
			"robots": [{
				"type": robot_type,
				"start": list(motion["states"][0]),
				"goal": list(motion["states"][-1]),
			}]
		}
		filename_env = str(p / "env.yaml")
		with open(filename_env, 'w') as f:
			yaml.dump(env, f, Dumper=yaml.CSafeDumper)

		subprocess.run(["python3",
			"../benchmark/{}/visualize.py".format(robot_type),
			str(filename_env),
			"--result", str(filename_result),
			"--video", str(output_file)])

def merge_motions(folder: str, limit: int = None):

	file_names = [str(p) for p in Path(folder).glob("**/*.yaml")]
	if limit is not None:
		random.shuffle(file_names)
	merged_motions = []
	for file_name in file_names:
		with open(file_name) as f:
			motions = yaml.load(f, Loader=yaml.CSafeLoader)
			merged_motions.extend([m for m in motions if m["T"] <= 100])
			print(len(merged_motions))
			if limit is not None and len(merged_motions) > limit:
				break
	return merged_motions


def plot_stats(motions: list, robot_type: str, filename: str):
	all_actions = []
	all_states = []
	all_start_states = []
	all_end_states = []
	for m in motions:
		all_actions.extend(m["actions"])
		all_states.extend(m["states"])
		all_start_states.append(m["x0"])
		all_end_states.append(m["xf"])
	

	import numpy as np
	import matplotlib.pyplot as plt
	from matplotlib.backends.backend_pdf import PdfPages

	all_actions = np.array(all_actions)
	all_states = np.array(all_states)
	all_start_states = np.array(all_start_states)
	all_end_states = np.array(all_end_states)

	pp = PdfPages(filename)

	r = robots.create_robot(robot_type)
	
	for k, a in enumerate(r.action_desc):
		fig, ax = plt.subplots()
		ax.set_title("Action: " + a)
		ax.hist(all_actions[:,k])
		pp.savefig(fig)
		plt.close(fig)

	for k, s in enumerate(r.state_desc):
		fig, ax = plt.subplots()
		ax.set_title("state: " + s)
		ax.hist(all_states[:,k])
		pp.savefig(fig)
		plt.close(fig)

	for k, s in enumerate(r.state_desc):
		fig, ax = plt.subplots()
		ax.set_title("start state: " + s)
		ax.hist(all_start_states[:,k])
		pp.savefig(fig)
		plt.close(fig)

	for k, s in enumerate(r.state_desc):
		fig, ax = plt.subplots()
		ax.set_title("end state: " + s)
		ax.hist(all_end_states[:,k])
		pp.savefig(fig)
		plt.close(fig)

	pp.close()




def main() -> None:
	parser = argparse.ArgumentParser()
	# parser.add_argument("motions", help="file containing the motions (YAML)")
	parser.add_argument("robot_type", help="name of robot type to generate motions for")
	args = parser.parse_args()


	out_path = Path("../cloud/motions")
	out_path.mkdir(parents=True, exist_ok=True)

	tmp_path = Path("../results/tmp/motions/{}".format(args.robot_type))
	tmp_path.mkdir(parents=True, exist_ok=True)

	# # load existing motions
	# with open(out_path / "{}_sorted.yaml".format(args.robot_type)) as f:
	# 	sorted_motions = yaml.load(f, Loader=yaml.CSafeLoader)

	# with open(out_path / "{}_sorted.msgpack".format(args.robot_type), 'wb') as file:
	# 	msgpack.pack(sorted_motions, file)


	# exit()

	motions = merge_motions(tmp_path) #, 10000)
	# tmp_path2 = Path("../results/tmp/motions/quadrotor_0_server")
	# motions.extend(merge_motions(tmp_path2))


	# now sort the primitives
	# sorted_motions = motions
	sorted_motions = sort_primitives(motions, args.robot_type)
	# with open(out_path / "{}_sorted.yaml".format(args.robot_type), 'w') as file:
	# 	yaml.dump(sorted_motions, file, Dumper=yaml.CSafeDumper)

	with open(out_path / "{}_sorted.msgpack".format(args.robot_type), 'wb') as file:
		msgpack.pack(sorted_motions, file)

	# visualize the top 10
	for k, m in enumerate(sorted_motions[0:10]):
		visualize_motion(m, args.robot_type, tmp_path / "top_{}.mp4".format(k))

	# with open(out_path / "{}_sorted.yaml".format(args.robot_type)) as f:
	# 	sorted_motions = yaml.load(f, Loader=yaml.CSafeLoader)

	print(len(sorted_motions))

	# plot_stats(sorted_motions, args.robot_type, tmp_path / "stats.pdf")

	exit()

	filename_motions_sorted = Path(args.motions).stem + "_sorted.yaml"
	used_motions = sort_primitives(motions, args.robot_type)
	with open(filename_motions_sorted, 'w') as file:
		yaml.dump(used_motions, file, Dumper=yaml.CSafeDumper)

	exit()


	robot_type = "unicycle_second_order_0"



	used_motions = sort_primitives(motions, robot_type, 10)

	with tempfile.TemporaryDirectory() as tmpdirname:
		p = Path(tmpdirname)

		for k, m in enumerate(used_motions):
			for s in m["states"]:
				s[0] += 2
				s[1] += 2

			# convert to result file format
			result = {
				"result": [{
						"states": m["states"],
						"actions": m["actions"],
				}]}
			filename_result = str(p / "result.yaml")
			with open(filename_result, 'w') as f:
				yaml.dump(result, f, Dumper=yaml.CSafeDumper)

			# generate environment file
			env = {
				"environment":{
					"min": [0, 0],
					"max": [4, 4],
					"obstacles": []
				},
				"robots": [{
					"type": robot_type,
					"start": list(m["states"][0]),
					"goal": list(m["states"][-1]),
				}]
			}
			filename_env = str(p / "env.yaml")
			with open(filename_env, 'w') as f:
				yaml.dump(env, f, Dumper=yaml.CSafeDumper)

			import subprocess
			subprocess.run(["python3",
				"../benchmark/unicycle_second_order_0/visualize.py",
				str(filename_env),
				"--result", str(filename_result),
				"--video", "../results/test/{}.mp4".format(k)])

	for m in used_motions:
		print(m["x0"], m["xf"])

if __name__ == "__main__":
	main()