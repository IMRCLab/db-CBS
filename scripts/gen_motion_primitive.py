import numpy as np
from scp import SCP
import robots
import yaml
import multiprocessing as mp
import tqdm
import itertools
import argparse

import sys, os
sys.path.append(os.getcwd())
from motionplanningutils import RobotHelper


# two point boundary value problem
def TPBVP_fixed_time(robot, x0, xf, T):
	scp = SCP(robot)

	# initialize with random rollout
	states = np.empty((T,len(robot.state_desc)))
	states[0] = x0
	actions = np.empty((T-1,len(robot.action_desc)))
	for k in range(T-1):
		actions[k] = np.random.uniform(robot.min_u, robot.max_u)
		states[k+1] = robot.step(states[k], actions[k])

	# states = np.tile(x0, (T, 1))
	# actions = np.zeros((T-1, 2))
	# states[1:] += np.random.normal(0, 0.001, states.shape)[1:]
	# actions += np.random.normal(0, 0.001, actions.shape)
	X, U, val = scp.min_xf(states, actions, x0, xf, 10, trust_x=0.1, trust_u=0.1)
	if len(X) > 1:
		return X[-1], U[-1], val

	return None, None, None

def gen_random_motion(robot_type):
	robot = robots.create_robot(robot_type)
	rh = RobotHelper(robot_type)
	while True:
		x0 = np.array(rh.sampleUniform())
		x0[0:2] = 0 # set position part to 0
		xf = np.array(rh.sampleUniform())
		T = np.random.choice([8, 16, 32])
		print(T)

		X, U, _ = TPBVP_fixed_time(robot, x0, xf, T)
		print(X)
		if X is not None:
			r = dict()
			r['x0'] = x0.tolist()
			r['xf'] = X[-1].tolist()
			r['states'] = X.tolist()
			r['actions'] = U.tolist()
			r['T'] = int(T)
			return r


def gen_motion(robot, x0, xf):
	print("Try ", xf)
	for T in range(2, 32):
	# for T in [32]:
		X, U, val = TPBVP_fixed_time(robot, x0, xf, T)
		if X is not None and val < 1e6:
			r = dict()
			r['x0'] = x0.tolist()
			r['xf'] = X[-1].tolist()
			r['states'] = X.tolist()
			r['actions'] = U.tolist()
			r['T'] = int(T)
			return r

def main():
	parser = argparse.ArgumentParser()
	parser.add_argument("robot_type", help="name of robot type to generate motions for")
	parser.add_argument("output", help="output file (YAML)")
	parser.add_argument("--N", help="number of motions", default=100, type=int)
	args = parser.parse_args()

	rh = RobotHelper(args.robot_type)

	motions = []
	# tasks = list(itertools.repeat(robot, N))
	# print(tasks)
	# for k in range(N):
	# 	print(k)
	# 	motion = gen_random_motion(robot)
	# 	motions.append(motion)

	mp.set_start_method('spawn')
	with mp.Pool() as p:
		for motion in tqdm.tqdm(p.imap_unordered(gen_random_motion, itertools.repeat(args.robot_type, args.N))):
			motion['distance'] = rh.distance(motion['x0'], motion['xf'])
			motion['name'] = 'm{}'.format(len(motions))
			motions.append(motion)

	# for x in [-0.25, 0, 0.25]:
	# 	for y in [-0.25, 0, 0.25]:
	# 		for yaw in np.linspace(-np.pi, np.pi, 8):
	# 			motion = gen_motion(robot, 
	# 				np.array([0, 0, 0], dtype=np.float32),
	# 				np.array([x, y, yaw], dtype=np.float32))
	# 			if motion is not None:
	# 				print(x,y, yaw)
	# 				motions.append(motion)

	with open(args.output, 'w') as file:
		yaml.dump(motions, file)


if __name__ == '__main__':
	main()