import argparse
import yaml
import numpy as np

import sys
import os
sys.path.append(os.getcwd())
from motionplanningutils import CollisionChecker, RobotHelper
import robots


def extract_valid_motions(filename_env: str, filename_result: str, validity_checked=False):
	# read robot type
	with open(filename_env) as f:
		env = yaml.safe_load(f)

	robot_node = env["robots"][0]
	robot = robots.create_robot(robot_node["type"])

	def check_array(a, b):
		return np.allclose(a, b, rtol=0.01, atol=1e-2)

	# load result
	with open(filename_result) as f:
		result = yaml.safe_load(f)

	states = np.array(result["result"][0]["states"])
	actions = np.array(result["result"][0]["actions"])

	# dynamics
	T = states.shape[0]
	valid = np.full((T,), True)
	if not validity_checked:
		for t in range(T-1):
			state_desired = robot.step(states[t], actions[t])
			valid[t] &= check_array(states[t+1], state_desired)
		# state limits
		for t in range(T):
			if not robot.valid_state(states[t]):
				valid[t] = False
		# action limits
		for t in range(T-1):
			if (actions[t] > robot.max_u + 1e-2).any() or (actions[t] < robot.min_u - 1e-2).any():
				valid[t] = False

	motions = []
	start_t = 0
	eucledian_distance = 0

	for t in range(T):
		if t > 0:
			if robot.is2D:
				eucledian_distance += np.linalg.norm(states[t-1][0:2] - states[t][0:2])
			else:
				eucledian_distance += np.linalg.norm(states[t-1][0:3] - states[t][0:3])
		if not valid[t] or t == T-1 or eucledian_distance > 0.5:
			if t - start_t > 5:
				# shift states
				if robot.is2D:
					states[start_t:, 0:2] -= states[start_t, 0:2]
				else:
					states[start_t:, 0:3] -= states[start_t, 0:3]
				# create motion
				motion = dict()
				motion['x0'] = states[start_t].tolist()
				motion['xf'] = states[t-1].tolist()
				motion['states'] = states[start_t:t].tolist()
				motion['actions'] = actions[start_t:t-1].tolist()
				motion['T'] = t-start_t-1
				motions.append(motion)
				start_t = t
				eucledian_distance = 0
		if not valid[t]:
			start_t = t
			eucledian_distance = 0

	print("extract motions: {:.1f} % valid; split in {} motions".format(np.count_nonzero(valid) / T * 100, len(motions)))
	
	return motions
	


def check(filename_env: str, filename_result: str, file = None, expected_T=None) -> bool:

	with open(filename_env) as f:
		env = yaml.safe_load(f)

	robot_node = env["robots"][0]
	robot = robots.create_robot(robot_node["type"])

	x0 = np.array(robot_node["start"])
	xf = np.array(robot_node["goal"])

	cc = CollisionChecker()
	cc.load(filename_env)

	with open(filename_result) as f:
		result = yaml.safe_load(f)

	states = np.array(result["result"][0]["states"])
	actions = np.array(result["result"][0]["actions"])

	def check_array(a, b, msg):
		success = np.allclose(a, b, rtol=0.01, atol=1e-2)
		if not success:
			print("{} Is: {} Should: {} Delta: {}".format(msg, a, b, a-b), file=file)
		return success

	success = True
	if states.shape[1] != len(robot.state_desc):
		print("Wrong state dimension!", file=file)
		success = False
	if actions.shape[1] != len(robot.action_desc):
		print("Wrong action dimension!", file=file)
		success = False
	if states.shape[0] != actions.shape[0] + 1:
		print("number of actions not number of states - 1!", file=file)
		success = False
	
	success &= check_array(states[0], x0, "start state")
	success &= check_array(states[-1], xf, "end state")
	# dynamics
	T = states.shape[0]
	for t in range(T-1):
		state_desired = robot.step(states[t], actions[t])
		success &= check_array(states[t+1], state_desired, "Wrong dynamics at t={}".format(t))
	# state limits
	for t in range(T):
		if not robot.valid_state(states[t]):
			print("State invalid at t={} ({})".format(t, states[t]), file=file)
			success = False
	# action limits
	for t in range(T-1):
		if (actions[t] > robot.max_u + 1e-2).any() or (actions[t] < robot.min_u - 1e-2).any():
			print("Action outside bounds at t={} ({})".format(t, actions[t]), file=file)
			success = False
	# collisions
	for t in range(T):
		dist, _, _ = cc.distance(states[t])
		if dist < -0.03: # allow up to 3cm violation
			print("Collision at t={} ({})".format(t, dist), file=file)
			success = False

	if expected_T is not None:
		if T-1 not in expected_T:
			print("Expected T to be in {}, but is {}".format(expected_T, T-1), file=file)
			success = False

	return success


def compute_delta(filename_env: str, filename_result: str) -> float:

	with open(filename_env) as f:
		env = yaml.safe_load(f)

	robot_node = env["robots"][0]
	robot = robots.create_robot(robot_node["type"])

	x0 = np.array(robot_node["start"])
	xf = np.array(robot_node["goal"])

	rh = RobotHelper(robot_node["type"])

	with open(filename_result) as f:
		result = yaml.safe_load(f)

	states = np.array(result["result"][0]["states"])
	actions = np.array(result["result"][0]["actions"])

	deltas = []
	deltas.append(rh.distance(x0, states[0]))
	# dynamics
	T = states.shape[0]
	for t in range(T-1):
		state_desired = robot.step(states[t], actions[t])
		delta = rh.distance(state_desired, states[t+1])
		# print(t, delta, states[t], actions[t], states[t+1], state_desired)
		deltas.append(delta)
	deltas.append(rh.distance(xf, states[-1]))
	# idx = np.argmax(deltas) + 1
	# print(idx, states[idx], actions[idx], states[idx+1])
	return np.max(deltas)


def main() -> None:
	parser = argparse.ArgumentParser()
	parser.add_argument("env", help="file containing the environment (YAML)")
	parser.add_argument("result", help="file containing the result (YAML)")
	args = parser.parse_args()

	print(extract_valid_motions(args.env, args.result))

	# print(check(args.env, args.result))
	# print(compute_delta(args.env, args.result))


if __name__ == "__main__":
	main()