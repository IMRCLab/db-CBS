import sys
sys.path.append('./')
import numpy as np
import yaml
import argparse
from pathlib import Path
import subprocess


np.set_printoptions(linewidth=np.inf)
np.set_printoptions(suppress=True)

#### This script generates the init guess from the plan of the two single robots to the joint robot
#### moreover, it saves an html file for the meshcat animation of this init guess

def saveyaml(file_dir, data):
    with open(file_dir, 'w') as f:
        yaml.safe_dump(data, f, default_flow_style=None)

def normalize(vec):
    norm_v = np.linalg.norm(vec)
    if norm_v > 0:
        return np.array(vec)/norm_v
    else: 
        raise ValueError("Cannot normalize a zero vector.")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--env', type=str, help="environment")
    parser.add_argument('--joint_robot_env', default=None, type=str, help="environment")
    parser.add_argument("--num_robots", type=int, help="num of robots")
    parser.add_argument('--payload', default=None, type=str, help="payload pose in dbcbs")
    parser.add_argument('--dbcbs', type=str, help="dbcbs sol")
    parser.add_argument('--result', type=str, help="init_guess_cable.yaml")
    parser.add_argument('--output', type=str, help="init_guess_cable.html")

    args = parser.parse_args()
    num_robots = args.num_robots
    if args.joint_robot_env is None:
        path_to_env = args.env 
    else:
        path_to_env = args.joint_robot_env
    path_to_dbcbs = args.dbcbs
    html_path = args.output
    path_to_result = args.result

    # Load environment
    with open(path_to_env, "r") as f: 
        env = yaml.safe_load(f)

    # Load db_cbs states
    with open(path_to_dbcbs, "r") as f: 
        db_cbs_states = yaml.safe_load(f)

    robots_states = []
    robots_actions = []

    for i in range(num_robots):
        robots_states.append(np.array(db_cbs_states["result"][i]["states"]))
        action_array = np.array(db_cbs_states["result"][i]["actions"])
        robots_actions.append(action_array)

    # Determine the maximum number of rows
    max_rows_states = max(arr.shape[0] for arr in robots_states)
    max_rows_actions = max(arr.shape[0] for arr in robots_actions)

    # Pad the arrays with zeros to match the maximum number of rows
    padded_robot_states = [np.pad(arr, ((0, max_rows_states - arr.shape[0]), (0, 0)), 'edge') for arr in robots_states]
    padded_robot_actions = [np.pad(arr, ((0, max_rows_actions - arr.shape[0]), (0, 0)), 'edge') for arr in robots_actions]
    actions = np.concatenate(padded_robot_actions, axis=1)
    num_states = padded_robot_states[0].shape[0]

    # Initialize joint states with the new representation
    unicycles_joint_states = np.zeros((padded_robot_states[0].shape[0], 2 + num_robots + (num_robots - 1)))

    for i in range(num_states):
        # Add the first robot's state
        unicycles_joint_states[i, 0:3] = padded_robot_states[0][i, 0:3]  # px1, py1, alpha1

        # Add alpha for remaining robots
        for j in range(1, num_robots):
            unicycles_joint_states[i, 2 + j] = padded_robot_states[j][i, 2]  # alpha_j

        # Compute theta for each rod
        for j in range(num_robots - 1):
            pi = padded_robot_states[j][i]
            pi_next = padded_robot_states[j + 1][i]
            u = pi_next[0:2] - pi[0:2] 
            th = np.arctan2(u[1], u[0])
            
            # Add theta to the joint states
            unicycles_joint_states[i, 2 + num_robots + j] = th
    # Save the result to a YAML file
    result_yaml = dict()
    result_yaml["result"] = dict()
    result_yaml["result"]["states"] = unicycles_joint_states.tolist()
    result_yaml["result"]["actions"] = actions.tolist()
    result_yaml["result"]["num_action"] = len(actions.tolist())
    result_yaml["result"]["num_states"] = len(unicycles_joint_states.tolist())

    saveyaml(path_to_result, result_yaml)
    # Optional visualization setup
    script = "../scripts/visualize_unicycles.py"
    subprocess.run(["python3", script, "--robot", "point", "--env", path_to_env, "--result", path_to_result, "--output", html_path])
    print(html_path)
    print(path_to_result)


if __name__ == "__main__":
    main()
