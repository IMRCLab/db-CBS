import sys
sys.path.append('./')
import numpy as np
import yaml
from pathlib import Path
sys.path.append(str(Path(__file__).parent / "visualize_dintCables.py"))
sys.path.append(str(Path(__file__).parent))
import argparse
from visualize import main
import subprocess
import re

def saveyaml(file_dir, data):
    with open(file_dir, 'w') as f:
        yaml.safe_dump(data, f, default_flow_style=None)

def normalize(vec):
    norm_v = norm(vec)
    if norm_v > 0:
        return np.array(vec)/norm_v
    else: 
        raise("cannot divide by zero")

def norm(vec):
    return np.linalg.norm(np.array(vec))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--env', type=str, help="environment")
    parser.add_argument('--joint_robot_env',default=None, type=str, help="environment")
    parser.add_argument("--num_robots", type=int, help="num of robots")
    parser.add_argument('--payload', type=str, help="payload pose in dbcbs")
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
    path_to_payload = args.payload 

    # load env 
    with open(path_to_env, "r") as f: 
        env = yaml.safe_load(f)
    #load payload data
    with open(path_to_payload, "r") as f: 
        payload_yaml = yaml.safe_load(f)

    # load db_cbs states
    with open(path_to_dbcbs, "r") as f: 
        db_cbs_states = yaml.safe_load(f)

    robots_states = []
    robots_actions = []
    for i in range(num_robots):
        robots_states.append(np.array(db_cbs_states["result"][i]["states"]))
        robots_actions.append(np.array(db_cbs_states["result"][i]["actions"]))
    # payload states from dbCBS non linear opt
    p0_init = payload_yaml["payload"]

    # Determine the maximum number of rows
    max_rows_states = max(arr.shape[0] for arr in robots_states)
    max_rows_actions = max(arr.shape[0] for arr in robots_actions)

    # Pad the arrays with zeros to match the maximum number of rows
    padded_robot_states = [np.pad(arr, ((0, max_rows_states - arr.shape[0]), (0, 0)), 'edge') for arr in robots_states]
    padded_robot_actions = [np.pad(arr, ((0, max_rows_actions - arr.shape[0]), (0, 0)), 'edge') for arr in robots_actions]
    actions = np.concatenate(padded_robot_actions, axis=1)
    num_states = padded_robot_states[0].shape[0]

    payload_states = np.zeros((padded_robot_states[0].shape[0], 6 + 6*num_robots + 7*num_robots))
    for i in range(num_states):
        p0 = np.array(p0_init[i], dtype=np.float64)
        payload_states[i,0:3] = p0_init[i]
        for j in range(num_robots):
            pi = padded_robot_states[j][i,0:3]
            qi = normalize(p0-pi)
            payload_states[i, 6+ 6*j : 6 + 6*j+3] = qi
            payload_states[i, 6 + 6*num_robots + 7*j : 6 + 6*num_robots + 7*j + 4] =[0,0,0,1]

    print("payload: ","("+str(len(p0_init))+","+ str(len(p0_init[0]))+")", "actions: ", actions.shape, "states:", payload_states.shape)

    result_yaml = dict()
    result_yaml["result"] = dict()
    result_yaml["result"]["states"] = payload_states.tolist()
    result_yaml["result"]["actions"] = actions.tolist()
    result_yaml["result"]["num_action"] = len(actions.tolist())
    result_yaml["result"]["num_states"] = len(payload_states.tolist())

    saveyaml(path_to_result, result_yaml)

    if args.joint_robot_env is None:
        with open(path_to_env) as f:
            env_dict = yaml.safe_load(f)

        env_joint_robot = {"environment": env_dict["environment"], "robots": list()}
        env_joint_robot["robots"].append(env_dict["joint_robot"][0])
        env_joint_robot["robots"][0]["start"] = env_dict["joint_robot"][0]["start"]
        goal = np.zeros(payload_states.shape[1])
        goal[0:3] = payload_states[-1][0:3]
        goal[3::] = payload_states[0][3::]
        env_joint_robot["robots"][0]["goal"]  = env_dict["joint_robot"][0]["goal"]
        new_path_to_env = re.sub(r'[^/]+\.yaml$', 'env.yaml', path_to_env)
    
        saveyaml(new_path_to_env, env_joint_robot)
    else:
        with open(args.joint_robot_env) as f:
            env_joint_robot = yaml.safe_load(f)
        new_path_to_env = args.joint_robot_env


    script = "../scripts/visualize_payload.py"
    subprocess.run(["python3",
            script,
            "--robot", "point",
            "--env", new_path_to_env,
            "--result", path_to_result,
            "--output", html_path])
        

if __name__ == "__main__":
    main()