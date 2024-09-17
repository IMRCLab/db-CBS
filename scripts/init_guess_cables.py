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


#### This script generates the init guess from the plan of the two single robots to the joint robot
#### moreover, it saves an html file for the meshcat animation of this init guess
### from the main directory

def saveyaml(file_dir, data):
    with open(file_dir, 'w') as f:
        yaml.safe_dump(data, f, default_flow_style=None)

def computePayloadSt(r1, r2, r0):
    # compute the states from the estimated payload and the robots positions coming from dbCBS
    # note that even though I compute the velocities, I only use the geometric states for the initial guess
    p1 = np.array(r1[0:2], dtype=np.float64)
    p2 = np.array(r2[0:2], dtype=np.float64)
    p0 = np.array(r0[0:2], dtype=np.float64)
    u1 = p1 - p0
    th1 = np.arctan2(u1[1], u1[0])
    u2 = p2 - p0
    th2 = np.arctan2(u2[1], u2[0])
    dp1 = np.array(r1[2::])
    dp2 = np.array(r2[2::])
    dpi = np.concatenate((dp1, dp2))
    l1 = 0.5
    l2 = l1
    # J maps the joint space to task space r_dot = J(q)q_dot
    # q = [p0x p0y th1, th2]
    # q_dot = [dp0x, dp0y, dth1, dth2]
    # r = [p1x p1y p2x p2y] 
    # r = f(q)
    # f(q) = [p0x + l1cos(th1), p0y + l1sin(th1), p0x + l2cos(th2), p0y + l2sin(th2)] 
    # r_dot = [dp1x dp1y dp2x dp2y]
    J = np.array([[1, 0,  -l1*np.sin(th1),          0       ], 
                  [0, 1,  l1*np.cos(th1),          0       ], 
                  [1, 0,            0,      -l2*np.sin(th2)], 
                  [0, 1,            0,       l2*np.cos(th2)]])
    # then [dp0x dp0y, dth1, dth2] = J^-1 * r_dot
    dstates = np.linalg.inv(J)@dpi
    dp0 = dstates[0:2]
    dth1 = dstates[2]
    dth2 = dstates[3]
    return th1, th2, dp0, dth1, dth2

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
    parser.add_argument('--payload', type=str, help="payload pose in dbcbs")
    parser.add_argument('--dbcbs', type=str, help="dbcbs sol")
    parser.add_argument('--result', type=str, help="init_guess_cable.yaml")
    parser.add_argument('--output', type=str, help="init_guess_cable.html")

    args = parser.parse_args()
    path_to_env = args.env
    path_to_dbcbs = args.dbcbs
    html_path = args.output
    path_to_result = args.result
    path_to_payload = args.payload 
    
    # load db_cbs states: 
    with open(path_to_env, "r") as f: 
        env = yaml.safe_load(f)
    #load payload data
    with open(path_to_payload, "r") as f: 
        payload_yaml = yaml.safe_load(f)

    # load db_cbs states: 
    with open(path_to_dbcbs, "r") as f: 
        db_cbs_states = yaml.safe_load(f)

    r1_states = np.array(db_cbs_states["result"][0]["states"])
    r2_states = np.array(db_cbs_states["result"][1]["states"])


    r1_actions = np.array(db_cbs_states["result"][0]["actions"])
    r2_actions = np.array(db_cbs_states["result"][1]["actions"])

    # payload states from dbCBS non linear opt
    p0_init = payload_yaml["payload"]

    num_act1 = r1_actions.shape[0]
    num_act2 = r2_actions.shape[0]
    # to find which action is the shortest 
    # and fill it with zero to be the same size as the other action
    if num_act1 > num_act2:
        num_missing_actions = num_act1 - num_act2
        for i in range(num_missing_actions):
            r2_actions = np.append(r2_actions,[[0,0]], axis=0)
    elif num_act2 > num_act1:
        num_missing_actions = num_act2 - num_act1
        for i in range(num_missing_actions):
            r1_actions = np.append(r1_actions,[[0,0]], axis=0)

    states = np.zeros((r1_actions.shape[0]+1,8))
    actions = np.zeros((r1_actions.shape[0],4))

    for k, action in enumerate(actions):
        ac1 = r1_actions[k]
        ac2 = r2_actions[k]
        actions[k] = [ac1[0], ac1[1], ac2[0], ac2[1]]



    for k, state in enumerate(states):
        # the conditions here to check if one robot's state is longer than the other
        # then take the last state of the shortest 
        if k < len(r1_states):
            r1_state = r1_states[k]
        else:
            r1_state = r1_states[-1]    
        if k < len(r2_states):
            r2_state = r2_states[k]
        else:
            r2_state = r2_states[-1]
        th1, th2, dp0, dth1, dth2 = computePayloadSt(r1_state, r2_state, p0_init[k])
        # states[k] = [p0_init[k][0], p0_init[k][1], th1, th2, dp0[0], dp0[1], dth1, dth2]
        # I only use the geometric states for the initial guess
        states[k] = [p0_init[k][0], p0_init[k][1], th1, th2, 0, 0, 0, 0]

    print("payload: ","("+str(len(p0_init))+","+ str(len(p0_init[0]))+")", "actions: ", actions.shape, "states:", states.shape)

    result_yaml = dict()
    result_yaml["result"] = dict()
    result_yaml["result"]["states"] = states.tolist()
    result_yaml["result"]["actions"] = actions.tolist()
    result_yaml["result"]["num_action"] = len(actions.tolist())
    result_yaml["result"]["num_states"] = len(states.tolist())

    saveyaml(path_to_result, result_yaml)

    script = "../scripts/visualize_dintCables.py"
    subprocess.run(["python3",
                    script,
                    "--env", path_to_env,
                    "--result", path_to_result,
                    "--output", html_path])
        
if __name__ == "__main__":
    main()