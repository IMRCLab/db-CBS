import numpy as np
import yaml
import argparse
np.set_printoptions(linewidth=np.inf)
np.set_printoptions(suppress=True)

# This script is used to test the optimization using optimized solutions from the 
# rigid links model in dynoplan as an initial guess for the mujoco model of quads with tendons  

def loadyaml(file_dir):
    with open(file_dir, "r") as f: 
        return yaml.safe_load(f)

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

def get_quad_st(dyno_state, i, num_robots, l=0.5):
    quad_pose = np.zeros(7,)
    p0 = dyno_state[0:3]
    qci = dyno_state[9 + 6*i : 9 + 6*i + 3] #cable direction
    pos = p0 -l*qci
    quat = dyno_state[9 + 6*num_robots + 7*i : 9 + 6*num_robots + 7*i + 4]
    quad_pose[0:3] =  pos
    quad_pose[3:7] = [1,0,0,0] #quat
    
    quad_vel = np.zeros(6,)
    wci = dyno_state[9 + 6*i + 3: 9 + 6*i + 6]
    qcidot = np.cross(wci, qci)
    v0 = dyno_state[3:6]
    quad_vel[0:3] = [0,0,0] #v0 - l*qcidot
    quad_vel[3:6] = [0,0,0] #dyno_state[9+6*num_robots + 7*i + 4 : 9+6*num_robots + 7*i + 7]

    return quad_pose, quad_vel

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dynoplan', type=str, help="dynoplan_path")
    parser.add_argument('--result', type=str, help="output path")
    parser.add_argument('--num_robots', type=int, help="number of robots transporting the load")

    args = parser.parse_args()
    num_robots = args.num_robots
    path_to_dynoplan = args.dynoplan

    # load dynoplan-states
    dynoplan_states =loadyaml(path_to_dynoplan)
    dyno_states  = np.array(dynoplan_states["result"]["refstates"], dtype=np.float64) 
    dyno_states[:,2] += 0.5
    # u_nominal = 0.0356*9.81/4
    dyno_actions = np.array(dynoplan_states["result"]["actions_d"], dtype=np.float64)
    
    nb = num_robots + 1
    nq = 7*nb
    nv = 6*nb
    mujoco_actions = dyno_actions.copy()
    mujoco_states = np.zeros((dyno_states.shape[0], nq+nv))
    mujoco_states[:, 0:3] = dyno_states[:,0:3] # copy positions
    mujoco_states[:, 3:7] = [0,0,0,1] # always copy unit quaternions for the payload since we only consider point mass case for now, #TODO: add flag for rigid payload to update this also
    mujoco_states[:,nq : nq+3] = dyno_states[:,3:6] # copy velocities


    for k, dyno_state in enumerate(dyno_states):
        for i in range(num_robots):
            quad_pos, quad_vel = get_quad_st(dyno_state, i, num_robots)
            mujoco_states[k, 7*(i+1):7*(i+1) + 7] = quad_pos
            mujoco_states[k, 7*nb + 6*(i+1) : 7*nb + 6*(i+1) + 6] = quad_vel
    mujoco_dict = dict()
    mujoco_dict["result"] = dict()
    mujoco_dict["result"]["states"] = mujoco_states.tolist()  
    mujoco_dict["result"]["actions"] = mujoco_actions.tolist() 
    saveyaml(args.result, mujoco_dict)

if __name__ == "__main__":
    main()