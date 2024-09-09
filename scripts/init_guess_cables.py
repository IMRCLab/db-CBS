import sys
sys.path.append('./')
import numpy as np
import yaml
from pathlib import Path
sys.path.append(str(Path(__file__).parent / "visualize_dintCables.py"))
sys.path.append(str(Path(__file__).parent))
# import robot_python
print(sys.path)
from visualize import main
import subprocess


#### This script generates the init guess for the plan of the two single robots to the joint robot
#### moreover, it saves an html file for the meshcat animation of this initguess
### from the main directory
### cd buildRelease 
### python3 ../scripts/init_guess_cables.py
### init_guess_cables.yaml is created + a visualization for the init guess: cables_integrator2_2d_window.html

def saveyaml(file_dir, data):
    with open(file_dir, 'w') as f:
        yaml.safe_dump(data, f, default_flow_style=None)

def computePayloadSt(r1, r2, r0):
    # compute the states from the estimated payload and the robots positions coming from dbCBS
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
    return np.array(vec)/np.linalg.norm(np.array(vec))

def norm(vec):
    return np.linalg.norm(np.array(vec))


videoname = "cables_integrator2_2d_window"
html_path = f"../{videoname}.html"
path_to_env = "../example/cables_integrator2_2d_window.yaml"
path_to_dbcbs = "../2integrator2_2d_window_dbcbs.yaml"
path_to_payload = "../window_payload.yaml"
tmp = True
path_to_result = "../../../init_guess_cables.yaml"
if tmp:
    path_to_result = "../init_guess_cables.yaml"

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
# to find which action is the shortest and fill it with zero to be the same size as the other action
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
    # actions[k] = [0, 0, 0, 0]

# r = robot_python.robot_factory_with_env(model_yaml, path_to_env)
# for k, state in enumerate(states):
#     if k < len(states)-1:
#         print(states[k+1], states[k], actions[k])
#         r.step(states[k+1], states[k], actions[k], 0.1)


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
    states[k] = [p0_init[k][0], p0_init[k][1], th1, th2, 0, 0, 0, 0]

print("payload: ","("+str(len(p0_init))+","+ str(len(p0_init[0]))+")", "actions: ", actions.shape, "states:", states.shape)

# actions = np.zeros((256,4))
# # for k in range(actions.shape[0]):
# #     actions[k] = [0,0,0,0]
# import numpy as np
# import matplotlib.pyplot as plt

# def interpolate_states(states, new_size, wrap_indices):
#     """
#     Linearly interpolates the given states to a new size with consideration for angle normalization.
    
#     Parameters:
#         states (np.ndarray): The original state matrix of shape (n_states, n_samples).
#         new_size (int): The new number of samples after interpolation.
#         wrap_indices (list): List of state indices that are angles and need wrapping.
    
#     Returns:
#         np.ndarray: Interpolated state matrix of shape (n_states, new_size).
#     """
#     # Time vectors for original and interpolated data
#     old_time = np.linspace(0, 1, states.shape[1])
#     new_time = np.linspace(0, 1, new_size)
#     print(states.shape)
#     print(states.shape[0], new_size)
#     # exit()
#     # Initialize the interpolated state matrix
#     new_states = np.zeros((states.shape[0], new_size))
    
#     for i in range(states.shape[0]):
#         if i in wrap_indices:
#             # Unwrap angles for linear interpolation
#             unwrapped_angles = np.unwrap(states[i])
#             interpolated_angles = np.interp(new_time, old_time, unwrapped_angles)
#             # Wrap angles back into the -pi to pi range
#             new_states[i] = (interpolated_angles + np.pi) % (2 * np.pi) - np.pi
#         else:
#             # Linear interpolation for non-angular states
#             new_states[i] = np.interp(new_time, old_time, states[i])
    
#     return new_states

# def plot_states(original_states, interpolated_states):
#     """
#     Plots the original and interpolated states.
    
#     Parameters:
#         original_states (np.ndarray): The original state matrix.
#         interpolated_states (np.ndarray): The interpolated state matrix.
#     """
#     n_states = original_states.shape[0]
#     print(n_states)
#     fig, axes = plt.subplots(nrows=n_states, ncols=2, figsize=(12, 2 * n_states))
    
#     for i in range(n_states):
#         axes[i, 0].plot(original_states[i], label='Original')
#         axes[i, 1].plot(interpolated_states[i], label='Interpolated')
        
#         axes[i, 0].set_title(f'State {i+1} Original')
#         axes[i, 1].set_title(f'State {i+1} Interpolated')
        
#         axes[i, 0].legend()
#         axes[i, 1].legend()
    
#     plt.tight_layout()
#     plt.show()

# # Example usage
# n_states = 8
# n_samples = states.shape[0]
# new_samples = n_samples + 40

# # Indices 2 and 3 are angles (th1 and th2)
# wrap_indices = [2, 3]
# # Interpolate states
# interpolated_states = interpolate_states(states.T, new_samples, wrap_indices)
# interpolated_actions = interpolate_states(actions.T, new_samples-1, [])
# Plot original and interpolated states
# plot_states(states.T, interpolated_states)




result_yaml = dict()
result_yaml["result"] = dict()
# result_yaml["result"]["states"] = interpolated_states.T.tolist()
# result_yaml["result"]["actions"] = interpolated_actions.T.tolist()
result_yaml["result"]["states"] = states.tolist()
result_yaml["result"]["actions"] = actions.tolist()
result_yaml["result"]["num_action"] = len(actions.tolist())
result_yaml["result"]["num_states"] = len(states.tolist())

saveyaml(path_to_result, result_yaml)

script = "../scripts/visualize_dintCables.py"
# script = "../../../scripts/visualize_dintCables.py"
subprocess.run(["python3",
				script,
				"--env", path_to_env,
				"--result", path_to_result,
				"--output", html_path])
	
