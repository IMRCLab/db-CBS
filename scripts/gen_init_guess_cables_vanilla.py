import numpy as np      
import yaml

def saveyaml(file_dir, data):
    with open(file_dir + '.yaml', 'w') as f:
        yaml.safe_dump(data, f, default_flow_style=None)

def saveyaml2(file_dir, data):
    with open(file_dir + '.yaml', 'w') as f:
        yaml.safe_dump(data, f, default_flow_style=False)


state_length = 400
ths1 = np.linspace(-1.1,1.1, state_length)
ths2 = np.linspace(1.1,-1.1, state_length)
# ths1 = np.zeros((state_length,))
# ths1 = 1.25*np.ones((state_length,))
# ths2 = -1.25*np.ones((state_length,))

x = np.linspace(-1, 1, state_length)
y = np.zeros_like(x)
u1x = 0.1*np.ones((state_length-1))
u1y = 0.1*np.ones_like(u1x)
u2x = 0.1*np.ones((state_length-1))
u2y = 0.1*np.ones_like(u2x)


states = np.zeros((state_length,8))
actions = np.zeros((state_length-1,4))

for k, _ in enumerate(states):
    states[k] = np.array([x[k], y[k], ths1[k], ths2[k], 0, 0, 0, 0])

for k, _ in enumerate(actions):
    actions[k] = np.array([u1x[k], u1y[k], u2x[k], u2y[k]])


gentraj = dict()
gentraj['cost'] = 10
gentraj['feasible'] = 0
gentraj['result'] = dict()
gentraj['result']['states'] = states.tolist()
gentraj['result']['actions'] = actions.tolist()

saveyaml('../init_guess_cables',gentraj)