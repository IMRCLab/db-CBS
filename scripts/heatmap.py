import numpy as np
import matplotlib.pyplot as plt
import yaml
# plot options
fig = plt.figure()
ax = fig.add_subplot(111, aspect="equal")
ax.set_xlim([0, 8])
ax.set_ylim([0, 8])
# get the data
filename_motions = "../dynoplan/expanded_trajs_rev.yaml"
with open(filename_motions) as motions_file:
    motions = yaml.load(motions_file, Loader=yaml.CLoader)
tmp_trajs = motions["trajs"]
print(len(tmp_trajs))
N = len(tmp_trajs)
data = np.zeros((6,6))
states = []
costs = []
cost_list = []
r = np.round(np.random.rand(),1)
g = np.round(np.random.rand(),1)
X, Y = [], []
for i in range(0,N,10):
    state = tmp_trajs[i]["states"]
    costs.append(tmp_trajs[i]["cost"])
    states.append([state[0][0], state[0][1]])
    X.append(state[0][0])
    Y.append(state[0][1])
costs_norm = (costs-np.min(costs))/(np.max(costs)-np.min(costs))
# plot the data
ax.scatter(1, 2.5, s=4, color='r')
ax.scatter(4, 2.5, s=4, color='r')
plt.scatter(x=X, y=Y, s=3, c=costs, cmap="summer") 
plt.colorbar(label="gScore", orientation="vertical") 
plt.show() 

