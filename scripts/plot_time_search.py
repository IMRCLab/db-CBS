import yaml
import matplotlib.pyplot as plt
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib import gridspec
import matplotlib.lines as mlines
# I - for the computation time plot
def read_yaml(file_path):
    # Read and parse the YAML file
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    # time_nearestMotion 
    # time_hfun 
    try:
        return {
            "time_collision_heuristic": data['data']['time_collision_heuristic'],
            "time_collisions": data['data']['time_collisions'],
            "time_nearestNode": data['data']['time_nearestNode'],
            "time_rebuild_focal_set": data['data']['time_rebuild_focal_set'],
            # "time_search": data['data']['time_search'],
        }
    except KeyError as e:
        print(f"Error: Missing expected key {e} in the YAML data.")
        return None

def time_analysis_plot(data_iterations):
    instance_names = {
    "alcove_unicycle_sphere": "alcove-u1",
    "gen_p10_n8_4_hetero": "hetero8",
    "drone4-C": "drone4-C",
    "drone4-R": "drone4-R",
    "drone8-C": "drone8-C",
    "drone8-R": "drone8-R",
    #  "wall8-C": "wall8-C-0.5",
    #  "wall8-R": "wall8-R-0.5",
    }

    # Data for plotting
    categories = {
        'Search-FH': 'time_collision_heuristic',
        'Search-Update FS': 'time_rebuild_focal_set',
        'Search-Collision': 'time_collisions',
        'Search-NN': 'time_nearestNode'
    }
    colors = ['skyblue', 'orange', 'grey', 'red']
    # labels = [f'Iteration {i + 1}' for i in range(len(data_iterations))] # to do: isntance name
    labels = list(instance_names.values())
    # Initialize plot
    fig, ax = plt.subplots()
    width = 0.4  # Bar width
    x_positions = range(len(data_iterations))  # Positions for each bar
    # Create stacked bars for each iteration
    for category_index, (category, key) in enumerate(categories.items()):
        bottoms = [sum(data[categories[c]] for c in list(categories.keys())[:category_index]) for data in data_iterations]
        values = [data[key] for data in data_iterations]
        ax.bar(x_positions, values, width, label=category, color=colors[category_index], bottom=bottoms)
    
    # Add legend, title, and labels
    ax.grid(which='both', axis='x', linestyle='dashed')
    ax.grid(which='major', axis='y', linestyle='dashed')
    ax.set_ylabel("Time [s]")
    ax.set_xticks(x_positions)
    ax.set_xticklabels(labels)
    ax.legend(loc='upper left')
    plt.tight_layout()
    plt.grid(True)
    # Show the plot
    plt.show()


def delta_time_analysis_plot(data_iterations):
    instance_names = [  # small, big delta order for each problem
    # "swap2-u1-0.3", # 100
    # "swap2-u1-0.5",
    "swap4-u1-0.3", # 100
    "swap4-u1-0.5",
    "swap4-c1-0.4",
    "swap4-c1-0.6", # 120
    "swap4-u2-0.3", # 100
    "swap4-u2-0.5"
    ]
    # Data for plotting
    categories = {
        'Search-FH': 'time_collision_heuristic',
        'Search-Update FS': 'time_rebuild_focal_set',
        'Search-Collision': 'time_collisions',
        'Search-NN': 'time_nearestNode'
        # 'Search' : 'time_search'
    }
    colors = [
    '#0072B2',  # Blue
    '#D55E00',  # Vermilion
    '#009E73',  # Bluish Green
    '#E69F00',  # Orange
    # '#CC79A7',  # Reddish Purple
]
    labels = instance_names 
    # Initialize plot
    fig, ax = plt.subplots()
    width = 0.4 # Bar width
    x_positions = range(len(data_iterations))  # Positions for each bar
    # Create stacked bars for each iteration
    for category_index, (category, key) in enumerate(categories.items()):
        bottoms = [sum(data[categories[c]] for c in list(categories.keys())[:category_index]) for data in data_iterations]
        values = [data[key] for data in data_iterations]
        ax.bar(x_positions, values, width, label=category, color=colors[category_index], bottom=bottoms)
    
    # Add legend, title, and labels
    ax.grid(which='both', axis='x', linestyle='dashed')
    ax.grid(which='major', axis='y', linestyle='dashed')
    ax.set_ylabel("Time [ms]")
    ax.set_xticks(x_positions)
    ax.set_xticklabels(labels, rotation=45, ha='right')
    ax.legend(loc='upper left')
    plt.tight_layout()
    plt.grid(True)
    # Show the plot
    plt.show()

import numpy as np
import yaml
import matplotlib.pyplot as plt

def add_node_rewire_bar_chart(a, i):
    folder = "/home/akmarak-laptop/IMRC/db-CBS/results/tro-plots/add_node/"
    
    t_values = {key: [] for key in a}  # Store t values for a1 and a2
    cost_values = {key: [] for key in a}  # Store cost values for a1 and a2
    
    # colors = ['red', 'blue']
    # colors = ['#E69F00', '#56B4E9'] # ['#117733', '#332288'] # green, navy
    colors = ['#005A8C', '#D55E00']
    labels = ['Always Add', 'Rewire']
    i2 = [name[:-1] for name in i] 
    x = np.arange(len(i))  # Positions for bars
    
    for a_instance in a:
        for i_instance in i:
            yaml_file = f"{folder}{a_instance}/{i_instance}/db-ecbs/000/stats.yaml"
            try:
                with open(yaml_file, 'r') as file:
                    data = yaml.safe_load(file)
                    if 'd_t' in data["stats"][0]:
                        t_values[a_instance].append(data["stats"][0]['d_t'])
                        cost_values[a_instance].append(data["stats"][0]['d_cost'])
                    else:
                        print(f"Warning: 't, cost' not found in {yaml_file}")
            except FileNotFoundError:
                print(f"Error: {yaml_file} not found")
                t_values[a_instance].append(None)
                cost_values[a_instance].append(None)
    
    # fig, ax = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    fig, ax = plt.subplots(2, 1, sharex='all', sharey='none')
    bar_width = 0.4  # Width of each bar

    parameters = {'Cost': cost_values, 'Time': t_values}
    
    for idx, (param, values) in enumerate(parameters.items()):
        for j in range(2):  # Two bars per category
            ax[idx].bar(x + j * bar_width, values[a[j]], 
                        color=colors[j], width=bar_width, alpha=0.8, label=labels[j])

        # ax[idx].grid(axis='y', linestyle='dashed')
        ax[idx].grid(which='both', axis='x', linestyle='dashed')
        ax[idx].grid(which='major', axis='y', linestyle='dashed')
        ax[idx].set_ylabel(f"{param} [s]")

    ax[0].legend()
    # X-axis settings
    ax[-1].set_xticks(x + bar_width / 2)
    ax[-1].set_xticklabels(i2)
    # i2 = [
    #     "alcove-u1",
    #     "swap4-u1",
	# 	"swap4-car",
    #     "drone4",
    #     "drone8",
    #     "drone12",
    #     "drone16",
    # ]
    # ax[-1].set_xticklabels(i2)

    plt.tight_layout()
    plt.grid(True)
    # plt.savefig('addnode_rewire.pdf')
    plt.show()

# if the folder has many iterations
def add_cost_and_time_over_robots_plot_itr(a, i, itr):
    # folder = "/home/akmarak-laptop/IMRC/db-CBS/results/add_node/"
    folder = "/home/akmarak-laptop/IMRC/db-CBS/results/heuristics/"
    t_values = {key: [] for key in a}  # Store t values for a1 and a2
    cost_values = {key: [] for key in a}  # Store t values for a1 and a2
    colors = ['green', 'red'] # ['red', 'blue']
    labels = ['L1', 'L2'] # ['Always Add', 'Rewire']
    # x = np.array([1, 2, 3, 4]) 
    x = np.array([1, 2, 3, 4]) 
    for a_instance in a:
        for i_instance in i:
            t = 0
            cost = 0
            valid = False
            for it in range(itr):
                yaml_file = folder + a_instance + "/" + i_instance + "/db-ecbs/00" + str(it) + "/stats.yaml" 
                try:
                    with open(yaml_file, 'r') as file:
                        data = yaml.safe_load(file)
                        # stats = data["stats"]
                        if 'd_t' in data["stats"][0]:
                            t += data["stats"][0]['d_t']
                            cost += data["stats"][0]['d_cost']
                            valid = True
                        else:
                            print(f"Warning: 't, cost' not found in {yaml_file}")
                except FileNotFoundError:
                    print(f"Error: {yaml_file} not found")
                    t_values[a_instance].append(None)  # Keep structure for plotting
                    cost_values[a_instance].append(None)  # Keep structure for plotting
            if(valid):
                t_values[a_instance].append(t / itr)  # take the average
                cost_values[a_instance].append(cost / itr)  
    print(t_values)
    fig, ax = plt.subplots(2, 1, sharex='all', sharey='none')
    for i in range(2):
    #   ax[i].set_xscale('log')
      ax[i].grid(which='both', axis='x', linestyle='dashed')
      ax[i].grid(which='major', axis='y', linestyle='dashed')

    parameters = {'p': cost_values, 't': t_values}
    for idx, (param, values) in enumerate(parameters.items()):
        for i in range(2):  # Two lines for each parameter
            ax[idx].plot(x, values[a[i]], color=colors[i], linewidth=3, alpha=0.8, label=labels[i])

    ax[0].legend()
    ax[0].set_ylabel(r"Cost [s]")
    ax[1].set_ylabel("Time [s]")
    ax[-1].set_xticks(x)
    i2 = [
        # "drone2",
        "drone4",
        "drone8",
        # "drone10",
        "drone12",
        "drone16",
    ]
    ax[-1].set_xticklabels(i2)
    plt.show()


# stem plot style
# if the folder has many iterations
def add_cost_and_time_over_robots_plot_itr_stem(a, i, itr):
    folder = "/home/akmarak-laptop/IMRC/db-CBS/results/tro-plots/heuristics/"
    t_values = {key: [] for key in a}  # Store t values for a1 and a2
    cost_values = {key: [] for key in a}  # Store t values for a1 and a2
    colors = ['#0072B2', '#D55E00'] 
    labels = ['L1', 'L2'] 
    # x = np.array([1, 2, 3, 4]) 
    x = np.arange(1, len(i) + 1)
    for a_instance in a:
        for i_instance in i:
            t = 0
            cost = 0
            valid = False
            for it in range(itr):
                yaml_file = folder + a_instance + "/" + i_instance + "/db-ecbs/00" + str(it) + "/stats.yaml" 
                try:
                    with open(yaml_file, 'r') as file:
                        data = yaml.safe_load(file)
                        # stats = data["stats"]
                        if 'd_t' in data["stats"][0]:
                            t += data["stats"][0]['d_t']
                            cost += data["stats"][0]['d_cost']
                            valid = True
                        else:
                            print(f"Warning: 't, cost' not found in {yaml_file}")
                except FileNotFoundError:
                    print(f"Error: {yaml_file} not found")
                    t_values[a_instance].append(None)  # Keep structure for plotting
                    cost_values[a_instance].append(None)  # Keep structure for plotting
            if(valid):
                t_values[a_instance].append(t / itr)  # take the average
                cost_values[a_instance].append(cost / itr)  
    print(t_values)
    fig, ax = plt.subplots(2, 1, sharex='all', sharey='none')
    for i in range(2):
    #   ax[i].set_xscale('log')
      ax[i].grid(which='both', axis='x', linestyle='dashed')
      ax[i].grid(which='major', axis='y', linestyle='dashed')

    parameters = {'p': cost_values, 't': t_values}
    for idx, (param, values) in enumerate(parameters.items()):
        for i in range(2):  # Two lines for each parameter
            markerline, stemlines, baseline = ax[idx].stem(
                x, values[a[i]],
                linefmt=colors[i],
                markerfmt=f'{colors[i]}',
                basefmt="k-"
            )
            # Optional styling
            # plt.setp(markerline, markersize=8)
            # plt.setp(stemlines, linewidth=2)
    # Create custom legend
    legend_handles = [
        mlines.Line2D([], [], color=colors[0], marker='o', linestyle='-', linewidth=2, label=labels[0]),
        mlines.Line2D([], [], color=colors[1], marker='o', linestyle='-', linewidth=2, label=labels[1])
    ]
    ax[0].legend(handles=legend_handles)
    # ax[0].legend()
    ax[0].set_ylabel(r"Cost [s]")
    ax[1].set_ylabel("Time [s]")
    ax[-1].set_xticks(x)
    i2 = [
        "alcove-u1",
        "swap4-u1",
		"swap4-car",
        "drone4",
        "drone8",
        "drone12",
        "drone16",
    ]
    ax[-1].set_xticklabels(i2)
    plt.tight_layout()
    plt.show()

# to plot robot trajectories in NewralSwarm-2 style
def get_state(X, t):
  if t < len(X):
    return X[t]
  else:
    return X[-1]
  
def plot(filename_env, filename_res):
#   scale = 2.5
#   fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8*scale,2.4*scale),sharex='row', sharey='row', gridspec_kw={'height_ratios': [5,1]})
#   fig = plt.figure(figsize=(8, 6)) 
  gs = gridspec.GridSpec(2, 1, height_ratios=[5, 1]) 
  ax1 = plt.subplot(gs[0])
  ax2 = plt.subplot(gs[1])
  # 1. read the environment, get robot types
  with open(filename_env) as env_file:
    env = yaml.safe_load(env_file)
  robot_types = []
  for r in env["robots"]:
    robot_types.append(r["type"])
  dt = 0.1
  # 2. read the trajectory, adjust the circle radius
  with open(filename_res) as motions_file:
    results = yaml.safe_load(motions_file)
 # get max T
  T = 0
  for i in range(len(results["result"])):
    T = max(T, len(results["result"][i]["states"]))

  for i in range(len(results["result"])):
    states = results["result"][i]["states"]
    X = [s[0] for s in states]
    Y = [s[1] for s in states]
    Z = [s[2] for s in states]
    F = [s[6] for s in states]
    qX = []
    qZ = []
    qU = []
    qV = []
    for k in np.arange(0, len(X)-1, int(0.5 / dt)):
       qX.append(X[k])
       qZ.append(Z[k])
       qU.append((X[k+1] - X[k]) / dt)
       qV.append((Z[k+1] - Z[k]) / dt)
    #    u = (X[k+1] - X[k]) / dt
    #    v = (Z[k+1] - Z[k]) / dt
    #    magnitude = np.sqrt(u**2 + v**2)  # Compute magnitude
    #    qU.append(u / magnitude)
    #    qV.append(v / magnitude)
    robot_rad = 0.1 # assumes a small robot
    # plot trajectory
    line = ax1.plot(X, Z,alpha=0.5)
    color = line[0].get_color()
    if(robot_types[i] == "integrator2_3d_large_v0"):
        robot_rad = 0.15
    # plot outline
    ax1.add_artist(mpatches.Circle([states[int(T/2)][0], states[int(T/2)][2]], robot_rad, color=color, alpha=0.4))
    # ax1.quiver(qX, qZ, qU, qV, angles='xy', scale_units='xy', scale=5, color='r', width=0.005)
    ax1.quiver(qX,qZ,qU,qV,angles='xy', scale_units='xy',scale=5, color=color, width=0.01)
    ax1.set_aspect('auto')
    ax1.set_xlim([-0.5,1])
    ax1.set_ylim([1,2])
    ax1.set_xticklabels([])
    ax1.set_yticklabels([])
    ax1.set_xlabel('X')
    ax1.set_ylabel('Z')
    ax2.plot([i * dt for i in range(len(X))], F, color)
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel(r"$\psi$ [g]")

  plt.tight_layout()
#   plt.savefig('swap3_drone.pdf')
  plt.show()


def main():
   
    # 1. Time analysis plot
    # path = "/home/akmarak-laptop/IMRC/db-CBS/results/tro-plots/time/"
    # instances = ["alcove_unicycle_sphere", "gen_p10_n8_4_hetero", "drone4-C", "drone4-R","wall8-C", "wall8-R" ] 
    # algorithms = [
    #     "db-ecbs-residual",
    #     "db-ecbs-conservative"
    # ]
    # file_name = "time_search.yaml"
    # file_paths = []
    # # Generate paths by combining the base path, instance, and algorithm
    # for algo in algorithms:
    #     for instance in instances:
    #         file_paths.append(path + instance + "/" + algo + "/000/" + file_name)
    # # Read data from each YAML file
    # data_iterations = []
    # for file_path in file_paths:
    #     if os.path.exists(file_path):
    #         data = read_yaml(file_path)
    #         if data:
    #             data_iterations.append(data)

    # time_analysis_plot(data_iterations)

    # 2. plot for always add vs. rewire
    # a = ["always_add", "rewire"]
    # i = [
    #     "drone2c",
    #     "drone4c",
    #     "drone8c",
    #     "drone10c",
    # ]
    # add_node_rewire_bar_chart(a, i)
    
    # 2. plot for L1, L2 comparison
    # a = ["L1", "L2"] # ["always_add", "rewire"]
    # i = [
    #     "alcove_unicycle",
    #     "swap4_unicycle",
	# 	"swap4_trailer",
    #     "drone4c",
    #     "drone8c",
    #     "drone12c",
    #     "drone16c",
    # ]
    # # add_cost_and_time_over_robots_plot_itr(a, i, 2)
    # add_cost_and_time_over_robots_plot_itr_stem(a, i, 2)

    # 3. NeuralSwarm2 plot
    # env_file = "/home/akmarak-laptop/IMRC/db-CBS/example/swap3_drone.yaml"
    # res_file = "/home/akmarak-laptop/IMRC/db-CBS/results/tro-plots/swap3_drone/db-ecbs/000/result_dbecbs_opt.yaml"
    # plot(env_file, res_file)

    # 4. delta value analysis w.r.t time
    # path = "/home/akmarak-laptop/IMRC/db-CBS/results/tro-plots/delta-time/"
    # instances = ["swap4_unicycle", "swap4_trailer", "swap4_unicycle2"] # "drone4-C", "drone4-R"
    # folder = [
    #     "small-delta",
    #     "big-delta"
    # ]
    # file_name = "time_search.yaml"
    # file_paths = []
    # # Generate paths by combining the base path, instance, and algorithm
    # for instance in instances:
    #     for f in folder:
    #         file_paths.append(path + f + "/" + instance + "/db-ecbs/000/" + file_name)
    # # Read data from each YAML file
    # data_iterations = []
    # for file_path in file_paths:
    #     if os.path.exists(file_path):
    #         data = read_yaml(file_path)
    #         if data:
    #             data_iterations.append(data)
    #         else:
    #             print(file_path)
    #     else: 
    #         print(file_path)
    # delta_time_analysis_plot(data_iterations)

    
if __name__ == "__main__":
  main()