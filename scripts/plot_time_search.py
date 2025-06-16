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
# read stats file, used for optimization/discrete time plot 
def read_yaml_stats(file_path):
    # Read and parse the YAML file
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    try:
        return {
            "discrete-search": data['stats']['duration_discrete'],
            "optimization": data['stats']['duration_opt'],
        }
    except KeyError as e:
        print(f"Error: Missing expected key {e} in the YAML data.")
        return None

# analysis for optimization complexity, which we believe is O(K(nx+nu)^3)
def analysis_optimization_complexity(itr = 5):
    folder = "/home/akmarak-laptop/IMRC/db-CBS/results/tro-plots/optimization_complexity/"
    file_name = "result_dbecbs.yaml" # only discrete search is of interest
    stats_name = "stats.yaml"
    mean_max_timesteps = []
    mean_duration_opts = []
    for i in range(10):
        max_timesteps = []
        duration_opts = []
        for it in range(itr):
            yaml_path = os.path.join(folder, f"gen_p10_n2_{i}_hetero", "db-ecbs", f"00{it}", file_name)
            stats_path = os.path.join(folder, f"gen_p10_n2_{i}_hetero", "db-ecbs", f"00{it}", stats_name)
            try:
                with open(yaml_path, 'r') as file:
                    data = yaml.safe_load(file)
                with open(stats_path, 'r') as stats_file:
                    stats = yaml.safe_load(stats_file)    
                stats = stats["stats"][0]
                duration_opt = stats["duration_opt"]

                max_timestep = 0
                for res_i in range(len(data["result"])):
                    max_timestep = max(max_timestep, len(data["result"][res_i]["actions"]))

                max_timesteps.append(max_timestep)
                duration_opts.append(duration_opt)
            except FileNotFoundError:
                print(f"Error: File not found: {yaml_path}")

        # Compute means for this i
        mean_max_timestep = np.mean(max_timesteps) if max_timesteps else float('nan')
        mean_duration_opt = np.mean(duration_opts) if duration_opts else float('nan')

        print(f"i={i}: Mean max_timestep = {mean_max_timestep:.2f}, Mean duration_opt = {mean_duration_opt:.2f}")
        mean_max_timesteps.append(mean_max_timestep)
        mean_duration_opts.append(mean_duration_opt)

    # Now sort mean_max_timesteps and reorder mean_duration_opts accordingly
    sorted_pairs = sorted(zip(mean_max_timesteps, mean_duration_opts), key=lambda x: x[0])
    mean_max_timesteps_sorted, mean_duration_opts_sorted = zip(*sorted_pairs)

    # Convert back to list if needed
    mean_max_timesteps_sorted = list(mean_max_timesteps_sorted)
    mean_duration_opts_sorted = list(mean_duration_opts_sorted)

    print("Sorted mean_max_timesteps:", mean_max_timesteps_sorted)
    print("Reordered mean_duration_opts:", mean_duration_opts_sorted)    
    plt.plot(mean_max_timesteps_sorted[:8], mean_duration_opts_sorted[:8])
    plt.xlabel('Time Steps')
    plt.ylabel('Optimization Runtime [s]')
    plt.show()

 # analysis for optimization/discrete search time   
def time_analysis_plot_optimization(instances, itr, dec_var=False):
    folder = "/home/akmarak-laptop/IMRC/db-CBS/results/tro-plots/optimization_scalability/"
    time_keys = ["duration_discrete", "duration_opt"]
    labels = ["Discrete Search", "Optimization"]
    colors = ['orange', 'grey']

    means = {key: [] for key in time_keys}
    stds = {key: [] for key in time_keys}
    timesteps_means = []  # mean value for several iterations for each problem instance
    instance_labels = []

    for instance in instances:
        instance_labels.append(instance)
        durations = {key: [] for key in time_keys}
        timesteps = []
        for it in range(itr):
            yaml_path = os.path.join(folder, instance, "db-ecbs-residual", f"00{it}", "stats.yaml")
            try:
                with open(yaml_path, 'r') as file:
                    data = yaml.safe_load(file)

                    if isinstance(data, dict) and "stats" in data and isinstance(data["stats"], list) and data["stats"]:
                        stats = data["stats"][0]
                        if all(key in stats for key in time_keys):
                            for key in time_keys:
                                durations[key].append(stats[key])
                        else:
                            print(f"Warning: Missing one of {time_keys} in {yaml_path}")
                        if dec_var:
                            timesteps.append(stats["discrete cost"])    
                    else:
                        print(f"Warning: Invalid or empty stats in {yaml_path}")

            except FileNotFoundError:
                print(f"Error: File not found: {yaml_path}")
            except yaml.YAMLError as e:
                print(f"YAML error in {yaml_path}: {e}")
            except Exception as e:
                print(f"Unexpected error reading {yaml_path}: {e}")
            
        # Compute mean and std deviation
        for key in time_keys:
            # if durations[key]:
                # print(durations[key])
                # means[key].append(np.mean(durations[key]))
                # stds[key].append(np.std(durations[key]))
            # to get stats in [minute]
            if durations[key]:
                mean_minutes = np.mean(durations[key]) / 60
                std_minutes = np.std(durations[key]) / 60
                means[key].append(mean_minutes)
                stds[key].append(std_minutes)
            else:
                means[key].append(0)
                stds[key].append(0)
        if dec_var:
            timesteps_mean = np.mean(timesteps)
            timesteps_means.append(timesteps_mean)
    if dec_var: 
        nx = 3
        nu = 3
        timesteps_means = [round(x) for x in timesteps_means]
        x = [value * (nx + nu) for value in timesteps_means] # assumes double integrator with u = (ax, ay, az)
        instance_labels = [f"{a}-{b}" for a, b in zip(instance_labels, x)]
    else:    
        x = np.arange(len(instances))
    fig, ax = plt.subplots()
    for i, key in enumerate(time_keys):
        mean_vals = np.array(means[key])
        std_vals = np.array(stds[key])
        ax.plot(x, mean_vals, color=colors[i], label=labels[i], linewidth=2)
        ax.fill_between(
            x,
            mean_vals - std_vals,
            mean_vals + std_vals,
            color=colors[i],
            alpha=0.3
        )
    # exit()
    ax.set_xticks(x)
    ax.set_xticklabels(instance_labels, rotation = 45, ha='right')
    ax.set_ylabel("Time [min]")
    ax.legend()
    # ax.grid(True, linestyle='dashed', alpha=0.5)
    ax.grid(which='both', axis='x', linestyle='dashed')
    ax.grid(which='both', axis='y', linestyle='dashed')
    plt.tight_layout()
    plt.show()

# optimization scalability (time vs. decision varibales), complexity (time vs. number of timesteps)
def combined_optimization_analysis(
    plot_complexity=False,
    plot_scaling=False,
    complexity_itr=5,
    scaling_instances=None,
    scaling_itr=5,
    dec_var=False
):
    if scaling_instances is None:
        scaling_instances = []

    fig, axs = None, []
    num_plots = int(plot_complexity) + int(plot_scaling)
    if num_plots > 1:
        fig, axs = plt.subplots(num_plots, 1, figsize=(8, 5 * num_plots))
        axs = axs if isinstance(axs, np.ndarray) else [axs]

    plot_index = 0

    # --- Complexity Plot ---
    if plot_complexity:
        folder = "/home/akmarak-laptop/IMRC/db-CBS/results/tro-plots/optimization_complexity/"
        file_name = "result_dbecbs.yaml"
        stats_name = "stats.yaml"
        mean_max_timesteps = []
        mean_duration_opts = []

        for i in range(10):
            max_timesteps = []
            duration_opts = []
            for it in range(complexity_itr):
                yaml_path = os.path.join(folder, f"gen_p10_n2_{i}_hetero", "db-ecbs", f"00{it}", file_name)
                stats_path = os.path.join(folder, f"gen_p10_n2_{i}_hetero", "db-ecbs", f"00{it}", stats_name)
                try:
                    with open(yaml_path, 'r') as file:
                        data = yaml.safe_load(file)
                    with open(stats_path, 'r') as stats_file:
                        stats = yaml.safe_load(stats_file)["stats"][0]
                    duration_opt = stats["duration_opt"]
                    max_timestep = max(len(res["actions"]) for res in data["result"])
                    max_timesteps.append(max_timestep)
                    duration_opts.append(duration_opt)
                except FileNotFoundError:
                    print(f"Error: File not found: {yaml_path}")
                except Exception as e:
                    print(f"Error reading files for i={i}, it={it}: {e}")

            mean_max_timesteps.append(np.mean(max_timesteps) if max_timesteps else float('nan'))
            mean_duration_opts.append(np.mean(duration_opts) if duration_opts else float('nan'))

        sorted_pairs = sorted(zip(mean_max_timesteps, mean_duration_opts), key=lambda x: x[0])
        mean_max_timesteps_sorted, mean_duration_opts_sorted = zip(*sorted_pairs)
        mean_max_timesteps_sorted = list(mean_max_timesteps_sorted)
        mean_duration_opts_sorted = list(mean_duration_opts_sorted)

        ax = axs[plot_index] if num_plots > 1 else plt
        ax.tick_params(axis='both', labelsize=11) # little bigger ticks
        ax.plot(mean_max_timesteps_sorted[:8], mean_duration_opts_sorted[:8])
        ax.set_xlabel('Time Steps')
        ax.set_ylabel('Optimization Runtime [s]')
        # ax.set_title("Optimization Complexity")
        # ax.grid(True)
        ax.grid(which='both', axis='x', linestyle='dashed')
        ax.grid(which='both', axis='y', linestyle='dashed')
        plot_index += 1

    # --- Scaling Plot ---
    if plot_scaling:
        folder = "/home/akmarak-laptop/IMRC/db-CBS/results/tro-plots/optimization_scalability/"
        time_keys = ["duration_discrete", "duration_opt"]
        labels = ["Discrete Search", "Optimization"]
        colors = ['orange', 'grey']
        means = {key: [] for key in time_keys}
        stds = {key: [] for key in time_keys}
        timesteps_means = []
        instance_labels = []

        for instance in scaling_instances:
            # instance_labels.append(instance)
            instance_labels.append(instance[:-1])
            durations = {key: [] for key in time_keys}
            timesteps = []
            for it in range(scaling_itr):
                yaml_path = os.path.join(folder, instance, "db-ecbs-residual", f"00{it}", "stats.yaml")
                try:
                    with open(yaml_path, 'r') as file:
                        data = yaml.safe_load(file)
                        if isinstance(data, dict) and "stats" in data and isinstance(data["stats"], list) and data["stats"]:
                            stats = data["stats"][0]
                            if all(key in stats for key in time_keys):
                                for key in time_keys:
                                    durations[key].append(stats[key])
                            if dec_var:
                                timesteps.append(stats["discrete cost"])
                except FileNotFoundError:
                    print(f"Error: File not found: {yaml_path}")
                except yaml.YAMLError as e:
                    print(f"YAML error in {yaml_path}: {e}")
                except Exception as e:
                    print(f"Unexpected error reading {yaml_path}: {e}")

            for key in time_keys:
                if durations[key]:
                    means[key].append(np.mean(durations[key]) / 60)  # in minutes
                    stds[key].append(np.std(durations[key]) / 60)
                else:
                    means[key].append(0)
                    stds[key].append(0)
            if dec_var:
                timesteps_means.append(np.mean(timesteps))

        if dec_var:
            nx = 3
            nu = 3
            timesteps_means = [round(x) for x in timesteps_means]
            x = [value * (nx + nu) for value in timesteps_means]
            instance_labels = [f"{a}-{b}" for a, b in zip(instance_labels, x)]
        else:
            x = np.arange(len(scaling_instances))

        ax = axs[plot_index] if num_plots > 1 else plt
        for i, key in enumerate(time_keys):
            mean_vals = np.array(means[key])
            std_vals = np.array(stds[key])
            ax.plot(x, mean_vals, color=colors[i], label=labels[i], linewidth=2)
            ax.fill_between(x, mean_vals - std_vals, mean_vals + std_vals, color=colors[i], alpha=0.3)

        ax.tick_params(axis='both', labelsize=11) # little bigger ticks
        ax.set_xticks(x)
        ax.set_xticklabels(instance_labels, rotation=45, ha='right')
        ax.set_ylabel("Time [min]")
        # ax.set_title("Optimization Scalability")
        ax.legend()
        ax.grid(which='both', axis='x', linestyle='dashed')
        ax.grid(which='both', axis='y', linestyle='dashed')

    if num_plots > 1:
        plt.tight_layout()
        plt.show()
    elif num_plots == 1:
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
    
    # fig, ax = plt.subplots(2, 1, sharex='all', sharey='none')
    fig, ax = plt.subplots(2, 1, sharex='all', sharey='none')
    bar_width = 0.3  # Width of each bar

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
   
    # 1. Time analysis plot for optimization/discrete time
    # i = [
        # "drone2c",
        # "drone4c",
        # "drone8c",
        # "drone10c",
        # "drone12c",
        # "drone16c",
    # ]
    # time_analysis_plot_optimization(i, itr=5, dec_var=True) # considers average time, itr = 5

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

    # 5. optimization complexity anaylsis
    # analysis_optimization_complexity(3) # time steps vs. optimization runtime for gen2_hetero case

    # 6. combined scalability, complexity analysis. Scalability - drone examples, Complexity - gen_2_hetero
    instances = [
        "drone2c",
        "drone4c",
        "drone8c",
        "drone10c",
        "drone12c",
        "drone16c",
    ]
    combined_optimization_analysis(
        plot_complexity=True,
        plot_scaling=True,
        scaling_instances=instances,
        scaling_itr=5,
        dec_var=True)

    
if __name__ == "__main__":
  main()