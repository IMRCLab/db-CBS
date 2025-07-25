import yaml
import matplotlib.pyplot as plt
from pathlib import Path
import numpy as np
import matplotlib.ticker as ticker

from matplotlib.backends.backend_pdf import PdfPages
import subprocess
# Configuration
environments = [
    # "window_2robots_unicycle",
    # "window_3robots_unicycle",
    # "window_2robots_unicycle",
    # "window_2robots_unicycle",
    # "window_3robots_unicycle",
    # "window_4robots_unicycle",

    # "window_2robots",
    # "forest_2robots",
    "window_2robots",
    "window_3robots",
    "window_4robots",
    # "window_3robots",
    # "window_4robots",
    # "forest_4robots",


    # "window_5robots_unicycle",
    # "forest_6robots_unicycle",
    # "window_4robots_unicycle",
    # "window_5robots_unicycle",
]  # Add more environments as needed
results_path = "../stats_db_good/"
trials = [i for i in range(10)]

# Colors for plotting
meancolors = ['r', 'g', 'b', 'm', 'c']  # Extend if needed
stdcolors = ['r', 'g', 'b', 'm', 'c']

# Initialize PDF
# with PdfPages('result_logscale.pdf') as pdf:
T = 350
dt = 0.1
fig, ax = plt.subplots(figsize=(6, 5))  # Single figure for all plots
ax.grid('True', which='both', axis='x', linestyle='dashed')
ax.grid(which='major', axis='y', linestyle='dashed')
for idx, env in enumerate(environments): 
    costs = []
    # Process each trial
    for trial in trials:
        costs_per_run = np.zeros(int(T / dt)) * np.nan
        env_path = Path(results_path + env + "/" + "00" + str(trial) + "/iteration_cost.yaml")
        if env_path.exists():
            with open(env_path, 'r') as f:
                data = yaml.safe_load(f)
                runs = data["runs"]
            total_time_per_iteration = runs[0]["duration_opt"] + runs[0]["duration_discrete"]
            for k, run in enumerate(runs):
                if k == 0:
                    index = int(total_time_per_iteration / dt)
                else:
                    total_time_per_iteration += run["duration_opt"] + run["duration_discrete"]
                    index = int(total_time_per_iteration / dt)
                    # costs_per_run[index:] = run["cost_joint"]

                costs_per_run[index:] = run["cost_joint"] 
        costs.append(costs_per_run)
    costs = np.array(costs)
    times = np.arange(0, T, dt)


    rs = costs.shape[0]
    cs = costs.shape[1]
    indices = []
    for c in range(cs):
        nanNums = 0
        for r in range(rs):
            if np.isnan(costs[r, c]):
                nanNums += 1
        if nanNums > 5:
            # print("true", nanNums) 
            indices.append(c)

    costs = np.delete(costs, indices, axis=1)
    times = np.delete(times, indices, axis=0)
    costs = costs[:, int(cs/12):]
    times = times[int(cs/12):]
    # exit()
    mean = np.nanmean(costs, axis=0)
    std = np.nanstd(costs, axis=0)

    ax.plot(times, mean, label=env, color=meancolors[idx % len(meancolors)], lw=1.5)
    ax.fill_between(times, mean+std, mean-std,color=stdcolors[idx % len(stdcolors)], alpha=0.1)
    ax.legend()
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Cost")

    ax.set_xscale('log')  # Keep logarithmic scale
    # ax.xaxis.set_major_locator(ticker.LogLocator(base=10.0))  # Ensure ticks are at powers of 10
    # ax.xaxis.set_minor_locator(ticker.LogLocator(base=10.0, subs=np.arange(2, 10) * 0.1, numticks=10))  # Minor ticks
    # ax.xaxis.set_major_formatter(ticker.LogFormatterSciNotation())  # Format as 10^n
    # Save to PDF
    result_path =  Path("..") 
    fig.savefig(result_path / 'plot1.pdf')

    subprocess.call(['pdfcrop', result_path / 'plot1.pdf', result_path / 'plot1.pdf'])
    plt.close()

