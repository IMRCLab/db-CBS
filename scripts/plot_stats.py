#!/usr/bin/env python3
import argparse
from mimetypes import init
import numpy as np
import matplotlib.pyplot as plt
import yaml
from matplotlib.backends.backend_pdf import PdfPages
# from matplotlib.backends.backend_pgf import PdfPages
from matplotlib.cm import get_cmap
from collections import defaultdict

class Report:
  def __init__(self, filename, T, dt):

    tex_fonts = {
        # Use LaTeX to write all text
        "text.usetex": True,
        "font.family": "serif",
        # "axes.labelsize": 12,
        "font.size": 11,
        # Make the legend/label fonts a little smaller
        # "legend.fontsize": 10,
        # "xtick.labelsize": 10,
        # "ytick.labelsize": 10
    }

    plt.rcParams.update(tex_fonts)


    self.pp = PdfPages(filename)
    self.fig = None
    cmap = get_cmap("Dark2")
    self.alg_dict = {
      'sst': {'idx': 0, 'color': cmap.colors[0], 'name': 'SST*'},
      'sbpl': {'idx': 1, 'color': cmap.colors[1], 'name': 'SBPL'},
      'komo': {'idx': 2, 'color': cmap.colors[2], 'name': 'geom. RRT*+KOMO'},
      'dbAstar-komo': {'idx': 3, 'color': cmap.colors[3], 'name': 'kMP-db-A*'},
    }
    self.color_dict = {
      'sst': cmap.colors[0],
      'sbpl': cmap.colors[1],
      'komo': cmap.colors[2],
      'dbAstar-komo': cmap.colors[3],
    }
    self.T = T
    self.dt = dt
    self.times = np.arange(0, self.T, self.dt)
    self.stats = dict()

  def load_stat_files(self, exp_name, algo, filenames):
    costs = []
    for filename in filenames:
      costs.append(load_data(filename, self.T, self.dt))

    # convert to 2D array
    costs = np.array(costs)
    key = (exp_name, algo)
    self.stats[key] = costs

  def add_time_cost_plot(self, exp_name):
    self._add_page()
    self.fig, self.ax = plt.subplots()
    self.ax.set_title(exp_name)

    for (exp_name_stats, algo), costs in self.stats.items():
      if exp_name_stats != exp_name:
        continue
      #   mean = costs.mean(axis=0)
      mean = np.nanmean(costs, axis=0)
      #   std = costs.std(axis=0)
      std = np.nanstd(costs, axis=0)

      self.ax.plot(self.times, mean, label=self.alg_dict[algo]['name'], color=self.color_dict[algo])
      self.ax.fill_between(self.times, mean+std, mean-std, color=self.color_dict[algo], alpha=0.5)
    self.ax.legend()
    self.ax.set_xlabel("Time [s]")
    self.ax.set_ylabel("Cost [s]")

  def add_success_over_time_plot(self, exp_name):
    self._add_page()
    self.fig, self.ax = plt.subplots()
    self.ax.set_title(exp_name)
    self.ax.set_xscale('symlog')
    self.ax.grid(which='both', axis='x', linestyle='dashed')#color='r', linestyle='-', linewidth=2)
    self.ax.grid(which='major', axis='y', linestyle='dashed')#color='r', linestyle='-', linewidth=2)

    for (exp_name_stats, algo), costs in self.stats.items():
      if exp_name_stats != exp_name:
        continue

      success = np.count_nonzero(~np.isnan(costs), axis=0) / 5 * 100

      self.ax.plot(self.times, success, label=self.alg_dict[algo]['name'], color=self.color_dict[algo], linewidth=3, alpha=0.8)
    self.ax.legend()
    self.ax.set_xlabel("Time [s]")
    self.ax.set_ylabel("Success [%]")


  def add_success_and_cost_over_time_plot(self, exp_name):
    self._add_page()
    self.fig, ax = plt.subplots(2, 1, sharex='all', sharey='none')
    ax[0].set_title(exp_name)

    for i in range(2):
      ax[i].set_xscale('log')
      ax[i].grid(which='both', axis='x', linestyle='dashed')
      ax[i].grid(which='major', axis='y', linestyle='dashed')

    for (exp_name_stats, algo), costs in self.stats.items():
      if exp_name_stats != exp_name:
        continue

      success = np.count_nonzero(~np.isnan(costs), axis=0) / 10 * 100
      median = np.nanmedian(costs, axis=0)
      percentileH = np.nanpercentile(costs, 75, axis=0)
      percentileL = np.nanpercentile(costs, 25, axis=0)
      # std = np.nanstd(costs, axis=0)

      ax[1].plot(self.times, median, label=self.alg_dict[algo]['name'], color=self.color_dict[algo], linewidth=3, alpha=0.8)
      ax[1].fill_between(self.times, percentileH, percentileL, color=self.color_dict[algo], alpha=0.5)

      ax[0].plot(self.times, success, label=self.alg_dict[algo]['name'], color=self.color_dict[algo], linewidth=3, alpha=0.8)
    ax[0].legend()
    ax[0].set_ylabel(r"Success [\%]")
    ax[1].set_ylabel("Cost [s]")

    ax[1].set_xlabel("Time [s]")

  def add_initial_time_cost_plot(self, exp_name):
    self._add_page()
    self.fig, self.ax = plt.subplots()
    self.ax.set_title(exp_name)

    for (exp_name_stats, algo), costs in self.stats.items():
      if exp_name_stats != exp_name:
        continue
      initial_costs = []
      initial_times = []
      for k in range(costs.shape[0]):
        l = costs[k, np.isfinite(costs[k])]
        if len(l) > 0:
          initial_cost = l[0]
          initial_costs.append(initial_cost)
          initial_time = self.times[np.isfinite(costs[k])][0]
          initial_times.append(initial_time)
      self.ax.scatter(initial_times, initial_costs, label=algo, color=self.color_dict[algo])
    self.ax.legend()
    self.ax.set_xlabel("Time for first solution [s]")
    self.ax.set_ylabel("Cost of first solution [s]")

  def add_success_rate_plot(self, exp_name):
    self._add_page()
    self.fig, self.ax = plt.subplots()
    self.ax.set_title(exp_name)

    success_dict = defaultdict(int)
    for (exp_name_stats, algo), costs in self.stats.items():
      if exp_name_stats != exp_name:
        continue
      for k in range(costs.shape[0]):
        l = costs[k, np.isfinite(costs[k])]
        if len(l) > 0:
          success_dict[algo] += 1
    names = self.color_dict.keys()
    y = []
    for name in names:
      y.append(success_dict[name])

    self.ax.bar(range(len(y)), y)
    self.ax.set_xticks(range(len(y)))
    self.ax.set_xticklabels(names)

  def add_boxplot_initial_time_plot(self, exp_name):
    self._add_page()
    self.fig, self.ax = plt.subplots()
    self.ax.set_title(exp_name)

    result = []
    names = self.color_dict.keys()
    result_dict = defaultdict(list)
    for (exp_name_stats, algo), costs in self.stats.items():
      if exp_name_stats != exp_name:
        continue
      initial_times = []
      for k in range(costs.shape[0]):
        l = costs[k, np.isfinite(costs[k])]
        if len(l) > 0:
          initial_time = self.times[np.isfinite(costs[k])][0]
          initial_times.append(initial_time)
      result_dict[algo] = initial_times
    # print(result_dict.values())
    result = []
    for name in names:
      result.append(result_dict[name])
    self.ax.boxplot(result)
    self.ax.set_xticks(range(1, len(names)+1))
    self.ax.set_xticklabels(names)
    # self.ax.legend()
    self.ax.set_ylabel("Time for first solution [s]")

  def add_boxplot_initial_cost_plot(self, exp_names):
    self._add_page()
    self.fig, ax = plt.subplots(1, len(exp_names), sharex='all', sharey='none', squeeze=False)
    for i, exp_name in enumerate(exp_names):
      ax[0,i].set_title(exp_name)

      result = []
      names = self.color_dict.keys()
      result_dict = defaultdict(list)
      for (exp_name_stats, algo), costs in self.stats.items():
        if exp_name_stats != exp_name:
          continue
        initial_costs = []
        for k in range(costs.shape[0]):
          l = costs[k, np.isfinite(costs[k])]
          if len(l) > 0:
            initial_cost = l[0]
            initial_costs.append(initial_cost)
        result_dict[algo] = initial_costs
      # print(result_dict.values())
      result = []
      colors = []
      for name in names:
        result.append(result_dict[name])
        colors.append(self.color_dict[name])
      bplot = ax[0,i].boxplot(result, patch_artist=True)
      for patch, color in zip(bplot['boxes'], colors):
        patch.set_facecolor(color)
      ax[0,i].set_xticks(range(1, len(names)+1))
      ax[0,i].set_xticklabels(names)
      ax[0,i].yaxis.grid(True)
      # ax[0,i].legend()
    ax[0,0].set_ylabel("Cost for first solution [s]")


  def add_barplot_initial_cost_plot(self, exp_names):
    self._add_page()
    self.fig, ax = plt.subplots(2, len(exp_names), sharex='all', sharey='none', squeeze=False)
    for i, exp_name in enumerate(exp_names):
      # ax[0,i].set_title(exp_name)
      ax[0,i].yaxis.grid(True)
      ax[0,i].set_xticks([])
      ax[1,i].yaxis.grid(True)
      ax[1,i].set_xticks([])

      for (exp_name_stats, algo), costs in self.stats.items():
        if exp_name_stats != exp_name:
          continue
        initial_costs = []
        initial_times = []
        for k in range(costs.shape[0]):
          l = costs[k, np.isfinite(costs[k])]
          if len(l) > 0:
            initial_cost = l[0]
            initial_costs.append(initial_cost)
            initial_time = self.times[np.isfinite(costs[k])][0]
            initial_times.append(initial_time)
        ax[0,i].bar(
          self.alg_dict[algo]['idx'],
          np.mean(initial_costs),
          yerr=np.std(initial_costs),
          color=self.alg_dict[algo]['color'])

        ax[1,i].bar(
          self.alg_dict[algo]['idx'],
          np.mean(initial_times),
          yerr=np.std(initial_times),
          color=self.alg_dict[algo]['color'])

        ax[1,i].set_xlabel(exp_name)
        
        
    ax[0,0].set_ylabel("Cost for first solution [s]")
    ax[1,0].set_ylabel("Time for first solution [s]")


  def close(self):
    self._add_page()
    self.pp.close()

  def _add_page(self):
    if self.fig is not None:
      self.pp.savefig(self.fig)
      plt.close(self.fig)
      self.fig = None


def load_data(filename, T, dt):
  with open(filename) as f:
    stats = yaml.safe_load(f)

  costs = np.zeros(int(T / dt)) * np.nan
  if stats is not None and "stats" in stats and stats["stats"] is not None:
    for d in stats["stats"]:
        idx = int(d["t"] / dt)
        costs[idx:] = d["cost"]
  return costs


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("stats", nargs='*', help="yaml files with stats")
  args = parser.parse_args()

  # plt.rcParams.update({
  #   "text.usetex": True,
  #   "font.family": "sans-serif",
  #   "font.sans-serif": ["Helvetica"]})

  fig, ax = plt.subplots()

  T = 61
  dt = 0.1
  times = np.arange(0, T, dt)
  
  costs = []
  for filename in args.stats:
    costs.append(load_data(filename, T, dt))

  # convert to 2D array
  costs = np.array(costs)
#   mean = costs.mean(axis=0)
  mean = np.nanmean(costs, axis=0)
#   std = costs.std(axis=0)
  std = np.nanstd(costs, axis=0)

  ax.plot(times, mean, color='blue')
  ax.fill_between(times, mean+std, mean-std, facecolor='blue', alpha=0.5)
  plt.show()


if __name__ == "__main__":
  main()
