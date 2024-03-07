import yaml
from pathlib import Path
import plot_stats
import argparse
from tabulate import tabulate

def run_benchmark_stats(instances, algs, trials, T):
	results_path = Path("../results")

	report = plot_stats.Report(results_path / "stats.pdf", trials, T, dt=0.1)

	for instance in instances:
		for alg in algs:
			result_folder = results_path / instance / alg
			stat_files = [str(p) for p in result_folder.glob("**/stats.yaml")]
			if len(stat_files) > 0:
				report.load_stat_files(instance, alg, stat_files)

	report.add_barplot_initial_cost_plot(instances)
	for instance in instances:
		report.add_success_and_cost_over_time_plot(instance)
		# report.add_time_cost_plot(instance)
		# report.add_success_over_time_plot(instance)
		# report.add_initial_time_cost_plot(instance)
		# # report.add_success_rate_plot(instance)
		# report.add_boxplot_initial_time_plot(instance)
		# report.add_boxplot_initial_cost_plot([instance])
	
	report.close()

def exp_nodes_table(instances, algs):
	results_path = Path("../results")
	all_data = []
	for instance in instances:
		per_instance = []
		per_instance.append(instance)
		for alg in algs:
			result_folder = results_path / instance / alg
			stat_files = [str(p) for p in result_folder.glob("**/expanded_nodes.yaml")]
			if len(stat_files) > 0:
				with open(str(stat_files[0])) as f: # for a single trial
					data = yaml.safe_load(f)
				per_instance.append(data["nodes"])
			else: 
				per_instance.append('*')
		all_data.append(per_instance)
		
	col_names = ["instance"]
	col_names[1:] = algs
	
	# df = pd.DataFrame(all_data, columns=col_names)
	with open(results_path / "node_expansion_stats.txt", 'w') as f:
		f.write(tabulate(all_data, headers=col_names, tablefmt="fancy_grid"))

def export_table_txt(instances, algs):
	results_path = Path("../results")
	all_data = []
	for instance in instances:
		per_instance = []
		per_instance.append(instance)
		for alg in algs:
			result_folder = results_path / instance / alg
			stat_files = [str(p) for p in result_folder.glob("**/stats.yaml")]
			if len(stat_files) > 0:
				with open(str(stat_files[0])) as f: # for a single trial
					data = yaml.safe_load(f)
				if (data["stats"]) != None:
					data = (data["stats"])[0]
					per_instance.extend([f'{data["t"]:.2f}', data["cost"], data["discrete_search_cost"], data["expanded_nodes"]])
				else:
					per_instance.extend(['*', '*', '*', '*'])
			else: 
				per_instance.extend(['*', '*', '*', '*'])
		all_data.append(per_instance)
		
	col_names = ["instances"]
	for i in range(len(algs)):
		col_names.extend(["t (" + algs[i] + ")", "cost (" + algs[i] + ")", "ds_cost (" + algs[i] + ")", "nodes (" + algs[i] + ")"])

	# df = pd.DataFrame(all_data, columns=col_names)
	with open(results_path / "results_nodes.txt", 'w') as f:
		f.write(tabulate(all_data, headers=col_names, tablefmt="fancy_grid"))

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('instances', help="instances")
    parser.add_argument('algs', help="algorithms")
    args = parser.parse_args()

    trials = 1
    timelimit = 5*60

    run_benchmark_stats(args.instances, args.algs, trials, timelimit)

if __name__ == '__main__':
	main()
