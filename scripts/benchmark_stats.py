import yaml
from pathlib import Path
import plot_stats


def main():
	results_path = Path("../results")

	instances = [
		"parallelpark",
		"bugtrap",
		"wall",
	]
	algs = [
		"sst",
		# "sbpl",
		# "komo",
		# "dbAstar-komo",
		# "dbAstar-scp",
	]

	report = plot_stats.Report(results_path / "stats.pdf", T=60, dt=0.1)

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

if __name__ == '__main__':
	main()
