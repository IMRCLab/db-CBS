from pathlib import Path
import yaml
import numpy as np
import subprocess

def compute_results(instances, algs, results_path, trials, T, regret):
	all_result = dict()
	if isinstance(trials, int):
		trials = [trials]*len(instances)
	print(trials)
	for instance, itrials in zip(instances, trials):
		print(instance)
		result = dict()
		for alg in algs:
			if not regret:
				result_folder = results_path / instance / alg
				stat_files = [str(p) for p in result_folder.glob("**/stats.yaml")]
			else:
				stat_files = [str(p) for p in results_path.glob(instance + "/"+alg+"/**/stats.yaml")]
			initial_time_regrets = []
			final_regrets = []

			# load data
			initial_times = []
			initial_time_regrets = []
			initial_costs = []
			initial_regrets = []
			final_costs = []
			final_regrets = []

			for stat_file in stat_files:
				final_cost_base = None
				initial_time_base = None
				if regret:
					stat_file_base = stat_file.replace(alg, "db-cbs")
					if Path(stat_file_base).exists():
						with open(stat_file_base) as sf:
							stats = yaml.safe_load(sf)
						if stats is not None and "stats" in stats and stats["stats"] is not None:
							for k, d in enumerate(stats["stats"]):
								# skip results that were after our time horizon
								if d["t"] > T:
									break
								if k == 0:
									initial_time_base = d["t"]
								final_cost_base = d["cost"]

				with open(stat_file) as sf:
					stats = yaml.safe_load(sf)
				if stats is not None and "stats" in stats and stats["stats"] is not None:
					last_cost = None
					for k, d in enumerate(stats["stats"]):
						# skip results that were after our time horizon
						if d["t"] > T:
							break
						if k == 0:
							initial_times.append(d["t"])
							initial_costs.append(d["cost"])
							if initial_time_base is not None:
								initial_time_regrets.append((d["t"] - initial_time_base)/d["t"] * 100)
								initial_regrets.append((d["cost"] - final_cost_base)/d["cost"] * 100)

						last_cost = d["cost"]
					if last_cost is not None:
						final_costs.append(last_cost)
					if last_cost is not None and final_cost_base is not None:
						final_regrets.append((last_cost - final_cost_base)/last_cost * 100)

			result[alg] = {
				'success': len(initial_times)/itrials,
				't^st_median': np.median(initial_times) if len(initial_times) > 0 else None,
				'tr^st_median': np.median(initial_time_regrets) if len(initial_time_regrets) > 0 else None,
				'J^st_median': np.median(initial_costs) if len(initial_costs) > 0 else None,
				'Jr^st_median': np.median(initial_regrets) if len(initial_regrets) > 0 else None,
				'J^f_median': np.median(final_costs) if len(initial_costs) > 0 else None,
				'Jr^f_median': np.median(final_regrets) if len(final_regrets) > 0 else None,
			}

			if alg == "s2m2" and len(initial_times) == 0 and "unicycle_sphere" not in instance:
				for key in result[alg].keys():
					result[alg][key] = '*'
		all_result[instance] = result
	return all_result

def gen_pdf(output_path):
	# run pdflatex
	subprocess.run(['pdflatex', output_path.with_suffix(".tex")], check=True, cwd=output_path.parent)
	# delete temp files
	output_path.with_suffix(".aux").unlink()
	output_path.with_suffix(".log").unlink()

def print_and_highlight_best(out, key, result, alg, algs, digits=1):
	out += " & "
	is_best = False
	if result[alg][key] is not None and result[alg][key] is not "*":
		# we only look at one digit
		is_best = np.array([round(result[alg][key],1) <= round(result[other][key],1) for other in algs if result[other][key] is not None and result[other][key] is not "*"]).all()
	if is_best:
		out += r"\bfseries "
	if result[alg][key] == "*":
		out += r"$\star$"
	elif result[alg][key] is not None:
		out += ("{:."+str(digits)+"f}").format(result[alg][key])
	else:
		out += r"\textemdash"
	return out

def print_and_highlight_best_max(out, key, result, alg, algs, digits=1):
	out += " & "
	is_best = False
	if result[alg][key] is not None and result[alg][key] is not "*":
		# we only look at one digit
		is_best = np.array([round(result[alg][key],1) >= round(result[other][key],1) for other in algs if result[other][key] is not None and result[other][key] is not "*"]).all()
	if is_best:
		out += r"\bfseries "
	if result[alg][key] == "*":
		out += r"$\star$"
	elif result[alg][key] is not None:
		out += ("{:."+str(digits)+"f}").format(result[alg][key])
	else:
		out += r"\textemdash"
	return out

def get_alg_name(alg_key):
	# all algorithms we consider so far
	mapping = {
		"sst": "SST*",
		"s2m2": "S2M2",
		"k-cbs": "k-CBS",
		"db-cbs": "db-CBS",
		"db-ecbs": "db-ECBS",
	}

	if alg_key in mapping:
		return mapping[alg_key]
	return alg_key.upper().replace("-", "_")

def write_table(rows, algs, results_path, fname, trials, T, regret=False):

	result = compute_results(rows, algs, results_path, trials, T, regret)
	output_path = Path(results_path) / Path(fname)
	with open(output_path.with_suffix(".tex"), "w") as f:

		f.write(r"\documentclass{standalone}")
		f.write("\n")
		f.write(r"\begin{document}")
		f.write("\n")
		f.write(r"% GENERATED - DO NOT EDIT - " + output_path.name + "\n")

		alg_names = {key: get_alg_name(key) for key in algs}

		out = r"\begin{tabular}{c || c"
		for alg in algs:
			if alg == "sst" and not regret:
				out += r" || r|r|r|r"
			else:
				out += r" || r|r|r"
		out += "}\n"
		f.write(out)
		out = r"\# & Instance"
		for k, alg in enumerate(algs):
			if k == len(algs) - 1:
				if alg == "sst" and not regret:
					out += r" & \multicolumn{4}{c}{"
				else:
					out += r" & \multicolumn{3}{c}{"
			else:
				if alg == "sst" and not regret:
					out += r" & \multicolumn{4}{c||}{"
				else:
					out += r" & \multicolumn{3}{c||}{"
			out += alg_names[alg]
			out += r"}"
		out += r"\\"
		f.write(out)
		out = r"& "
		if not regret:
			for alg in algs:
				if alg == "sst":
					out += r" & $p$ & $t^{\mathrm{st}} [s]$ & $J^{\mathrm{st}} [s]$ & $J^{f} [s]$"
				else:
					out += r" & $p$ & $t^{\mathrm{st}} [s]$ & $J^{\mathrm{st},f} [s]$"
		else:
			for alg in algs:
				out += r" & $p$ & $t_r^{\mathrm{st}} [\%]$ & $J_r^{f} [\%]$"
		out += r"\\"
		f.write(out)
		f.write(r"\hline")

		for r_number, row in enumerate(rows):

			out = ""
			out += r"\hline"
			out += "\n"
			out += "{} & ".format(r_number+1)
			out += "{} ".format(row.replace("_", "\_"))

			for alg in algs:

				if not regret:
					out = print_and_highlight_best_max(out, 'success', result[row], alg, algs)
					out = print_and_highlight_best(out, 't^st_median', result[row], alg, algs)
					out = print_and_highlight_best(out, 'J^f_median', result[row], alg, algs)
					if alg == "sst":
						out = print_and_highlight_best(out, 'J^f_median', result[row], alg, algs)
				else:
					out = print_and_highlight_best_max(out, 'success', result[row], alg, algs)
					out = print_and_highlight_best(out, 'tr^st_median', result[row], alg, algs)
					out = print_and_highlight_best(out, 'Jr^f_median', result[row], alg, algs)

			out += r"\\"
			f.write(out)

		f.write("\n")
		f.write(r"\end{tabular}")
		f.write("\n")
		f.write(r"\end{document}")

	# run pdflatex
	gen_pdf(output_path)

# to benchmark db-cbs, db-ecbs
def write_table_test(instances, algs, trials, timelimit):
	# trials = [trials]*4 + [10*trials]*7

	alg_names = {key: get_alg_name(key) for key in algs}

	result = compute_results(instances, algs, Path("../results"), trials, timelimit, True)
	output_path = Path("../results/benchmark_table.pdf")
	with open(output_path.with_suffix(".tex"), "w") as f:

		f.write(r"\documentclass{standalone}")
		f.write("\n")
		f.write(r"\begin{document}")
		f.write("\n")
		f.write(r"% GENERATED - DO NOT EDIT - " + output_path.name + "\n")

		out = r"\begin{tabular}{c || c"
		for alg in algs:
			out += r" || r|r|r|r"
		out += "}\n"
		f.write(out)
		out = r"\# & Instance"
		for k, alg in enumerate(algs):
			if k == len(algs) - 1:
				out += r" & \multicolumn{4}{c}{"
			else:
				out += r" & \multicolumn{4}{c||}{"
			out += alg_names[alg]
			out += r"}"
		out += r"\\"
		f.write(out)
		out = r"& "
		for alg in algs:
			out += r" & $p$ & $t [s]$ & $J [s]$ & $r [\%]$"
		out += r"\\"
		f.write(out)
		f.write(r"\hline")

		r_number = 0
		for instance in instances:

			if instance == "<<HLINE>>":
				f.write(r"\hline")
				f.write("\n")
				continue

			out = ""
			out += r"\hline"
			out += "\n"
			out += "{} & ".format(r_number+1)
			out += "{} ".format(instance.replace("_", "\_"))

			for alg in algs:

				out = print_and_highlight_best_max(out, 'success', result[instance], alg, algs)
				out = print_and_highlight_best(out, 't^st_median', result[instance], alg, algs)
				out = print_and_highlight_best(out, 'J^st_median', result[instance], alg, algs)
				out = print_and_highlight_best(out, 'Jr^st_median', result[instance], alg, algs, digits=0)

			out += r"\\"
			f.write(out)
			r_number += 1

		f.write("\n")
		f.write(r"\end{tabular}")
		f.write("\n")
		f.write(r"\end{document}")

	gen_pdf(output_path)


def main():
	results_path = Path("../results")

	rows = [
		"swap1_unicycle",
	]
	algs = [
		"db-cbs",
		"db-ecbs",
	]

	write_table(rows, algs, results_path, "table.pdf", 5, 5*60)
	# write_table_test(rows, algs, 2, 5*60)

if __name__ == '__main__':
	main()
