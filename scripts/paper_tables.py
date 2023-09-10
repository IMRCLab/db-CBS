from pathlib import Path

import benchmark_table
from benchmark_table import write_table

def write_table1(trials, timelimit):
	instances = [
		"swap2_unicycle_sphere",
		"alcove_unicycle_sphere",
		"at_goal_unicycle_sphere",
		"window4_unicycle_sphere",
	]
	algs = [
		"sst",
		"s2m2",
		"k-cbs",
		"db-cbs",
	]

	write_table(instances, algs, Path("../results"), "paper_table1.pdf", trials, timelimit)


def write_table4(trials, timelimit):
	instances = [
		"swap1_unicycle",
		"swap1_double_integrator",
		"swap1_trailer",
		"swap1_unicycle2",

		"swap2_unicycle",
		"swap2_double_integrator",
		"swap2_trailer",
		"swap2_unicycle2",

		"swap3_unicycle",
		"swap3_double_integrator",
		"swap3_trailer",
		"swap3_unicycle2",

		"swap4_unicycle",
		"swap4_double_integrator",
		"swap4_trailer",
		"swap4_unicycle2",
	]
	algs = [
		"sst",
		"k-cbs",
		"db-cbs",
	]

	robots = [1,2,3,4]
	dynamics = ["unicycle", "double_integrator", "trailer", "unicycle2"]

	r = benchmark_table.compute_results(instances, algs, Path("../results"), trials, timelimit)
	print(r)

	output_path = Path("../results/paper_table4.pdf")
	with open(output_path.with_suffix(".tex"), "w") as f:

		f.write(r"\documentclass{standalone}")
		f.write("\n")
		f.write(r"\begin{document}")
		f.write("\n")

		alg_names = {
			"sst": "SST*",
			"k-cbs": "k-CBS",
			"db-cbs": "db-CBS",
		}

		dyn_names = {
			"unicycle": "unicycle",
			"double_integrator": "double integrator",
			"trailer": "trailer",
			"unicycle2": "unicycle2",
		}

		out = r"\begin{tabular}{c "
		for d in dynamics:
			out += r" || r|r|r"
		out += "}"
		f.write(out)
		out = r"N "
		for k, d in enumerate(dynamics):
			out += r" & \multicolumn{3}{c||}{"
			out += dyn_names[d]
			out += r"}"
		out += r"\\"
		f.write(out)

		out = ""
		for _ in dynamics:
			for k, alg in enumerate(algs):
				out += r"& "
				out += alg_names[alg]
		out += r"\\"
		f.write(out)

		for n in robots:
			out = str(n)
			for d in dynamics:
				for alg in algs:
					out = benchmark_table.print_and_highlight_best(out, 't^st_median', r["swap{}_{}".format(n, d)], alg, algs)
			out += r"\\"
			f.write(out)

		f.write(r"\end{tabular}")
		f.write(r"\end{document}")

	benchmark_table.gen_pdf(output_path)

if __name__ == '__main__':
	trials = 1
	timelimit = 5*60
	write_table1(trials, timelimit)
	write_table4(trials, timelimit)
