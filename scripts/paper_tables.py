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

def write_table2(trials, timelimit):
	instances = [
		"gen_p10_n2_*_unicycle_sphere",
		"gen_p10_n4_*_unicycle_sphere",
		"gen_p10_n8_*_unicycle_sphere",
	]
	algs = [
		"sst",
		"s2m2",
		"k-cbs",
		"db-cbs",
	]

	write_table(instances, algs, Path("../results"), "paper_table2.pdf", 10, timelimit, True)

def write_table3(trials, timelimit):
	instances = [
		"gen_p10_n2_*_hetero",
		"gen_p10_n4_*_hetero",
		"gen_p10_n8_*_hetero",
	]
	algs = [
		"sst",
		"k-cbs",
		"db-cbs",
	]

	write_table(instances, algs, Path("../results"), "paper_table3.pdf", 10, timelimit, True)

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
		f.write(r"% GENERATED - DO NOT EDIT - " + output_path.name + "\n")

		alg_names = {
			"sst": "$\star$",
			"k-cbs": "$\dagger$",
			"db-cbs": "$\ddagger$",
		}

		dyn_names = {
			"unicycle": "unicycle $1^{\mathrm{st}}$",
			"double_integrator": "double int.",
			"trailer": "car with trailer",
			"unicycle2": "unicycle $2^{\mathrm{nd}}$",
		}

		out = r"\begin{tabular}{c "
		for d in dynamics:
			out += r" || r|r|r"
		out += "}\n"
		f.write(out)
		out = r"N "
		for k, d in enumerate(dynamics):
			if k == len(dynamics) - 1:
				out += r" & \multicolumn{3}{c}{"
			else:
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
		f.write(r"\hline")

		for n in robots:
			out = ""
			out += r"\hline"
			out += str(n)
			for d in dynamics:
				for alg in algs:
					out = benchmark_table.print_and_highlight_best(out, 't^st_median', r["swap{}_{}".format(n, d)], alg, algs)
			out += r"\\"
			f.write(out)

		f.write("\n")
		f.write(r"\end{tabular}")
		f.write("\n")
		f.write(r"\end{document}")

	benchmark_table.gen_pdf(output_path)

def write_table5(trials, timelimit):
	instances = [
		"swap2_unicycle_sphere",
		"alcove_unicycle_sphere",
		"at_goal_unicycle_sphere",
		# "window4_unicycle_sphere",

		"<<HLINE>>",

		"gen_p10_n2_*_unicycle_sphere",
		"gen_p10_n4_*_unicycle_sphere",
		"gen_p10_n8_*_unicycle_sphere",

		"<<HLINE>>",

		"gen_p10_n2_*_hetero",
		"gen_p10_n4_*_hetero",
		"gen_p10_n8_*_hetero",
	]
	trials = [1]*4 + [10]*7
	algs = [
		"sst",
		"s2m2",
		"k-cbs",
		"db-cbs",
	]

	instance_names = {
		'swap2_unicycle_sphere': "swap",
		'alcove_unicycle_sphere': "alcove",
		'at_goal_unicycle_sphere': "at goal",
		'window4_unicycle_sphere': "window4",
		'gen_p10_n2_*_unicycle_sphere': "rand (N=2)",
		'gen_p10_n4_*_unicycle_sphere': "rand (N=4)",
		'gen_p10_n8_*_unicycle_sphere': "rand (N=8)",
		'gen_p10_n2_*_hetero': "rand hetero (N=2)",
		'gen_p10_n4_*_hetero': "rand hetero (N=4)",
		'gen_p10_n8_*_hetero': "rand hetero (N=8)",
	}

	alg_names = {
		"sst": "SST*",
		"s2m2": "S2M2",
		"k-cbs": "k-CBS",
		"db-cbs": "db-CBS",
	}

	result = benchmark_table.compute_results(instances, algs, Path("../results"), trials, timelimit, True)
	output_path = Path("../results/paper_table5.pdf")
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
			if instance in instance_names:
				out += instance_names[instance]
			else:
				out += "{} ".format(instance.replace("_", "\_"))

			for alg in algs:

				out = benchmark_table.print_and_highlight_best_max(out, 'success', result[instance], alg, algs)
				out = benchmark_table.print_and_highlight_best(out, 't^st_median', result[instance], alg, algs)
				out = benchmark_table.print_and_highlight_best(out, 'J^st_median', result[instance], alg, algs)
				out = benchmark_table.print_and_highlight_best(out, 'Jr^st_median', result[instance], alg, algs, digits=0)

			out += r"\\"
			f.write(out)
			r_number += 1

		f.write("\n")
		f.write(r"\end{tabular}")
		f.write("\n")
		f.write(r"\end{document}")

	benchmark_table.gen_pdf(output_path)

if __name__ == '__main__':
	trials = 1
	timelimit = 5*60
	write_table1(trials, timelimit)
	write_table2(trials, timelimit)
	write_table3(trials, timelimit)
	write_table4(trials, timelimit)
	write_table5(trials, timelimit)


