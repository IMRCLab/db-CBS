from pathlib import Path

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
		"s2m2",
		"k-cbs",
		"db-cbs",
	]

	write_table(instances, algs, Path("../results"), "paper_table4.pdf", trials, timelimit)

if __name__ == '__main__':
	trials = 1
	timelimit = 5*60
	write_table1(trials, timelimit)
	write_table4(trials, timelimit)
