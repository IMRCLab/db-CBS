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

if __name__ == '__main__':
	trials = 1
	timelimit = 5*60
	write_table1(trials, timelimit)
	write_table2(trials, timelimit)

