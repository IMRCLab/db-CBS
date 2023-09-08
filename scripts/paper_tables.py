from pathlib import Path

from benchmark_table import write_table

def write_table1(trials, timelimit):
	instances = [
		"swap2_unicycle_sphere",
		"alcove_unicycle_sphere",
	]
	algs = [
		"sst",
		"s2m2",
		"k-cbs",
		"db-cbs",
	]

	write_table(instances, algs, Path("../results/paper_table1.pdf"), trials, timelimit)

if __name__ == '__main__':
	trials = 1
	timelimit = 5*60
	write_table1(trials, timelimit)
