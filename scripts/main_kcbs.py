import argparse
import subprocess
import tempfile
from pathlib import Path
import yaml

def run_kcbs(filename_env, folder, timelimit, cfg):

	with tempfile.TemporaryDirectory() as tmpdirname:
		p = Path(tmpdirname)
		filename_cfg = p / "cfg.yaml"
		with open(filename_cfg, 'w') as f:
			yaml.dump(cfg, f, Dumper=yaml.CSafeDumper)
		result = subprocess.run(["./main_kcbs", 
			"-i", filename_env,
			"-o", "{}/result_kcbs.yaml".format(folder),
			"--stats", "{}/stats.yaml".format(folder),
			"--timelimit", str(timelimit),
			"-p", "k-cbs",
			"-c", str(filename_cfg)])
		if result.returncode != 0:
			print("KCBS failed")

def main():
	parser = argparse.ArgumentParser()
	parser.add_argument("env", help="file containing the environment (YAML)")
	args = parser.parse_args()

	for i in range(1):
		run_kcbs(args.env, i)


if __name__ == '__main__':
	main()
