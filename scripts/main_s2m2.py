import argparse
import subprocess
from pathlib import Path


def run_s2m2(filename_env, folder, cfg):
	s2sm_script = Path().resolve().parent / "s2m2/main_s2m2_original.py" 
	result = subprocess.run(["python3",
		s2sm_script, 
		filename_env,
		folder,
		str(cfg),
		])
	if result.returncode != 0:
		print("S2SM failed")
		
def main():
	parser = argparse.ArgumentParser()
	parser.add_argument("env", help="file containing the environment (YAML)")
	args = parser.parse_args()

	for i in range(1):
		run_s2m2(args.env, i)


if __name__ == '__main__':
	main()
