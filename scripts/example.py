import yaml
import argparse
import shutil
from pathlib import Path

from main_ompl import run_ompl

def run_example(env,result_folder,timelimit,cfg):
    result_folder = Path(result_folder)
    if result_folder.exists():
        print("Warning! {} exists already. Deleting...".format(result_folder))
        shutil.rmtree(result_folder)
    result_folder.mkdir(parents=True, exist_ok=False)
    with open(cfg) as f:
        cfg = yaml.safe_load(f)
    mycfg = cfg['sst']
    mycfg = mycfg['default']
    run_ompl(env, str(result_folder), timelimit, mycfg)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('env', help="path to env.yaml file")
    parser.add_argument('result_folder', help="output folder")
    parser.add_argument('timelimit', default = 5 * 60, help="timelimit")
    parser.add_argument('cfg', help="path to algorithm cfgs")

    args = parser.parse_args()
    run_example(args.env, args.result_folder,args.timelimit,args.cfg)


if __name__ == "__main__":
    main()