import argparse
from pathlib import Path
import yaml
import sys
import shutil
# converts TRO(18) problem instance format into db-ecbs
# python3 convert_tro_format.py folder instance1 instance2 etc.,
def main_conversion(folder, instances):
    folder = Path(folder)
    num_zeros = 3 # for vx, vy, vz
    for instance in instances:
        new_format_instance = {}
        # set the environment
        new_format_instance["environment"] = {}
        new_format_instance["environment"]["min"] = [0,0,0] 
        if instance == 'drone32b':
            new_format_instance["environment"]["max"] = [13,13,3] 
        elif instance == 'drone32c' or instance == 'swap50':
            new_format_instance["environment"]["max"] = [7.5,6.5,2.5] 
        else: 
            new_format_instance["environment"]["max"] = [6,6,6] 
        new_format_instance["environment"]["obstacles"] = []
        if (folder / (instance + ".bt")).is_file() and (folder / (instance + ".stl")).is_file():
            obstacle = {}
            obstacle["type"] = "octomap"
            obstacle["octomap_file"] = "../octomaps/" + instance + ".bt"
            obstacle["octomap_stl"] = "../meshes/" + instance + ".stl"
            obstacle["center"] = []
            obstacle["size"] = []
            new_format_instance["environment"]["obstacles"].append(obstacle)
            # copy the octomap and stl into proper folders
            if Path(folder.parent.parent / ("octomaps/" + instance + ".bt")).is_file() == False:
                shutil.copy(folder / (instance + ".bt"), folder.parent.parent / ("octomaps/" + instance + ".bt"))
            if Path(folder.parent.parent / ("meshes/" + instance + ".stl")).is_file() == False:
                shutil.copy(folder / (instance + ".stl"), folder.parent.parent / ("meshes/" + instance + ".stl"))
        else: 
            print("No octomap provided (.bt nor .stl)!")

        # set robots
        with open(folder / (instance + "_addVertices.yaml")) as f: 
            data = yaml.safe_load(f)
        vertices = data["vertices"]
        with open(folder / (instance + "_agents.yaml")) as f: 
            data = yaml.safe_load(f)
        agents = data["agents"]
        # set robots
        new_format_instance["robots"] = []
        for agent in agents:
            per_agent = {}
            i = [i for i, _ in enumerate(vertices) if _['name'] == agent["start"]][0]
            j = [j for j, _ in enumerate(vertices) if _['name'] == agent["goal"]][0]
            if (i >=0 and j >=0):
                vertices[i]["pos"].extend([0] * num_zeros)
                vertices[j]["pos"].extend([0] * num_zeros)
                per_agent["type"] = "integrator2_3d_v0"
                per_agent["start"] = list(vertices[i]["pos"])
                per_agent["goal"] = list(vertices[j]["pos"])
            else: 
                sys.exit("Fail to get robots")
            new_format_instance["robots"].append(per_agent)

        # save into example folder
        f = open(folder.parent / (instance + '.yaml'), 'w+') 
        yaml.dump(new_format_instance, f, allow_unicode=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('folder', help="folder with old-format instances")
    parser.add_argument('instances', nargs='+', type=str, default=[], help="list of old-format instances")
    args = parser.parse_args()

    main_conversion(args.folder, args.instances)

if __name__ == '__main__':
	main()