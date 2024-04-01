import yaml
import glob
import sys

files = glob.glob("/home/akmarak-laptop/IMRC/db-CBS/example/*.yaml") # list of all .yaml files in a directory 

def read_yaml_file(filename):
    with open(filename, 'r') as stream:
        env = yaml.safe_load(stream)
        if "robots" in env:
            for robot in env["robots"]:
                if robot["type"] == "unicycle_first_order_0_sphere":
                    robot["type"] = "unicycle1_sphere_v0"
                elif robot["type"] == "unicycle_first_order_0":
                    robot["type"] = "unicycle1_v0"
                elif robot["type"] == "car_first_order_with_1_trailers_0":
                    robot["type"] = "car1_v0"
                elif robot["type"] == "double_integrator_0":
                    robot["type"] = "integrator2_2d_v0"
                elif robot["type"] == "unicycle_second_order_0":
                    robot["type"] = "unicycle2_v0"
            with open(filename, "w") as f:
                yaml.dump(env, f)

        # print(len(env["robots"]))
        # exit()
        # try:
        #     print(yaml.safe_load(stream))
        # except yaml.YAMLError as exc:
        #     print(exc)

for file in files:
    print(file)
    read_yaml_file(file)