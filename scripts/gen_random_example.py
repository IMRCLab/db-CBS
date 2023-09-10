import yaml
import numpy as np
import tempfile
import subprocess
import copy

def check_problem(cfg):
    with tempfile.TemporaryDirectory() as tmpdirname:
        print('created temporary directory', tmpdirname)

        # inflate the obstacles so that robots aren't too close
        cfg_copy = copy.deepcopy(cfg)
        for o in cfg_copy["environment"]["obstacles"]:
            o["size"][0] += 0.1
            o["size"][1] += 0.1

        with open(tmpdirname + "/env.yaml", "w") as f:
            yaml.dump(cfg_copy, f)

        result = {
            "result":
            [
                {
                    "states": [r["start"], r["goal"]],
                    "actions": [[0,0]]
                }
                for r in cfg["robots"]
            ]
        }
        print(result)

        with open(tmpdirname + "/result.yaml", "w") as f:
            yaml.dump(result, f)


        out = subprocess.run(["./main_check_multirobot",
                    "--result_file", tmpdirname + "/result.yaml",
                    "--env_file", tmpdirname + "/env.yaml",
                    "--models_base_path" , "../dynoplan/dynobench/models/",
                    "--traj_tol" , "9999999",
                    "--col_tol" , "1e-9"])
        return out.returncode == 0


def gen_env(min, max, obs_density, N, filename):

    r = dict()
    r["environment"] = dict()
    r["environment"]["min"] = min.tolist()
    r["environment"]["max"] = max.tolist()

    area = np.prod(max - min)
    print(area)

    filled_area = 0
    obs = []
    while filled_area < obs_density*area:
        size = np.random.normal([0.75, 0.75], 0.5)
        if np.any(size < 0.1):
             continue
        filled_area += np.prod(size)
        center = np.random.uniform(min+size/2, max-size/2)
        p1 = center - size/2
        p2 = center + size/2
        collision = False
        for o in obs:
            p1o = np.asarray(o["center"]) - np.asarray(o["size"]) / 2
            p2o = np.asarray(o["center"]) + np.asarray(o["size"]) / 2
            if p1[0] < p2o[0] and p1o[0] < p2[0] and p1[1] < p2o[1] and p1o[1] < p2[1]:
                collision = True
                break
        if collision:
            continue

        print(size, filled_area)
        obs.append({
             "type": "box",
             "center": center.tolist(),
             "size": size.tolist()
        })
    r["environment"]["obstacles"] = obs

    r["robots"] = []
    while len(r["robots"]) < N:
        type = str(np.random.choice(["unicycle_first_order_0", "car_first_order_with_1_trailers_0", "double_integrator_0"]))
        # type = str(np.random.choice(["unicycle_first_order_0", "double_integrator_0"]))
        # type = str(np.random.choice(["unicycle_first_order_0_sphere"]))

        # first pick a type, then try to fit it
        while True:
            if type == "unicycle_first_order_0" or "unicycle_first_order_0_sphere":
                start = np.random.uniform([min[0]+0.5, min[1]+0.5, -np.pi], [max[0]-0.5, max[1]-0.5, np.pi])
                goal = np.random.uniform([min[0]+0.5, min[1]+0.5, -np.pi], [max[0]-0.5, max[1]-0.5, np.pi])
            if type == "car_first_order_with_1_trailers_0":
                start = np.random.uniform([min[0]+0.5, min[1]+0.5, -np.pi, -0.4], [max[0]-0.5, max[1]-0.5, np.pi, 0.4])
                goal = np.random.uniform([min[0]+0.5, min[1]+0.5, -np.pi, -0.4], [max[0]-0.5, max[1]-0.5, np.pi, 0.4])
                start[3] = start[2] + start[3]
                goal[3] = goal[2] + goal[3]
            elif type == "double_integrator_0":
                start = np.random.uniform([min[0]+0.5, min[1]+0.5, 0, 0], [max[0]-0.5, max[1]-0.5, 0, 0])
                goal = np.random.uniform([min[0]+0.5, min[1]+0.5, 0, 0], [max[0]-0.5, max[1]-0.5, 0, 0])

            if np.linalg.norm(start - goal) < 2:
                continue

            r["robots"].append({
                "type": type,
                "start": start.tolist(),
                "goal": goal.tolist()
            })
            if not check_problem(r):
                r["robots"].pop()
            else:
                break

    with open(filename, "w") as f:
        yaml.dump(r, f)

def main():
    min = np.array([0,0])
    max = np.array([5,5])
    obs_density = 10 # percent
    K = 10 # num instances

    for N in [2, 4, 8]:
        for k in range(K):
            # filename = "../example/gen_p{}_n{}_{}_unicycle_sphere.yaml".format(obs_density, N, k)
            filename = "../example/gen_p{}_n{}_{}_hetero.yaml".format(obs_density, N, k)
            gen_env(min, max, obs_density / 100.0, N, filename)

if __name__ == '__main__':
    main()