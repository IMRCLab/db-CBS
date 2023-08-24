import yaml
import numpy as np
import tempfile
import subprocess

def check_problem(cfg):
    with tempfile.TemporaryDirectory() as tmpdirname:
        print('created temporary directory', tmpdirname)

        with open(tmpdirname + "/env.yaml", "w") as f:
            yaml.dump(cfg, f)

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
        size = np.random.normal([0.5, 0.5], 0.5)
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
        start = np.random.uniform([min[0]+0.5, min[1]+0.5, -np.pi], [max[0]-0.5, max[1]-0.5, np.pi])
        goal = np.random.uniform([min[0]+0.5, min[1]+0.5, -np.pi], [max[0]-0.5, max[1]-0.5, np.pi])
        r["robots"].append({
            "type": "unicycle_first_order_0",
            "start": start.tolist(),
            "goal": goal.tolist()
        })
        if not check_problem(r):
            r["robots"].pop()

    with open(filename, "w") as f:
        yaml.dump(r, f)

def main():
    min = np.array([0,0])
    max = np.array([5,5])
    obs_density = 10 # percent
    N = 2 # number of robots
    K = 1 # num instances

    for N in [2, 4, 8, 16]:
        for k in range(K):
            filename = "../example/gen_p{}_n{}_{}.yaml".format(obs_density, N, k)
            gen_env(min, max, obs_density / 100.0, N, filename)

if __name__ == '__main__':
    main()