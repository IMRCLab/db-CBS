import os
import yaml
from pathlib import Path
import numpy as np


def process_all_runs(root_dir, problems, algorithms, file_name_options, anytime=False):
  for problem in problems:
    for alg in algorithms:
      if problem.endswith("hetero") and alg == "s2m2":
          print(f"Skipping {problem}/{alg}")
          continue

      for run_id in range(5):  # 000 to 004
        run_folder = f"{run_id:03d}"
        run_dir = os.path.join(root_dir, problem, alg, run_folder)

        # Try to find and load result file
        result_data = None
        for file_name in file_name_options:
          result_path = os.path.join(run_dir, file_name)
          if os.path.isfile(result_path):
              try:
                  with open(result_path, "r") as f:
                      result_data = yaml.safe_load(f)
              except Exception as e:
                  print(f"Error reading {result_path}: {e}")
              break  # stop after the first found file

        if result_data is None:
            print(f"No result file found in {run_dir}")
            continue

        # Try to load stats.yaml
        stats_path = os.path.join(run_dir, "stats.yaml")
        stats_data = None
        has_cost = False
        # check, sometimes file exists, but can be empty  
        if os.path.isfile(stats_path):
            try:
                with open(stats_path, "r") as f:
                    stats_data = yaml.safe_load(f)
                has_cost = False
                if isinstance(stats_data, dict) and "stats" in stats_data:
                    stats_list = stats_data["stats"]
                    if isinstance(stats_list, list) and len(stats_list) > 0:
                        has_cost = "cost" in stats_list[0]
            except Exception as e:
                print(f"Error reading {stats_path}: {e}")
        if has_cost == False:
            continue
        # if the computation was succesfull, then do the energy cost estimation 
        else:
           energy_cost = 0 # for each robot
           for i in range(len(result_data["result"])):
              print(f"robot:  {i}")
              actions = result_data["result"][i]["actions"]
              actions = np.array(actions)
              energy_cost += np.sum(np.linalg.norm(actions, axis=1)**2)
        
        stats_data["stats"][len(stats_data["stats"])-1]["energy_cost"] = float(energy_cost) # append to the end,
        # since we might have anytime solutions, but always keep the best
        # check if you have solution from the first successful run
        if anytime: 
          for file in os.listdir(run_dir):
              if file.startswith('optimization_') and file.endswith('.yaml'): # solution of the first successful run
                  file_path = os.path.join(run_dir, file)
                  with open(file_path, 'r') as f:
                    first_result_data = yaml.safe_load(f)
                  first_energy_cost = 0 # for each robot
                  for i in range(len(first_result_data["result"])):
                     print(f"robot:  {i}")
                     actions = first_result_data["result"][i]["actions"]
                     actions = np.array(actions)
                     first_energy_cost += np.sum(np.linalg.norm(actions, axis=1)**2)
          stats_data["stats"][0]["energy_cost"] = float(first_energy_cost) # append to the first successful itr.
        # now dump all and save
        with open(stats_path, "w") as f:
            yaml.safe_dump(stats_data, f, default_flow_style=False, sort_keys=False)
        print(f"Updated energy cost in {stats_path}")
        # ----- Your computation here -----
        print(f"\nProcessing: {problem}/{alg}/{run_folder}")
        print(f"  Result keys: {list(result_data.keys()) if result_data else 'None'}")
        print(f"  Has cost in stats: {has_cost}")
        print(f"  Energy cost: {energy_cost}")

def main():
  # 2D case
  # results_path = Path("../results/tro-plots/2d-5/")
  # base_patterns = [
    # "gen_p10_n2_*_unicycle_sphere",
    # "gen_p10_n4_*_unicycle_sphere",
    # "gen_p10_n8_*_unicycle_sphere",
    # "gen_p10_n2_*_hetero",
    # "gen_p10_n4_*_hetero",
    # "gen_p10_n8_*_hetero",
  # ]
# 
  # instances = [
      # pattern.replace("*", str(i))
      # for pattern in base_patterns
      # for i in range(10)
  # ]
  # instances = []
  # instances.append("swap2_unicycle_sphere")
  # instances.append("alcove_unicycle_sphere")
  # instances.append("at_goal_unicycle_sphere")    
  # 
  # algs = [
    # "sst",
    # "s2m2",
    # "k-cbs",
    # "db-cbs",
    # "db-ecbs",
  # ]
# 
  # file_names = [
      # "result_ompl.yaml",
      # "result_kcbs.yaml",
      # "result_s2sm.yaml",
      # "result_dbcbs_opt.yaml",
      # "result_dbecbs_opt.yaml",
  # ]
# 
# 3D cases - Wall, Window examples
  results_path = Path("../results/tro-plots/3d-test/")
  instances = [
    # "drone2c",
    # "drone4c",
    # "drone8c",
    # "drone10c",
    # "drone12c",
    "drone16c",   
    # "wall_drone8c",
    # "wall_drone10c",
  ]
  
  file_names = [
      "result_dbecbs_opt.yaml",
  ]
  algs = [
     "db-ecbs-conservative",
     "db-ecbs-residual",
  ]
  process_all_runs(results_path, instances, algs, file_names, anytime=True)

if __name__ == '__main__':
  main()
