import os
import yaml
import numpy as np
import argparse
import math
from collections import defaultdict

def transform_environments(original_envs):
    """Transforms the environment dictionary to the desired format."""
    transformed_envs = {}
    for group, env_list in original_envs.items():
        transformed_envs[group] = [
            f"{group}, {env.split('_')[1][0]} robots" for env in env_list
        ]
    return transformed_envs

# # Original dictionary
# environments = {
#     "Window": ["window_2robots", "window_3robots", "window_4robots", "window_5robots", "window_6robots"],
#     "Wall": ["wall_2robots", "wall_3robots", "wall_4robots", "wall_5robots", "wall_6robots"],
#     "Forest": ["forest_2robots", "forest_3robots", "forest_4robots", "forest_5robots", "forest_6robots"],
# }


def map_to_final_format(data_stats_db_done, data_alt_structure, environments, new_envs):
    """Maps data_stats_db_done and data_alt_structure to the final format."""
    data = {}
    envs_keys = list(new_envs.keys())
    envs_items = list(new_envs.items())
    i = 0
    # print(new_envs)
    for env_group, env_list in environments.items():
        j = 0
        for env in env_list:
            formatted_env = env.replace(" ", "_").lower()
            env_list_new = new_envs[envs_keys[i]]
            data[env_list_new[j]] = {
                "Ours": {
                    "UR": data_stats_db_done.get(formatted_env, {}).get(
                        "UR", {"success": None, "cost": None, "time": None}
                    ),
                    "MP": data_stats_db_done.get(formatted_env, {}).get(
                        "MP", {"success": None, "cost": None, "time": None}
                    ),
                },
                "BL": {
                    "UR": data_alt_structure.get(formatted_env, {}).get(
                        "UR", {"success": None, "cost": None, "time": None}
                    ),
                    "MP": data_alt_structure.get(formatted_env, {}).get(
                        "MP", {"success": None, "cost": None, "time": None}
                    ),
                },
            }
            j = j+1
        i = i+1

    return data


def process_db_data(base_path, environments, robot_types):
    """Processes the stats_db_done folder structure."""
    data = {}

    def parse_trajectory_opt_check(file_path):
        """Parses trajectory_opt.check.txt to compute success rate."""
        with open(file_path, 'r') as f:
            lines = f.readlines()
        successes = sum(1 for line in lines if line.strip().endswith("OK"))
        total = len([line for line in lines if "trajectory_opt.yaml" in line])
        return (successes / total) * 100 if total > 0 else 0

    def parse_result_dbcbs_opt(file_path):
        """Parses result_dbcbs_opt.yaml for cost."""
        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)
        return float(data["cost"])
        # return data["cost"]

    def parse_dbcbs_stats(file_path):
        """Parses dbcbs_stats.yaml for time (duration_opt + duration_discrete)."""
        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)
        data = data["runs"][0]
        return data.get('duration_opt', 0) + data.get('duration_discrete', 0)

    def parse_dbcbs_stats_1iter(file_path):
        """Parses dbcbs_stats.yaml for time (duration_opt + duration_discrete)."""
        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)
        return data.get('total_time', 0)

    for env_group, env_list in environments.items():
        for env in env_list:
            data[env] = {}
            for robot_type in robot_types:
                is_mp = robot_type == "MP"
                env_name = f"{env}{'_unicycle' if robot_type == 'UR' else ''}"
                full_path = os.path.join(base_path, env_name)
                if not os.path.exists(full_path):
                    print(f"Directory not found: {full_path}")
                    continue
                trials = sorted(os.listdir(full_path))
                costs, times, errors = [], [], []
                successes = 0
                total_trials = len(trials)

                for trial in trials:
                    trial_path = os.path.join(full_path, trial)
                    trajectory_check_path = os.path.join(trial_path, 'trajectory_opt.check.txt')
                    result_dbcbs_path = os.path.join(trial_path, 'result_dbcbs_opt.yaml')
                    dbcbs_stats_path = os.path.join(trial_path, 'iteration_cost.yaml')
                    dbcbs_stats_1iter_path = os.path.join(trial_path, 'dbcbs_stats.yaml')
                    trajectory_opt_path = os.path.join(trial_path, 'trajectory_opt.yaml')

                    if os.path.exists(trajectory_check_path):
                        if "OK" in open(trajectory_check_path).read():
                            successes += 1
                    if os.path.exists(result_dbcbs_path):
                        costs.append(parse_result_dbcbs_opt(result_dbcbs_path))
                    if os.path.exists(dbcbs_stats_path):
                        times.append(parse_dbcbs_stats(dbcbs_stats_path))
                    elif os.path.exists(dbcbs_stats_1iter_path):
                        times.append(parse_dbcbs_stats_1iter(dbcbs_stats_1iter_path))
                    if os.path.exists(trajectory_opt_path):
                        mean_error, std_error = parse_trajectory_opt(trajectory_opt_path, is_mp)
                        errors.append((mean_error, std_error))

                success_rate = (successes / total_trials) * 100 if total_trials > 0 else 0
                data[env][robot_type] = {
                    "success": float(success_rate) if not math.isnan(success_rate) else None,
                    "cost": [float(np.mean(costs)), float(np.std(costs))] if not (math.isnan(np.mean(costs)) or math.isnan(np.std(costs))) else None,
                    "time": [float(np.mean(times)), float(np.std(times))] if not (math.isnan(np.mean(times)) or math.isnan(np.std(times))) else None,
                }

                # print(env, robot_type, data[env][robot_type])
    return data

def parse_trajectory_opt(file_path, is_mp):
    """Parses trajectory_opt.yaml for error."""
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)
    refstates = np.array(data['result']['refstates'])
    states = np.array(data['result']['states'])
    if is_mp:
        # Compute error for MP: First 3 columns and all rows
        errors = [np.linalg.norm(refstates[i, :3] - states[i, :3]) for i in range(len(refstates))]
    else:
        # Compute error for UR: Full comparison
        errors = np.linalg.norm(refstates - states, axis=1)
    return np.mean(errors), np.std(errors)

def parse_output_trajopt(file_path):
    """Parses output.trajopt.yaml for feasibility and cost."""
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)
    feasible = data.get("feasible", 0)
    cost = data.get("cost", float("inf"))
    return feasible == 1, cost

def process_coltrans_data(base_path, environments, robot_types):
    """Processes the alternate folder structure."""
    data = {}

    for env_group, env_list in environments.items():
        for env in env_list:
            data[env] = {}
            for robot_type in robot_types:
                is_mp = robot_type == "MP"
                env_name = f"{env}{'_unicycle' if robot_type == 'UR' else ''}"
                full_path = os.path.join(base_path, env_name)
                if not os.path.exists(full_path):
                    print(f"Directory not found: {full_path}")
                    continue
                
                geom_path = os.path.join(full_path, 'geom')
                opt_path = os.path.join(full_path, 'opt')
                geom_trials = sorted(os.listdir(geom_path))
                if os.path.exists(opt_path):
                    opt_trials = sorted(os.listdir(opt_path))
                else:
                    opt_trials = []
                total_trials = len(geom_trials)
                successes = 0

                costs, times, errors = [], [], []

                for trial in opt_trials:
                    opt_trial_path = os.path.join(opt_path, trial)
                    geom_trial_path = os.path.join(geom_path, trial)

                    # Parse output.trajopt.yaml for feasibility
                    output_trajopt_path = os.path.join(opt_trial_path, 'output.trajopt.yaml')
                    if not os.path.exists(output_trajopt_path):
                        continue  # Skip trial if no output.trajopt.yaml exists

                    feasible, cost = parse_output_trajopt(output_trajopt_path)
                    if not feasible:
                        continue  # Skip trial if it's marked as not feasible
                    
                    successes += 1
                    costs.append(cost)

                    # Parse stats.yaml for time
                    opt_stats_path = os.path.join(opt_trial_path, 'stats.yaml')
                    geom_stats_path = os.path.join(geom_trial_path, 'stats.yaml')
                    opt_time = geom_time = 0

                    if os.path.exists(opt_stats_path):
                        with open(opt_stats_path, 'r') as f:
                            opt_stats = yaml.safe_load(f)
                        opt_time = opt_stats.get('opt_time', 0)

                    if os.path.exists(geom_stats_path):
                        with open(geom_stats_path, 'r') as f:
                            geom_stats = yaml.safe_load(f)
                        geom_time = geom_stats.get('geom_time', 0)

                    total_time = opt_time + geom_time
                    times.append(total_time)

                    # Parse trajectory_opt.yaml for error
                    trajectory_opt_path = os.path.join(opt_trial_path, 'trajectory_opt.yaml')
                    if os.path.exists(trajectory_opt_path):
                        mean_error, std_error = parse_trajectory_opt(trajectory_opt_path, is_mp)
                        errors.append((mean_error, std_error))

                success_rate = (successes / total_trials) * 100 if total_trials > 0 else 0
                data[env][robot_type] = {
                    "success": float(success_rate) if not math.isnan(success_rate) else None,
                    "cost": [float(np.mean(costs)), float(np.std(costs))] if not (math.isnan(np.mean(costs)) or math.isnan(np.std(costs))) else None,
                    "time": [float(np.mean(times)), float(np.std(times))] if not (math.isnan(np.mean(times)) or math.isnan(np.std(times))) else None,
                }
                # print(env, robot_type, data[env][robot_type])

    return data


def main():
    parser = argparse.ArgumentParser(description="Process environment statistics.")
    parser.add_argument("--db_dir", required=True, default=None, help="Path to the 'stats_db_done' folder.")
    parser.add_argument("--coltrans_dir", required=False, default=None, help="Path to the alternate folder structure.")
    parser.add_argument("--out", help="output_yaml.")
    args = parser.parse_args()
    out = args.out
    environments = {
        # "Window" : [
        #     "window_4robots_25", 
        #     "window_4robots_100", 
        #     "window_4robots_200", 
        #     "window_4robots_500", 
        #     "window_4robots_1000", 
        # ],
        # "Wall" : [
        #     "wall_2robots", 
        #     "wall_3robots", 
        #     "wall_4robots", 
        #     "wall_5robots", 
        #     "wall_6robots", 
        # ],
        "Forest" : [
            "forest_6robots", 
        ],
        # "Forest": [ 
        #            "forest_4robots", 
        #            "forest_4robots_cable", 
        #            "forest_4robots_db", 
        #            "forest_4robots_db_orig"
        #            ],
        # "Wall" : [
        #     "wall_2robots_unicycle", "wall_3robots_unicycle", "wall_4robots_unicycle", "wall_5robots_unicycle", "wall_6robots_unicycle"
        # ]
        # "Window" : [
        #     "window_2robots", "window_3robots", "window_4robots", "window_5robots", "window_6robots"
        # ],
        # "Forest" : [
        #     "forest_2robots", "forest_3robots", "forest_4robots", "forest_5robots", "forest_6robots"
        # ],
    }
    # robot_types = ["UR", "MP"]
    # robot_types = ["UR"]
    robot_types = ["MP"]

    if args.db_dir is not None: 
        print(f"Processing {args.db_dir}")
        data_db = process_db_data(args.db_dir, environments, robot_types)
        print(data_db)
    else:
        print("db is None")
        data_db = {}

    if args.coltrans_dir is not None: 
        print(f"Processing {args.coltrans_dir}")
        data_coltrans = process_coltrans_data(args.coltrans_dir, environments, robot_types)
        print(data_coltrans)
    else:
        print("coltransplanning is None")
        data_coltrans = {}
    # Combine the data into a single dictionary
    combined_data = {
        "stats_db": data_db,
        "stats_coltransplanning": data_coltrans,
    }

    # Save the combined data to a single YAML file
    with open(f"../combined_data_{out}.yaml", "w") as file:
        yaml.dump(combined_data, file, default_flow_style=False)

    print(f"Data has been saved to 'combined_data_{out}.yaml'.")
    new_envs = transform_environments(environments)

    final_data = map_to_final_format(data_db, data_coltrans, environments, new_envs)

    # Optionally, save the data to a YAML file
    with open(f"../final_data_{out}.yaml", "w") as file:
        yaml.dump(final_data, file, default_flow_style=False)

    print(f"Data saved to 'final_data_{out}.yaml'.")

if __name__ == "__main__":
    main()
