#!/usr/bin/env python
# -*- coding: utf-8 -*-
import msgpack
import yaml
import numpy as np

def cut_prim(robots, steps=10):
    data = []
    for robot in robots.keys():
        states = robots[robot]["states"]
        actions = robots[robot]["actions"]
        state_counter = 0
        states_size = len(states)
        while state_counter < states_size-1:
            robot_dict = {"states": [], "actions": []}
            start = state_counter
            goal = state_counter + steps if state_counter + steps < states_size else states_size
            robot_dict["states"] = states[start:goal]
            robot_dict["actions"] = actions[start:goal-1]
            data.append(robot_dict)
            state_counter += steps
    return data

def compute_unicycle_st(state, num_robots, robot_idx, init_state):
    px = state[0]
    py = state[1]
    l = 0.5
    for k in range(robot_idx):
        th = state[2 + num_robots + k]
        px += l * np.cos(th)
        py += l * np.sin(th)

    alpha = state[2 + robot_idx]
    return [px - init_state[0], py - init_state[1], alpha]

def loadyaml(file_in):
    with open(file_in, "r") as f:
        return yaml.safe_load(f)

def saveyaml(file_dir, data):
    with open(file_dir, "w") as f:
        yaml.safe_dump(data, f, default_flow_style=None)

def main():
    # msg_file = "/path/to/your/input.msgpack"
    # new_msg_file = "/path/to/your/output.msgpack"
    # new_primitives = [("/path/to/your/yaml_file.yaml", 3, "unicycle")]

    # msg_file = "/home/khaledwahba94/imrc/db-CBS/new_format_motions/unicycle1_v0/unicycle1_v0_working.msgpack"
    msg_file = "/home/khaledwahba94/imrc/db-CBS/new_format_motions/unicycle1_v0/unicycle1_v0.msgpack"
    new_msg_file = "/home/khaledwahba94/imrc/db-CBS/new_format_motions/unicycle1_v0/unicycle1_v0.msgpack"

    new_primitives = [
        # joint system primitives, num_robots, system type
        # ("/home/khaledwahba94/imrc/db-CBS/stats_db/window_2robots_unicycle/000/output", 2, "unicycle"),
        # ("/home/khaledwahba94/imrc/db-CBS/stats_db/window_3robots_unicycle/000/output", 3, "unicycle"),
        # ("/home/khaledwahba94/imrc/db-CBS/stats_db/window_4robots_unicycle/000/output", 4, "unicycle"),
        # ("/home/khaledwahba94/imrc/db-CBS/stats_db/window_5robots_unicycle/000/output", 5, "unicycle"),
        ("/home/khaledwahba94/imrc/db-CBS/stats_db/forest_2robots_unicycle/000/output", 2, "unicycle"),
        ("/home/khaledwahba94/imrc/db-CBS/stats_db/forest_3robots_unicycle/000/output", 3, "unicycle"),
        ("/home/khaledwahba94/imrc/db-CBS/stats_db/forest_4robots_unicycle/000/output", 4, "unicycle"),
        ("/home/khaledwahba94/imrc/db-CBS/stats_db/forest_5robots_unicycle/000/output", 5, "unicycle"),

        ]

    # Unpack existing msgpack
    with open(msg_file, "rb") as data_file:
        byte_data = data_file.read()

    data_loaded = msgpack.unpackb(byte_data)
    total_primitives_before = len(data_loaded["data"])
    print("total primitives before: ", total_primitives_before)
    # Process new primitives
    for path, num_robots, robot_type in new_primitives:
        print(path)
        if robot_type == "unicycle":
            file_data = loadyaml(path)
            states = file_data["result"][0]["states"]
            actions = np.array(file_data["result"][0]["actions"])
            init_state = states[0]
            robots = {}

            for idx in range(num_robots):
                robots[str(idx)] = {"states": [], "actions": []}
                for k, state in enumerate(states):
                    robots[str(idx)]["states"].append(compute_unicycle_st(state, num_robots, idx, init_state))
                    if k < len(states) - 1:
                        robots[str(idx)]["actions"].append(actions[k, 2 * idx:2 * idx + 2].tolist())
            cut_primitives = cut_prim(robots, steps=10)
            len_before = len(data_loaded["data"]) 
            data_loaded["data"].extend(cut_primitives)
            # print(cut_primitives)
            print("added prims: ",len(data_loaded["data"]) - len_before)
    total_primitives_after = len(data_loaded["data"])

    # Save updated msgpack
    with open(new_msg_file, "wb") as file:
        msgpack.pack(data_loaded, file)
    total_primitives_after = len(data_loaded["data"])
    print("total primitives after: ", total_primitives_after)
    print("increase: ", total_primitives_after - total_primitives_before)

if __name__ == "__main__":
    main()
