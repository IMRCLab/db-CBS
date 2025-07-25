#!/usr/bin/env python
# -*- coding: utf-8 -*-
import msgpack
import yaml
import numpy as np
from numpy import random

def loadyaml(file_in):
    with open(file_in, "r") as f:
        return yaml.safe_load(f)

def saveyaml(file_dir, data):
    with open(file_dir, "w") as f:
        yaml.safe_dump(data, f, default_flow_style=None)



def split_primitive(states, actions, min_chunks=3, max_chunks=6):
    """
    Splits a primitive into smaller chunks of states and actions.

    Args:
        states (list): List of states in the primitive.
        actions (list): List of actions in the primitive.
        min_chunks (int): Minimum number of chunks to split into.
        max_chunks (int): Maximum number of chunks to split into.

    Returns:
        list: List of new primitives as dictionaries.
    """
    num_states = len(states)
    num_chunks = random.randint(min_chunks, max_chunks)
    # num_chunks = 2
    chunk_size = num_states // num_chunks
    remainder = num_states % num_chunks

    new_primitives = []
    start_idx = 0
    for i in range(num_chunks):
        extra = 1 if i < remainder else 0
        end_idx = start_idx + chunk_size + extra
        new_prim = {
            "states": states[start_idx:end_idx],
            "actions": actions[start_idx:end_idx - 1],  # actions are one less than states
        }
        new_primitives.append(new_prim)
        start_idx = end_idx

    return new_primitives



def main():
    # msg_file = "/path/to/your/input.msgpack"
    # new_msg_file = "/path/to/your/output.msgpack"
    # new_primitives = [("/path/to/your/yaml_file.yaml", 3, "unicycle")]

    # msg_file = "/home/khaledwahba94/imrc/db-CBS/new_format_motions/unicycle1_v0/unicycle1_v0_working.msgpack"
    msg_file = "/home/khaledwahba94/imrc/db-CBS/new_format_motions/quad3d_v0_old/quad3d_v0.msgpack"
    new_msg_file = "/home/khaledwahba94/imrc/db-CBS/new_format_motions/quad3d_v0_old/quad3d_v0_cut.msgpack"

    # Unpack existing msgpack
    with open(msg_file, "rb") as data_file:
        byte_data = data_file.read()

    data_loaded = msgpack.unpackb(byte_data)
    prims = data_loaded["data"]
    new_prims = dict()
    new_prims["data"] = []
    total_primitives_before = len(data_loaded["data"])
    print("total primitives before: ", total_primitives_before)
    # Process new primitives
    
    for prim in prims:
        states = prim["states"]
        actions = prim["actions"]
        split_prims = split_primitive(states, actions, min_chunks=2, max_chunks=5)
        new_prims["data"].extend(split_prims)

    total_primitives_after = len(new_prims["data"])
    print("Total primitives after:", total_primitives_after)

    # Save updated msgpack
    with open(new_msg_file, "wb") as file:
        msgpack.pack(new_prims, file)

if __name__ == "__main__":
    main()
