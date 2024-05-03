#!/usr/bin/env python
# -*- coding: utf-8 -*-
import msgpack
import yaml
import numpy as np
# convert from db_cbs submission format to Quim's new format
msg_file = "/home/akmarak-laptop/IMRC/db-CBS/motions/double_integrator_0_sorted.msgpack"

with open(msg_file, "rb") as data_file:
    byte_data = data_file.read()

data_loaded = msgpack.unpackb(byte_data)
all = {}
all['data'] = []
for i in range(len(data_loaded)):
# for i in range(5000):
    one = {}
    one["states"], one["actions"] = [], []
    for j in range(len(data_loaded[i]["states"])):
        one["states"].append(data_loaded[i]["states"][j])

    for k in range(len(data_loaded[i]["actions"])):
        if any(type(item) == str for item in data_loaded[i]["actions"][k]):
            print(f"k i {k} {i}")
            print(data_loaded[i]["actions"][k])
            for l in range(len(data_loaded[i]["actions"][k])):
                data_loaded[i]["actions"][k][l] = float(data_loaded[i]["actions"][k][l])
            print(data_loaded[i]["actions"][k])
        one["actions"].append(data_loaded[i]["actions"][k])
    all['data'].append(one)

new_name = "/home/akmarak-laptop/IMRC/motions_dynoplan/integrator2_2d_v0"

with open(new_name + ".msgpack", 'wb') as file:
	msgpack.pack(all,file)
     
with open(new_name + ".yaml", 'w') as file:
    documents = yaml.dump(all, file)

# understanding/testing Quim's new format
# msg_file = "/home/akmarak-laptop/IMRC/dynoplan/data/motion_primitives/unicycle1_v0/unicycle1_v0__ispso__2023_04_03__14_56_57.bin.less.bin.msgpack"
# # Read msgpack file
# with open(msg_file, "rb") as data_file:
#     byte_data = data_file.read()

# data_loaded = msgpack.unpackb(byte_data)
# print(len(data_loaded['data']))
# print(len(data_loaded['data'][0]['states']))

# all = {}
# all['data'] = []
# for i in range(len(data_loaded["data"])):
#     one = {}
#     one["states"], one["actions"] = [], []
#     for j in range(len(data_loaded["data"][i]["states"])):
#         one["states"].append(data_loaded["data"][i]["states"][j])
#     for k in range(len(data_loaded["data"][i]["actions"])):
#         one["actions"].append(data_loaded["data"][i]["actions"][k])
#     all['data'].append(one)

# print(len(all['data']))
# print(len(all['data'][0]['states']))
# new_name = "/home/akmarak-laptop/IMRC/db-CBS/motions/test"

# with open(new_name + ".msgpack", 'wb') as file:
# 	msgpack.pack(data,file)
     
# with open(new_name + ".yaml", 'w') as file:
#     documents = yaml.dump(data, file)
