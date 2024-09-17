#!/usr/bin/env python
# -*- coding: utf-8 -*-
import msgpack
import yaml
import numpy as np
# convert from db_cbs submission format to Quim's new format

# msg_file = "/home/khaledwahba94/imrc/db-CBS/old_format_motions/quad3d_v0/quad3d_v0_all3.bin.im.bin.sp1.bin.ca.bin.msgpack"
msg_file = "/home/khaledwahba94/imrc/test_dynoplan/motions/double_integrator_0_sorted.msgpack"
new_name = "/home/khaledwahba94/imrc/db-CBS/new_format_motions/integrator2_2d_v0/integrator2_2d_v0"

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

with open(new_name + ".msgpack", 'wb') as file:
	msgpack.pack(all,file)
     
with open(new_name + ".yaml", 'w') as file:
    documents = yaml.dump(all, file)
