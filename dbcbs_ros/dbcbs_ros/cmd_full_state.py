#!/usr/bin/env python

import numpy as np
from pathlib import Path

from crazyflie_py import *
from crazyflie_py.uav_trajectory import Trajectory
import yaml

def parse_data(yaml_path,Z):
    with open(yaml_path, 'r') as ymlfile:
        data = yaml.safe_load(ymlfile)['result']  # a list  elements are dictionaries
    num_traj = len(data) # number of trajectories
    print('number of trajectories',num_traj)
    if len(data[0]['states'][0]) == 4:
        print("The length of data[0]['states'] is 4.------------")
    else:
        print("The length of data[0]['states'] is",len(data[0]['states'][0]))
        quit()

    num_waypoints = max([len(trajectory['states']) for trajectory in data])
    print("Minimum number of waypoints:", num_waypoints)

    states_list = []
    velocity_list = []
    acceleration_list = []
    for trajectory in data:
        states = [row[0:2] + [Z] for row in trajectory['states']]  
        velocity = [row[2:4] + [0] for row in trajectory['states']]  
        acceleration = [row[0:2] + [0] for row in trajectory['actions']]
        while len(states) < num_waypoints:
            states.append(states[-1])  # Append the last line of states
            
        while len(velocity) < num_waypoints:
            velocity.append([0, 0, 0])  # Append [0, 0, 0] to velocity
            
        while len(acceleration) < num_waypoints:
            acceleration.append([0, 0, 0])  # Append [0, 0, Z] to acceleration
        states_list.append(np.array(states, dtype=np.float64))
        velocity_list.append(np.array(velocity, dtype=np.float64))
        acceleration_list.append(np.array(acceleration, dtype=np.float64))

    file_name = yaml_path.stem
    print(f'load {file_name} finish')
    return num_traj,num_waypoints,states_list,velocity_list,acceleration_list

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs   # CrazyflieServer.crazyflies[0] --> Crazyflie

    rate = 10.0
    Z = 0.5

    allcfs.takeoff(targetHeight=Z, duration=Z+1.0)
    timeHelper.sleep(Z+2.0)

    # parse data
    yaml_path = Path(__file__).parent / "data/swap4_1.yaml"
    num_traj,num_waypoints,states_list,velocity_list,acceleration_list = parse_data(yaml_path,Z)

    # check the num of UAV <= states_list
    if num_traj < len(allcfs.crazyflies):
        print(f'not enough trajectory for {len(allcfs.crazyflies)} crazyfile')
        quit()

    # execute trajectory
    for state_id in range(num_waypoints):
        for drone_id in range(len(allcfs.crazyflies)):
            cf = allcfs.crazyflies[drone_id]   
            pos = states_list[drone_id][state_id]
            vel = velocity_list[drone_id][state_id]
            acc = acceleration_list[drone_id][state_id]
            print('drone_id',drone_id,'pos:',pos,'vel',vel,'acc',acc)
            cf.cmdFullState(pos, vel, acc, 0, np.zeros(3))  
        timeHelper.sleepForRate(rate)

    for cf in allcfs.crazyflies:
        cf.notifySetpointsStop()

    allcfs.land(targetHeight=0.02, duration=3.0)
    timeHelper.sleep(3.0)

if __name__ == "__main__":
    main()
