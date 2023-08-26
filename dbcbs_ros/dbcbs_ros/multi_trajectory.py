#!/usr/bin/env python

import numpy as np
from pathlib import Path
from crazyflie_py import *
import yaml

height = 0.5

def multi_traj():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs  # CrazyflieServer

    # load yaml file contains smooth waypoint
    yaml_path = Path(__file__).parent / "data/result_ompl2.yaml"
    with open(yaml_path, 'r') as ymlfile:
        data = yaml.safe_load(ymlfile)['result']  # a list  elements are dictionaries
    n = len(data) # number of distinct trajectories
    states_list = []
    for i in range(n):
        states = data[i]['states']
        states_list.append(states)

    # check the num of UAV <= states_list
    if n < len(allcfs.crazyflies):
        print(f'not enough trajectory for {len(allcfs.crazyflies)} crazyfile')
        quit()
    allcfs.takeoff(targetHeight=0.5, duration=2.0)
    timeHelper.sleep(2)
    print('number of waypoints:',len(states_list[0]))
    for state_id in range(len(states_list[0])):  # can goto synchronized for multi drones?
        for drone_id in range(len(allcfs.crazyflies)): 
            togo_time = 5.0
            if state_id ==1:
                pos = np.append(np.array(states_list[drone_id][state_id]), height)
                print('drone_id',drone_id,'initial pos:',pos)
                allcfs.crazyflies[drone_id].goTo(pos, 0, togo_time)
                # print('drone_id',drone_id, 'len(allcfs.crazyflies)',len(allcfs.crazyflies))
                if drone_id == len(allcfs.crazyflies)-1:
                    print('if drone_id == len(allcfs.crazyflies)')
                    timeHelper.sleep(togo_time/2)
                    # quit()
            else:
                pos = np.append(np.array(states_list[drone_id][state_id]), height)
                allcfs.crazyflies[drone_id].goTo(pos, 0, togo_time)
                print('drone_id',drone_id,'state_id',state_id)
    timeHelper.sleep(togo_time + 1)
    allcfs.land(targetHeight=0.06, duration=2.0)

def main():
    multi_traj()

if __name__ == "__main__":
    main()
