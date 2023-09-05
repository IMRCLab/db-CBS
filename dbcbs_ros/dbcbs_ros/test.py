#!/usr/bin/env python

import numpy as np
from pathlib import Path

from crazyflie_py import *
from crazyflie_py.uav_trajectory import Trajectory
import yaml

def test_1UAV(): # that is not using the first trajectory
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    cf_number = 0
    cf = allcfs.crazyflies[cf_number]   # CrazyflieServer.crazyflies[0] --> Crazyflie

    rate = 10.0
    Z = 0.5

    cf.takeoff(targetHeight=Z, duration=Z+1.0)
    timeHelper.sleep(Z+2.0)

    trajpath = Path(__file__).parent / "data/result_ompl2.yaml"
    # load yaml file contains smooth waypoint
    yaml_path = trajpath
    with open(yaml_path, 'r') as ymlfile:
        data = yaml.safe_load(ymlfile)['result']  # a list  elements are dictionaries
    n = len(data) # number of trajectories
    states_list = []
    for i in range(n):
        states = data[i]['states']
        states_list.append(states)
    print('number of waypoints:',len(states_list[0]))
    print('load finish')
    # check the num of UAV <= states_list
    if n < len(allcfs.crazyflies):
        print(f'not enough trajectory for {len(allcfs.crazyflies)} crazyfile')
        quit()

    for state_id in range(len(states_list[0])):
        if state_id ==1:
            pos = np.array(cf.initialPosition) + np.append(np.array(states_list[0][state_id]), Z)
            timeHelper.sleepForRate(Z+1.0)
        else:
            pos = np.append(np.array(states_list[0][state_id]), Z)
        print('drone_id',cf_number,'initial pos:',pos)
        # pos = np.array(cf.initialPosition) + np.array([1, 1, Z])
        cf.cmdFullState(pos, np.zeros(3), np.zeros(3), 0, np.zeros(3))
        timeHelper.sleepForRate(rate)

    cf.notifySetpointsStop()
    # TODO bug!: the takeoff and land must be at 0,0
    # cf.takeoff(targetHeight=Z+1, duration=Z+1.0)  # this is really another bug! cause when 8 is ok, for the trajectory here is not ok
    cf.land(targetHeight=0.03, duration=Z+1.0)
    timeHelper.sleep(Z+2.0)

def test_multi_UAV():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs   # CrazyflieServer.crazyflies[0] --> Crazyflie

    rate = 10.0
    rate = 3.0
    Z = 0.5

    allcfs.takeoff(targetHeight=Z, duration=Z+1.0)
    timeHelper.sleep(Z+2.0)

    trajpath = Path(__file__).parent / "data/db_cvbs_opt_4.yaml"
    # load yaml file contains smooth waypoint
    yaml_path = trajpath
    with open(yaml_path, 'r') as ymlfile:
        data = yaml.safe_load(ymlfile)['result']  # a list  elements are dictionaries
    n = len(data) # number of trajectories
    states_list = []
    for i in range(n):
        states = data[i]['states']
        states_list.append(states)
    print('number of waypoints:',len(states_list[0]))
    print('load finish')
    # check the num of UAV <= states_list
    if n < len(allcfs.crazyflies):
        print(f'not enough trajectory for {len(allcfs.crazyflies)} crazyfile')
        quit()

    for state_id in range(len(states_list[0])):
    # for state_id in range(6):
        for drone_id in range(len(allcfs.crazyflies)):
            cf = allcfs.crazyflies[drone_id]   
            pos = np.append(np.array(states_list[drone_id][state_id]), Z)
            print('drone_id',drone_id,'initial pos:',pos)
            cf.cmdFullState(pos, np.zeros(3), np.zeros(3), 0, np.zeros(3))
        if state_id == 0:
            timeHelper.sleep(Z+1.0)    
        timeHelper.sleepForRate(rate)

    for cf in allcfs.crazyflies:
        cf.notifySetpointsStop()

    allcfs.land(targetHeight=0.03, duration=Z+1.0)
    timeHelper.sleep(Z+2.0)

def test_vel_acc():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs   # CrazyflieServer.crazyflies[0] --> Crazyflie

    rate = 10.0
    rate = 3.0
    Z = 0.5

    # allcfs.takeoff(targetHeight=Z, duration=Z+1.0)
    # timeHelper.sleep(Z+2.0)
    # parse data
    trajpath = Path(__file__).parent / "data/result_dbcbs_opt.yaml"
    # load yaml file contains smooth waypoint
    yaml_path = trajpath
    with open(yaml_path, 'r') as ymlfile:
        data = yaml.safe_load(ymlfile)['result']  # a list  elements are dictionaries
    n = len(data) # number of trajectories
    if len(data[0]['states'][0]) == 4:
        print("The length of data[0]['states'] is 4.------------")
        states_list = []
        velocity_list = []
        acceleration_list = []
        for trajectory in data:
            states = [row[0:2] + [Z] for row in trajectory['states']]  
            velocity = [row[2:4] + [0.0] for row in trajectory['states']]  
            acceleration = [row[0:2] + [Z] for row in trajectory['actions']]
            states_list.append(states)
            velocity_list.append(velocity)
            acceleration_list.append(acceleration)
        # print(states_list)
    elif len(data[0]['states'][0]) == 2:
        print("The length of data[0]['states'] is 2.")

    states_array = np.array(states_list, dtype=np.float64)
    velocity_array = np.array(velocity_list, dtype=np.float64)
    acceleration_array = np.array(acceleration_list, dtype=np.float64)
    num_waypoints = states_array.shape[1]
    print('number of waypoints:',num_waypoints)
    file_name = yaml_path.stem
    print(f'load {file_name} finish')

    # check the num of UAV <= states_list
    if n < len(allcfs.crazyflies):
        print(f'not enough trajectory for {len(allcfs.crazyflies)} crazyfile')
        quit()


    # ------ run data
    for state_id in range(num_waypoints):
    # for state_id in range(6):
        for drone_id in range(len(allcfs.crazyflies)):
            cf = allcfs.crazyflies[drone_id]   
            pos = states_array[drone_id][state_id]
            vel = velocity_array[drone_id][state_id]
            acc = acceleration_array[drone_id][state_id]
            print('drone_id',drone_id,'pos:',pos,'vel',vel,'acc',acc)
            cf.cmdFullState(pos, vel, acc, 0, np.zeros(3))
        # if state_id == 0:
        #     timeHelper.sleep(Z+1.0)    
        timeHelper.sleepForRate(rate)

    for cf in allcfs.crazyflies:
        cf.notifySetpointsStop()

    allcfs.land(targetHeight=0.03, duration=Z+1.0)
    timeHelper.sleep(Z+2.0)



def main():
    # test_1UAV()
    # test_multi_UAV()
    test_vel_acc()


if __name__ == "__main__":
    main()
