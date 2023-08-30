#!/usr/bin/env python

import numpy as np
from pathlib import Path

from crazyflie_py import *
from crazyflie_py.uav_trajectory import Trajectory
import yaml

def test_with_one_point():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]   # CrazyflieServer.crazyflies[0] --> Crazyflie

    rate = 30.0
    Z = 0.5

    cf.takeoff(targetHeight=Z, duration=Z+1.0)
    timeHelper.sleep(Z+2.0)

    pos = np.array(cf.initialPosition) + np.array([1, 1, Z])
    cf.cmdFullState(pos, np.zeros(3), np.zeros(3), 0, np.zeros(3))
    timeHelper.sleepForRate(rate)
    # quit()



    cf.notifySetpointsStop()
    cf.land(targetHeight=0.03, duration=Z+1.0)
    timeHelper.sleep(Z+2.0)

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    cf = allcfs.crazyflies[0]   # CrazyflieServer.crazyflies[0] --> Crazyflie

    rate = 30.0
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
        print('drone_id',0,'initial pos:',pos)
        # pos = np.array(cf.initialPosition) + np.array([1, 1, Z])
        cf.cmdFullState(pos, np.zeros(3), np.zeros(3), 0, np.zeros(3))
        timeHelper.sleepForRate(rate)



    cf.notifySetpointsStop()
    # TODO bug!: the takeoff and land must be at 0,0
    # cf.takeoff(targetHeight=Z+1, duration=Z+1.0)  # this is really another bug! cause when 8 is ok, for the trajectory here is not ok
    cf.land(targetHeight=0.03, duration=Z+1.0)
    timeHelper.sleep(Z+2.0)


if __name__ == "__main__":
    main()
