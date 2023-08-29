#!/usr/bin/env python

import numpy as np
from pathlib import Path

from crazyflie_py import *
from crazyflie_py.uav_trajectory import Trajectory
import yaml

def executeTrajectory(timeHelper, allcfs, trajpath, Z, rate=100):
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

    togo_time = 5.0
    zero_vec = np.array([0.0,0.0,0.0])
    for state_id in range(len(states_list[0])):  # can goto synchronized for multi drones?
        for drone_id in range(len(allcfs.crazyflies)): 
            cf = allcfs.crazyflies[drone_id]
            # make sure all the crazyfiles are at the right intial position
            if state_id ==1:
                pos = np.append(np.array(states_list[drone_id][state_id]), Z) + np.array(cf.initialPosition) 
                # print('drone_id',drone_id,'initial pos:',pos)
                # cf.goTo(pos, 0, togo_time)
                cf.cmdPosition(pos, yaw=0.0)
                # pos vel acc yaw omega
                # cf.cmdFullState(pos,zero_vec,zero_vec,0,zero_vec)   # must be something here wrong, so it will go to 0,0,0
                if drone_id == len(allcfs.crazyflies)-1:
                    print('if drone_id == len(allcfs.crazyflies)')
                    timeHelper.sleep(togo_time/2)
                    # quit()
            # run
            else:
                pos = np.append(np.array(states_list[drone_id][state_id]), Z)
                # cf.goTo(pos, 0, togo_time)
                cf.cmdPosition(pos, yaw=0.0)
                # cf.cmdFullState(pos,zero_vec,zero_vec,0,zero_vec)
                print('drone_id',drone_id,'state_id',state_id)


            timeHelper.sleepForRate(rate)

def setUp(extra_args=""):
    crazyflies_yaml = """
    crazyflies:
    - channel: 100
      id: 1
      initialPosition: [1.0, 0.0, 0.0]
    """
    swarm = Crazyswarm(crazyflies_yaml=crazyflies_yaml, args="--sim --vis null " + extra_args)
    timeHelper = swarm.timeHelper
    return swarm.allcfs, timeHelper


def test_cmdFullState_zeroVel(timeHelper, allcfs,Z):
    cf = allcfs.crazyflies[0]
    print('cf.initialPosition',cf.initialPosition)

    pos = np.array(cf.initialPosition) + np.array([1, 1, Z])
    cf.cmdFullState(pos, np.zeros(3), np.zeros(3), 0, np.zeros(3))
    timeHelper.sleep(1.0)
    print('test_cmdFullState_zeroVel over')

    # assert np.all(np.isclose(cf.position(), pos))

def test_cmdPosition(timeHelper, allcfs,Z):
    cf = allcfs.crazyflies[0]
    print('cf.initialPosition',cf.initialPosition)
    pos = np.array(cf.initialPosition) + np.array([1, 1, Z])
    cf.cmdPosition(pos,yaw=0.0)
    timeHelper.sleep(1.0)
    print('test_cmdPosition over')
    # assert np.all(np.isclose(cf.position(), pos))


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    # cf = swarm.allcfs.crazyflies[0]
    allcfs = swarm.allcfs 

    rate = 30.0
    rate = 10.0
    rate = 1.0
    Z = 1.5


    # allcfs.takeoff(targetHeight=Z, duration=Z+1.0)
    # timeHelper.sleep(Z+2.0)
    trajpath = Path(__file__).parent / "data/result_ompl2.yaml"
    # test_cmdFullState_zeroVel(timeHelper, allcfs,Z)
    test_cmdPosition(timeHelper, allcfs,Z)
    # executeTrajectory(timeHelper, allcfs, trajpath,Z, rate)

    # for cf in allcfs.crazyflies:
    #     cf.notifySetpointsStop()
    # allcfs.land(targetHeight=0.03, duration=Z+1.0)
    # timeHelper.sleep(Z+2.0)


if __name__ == "__main__":
    main()
