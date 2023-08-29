#!/usr/bin/env python

import numpy as np
from pathlib import Path

from crazyflie_py import *
from crazyflie_py.uav_trajectory import Trajectory

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


if __name__ == "__main__":
    main()
