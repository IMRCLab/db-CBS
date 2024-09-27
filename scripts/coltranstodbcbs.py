import yaml
from pathlib import Path
import numpy as np
import argparse
import os
import re

def saveyaml(file_out, data):
    with open(file_out, "w") as f:
        yaml.safe_dump(data,f,default_flow_style=None)

def loadyaml(file_in):
    with open(file_in, "r") as f: 
        file_out = yaml.safe_load(f)
    return file_out


def createRobotDict(coltrans_dict):
    start = coltrans_dict["robots"][0]["start"]
    goal = coltrans_dict["robots"][0]["goal"]
    num_robots =  coltrans_dict["robots"][0]["quadsNum"]
    l = coltrans_dict["robots"][0]["l"]

    robots = []
    for i in range(num_robots):
        robot_i_dict = dict()
        robot_i_dict["type"] = "quad3d_v0"
        p0_start =  np.array(start[0:3])
        p0_goal  =  np.array(goal[0:3])
        qi_start = np.array(start[6+6*i: 6+3+6*i])
        qi_goal  = np.array(goal[6+6*i: 6+3+6*i])

        pi_start = p0_start - l[i]*qi_start
        pi_goal  = p0_goal - l[i]*qi_goal

        robot_start = np.zeros(13,)
        robot_goal = np.zeros(13,)
        # position
        robot_start[0:3] = pi_start 
        robot_goal[0:3]  = pi_goal 
        # quaternion
        robot_start[3:7] = [0,0,0,1]   
        robot_goal[3:7]  = [0,0,0,1]
        # velocity
        robot_start[7:10] = [0,0,0] 
        robot_goal[7:10]  = [0,0,0] 
        # angular velocity
        robot_start[10:13] = [0,0,0] 
        robot_goal[10:13]  = [0,0,0] 


        robot_i_dict["start"] = robot_start.tolist()
        robot_i_dict["goal"] = robot_goal.tolist()
        robots.append(robot_i_dict)
    return robots


def create_db_CBS_env_for_benchmark(coltrans_dict):

    robots = createRobotDict(coltrans_dict)
    dbcbs_env_dict = dict()
    dbcbs_env_dict["name"] = coltrans_dict["name"]
    dbcbs_env_dict["environment"] = coltrans_dict["environment"]
    dbcbs_env_dict["robots"] = robots
    dbcbs_env_dict["joint_robot"] = coltrans_dict["robots"]
    
    return dbcbs_env_dict

# def create_payload_env_for_optimization(coltrans_dict):
#     payload_env_dict = dict()
#     payload_env_dict["name"] = coltrans_dict["name"]
#     payload_env_dict["environment"] = env_dict["environment"]
#     payload_env_dict["robots"] = robots_dict
    
#     return payload_env_dict



def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--coltrans', type=str, help="input environment from coltrans")
    parser.add_argument('--dbcbs', type=str, help="output folder directory for the environment")
    args = parser.parse_args()

    if os.listdir(args.coltrans): 
        coltrans_files = [os.path.join(args.coltrans, file) for file in os.listdir(args.coltrans)]

        for file in coltrans_files:
            print(f"Processing file: {file}")
            coltrans_dict = loadyaml(file)
            env_dict = dict()
            env_dict["name"] = coltrans_dict["name"]
            env_dict["environment"] = coltrans_dict["environment"]
            joint_robot_dict = coltrans_dict["robots"]
            dbcbs_env_dict = create_db_CBS_env_for_benchmark(coltrans_dict)

            # create environment to be used in the benchmark for dbcbs robots and optimization of the payload system
            dbcbs_path = re.sub(r'coltransplanning/', 'dbcbs/', file)
            saveyaml(dbcbs_path, dbcbs_env_dict)  

    else:
        print("Directory is empty.")

    args = parser.parse_args()
    


if __name__=="__main__":
    main()


