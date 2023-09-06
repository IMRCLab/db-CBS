#!/usr/bin/env python

import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
import yaml

# from crazyflie_py import *
# # from crazyflie_py.uav_trajectory import Trajectory
# import yaml

'''
want to see the trajectory cause it is now not that round enough
save the xyz format and use uav gen to visualise it 
'''
def plot_1yaml_2traj():
    # load yaml file contains smooth waypoint
    yaml_path = Path(__file__).parent / "data/db_cbs_opt_4.yaml"
    with open(yaml_path, 'r') as ymlfile:
        data = yaml.safe_load(ymlfile)['result']  # a list  elements are dictionaries
    print('load finish')
    
    
    # height = 0.5
    n = len(data) # number of distinct trajectories
    traj1 = data[0]
    states = traj1['states']  # list len 73
    # print(np.array(states))  # list 
    traj2 = data[1]
    states2 = traj2['states']  # list len 73
    print('states2',states2)


    number = 70
    # Extract x and y coordinates from the points
    x_coords1 = [point[0] for point in states[:number]]
    y_coords1 = [point[1] for point in states[:number]]
    x_coords2 = [point[0] for point in states2[:number]]
    y_coords2 = [point[1] for point in states2[:number]]
    # Plot the points
    # Plot the first set of points in blue
    plt.scatter(x_coords1, y_coords1, color='blue', label='States 1')
    # Plot the second set of points in red
    plt.scatter(x_coords2, y_coords2, color='red', label='States 2')  # [0.762263, 1.01913]
    plt.xlabel('X')
    plt.ylabel('Y')
    file_name = yaml_path.stem
    print(file_name)
    plt.title(f'{file_name}')
    # plt.title('Scatter Plot of Points')
    plt.grid(True)
    plt.show()

    # print('end')

def plot_all():
    # Get a list of YAML files in the data directory
    data_directory = Path(__file__).parent / "data"
    yaml_files = data_directory.glob("*.yaml")

    # Iterate through each YAML file
    for yaml_path in yaml_files:
        # Load YAML file containing smooth waypoints
        with open(yaml_path, 'r') as ymlfile:
            data = yaml.safe_load(ymlfile)['result']  # a list where elements are dictionaries
        
        # Extract file name without extension
        file_name = yaml_path.stem
        
        # Number of states to consider for each trajectory
        number = 70
        number = -1
        
        # Create a new figure for each YAML file
        plt.figure()
        
        # Plot all trajectories in the current YAML file
        for i, traj in enumerate(data):
            states = traj['states'][:number]
            
            # Extract x and y coordinates from the states
            x_coords = [point[0] for point in states]
            y_coords = [point[1] for point in states]
            
            # Plot the points for the current trajectory
            plt.scatter(x_coords, y_coords, label=f'Trajectory {i+1}')
        
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title(f'Scatter Plot of Points - {file_name}')
        plt.grid(True)
        plt.legend()

    # Show all figures
    plt.show()

def plot_one_figure():
    yaml_path = Path(__file__).parent / "data/swap4_1.yaml"
    with open(yaml_path, 'r') as ymlfile:
        data = yaml.safe_load(ymlfile)['result']  # a list where elements are dictionaries
    
    # Extract file name without extension
    file_name = yaml_path.stem
    
    # Number of states to consider for each trajectory
    # number = 20
    number = -1
    
    # Create a new figure for each YAML file
    plt.figure()
    
    # Plot all trajectories in the current YAML file
    for i, traj in enumerate(data):
        states = traj['states'][:number]
        
        # Extract x and y coordinates from the states
        x_coords = [point[0] for point in states]
        y_coords = [point[1] for point in states]
        
        # Plot the points for the current trajectory
        plt.scatter(x_coords, y_coords, label=f'Trajectory {i+1}')
        
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title(f'position Plot - {file_name}')
        plt.grid(True)
        plt.legend()

    # Create actions figure
    actions_flag = 0
    if actions_flag == 1:
        plt.figure(figsize=(12, 6))
        # Plot all trajectories in the current YAML file for actions
        for i, traj in enumerate(data):
            actions = traj['actions'][:number]

            # Plot the actions for the current trajectory
            plt.plot(range(len(actions)), actions, label=f'Trajectory {i + 1}')

        plt.xlabel('Time Step')
        plt.ylabel('Action Value')
        plt.title(f'Action Plot - {file_name}')
        plt.grid(True)
        plt.legend()

    # Show all figures
    plt.show()

def test():
    Z = 0.5
    yaml_path = Path(__file__).parent / "data/swap4.yaml"
    with open(yaml_path, 'r') as ymlfile:
        data = yaml.safe_load(ymlfile)['result']  # a list where elements are dictionaries
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
            states_list.append(np.array(states, dtype=np.float64))
            velocity_list.append(velocity)
            acceleration_list.append(acceleration)
        # print(states_list)
    elif len(data[0]['states'][0]) == 2:
        print("The length of data[0]['states'] is 2.")

    # states_array = np.array(states_list, dtype=np.float64)
    velocity_array = np.array(velocity_list, dtype=np.float64)
    acceleration_array = np.array(acceleration_list, dtype=np.float64)
    num_waypoints = states_list[0].shape[1]
    print('1')

def main():
    # plot_all()
    # plot_1yaml_2traj()
    plot_one_figure()
    # test()
if __name__ == "__main__":
    main()
