#!/usr/bin/env python

import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
import yaml 

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

    plt.show()

def test_load_data():
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
    plot_one_figure()
    # test_load_data()
if __name__ == "__main__":
    main()
