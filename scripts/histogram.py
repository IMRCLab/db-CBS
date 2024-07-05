import argparse
import yaml
import math 
import matplotlib.pyplot as plt

def histogram(env_file, trajs_file):
    # get environment info
    with open(env_file) as f:
        env = yaml.load(f, Loader=yaml.FullLoader)

    # get generated motions
    with open(trajs_file) as traj_file:
        trajs = yaml.load(traj_file, Loader=yaml.FullLoader) # len = number of motions

    distance = []
    for traj in trajs:
       start = traj["states"][0][:3] # only position
       goal = traj["states"][-1][:3]
       distance.append(math.dist(start,goal)) # Euclidean distance

    # Create a histogram
    plt.hist(distance, bins=30, alpha=0.7, color='blue', edgecolor='black')

    # Add titles and labels
    plt.title('Distance between Start/Goal')
    plt.xlabel('Value')
    plt.ylabel('Frequency')
    plt.show()

def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('env', help="input file containing map")
  parser.add_argument('--trajs', help="file with expanded trajectories during the search")

  args = parser.parse_args()

  histogram(args.env, args.trajs)

if __name__ == "__main__":
  main()