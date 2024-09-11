import argparse
import yaml
import matplotlib.pyplot as plt

def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("file", help="input file containing the solution")
  args = parser.parse_args()

  with open(args.file) as res_file:
    result = yaml.load(res_file, Loader=yaml.FullLoader)

  states = result["result"][0]["states"]
  actions = result["result"][0]["actions"]
  
  fig, axs = plt.subplots(6, 1, figsize=(8, 12)) 
  # for states
  axs[0].plot([sublist[3] for sublist in states], marker='o', color='r')
  axs[0].set_title('vx')

  axs[1].plot([sublist[4] for sublist in states], marker='x', color='g')
  axs[1].set_title('vy')

  axs[2].plot([sublist[5] for sublist in states], marker='s', color='b')
  axs[2].set_title('vz')

  # for actions
  axs[3].plot([sublist[0] for sublist in actions], marker='o', color='r')
  axs[3].set_title('ax')

  axs[4].plot([sublist[1] for sublist in actions], marker='x', color='g')
  axs[4].set_title('ay')

  axs[5].plot([sublist[2] for sublist in actions], marker='s', color='b')
  axs[5].set_title('az')

  plt.tight_layout()
  plt.savefig('../results/check_limits.png')
  plt.show()

if __name__ == "__main__":
  main()
