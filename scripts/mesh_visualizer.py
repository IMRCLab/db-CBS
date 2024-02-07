import numpy as np
from pathlib import Path

# visualization related
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
from meshcat.animation import Animation

import argparse
import yaml
import time
# copied from https://github.com/rdeits/meshcat-python/blob/master/src/meshcat/geometry.py#L83
# since the latest pip-version doesn't include it yet

# To do:
# support obstacles
# add mesh for other robot dynamics if possible -> hybrid

def visualize(env_file, result_file):
    vis = meshcat.Visualizer()
    vis.open()

    vis["/Cameras/default"].set_transform(
        tf.translation_matrix([0, 0, 0]).dot(
            tf.euler_matrix(0, np.radians(-30), np.radians(90))))
    
    vis["/Cameras/default/rotated/<object>"].set_transform(
        tf.translation_matrix([1, 0, 0]))


    with open(env_file) as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
        
    with open(result_file) as res_file:
        result = yaml.load(res_file, Loader=yaml.FullLoader)
    states = []
    robot_shapes = []
    for i in range(len(result["result"])):
        state = []
        robot_shapes.append(g.StlMeshGeometry.from_file('../meshes/cf2_assembly.stl'))
        for s in result["result"][i]["states"]:
          state.append(s)
        states.append(state)

    while True:
      max_k = len(max(states))
      for k in range(max_k):
        for l in range(len(states)): # for each robot
            if k >= len(states[l]):
              robot_state = states[l][-1]
            else:
              robot_state = states[l][k]
            robot_shape = robot_shapes[l]
            vis["Quadrotor" + str(l)].set_object(robot_shape, g.MeshLambertMaterial(color="green"))
            vis["Quadrotor" + str(l)].set_transform(tf.translation_matrix(robot_state[0:3]).dot(
                tf.quaternion_matrix(np.array([1,0,0,0]))))
        time.sleep(0.1)
      time.sleep(0.3)
            
def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("env", help="input file containing map")
  parser.add_argument("--result", help="output file containing solution")
#   parser.add_argument("--video", help="output file for video")
  args = parser.parse_args()

  visualize(args.env, args.result)

if __name__ == "__main__":
  main()
