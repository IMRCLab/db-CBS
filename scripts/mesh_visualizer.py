import numpy as np

# visualization related
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
from meshcat.animation import Animation

import argparse
import yaml
import time

def visualize(env_file, result_file):
    vis = meshcat.Visualizer()
    anim = Animation()

    res = vis.static_html()

    vis["/Cameras/default"].set_transform(
        tf.translation_matrix([0, 0, 0]).dot(
            tf.euler_matrix(0, np.radians(-30), np.radians(90))))
    
    vis["/Cameras/default/rotated/<object>"].set_transform(
        tf.translation_matrix([1, 0, 0]))


    with open(env_file) as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
        
    obstacles = data["environment"]["obstacles"]
    for k, obs in enumerate(obstacles):
      center = obs["center"]
      size = obs["size"]
      obs_type = obs["type"]
      if (obs_type == 'octomap'):
         vis[f"Obstacle{k}"].set_object(g.StlMeshGeometry.from_file('../meshes/map.stl'), g.MeshLambertMaterial(color="blue")) # hard-coded
      elif (obs_type == 'box'):
        vis[f"Obstacle{k}"].set_object(g.Mesh(g.Box(size)))
        vis[f"Obstacle{k}"].set_transform(tf.translation_matrix(center))
      else:
         print("Unknown Obstacle type!")

    with open(result_file) as res_file:
        result = yaml.load(res_file, Loader=yaml.FullLoader)
    states = []
    name_robot = 0
    for i in range(len(result["result"])):
        state = []
        for s in result["result"][i]["states"]:
          state.append(s)
        states.append(state)

        vis["Quadrotor" + str(name_robot)].set_object(g.StlMeshGeometry.from_file('../meshes/cf2_assembly.stl'), g.MeshLambertMaterial(color="green"))
      
        name_robot+=1
    max_k = len(max(states))
    for k in range(max_k):
      for l in range(len(states)): # for each robot
        with anim.at_frame(vis, 10*k) as frame:
          if k >= len(states[l]):
            robot_state = states[l][-1]
          else:
            robot_state = states[l][k]
          frame["Quadrotor" + str(l)].set_transform(tf.translation_matrix(robot_state[0:3]).dot(
              tf.quaternion_matrix(np.array([1,0,0,0]))))
      time.sleep(0.1)

    vis.set_animation(anim)
    res = vis.static_html()
    with open("octomap_test.html", "w") as f:
        f.write(res)
            
def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("env", help="input file containing map")
  parser.add_argument("--result", help="output file containing solution")
  args = parser.parse_args()

  visualize(args.env, args.result)

if __name__ == "__main__":
  main()
