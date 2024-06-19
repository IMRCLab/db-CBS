import numpy as np

# visualization related
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
from meshcat.animation import Animation
from pathlib import Path

import argparse
import yaml
import time
import os
def visualize(env_file, result_file, filename_video=None):
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
         octomap_stl = obs["octomap_stl"]
         vis[f"Obstacle{k}"].set_object(g.StlMeshGeometry.from_file(octomap_stl), g.MeshLambertMaterial(opacity=0.4, color=0xFFFFFF)) 
      elif (obs_type == 'box'):
        vis[f"Obstacle{k}"].set_object(g.Mesh(g.Box(size)))
        vis[f"Obstacle{k}"].set_transform(tf.translation_matrix(center))
      else:
         print("Unknown Obstacle type!")

    with open(result_file) as res_file:
        result = yaml.load(res_file, Loader=yaml.FullLoader)
    states = []
    name_robot = 0
    max_k = 0
    for i in range(len(result["result"])):
        state = []
        position = [] 
        for s in result["result"][i]["states"]:
          state.append(s)
        max_k = max(max_k, len(state))
        states.append(state)
        position = [[sublist[i] for sublist in state] for i in range(3)] # assumes 3D 
        position = np.array(position)
        vis["Quadrotor" + str(name_robot)].set_object(g.StlMeshGeometry.from_file('../meshes/cf2_assembly.stl'), g.MeshLambertMaterial(color=0x0000FF)) # blue
        vis["trajectory" + str(name_robot)].set_object(g.Line(g.PointsGeometry(position), g.LineBasicMaterial(color=0x00FF00))) # green
        name_robot+=1
    # max_k = len(max(states))
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

    # with open("octomap_" + os.path.basename(env_file).split('.')[0] + ".html", "w") as f:
    #     f.write(res)
    
    html_file = Path(result_file).with_suffix(".html")
    with open(html_file, "w") as f:
        f.write(res)
            
def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("env", help="input file containing map")
  parser.add_argument("--result", help="output file containing solution")
  parser.add_argument("--video", default = None, help="output file for video")
  args = parser.parse_args()

  visualize(args.env, args.result, args.video)

if __name__ == "__main__":
  main()
