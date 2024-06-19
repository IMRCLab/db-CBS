import numpy as np

# visualization related
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
from meshcat.animation import Animation
import argparse
import yaml
import time
import os
from pathlib import Path

# script visualizes motions/short trajectories expanded during the discrete search 
def visualize(env_file, trajs_file, result_file = None):
    vis = meshcat.Visualizer()
    anim = Animation()

    res = vis.static_html()

    vis["/Cameras/default"].set_transform(
        tf.translation_matrix([0, 0, 0]).dot(
            tf.euler_matrix(0, np.radians(-30), np.radians(90))))
    
    vis["/Cameras/default/rotated/<object>"].set_transform(
        tf.translation_matrix([1, 0, 0]))

    # get environment info
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
    # get expanded trajs
    with open(trajs_file) as traj_file:
        expanded_trajs = yaml.load(traj_file, Loader=yaml.FullLoader)
    traj_idx = 0
    for i in range(len(expanded_trajs["trajs"])): # each motion primitive/traj = several states
        traj = []
        position = [] 
        for s in expanded_trajs["trajs"][i]["states"]:
          traj.append(s)
        start = [traj[0][0], traj[0][1], traj[0][2]]  
        position = [[sublist[j] for sublist in traj] for j in range(3)] # assumes 3D 
        position = np.array(position)
        vis["expanded_trajs" + str(traj_idx)].set_object(g.Line(g.PointsGeometry(position), g.LineBasicMaterial(color=0x00ff00ff))) # magenta
        vis["sphere" + str(traj_idx)].set_object(g.Mesh(g.Sphere(0.01), g.MeshLambertMaterial(color=0x1000000))) # black
        vis["sphere" + str(traj_idx)].set_transform(tf.translation_matrix(start))
        traj_idx += 1
    # get results
    if result_file is not None:
      with open(result_file) as res_file:
          result = yaml.load(res_file, Loader=yaml.FullLoader)
      states = []
      name_robot = 0
      for i in range(len(result["result"])):
          state = []
          position = [] 
          for s in result["result"][i]["states"]:
            state.append(s)
          states.append(state)
          position = [[sublist[i] for sublist in state] for i in range(3)] # assumes 3D 
          position = np.array(position)
          vis["Quadrotor" + str(name_robot)].set_object(g.StlMeshGeometry.from_file('../meshes/cf2_assembly.stl'), g.MeshLambertMaterial(color=0x0000FF)) # blue
          vis["trajectory" + str(name_robot)].set_object(g.Line(g.PointsGeometry(position), g.LineBasicMaterial(color=0x00FF00))) # green
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
    result_folder = Path(trajs_file).resolve().parent
    vis.set_animation(anim)
    res = vis.static_html()
    with open(result_folder / ("octomap_exp_" + os.path.basename(env_file).split('.')[0] + ".html"), "w") as f:
        f.write(res)
            
def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("env", help="input file containing map")
  parser.add_argument("--trajs", help="file with expanded trajectories during the search")
  parser.add_argument("--result", help="output file containing solution")
  args = parser.parse_args()

  visualize(args.env, args.trajs, args.result)

if __name__ == "__main__":
  main()
