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
import matplotlib.pyplot as plt
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
         vis[f"Obstacle{k}"].set_object(g.StlMeshGeometry.from_file(octomap_stl), g.MeshLambertMaterial(opacity=0.8, color=0xFFFFFF)) 
      elif (obs_type == 'box'):
        vis[f"Obstacle{k}"].set_object(g.Mesh(g.Box(size)))
        vis[f"Obstacle{k}"].set_transform(tf.translation_matrix(center))
      else:
         print("Unknown Obstacle type!")

    add_walls = False
    if(add_walls):
        # Define boundaries, my 3D problem
      min_dim = data["environment"]["min"]
      max_dim = data["environment"]["max"]

      min_bounds = np.array([min_dim[0], min_dim[1], min_dim[2]])
      max_bounds = np.array([max_dim[0], max_dim[1], max_dim[2]])

      size = max_bounds - min_bounds
      floor_size = [size[0], size[1], 0.1]  # Thin floor
      wall_thickness = 0.1  # Thin walls

      # Add floor
      vis["floor"].set_object(g.Box(floor_size), g.MeshLambertMaterial(color=0x888888))
      vis["floor"].set_transform(
          meshcat.transformations.translation_matrix([(min_bounds[0] + max_bounds[0]) / 2,
                                                      (min_bounds[1] + max_bounds[1]) / 2,
                                                      min_bounds[2] - 0.05])  # Slightly below min z
      )

      # Add first wall (along x-axis)
      # wall_x_size = [wall_thickness, size[1], size[2]]
      # vis["wall_x"].set_object(g.Box(wall_x_size), g.MeshLambertMaterial(opacity=0.6, color=0xC0C0C0))
      # vis["wall_x"].set_transform(
      #     meshcat.transformations.translation_matrix([min_bounds[0] - wall_thickness / 2,
      #                                                 (min_bounds[1] + max_bounds[1]) / 2,
      #                                                 min_bounds[2] + size[2] / 2])
      # )

      # Add second wall (along y-axis)
      wall_y_size = [size[0], wall_thickness, size[2]]
      vis["wall_y"].set_object(g.Box(wall_y_size), g.MeshLambertMaterial(opacity=0.6, color=0xC0C0C0))
      vis["wall_y"].set_transform(
          meshcat.transformations.translation_matrix([(min_bounds[0] + max_bounds[0]) / 2,
                                                max_bounds[1] + wall_thickness / 2,
                                                min_bounds[2] + size[2] / 2])
      )

     # Add second wall (along x-axis)
      wall_x_size = [wall_thickness, size[1], size[2]]
      vis["wall_x"].set_object(g.Box(wall_x_size), g.MeshLambertMaterial(opacity=0.6, color=0xC0C0C0))
      vis["wall_x"].set_transform(
          meshcat.transformations.translation_matrix([max_bounds[0] - wall_thickness / 2,
                                                      (min_bounds[1] + max_bounds[1]) / 2,
                                                      min_bounds[2] + size[2] / 2])
      )

    with open(result_file) as res_file:
        result = yaml.load(res_file, Loader=yaml.FullLoader)
    # for the histogram
    if result.get("cluster_tracking"):
      clusters = result["cluster_tracking"]
      robots = list(range(0, len(result["result"]) + 1, 1))
      plt.bar(robots, clusters)
      # plt.xticks(ticks=range(min(clusters), max(clusters) + 1, 1))
      plt.xticks(np.arange(0, len(robots), 1))
      plt.title('Histogram for Robot Numbers')
      plt.xlabel('Robot ID')
      plt.ylabel('Frequency')
      plt.savefig(Path(result_file).with_suffix(".jpg"))
    # for the residual force
    if result.get("fa"):
      fa = result["fa"]
      plt.plot(fa)
      plt.title('Residual force')
      plt.xlabel('time')
      plt.ylabel('fa')
      fa_file = Path(result_file).with_stem(Path(result_file).stem + "_fa").with_suffix(".jpg")
      plt.savefig(fa_file)
    states = []
    name_robot = 0
    max_k = 0
    # start, goal states
    start_goal = True
    if(start_goal):
      robots = data["robots"]
      for r in range (len(robots)):
        start = robots[r]["start"][:3]
        goal = robots[r]["goal"][:3]
        vis["sphere" + str(r*2)].set_object(g.Mesh(g.Sphere(0.03), g.MeshLambertMaterial(opacity=0.4,color=0xFF0000))) 
        vis["sphere" + str(r*2)].set_transform(tf.translation_matrix(start))
        vis["box" + str(r*2 + 1)].set_object(g.Box([0.05, 0.05, 0.05]), g.MeshLambertMaterial(opacity=0.4, color=00000000))
        vis["box" + str(r*2 + 1)].set_transform(tf.translation_matrix(goal))

    for i in range(len(result["result"])):
        state = []
        position = [] 
        for s in result["result"][i]["states"]:
          state.append(s)
        max_k = max(max_k, len(state))
        states.append(state)
        position = [[sublist[i] for sublist in state] for i in range(3)] # assumes 3D 
        position = np.array(position)
        if(data["robots"][i]["type"] == "integrator2_3d_large_v0"):
          vis["Quadrotor" + str(name_robot)].set_object(g.StlMeshGeometry.from_file('../meshes/cf2_assembly.stl'), g.MeshLambertMaterial(color=0xFF0000)) # red
        else: 
          vis["Quadrotor" + str(name_robot)].set_object(g.StlMeshGeometry.from_file('../meshes/cf2_assembly.stl'), g.MeshLambertMaterial(color=0x0000FF)) # blue
        vis["trajectory" + str(name_robot)].set_object(g.Line(g.PointsGeometry(position), g.LineBasicMaterial(color=0x006400))) # green - 0x00FF00 
        name_robot+=1
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
