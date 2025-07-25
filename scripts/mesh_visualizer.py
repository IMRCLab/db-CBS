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

# To do:
# draw the path 
# add start/goal robots with different color
# html out file as arg

DnametoColor = {
    "red": 0xff0000,
    "green": 0x00ff00,
    "blue": 0x0000ff,
    "yellow": 0xffff00,
    "white": 0xffffff,
    "black": 0x000000,
    "cyan": 0x00ffff,
    "magenta": 0xff00ff,
    "orange": 0xffa500,
    "purple": 0x800080
}


def normalize(vec):
    norm_v = np.linalg.norm(vec)
    if norm_v > 0:
        return np.array(vec)/norm_v
    else: 
        raise ValueError("Cannot normalize a zero vector.")

def visualize(env_file, result_file, video_file, payload_file=None):
    vis = meshcat.Visualizer()
    # vis.open()
    anim = Animation()

    res = vis.static_html()
    with open("output.html", "w") as f:
        f.write(res)
    draw_payload = False
    print("payload file: ",payload_file)
    if payload_file is not None:
      with open(payload_file, "r") as f:
        payload_states = yaml.safe_load(f)

      if "payload" in payload_states and payload_states["payload"] is not None:
        pstates = np.array(payload_states["payload"])
        draw_payload = True
        vis["payload"].set_object(
                  g.Mesh(g.Sphere(0.01), g.MeshLambertMaterial(color="r",opacity=1.0)))

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
      vis[f"Obstacle_Box{k}"].set_object(g.Mesh(g.Box(size)))
      vis[f"Obstacle_Box{k}"].set_transform(tf.translation_matrix(center))

       

    with open(result_file) as res_file:
        result = yaml.load(res_file, Loader=yaml.FullLoader)
    states = []
    name_robot = 0
    for i in range(len(result["result"])):
      state = []
      for s in result["result"][i]["states"]:
        state.append(s)
      states.append(state)
      if "quad" in data["robots"][0]["type"]:
        vis["Quadrotor" + str(name_robot)].set_object(g.StlMeshGeometry.from_file('../meshes/cf2_assembly.stl'), g.MeshLambertMaterial(color="green"))
      elif "unicycle" in data["robots"][0]["type"]:
        vis["unicycle" + str(name_robot)].set_object(g.Mesh(g.Box([0.1, 0.05, 0.05]), material=g.MeshLambertMaterial(color=list(DnametoColor.items())[name_robot][1])))
      name_robot+=1
    
    # for i in range(len(states) - 1):
    #     vis[f"rod{i}"].set_object(
    #         g.Mesh(g.Box([0.6*0.5, 0.01, 0.01]), g.MeshLambertMaterial(color=0x000000))
    #     )

    max_k = 0
    for state in states: 
      if len(state) > max_k:
        max_k = len(state)
    for k in range(max_k):
      for l in range(len(states)): # for each robot
        with anim.at_frame(vis, k) as frame:
          if k >= len(states[l]):
            robot_state = states[l][-1]
          else:
            robot_state = states[l][k]
          if "quad" in data["robots"][0]["type"]:

            frame["Quadrotor" + str(l)].set_transform(tf.translation_matrix(robot_state[0:3]).dot(
                tf.quaternion_matrix(np.array([robot_state[6],robot_state[3],robot_state[4],robot_state[5]]))))
          elif "unicycle" in data["robots"][0]["type"]:
            frame["unicycle" + str(l)].set_transform(tf.translation_matrix([robot_state[0], robot_state[1], 0]).dot(
                tf.quaternion_matrix(tf.quaternion_from_euler(0,0,robot_state[2]))))



            # Add rods between robots
            if l < len(states) - 1:
                next_robot_state = states[l + 1][k] if k < len(states[l + 1]) else states[l + 1][-1]
                pos1 = np.array([robot_state[0], robot_state[1]])
                pos2 = np.array([next_robot_state[0], next_robot_state[1]])
                rod_length = np.linalg.norm(pos2 - pos1)
                rod_angle = np.arctan2(pos2[1] - pos1[1], pos2[0] - pos1[0])
                
                rod_center = pos1 + 0.5 * rod_length * normalize(pos2-pos1)
                vis[f"rod{l}"].set_object(
                    g.Mesh(g.Box([0.6*rod_length, 0.01, 0.01]), g.MeshLambertMaterial(color=0x000000))
                )


                frame[f"rod{l}"].set_transform(
                    tf.translation_matrix([rod_center[0], rod_center[1], 0]).dot(
                        tf.quaternion_matrix(tf.quaternion_from_euler(0, 0, rod_angle))
                    )
                )

          if draw_payload:
            frame["payload"].set_transform(tf.translation_matrix(pstates[k,0:3]).dot(tf.quaternion_matrix([1,0,0,0])))
    vis.set_animation(anim)
    res = vis.static_html()
    with open(video_file, "w") as f:
        f.write(res)

            
def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("--env", help="input file containing map")
  parser.add_argument("--payload", default=None, help="payload states path")
  parser.add_argument("--result", help="output file containing solution")
  parser.add_argument("--video", help="output file for video")
  args = parser.parse_args()

  visualize(args.env, args.result, args.video, args.payload)

if __name__ == "__main__":
  main()
