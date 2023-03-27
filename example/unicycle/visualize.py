#!/usr/bin/env python3
import argparse
import numpy as np
import yaml
import matplotlib.pyplot as plt
import matplotlib
from matplotlib.patches import Circle, Rectangle, Arrow
from matplotlib import animation
import matplotlib.animation as manimation
import os
import sys

def draw_box_patch(ax, center, size, angle = 0, **kwargs):
  xy = np.asarray(center) - np.asarray(size) / 2
  rect = Rectangle(xy, size[0], size[1], **kwargs)
  t = matplotlib.transforms.Affine2D().rotate_around(
      center[0], center[1], angle)
  rect.set_transform(t + ax.transData)
  ax.add_patch(rect)
  return rect


class Animation:
  def __init__(self, filename_env, filename_result = None):
    with open(filename_env) as env_file:
      env = yaml.safe_load(env_file)

    self.fig = plt.figure()  # frameon=False, figsize=(4 * aspect, 4))
    self.ax = self.fig.add_subplot(111, aspect='equal')
    self.ax.set_xlim(env["environment"]["min"][0], env["environment"]["max"][0])
    self.ax.set_ylim(env["environment"]["min"][1], env["environment"]["max"][1])

    # for obstacle in env["environment"]["obstacles"]:
    #   if obstacle["type"] == "box":
    #     draw_box_patch(
    #         self.ax, obstacle["center"], obstacle["size"], facecolor='gray', edgecolor='black')
    #   else:
    #     print("ERROR: unknown obstacle type")

    for robot in env["robots"]:  
      self.size = []
      for i in range(robot["numbers"]):
        self.size.append(np.array([0.5, 0.25])) 
      self.draw_robot(robot["start"], facecolor='red')
      self.draw_robot(robot["goal"], facecolor='none', edgecolor='red')

    self.robot_numbers = len(self.size)
    if filename_result is not None:
      with open(filename_result) as result_file:
        self.result = yaml.safe_load(result_file)

      T = 0
      for robot in self.result["result"]:
        T = max(T, len(robot["states"]))
      print("T", T)

      self.robot_patches = []
      for robot in self.result["result"]:
        state = robot["states"][0]
        patches = self.draw_robot(state, facecolor='blue')
        self.robot_patches.extend(patches)

      self.anim = animation.FuncAnimation(self.fig, self.animate_func,
                                frames=T,
                                interval=100,
                                blit=True)

  def save(self, file_name, speed):
    self.anim.save(
      file_name,
      "ffmpeg",
      fps=10 * speed,
      dpi=200),
      # savefig_kwargs={"pad_inches": 0, "bbox_inches": "tight"})

  def show(self):
    plt.show()

  def animate_func(self, i):
    print(i)
    for k, robot in enumerate(self.result["result"]):
      state = robot["states"][i]
      for j in range(self.robot_numbers):
          pos = state[3*j:3*j+2]
          yaw = state[3*j+2]
          xy = np.asarray(pos) - np.asarray(self.size[j]) / 2

          self.robot_patches[2*k+j].set_xy(xy)
          t = matplotlib.transforms.Affine2D().rotate_around(
              pos[0], pos[1], yaw)
          self.robot_patches[2*k+j].set_transform(t + self.ax.transData)

    return self.robot_patches

  def draw_robot(self, state, **kwargs):
    xy0 = state[0:2]
    yaw0 = state[2]
    xy1 = state[3:5]
    yaw1 = state[5]
    patch1 = draw_box_patch(self.ax, xy0, self.size[0], yaw0, **kwargs)
    patch2 = draw_box_patch(self.ax, xy1, self.size[1], yaw1, **kwargs)

    return [patch1, patch2]

def visualize(filename_env, filename_result = None, filename_video=None):
  anim = Animation(filename_env, filename_result)
  anim.show()
  # if filename_video is not None:
  #   anim.save(filename_video, 1)
  # else:
  #   anim.show()

def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("env", help="input file containing map")
  parser.add_argument("--result", help="output file containing solution")
  parser.add_argument("--video", help="output file for video")
  args = parser.parse_args()

  visualize(args.env, args.result, args.video)

if __name__ == "__main__":
  main()
