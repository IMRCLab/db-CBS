import math
import numpy as np
import rowan as rn
import yaml
import time
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
from meshcat.animation import Animation
# import viewer_utils
import argparse
import matplotlib.pyplot as plt
from pathlib import Path

def axis_angle_to_quaternion(axis, angle):
    axis = np.array(axis)
    axis = axis / np.linalg.norm(axis)
    angle /= 2
    w = np.cos(angle)
    x, y, z = axis * np.sin(angle)
    return np.array([w, x, y, z])

DnametoColor = {
    "red": 0xff0000,
    "green": 0x00ff00,
    "blue": 0x0000ff,
    "yellow": 0xffff00,
    "white": 0xffffff,
}


class Robot:
    def __init__(self, params, states=None):
        l, ri = params
        self.l = l
        self.ri = ri
        self.states = states

    def getPayload(self, i):
        xy = self.states[i,0:2]
        th = self.states[i,2]
        return np.array([*xy, 0]), th


    def getP1(self, i):
        p, th = self.getPayload(i)
        s = np.sin(th)
        c = np.cos(th)
        q = np.array([c, s, 0])
        p1 = p +  self.l/2*q
        return p1
    
    def getP2(self, i):
        p, th = self.getPayload(i)
        s = np.sin(th)
        c = np.cos(th)
        q = np.array([c, s, 0])
        p2 = p -  self.l/2*q
        return p2

class Visualizer:
    def __init__(self, robot, env):
        self.vis = meshcat.Visualizer()
        self.vis["/Cameras/default"].set_transform(
            tf.translation_matrix([1.0, 0, 0]).dot(
                tf.euler_matrix(np.radians(-40), np.radians(0), np.radians(-100))))
        self.vis["/Cameras/default/rotated/<object>"].set_transform(
            tf.translation_matrix([-2, 0, 2.5]))
        self.robot = robot
        self.obstacles = env["environment"]["obstacles"]
        # self._setObjects()
        # self.updateVis(0 ,"start")
        self.updateVis(0 ,"state")
        # self.updateVis(-1 ,"goal")

    def updateVis(self, state_id, prefix: str = "state", frame=None):
        quat_id = np.array([1,0,0,0])
        if frame is None:
            frame = self.vis
            frame[prefix+'_robot1'].set_object(g.Mesh(
                g.Sphere(self.robot.ri), g.MeshLambertMaterial(DnametoColor.get('red', 0xff11dd))))        
            self.vis[prefix+'_robot2'].set_object(g.Mesh(
                g.Sphere(self.robot.ri), g.MeshLambertMaterial(DnametoColor.get('green', 0xff11dd))))        
            self.vis[prefix+'_payload'].set_object(g.Mesh(
                g.Box([self.robot.l, 0.025, 0.025] ), g.MeshLambertMaterial(DnametoColor.get('blue', 0xff11dd))))        
            
            if self.obstacles is not None: 
                for idx, obstacle in enumerate(self.obstacles):
                    obsMat = g.MeshLambertMaterial(opacity=0.5, color=0x008000)
                    center = obstacle["center"]
                    shape = obstacle["type"]
                    if (shape == "sphere"):
                        radius = obstacle["radius"]
                        self.vis["obstacle" +
                                str(idx)].set_object(g.Mesh(g.Sphere(radius), material=obsMat))
                        self.vis["obstacle" +
                                str(idx)].set_transform(tf.translation_matrix(center))
                    elif shape == "box":
                        size = obstacle["size"]
                        self.vis["obstacle" +
                                str(idx)].set_object(g.Mesh(g.Box(size), material=obsMat))
                        self.vis["obstacle" +
                                str(idx)].set_transform(tf.translation_matrix(center))

        frame[prefix+'_robot1'].set_transform(
                    tf.translation_matrix(self.robot.getP1(state_id)).dot(
                        tf.quaternion_matrix(quat_id)))

        frame[prefix+'_robot2'].set_transform(
                tf.translation_matrix(self.robot.getP2(state_id)).dot(
                    tf.quaternion_matrix(quat_id)))
        p, th = self.robot.getPayload(state_id)
        quat = axis_angle_to_quaternion([0,0,1], th)
        frame[prefix+'_payload'].set_transform(
                tf.translation_matrix(p).dot(
                    tf.quaternion_matrix(quat)))
        

def bar_integrator2_2d():
    parser = argparse.ArgumentParser()
    # parser.add_argument('--robot', type=str, help="robot model: dintegratorcables")
    parser.add_argument('--env', type=str, help="environment")
    parser.add_argument('--result', type=str, help="result trajectory")
    parser.add_argument('--output', type=str, help="html animation")
    parser.add_argument("-i", "--interactive", action="store_true")  # on/off flag

    args = parser.parse_args()
    pathtoenv = args.env
    # robotname = args.robot
    with open(pathtoenv, "r") as file:
        env = yaml.safe_load(file)
    l = 0.5
    ri = 0.1
    start = env["robots"][0]["start"]
    goal = env["robots"][0]["goal"]
    obstacles = env["environment"]["obstacles"]

    result = args.result
    if start is not None or goal is not None: 
        if result is not None:
            with open(result, 'r') as file:
                path = yaml.safe_load(file)

            if "states" in path:
                states = path['states']
            elif "result" in path:
                states = path['result']['states']
                actions = path['result']['actions']
            else: 
                raise NotImplementedError("unknown result format")
        params = (l,ri)
        robot = Robot(params,states=np.array(states))

        visualizer = Visualizer(robot, env)
        if args.interactive:
            visualizer.vis.open()

        anim = Animation()
        for k, state in enumerate(states):
            with anim.at_frame(visualizer.vis, k) as frame:
                visualizer.updateVis(k, frame=frame)
        visualizer.vis.set_animation(anim)

        res = visualizer.vis.static_html()
        # save to a file
        print(args.output)
        with open(args.output, "w") as f:
            f.write(res)
    else:
        print('no results')



if __name__ == "__main__":
  bar_integrator2_2d()  