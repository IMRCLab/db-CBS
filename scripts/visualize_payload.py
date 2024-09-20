import math
import numpy as np
import rowan as rn
import yaml
import time
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
from meshcat.animation import Animation
import argparse
import matplotlib.pyplot as plt
from pathlib import Path


class Quad():
    def __init__(self, name, l):
        self.l = l  # length of cable
        self.name = name
        self.state = 0  # [quad_pos, quat, vel, w]
        self.state_ap = []

    def _updateState(self, state):
        self.state = state
        self.state_ap.append(state)


class Payload():
    def __init__(self, shape):
        self.shape = shape
        self.state = 0
        self.state_ap = []

    def _updateState(self, state):
        self.state = state
        self.state_ap.append(state)


class QuadPayloadRobot():
    def __init__(self, quadNum=1, pType="quad3dpayload"):
        # ASSUMPTION: CABLE LENTH IS ALWAYS 0.5
        self.quadNum = quadNum
        self.pType = pType
        l = (0.5 * np.ones(quadNum,)).tolist()
        self._addQuad(l)
        self._addPayload()

    def _addQuad(self, l):
        self.quads = dict()
        for i in range(self.quadNum):
            self.quads[str(i)] = Quad(str(i), l[i])

    def _addPayload(self):
        self.payload = Payload(self.pType)

    def updateFullState(self, state):
        _state = np.empty(19,)
        num_uavs = self.quadNum
        for name, quad in self.quads.items():
            if self.payload.shape == "quad3dpayload":
                self.state = state.copy()
                _state = state.copy()  # this is useful for the multiple uavs
            elif self.payload.shape == "point":
                # THIS FOR MULTIPLE UAVs FOR POINTMASS PAYLOAD:
                # state that defines UAV state: p_load, qc, v_load, wc, quat, w
                qcwcs = state[6:6 + 6 * num_uavs]
                quatws = state[6 + 6 * num_uavs:6 +
                               6 * num_uavs + 7 * num_uavs]
                i = 6 * int(name)
                qc = qcwcs[i: i + 3]
                wc = qcwcs[i + 3: i + 6]
                j = 7 * int(name)
                quat = quatws[j:j + 4]
                w = quatws[j + 4:j + 7]
                _state[0:3] = state[0:3]
                _state[3:6] = qc
                _state[6:9] = state[3:6]
                _state[9:12] = wc
                _state[12:16] = quat
                _state[16:19] = w

            elif self.payload.shape == "rigid":
                # THIS IS FOR MULTIPLE UAVS WITH RIGID PAYLOAD
                # NOT IMPLEMENTED
                print('NOT IMPLEMENTED, please use payload type as point')
                exit()
            else:
                print("Payload type doesn't exist")
                exit()

            self.quads[name] = self._updateRobotState(quad, _state)
        self.payload._updateState(state)

    def _updateRobotState(self, quad, state):
        # position, quaternion, velocity, angular velocity
        p_load = np.array(state[0:3])
        qc = np.array(state[3:6])
        v_load = np.array(state[6:9])
        wc = np.array(state[9:12])
        qc_dot = np.cross(wc, qc)
        quat = state[12:16]
        quat = np.array([quat[3], quat[0], quat[1], quat[2]])
        w = np.array(state[16:19])
        p_quad = p_load - quad.l * qc
        v_quad = v_load - quad.l * qc_dot
        quad_state = [*p_quad, *quat, *v_quad, *w]
        quad._updateState(quad_state)
        return quad


DnametoColor = {
    "red": 0xff0000,
    "green": 0x00ff00,
    "blue": 0x0000ff,
    "yellow": 0xffff00,
    "white": 0xffffff,
}


class Visualizer():
    def __init__(self, QuadPayloadRobot, env):
        self.vis = meshcat.Visualizer()
        self.vis["/Cameras/default"].set_transform(
            tf.translation_matrix([0.5, 0, 0]).dot(
                tf.euler_matrix(np.radians(-40), np.radians(0), np.radians(-100))))
        self.vis["/Cameras/default/rotated/<object>"].set_transform(
            tf.translation_matrix([-2, 0, 2.5]))

        self.QuadPayloadRobot = QuadPayloadRobot
        # self._addQuadsPayload()
        self._setObstacles(env["environment"]["obstacles"])
        self.env = env
        self.__setGoal()
        self.__setStart()

    def __setGoal(self):
        self._addQuadsPayload("goal", "red")
        state = self.env["robots"][0]["goal"]
        self.QuadPayloadRobot.updateFullState(state)
        self.updateVis(state, "goal")

    def __setStart(self):
        self._addQuadsPayload("start", "green")
        state = self.env["robots"][0]["start"]
        self.QuadPayloadRobot.updateFullState(state)
        self.updateVis(state, "start")

    def draw_traces(self, result, quadNum, pType, l, desired):
        if desired: 
            d = "_d"
            # result = np.delete(result, [6,7,8], axis=1)
            c = 0xff0022
        else:
            d = ""
            c = 0xffffff
        # trace payload:
        payload = result[:, :3].T
        self.vis["trace_payload"+d].set_object(
            g.Line(g.PointsGeometry(payload), g.LineBasicMaterial(color=c)))

        if pType == "quad3dpayload":
            qc = result[:, 3:6].T
            quad_pos = payload - 0.5 * qc
            self.vis["trace_quad"+d].set_object(
                g.Line(
                    g.PointsGeometry(quad_pos),
                    g.LineBasicMaterial()))
        elif pType == "point":
            qcwcs = result[:, 6:6 + 6 * quadNum]
            for i in range(quadNum):
                qc = qcwcs[:, 6*i: 6*i+3].T
                quad_pos = payload - l[i] * qc

                self.vis["trace_quad" + str(i) + d].set_object(
                    g.Line(g.PointsGeometry(quad_pos), g.LineBasicMaterial(color=c)))

    def _addQuadsPayload(self, prefix: str = "", color_name: str = ""):
        self.quads = self.QuadPayloadRobot.quads
        self.payload = self.QuadPayloadRobot.payload
        if self.payload.shape == "quad3dpayload" or self.payload.shape == "point":
            self.vis[prefix + self.payload.shape].set_object(g.Mesh(
                g.Sphere(0.01), g.MeshLambertMaterial(DnametoColor.get(color_name, 0xff11dd))))
        elif self.payload.shape == "rigid":
            # THIS IS FOR MULTIPLE UAVS WITH RIGID PAYLOAD
            # different state order
            # NOT IMPLEMENTED
            print('NOT IMPLEMENTED, please use payload type as point')
            exit()
        else:
            print("Payload type doesn't exist")
            exit()

        for name in self.quads.keys():
            self.vis[prefix + name].set_object(g.StlMeshGeometry.from_file(
                Path(__file__).parent.parent / 'meshes/cf2_assembly.stl'), g.MeshLambertMaterial(color=DnametoColor.get(color_name, 0xffffff)))
            self.vis[prefix + "cable_" + name].set_object(g.Box([0.005,0.005,self.quads[name].l]), g.MeshLambertMaterial(color=0x000000))            
            self.vis[prefix + name + "_sphere"].set_object(
                g.Mesh(g.Sphere(0.1), g.MeshLambertMaterial(opacity=0.1)))  # safety distance

    def _setObstacles(self, obstacles):
        for idx, obstacle in enumerate(obstacles):
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

    def updateVis(self, state, prefix: str = "", frame=None):
        if frame is None:
            frame = self.vis
        self.QuadPayloadRobot.updateFullState(state)
        payloadSt = self.payload.state
        # color of the payload trajectory
        point_color = np.array([1.0, 1.0, 1.0])
        full_state = np.array(
            self.payload.state_ap,
            dtype=np.float64)[
            :,
            0:3].T
        self.vis[prefix + 'points'].set_object(g.Points(
            g.PointsGeometry(full_state, color=point_color),
            g.PointsMaterial(size=0.01)
        ))
        if self.payload.shape == "quad3dpayload" or self.payload.shape == "point":
            frame[prefix + self.payload.shape].set_transform(
                tf.translation_matrix(payloadSt).dot(
                    tf.quaternion_matrix([1, 0, 0, 0])))
        else:
            frame[prefix + self.payload.shape].set_transform(
                tf.translation_matrix(payloadSt[0:3]).dot(
                    tf.quaternion_matrix(payloadSt[3:7])))

        for i, (name, quad) in enumerate(self.quads.items()):
            frame[prefix + name].set_transform(
                tf.translation_matrix(quad.state[0:3]).dot(
                    tf.quaternion_matrix(quad.state[3:7])))
            qc = state[6+6*i: 6+6*i+3]
            cablePos  = payloadSt[0:3] - (quad.l)*np.array(qc)/2
            cableQuat = rn.vector_vector_rotation(qc, [0,0,-1])
            frame[prefix + "cable_" + name].set_transform(tf.translation_matrix(cablePos).dot(
                                                                tf.quaternion_matrix(cableQuat)))
            frame[prefix + name + \
                "_sphere"].set_transform(tf.translation_matrix(quad.state[0:3]))


def quad3dpayload_meshcatViewer():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--robot',
        type=str,
        help="robot model: quad3dpayload, (point, rigid: for n robots)")
    parser.add_argument('--env', type=str, help="environment")
    parser.add_argument('--ref', type=str, help="reference trajectory")
    parser.add_argument('--result', type=str, help="result trajectory")
    parser.add_argument('--output', type=str, help="result trajectory")
    parser.add_argument(
        "-i",
        "--interactive",
        action="store_true")  # on/off flag

    args = parser.parse_args()
    pathtoenv = args.env
    robotname = args.robot
    with open(pathtoenv, "r") as file:
        env = yaml.safe_load(file)

    pType = robotname
    quadNum = env["robots"][0]["quadsNum"]
    lengths = env["robots"][0]["l"]
    start = env["robots"][0]["start"]
    goal = env["robots"][0]["goal"]
    obstacles = env["environment"]["obstacles"]
    quadsPayload = QuadPayloadRobot(quadNum=quadNum, pType=pType)

    fig, axs = plt.subplots(7)
    # payload
    # cable1
    # cable2
    # robot1
    # robot2
    # control1
    # control2
    print(args.result)
    if args.result is not None:
        with open(args.result, 'r') as file:
            __path = yaml.safe_load(file)

        if "states" in __path and "actions" in __path:
            __path = __path
        else:
            __path = __path["result"]


        xs = [ x[:6] for x in __path["states"] ]

        labels_x = ["x" + str(i) for i in range(len(xs[0]))]

        for i, l in enumerate(labels_x):
            xi = [x[i] for x in xs]
            axs[0].plot(xi, label=l)
        axs[0].legend()

        xs = [ x[6:6+6] for x in __path["states"] ]

        labels_x = ["x" + str(i) for i in range(len(xs[0]))]

        for i, l in enumerate(labels_x):
            xi = [x[i] for x in xs]
            axs[1].plot(xi, label=l)
        axs[1].legend()


        xs = [ x[6+6:6+6+6] for x in __path["states"] ]

        labels_x = ["x" + str(i) for i in range(len(xs[0]))]

        for i, l in enumerate(labels_x):
            xi = [x[i] for x in xs]
            axs[2].plot(xi, label=l)
        axs[2].legend()

        xs = [ x[6+6+6:6+6+6+7] for x in __path["states"] ]

        labels_x = ["x" + str(i) for i in range(len(xs[0]))]

        for i, l in enumerate(labels_x):
            xi = [x[i] for x in xs]
            axs[3].plot(xi, label=l)
        axs[3].legend()


        xs = [ x[6+6+6+7:6+6+6+7+7] for x in __path["states"] ]

        labels_x = ["x" + str(i) for i in range(len(xs[0]))]

        for i, l in enumerate(labels_x):
            xi = [x[i] for x in xs]
            axs[4].plot(xi, label=l)
        axs[4].legend()

        us = [ u[:4] for u in __path["actions"] ]

        labels_u = ["x" + str(i) for i in range(len(us[0]))]

        for i, l in enumerate(labels_u):
            ui = [u[i] for u in us]
            axs[5].plot(ui, label=l)
        axs[5].legend()


        us = [ u[4:4+4] for u in __path["actions"] ]

        labels_u = ["x" + str(i) for i in range(len(us[0]))]

        for i, l in enumerate(labels_u):
            ui = [u[i] for u in us]
            axs[6].plot(ui, label=l)
        axs[6].legend()


        # for u in __path["actions"]:
        #     print(u)
    print("visualizing")
    visualizer = Visualizer(quadsPayload, env)

    
    # point1 = [-0.473381,0.0529316,0.186414]
    # point2 = [-0.487709,0.0579852,0.186402]
    # point1 =  [1.14314,0.118284,0.451639]
    # point2=  [1.03288,0.118285,0.451639]

    # points = np.array([point1, point2]).T

    # visualizer.vis["col1"].set_object(
    #     g.Line(g.PointsGeometry(points), g.LineBasicMaterial()))


    # name = input("press any key on terminal to close: ")
    # print("closing")



    if args.interactive:
        plt.show()
        visualizer.vis.open()

    pathtoresult = args.result

    if args.result is not None:

        with open(pathtoresult, 'r') as file:
            path = yaml.safe_load(file)

        if "states" in path:
            states = path['states']
        elif "result" in path:
            states = path['result']['states']
            actions = path['result']['actions']
            # print("shape of actions: ", np.array(actions).shape)
        else: 
            raise NotImplementedError("unknown result format")
        
        visualizer._addQuadsPayload()
        
        if args.ref is not None: 
            with open(args.ref, 'r') as file: 
                refpath = yaml.safe_load(file)
            if "states" in refpath:
                states_d = refpath["states"]
            elif "result" in refpath:
                states_d = refpath["result"]["states"]
            else: 
                raise NotImplementedError("unknown result format")

            desired = True
            visualizer.draw_traces(np.array(states_d), quadNum, pType, lengths, desired)
        desired = False
        visualizer.draw_traces(np.array(states), quadNum, pType, lengths, desired)
        # print("shape of states: ", np.array(states).shape)

        anim = Animation()
        for k, state in enumerate(states):
            with anim.at_frame(visualizer.vis, k) as frame:
                visualizer.updateVis(state, frame=frame)
        visualizer.vis.set_animation(anim)

        res = visualizer.vis.static_html()
        # save to a file
        # Path(args.output).mkdir(exist_ok=True)
        with open(args.output, "w") as f:
            f.write(res)
    # else: 


        # point1 =  [-0.000187059,-0.202631,0.31554]
        # point2 = [-2.62275e-05,-0.16202,0.252487]


if __name__ == "__main__":
    quad3dpayload_meshcatViewer()