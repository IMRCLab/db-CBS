import argparse
import math
import rowan
import numpy as np
import yaml
import pathlib
from typing import List
from string import Template

# example of input:
# environment:
#   min: [0, 0]
#   max: [3.5, 1.5]
#   obstacles:
#     - type: box
#       center: [0.7, 0.2]
#       size: [0.5, 0.25]
#     - type: box
#       center: [2.7, 0.2]
#       size: [0.5, 0.25]
# robots:
#   - type: car_first_order_with_1_trailers_0
#     start: [0.7,0.6,0,0] # x,y,theta0,theta1
#     goal: [1.9,0.2,0,0] # x,y,theta0,theta1


class RobotUnicycle:
    def __init__(self, start: List[float], goal: List[float], ID: int):
        self.start = start
        self.goal = goal
        self.size = [0.5, .25]
        self.ID = ID

    def to_g(self) -> str:
        start_pos = self.start[:2] + [.5]
        goal_pos = self.goal[:2] + [.5]
        size3 = self.size + [1]

        out0 = ("start{} {{X:<t({}) d({} 0 0 1)>, shape:ssBox, size:"
                "[{} 0.05],  color:[1 0 0 .5]}}\n").format(
            self.ID, ' '.join(map(str, start_pos)),
            180 / math.pi * self.start[2], ' '.join(map(str, size3)))

        out0f = "fstart{}(start{}) {{shape:marker, size:[.2]}}\n".format(
            self.ID, self.ID)

        light0 = ("light_start{}(start{}) {{Q:<t(.3 0 0)>, shape:ssBox,"
                  "size:[.1 .25 1  0.05], color:[1 1 0 0.5]}}\n").format(
            self.ID, self.ID)

        out1 = ("robot{}(world) {{joint:transXYPhi, q:[{}], shape:ssBox,"
                "size:[{}  0.05], color:[0 0 1 .5], contact}}\n").format(
            self.ID, ' '.join(map(str, self.start)), ' '.join(map(str, size3)))

        light1 = ("light_robot{}(robot{}) {{Q:<t(.3 0 0)> shape:ssBox,"
                  "size:[.1 .25 1 0.05], color:[1 1 0 0.5]}}\n").format(
            self.ID, self.ID)

        out1f = "frobot{}(robot{}) {{shape:marker, size:[.2]}}\n".format(
            self.ID, self.ID)

        out2 = ("goal{} {{X: <t({}) d({} 0 0 1)>, shape:ssBox, size:"
                "[{} 0.05],  color:[0 1 0 .5]}}\n").format(
            self.ID, ' '.join(map(str, goal_pos)), 180 /
            math.pi * self.goal[2],
            ' '.join(map(str, size3)))
        out2f = "fgoal{}(goal{}) {{shape:marker, size:[.2]}}\n".format(
            self.ID, self.ID)

        light2 = ("light_goal{}(goal{}) {{Q:<t(.3 0 0)> shape:ssBox,"
                  "size:[.1 .25 1 0.05], color:[1 1 0 0.5]}}\n").format(
            self.ID, self.ID)

        return (out0 + out0f + out1 + out1f + out2 + out2f + light0 + light1
                + light2)


class RobotTrailer:
    def __init__(self, start: List[float], goal: List[float], ID: int):
        assert len(start) == 4
        assert len(goal) == 4
        self.start = start
        self.goal = goal
        # NOTE: size of car and trailer is hardcoded in the .g file
        self.ID = ID
        p = pathlib.Path(__file__).parent / "../src/car_with_trailer_base.g"
        self.include = "Include: '{}'\n".format(str(p.resolve()))
        self.name_robot = "R_"
        self.name_goal = "GOAL_"
        self.car_start = start[0:3] 
        # Conversion of theta1 between Quim and Valle
        # Check Quim Drawing!
        self.trailer_start = math.degrees(math.pi/2 - start[3] + start[2])

        self.car_goal = goal[0:3]
        self.trailer_goal = math.degrees(math.pi/2 - goal[3] + goal[2])

    def to_g(self) -> str:

        # robot
        out = "Prefix: \"{}\"\n".format(self.name_robot)
        out += self.include
        out += "Edit {}robot {{ q:[{}]}}\n".format(self.name_robot,
                                           ' '.join(map(str, self.car_start)))
        out += "Edit {}front_wheel {{ q:[{}] }}\n".format(
            self.name_robot, 0)
        out += "Edit {}arm {{ q:[{}] }}\n".format(self.name_robot, self.trailer_start * math.pi / 180)

        # goal
        out += "Prefix: \"{}\"\n".format(self.name_goal)
        out += self.include

        out += "Edit {}robot {{ joint:rigid, Q:<t({} 0) d({} 0 0 1)>  }}\n".format(
                self.name_goal, ' '.join(map(str, self.car_goal[:2])), math.degrees(self.car_goal[2]))
        # there is no goal in the from wheel
        out += "Edit {}front_wheel {{ joint:rigid }}\n".format(self.name_goal)
        out += "Edit {}arm {{ joint:rigid, Q:<t(0 0 0) d({} 0 0 1)>}}\n".format(
            self.name_goal, self.trailer_goal)

        return out


class RobotQuadrotor:
    def __init__(self, start: List[float], goal: List[float], ID: int):
        assert len(start) == 13
        assert len(goal) == 13
        self.start = start
        self.goal = goal
        self.ID = ID
        # p = pathlib.Path(__file__).parent / "../src/car_with_trailer_base.g"
        # self.include = "Include: '{}'\n".format(str(p.resolve()))
        # self.name_robot = "R_"
        # self.name_goal = "GOAL_"

    def to_g(self) -> str:

        t = Template((
            "world {}\n"
            "\n"
            "drone (world) { joint:free X:<t($sx $sy $sz) d($sangle $saxisx $saxisy $saxisz)> }\n"
            # "drone (world) { joint:free X:<t($sx $sy $sz)> }\n"
            "\n"
            " (drone){Q:<d(45 0 0 1)> shape:ssBox size:[0.094 0.005 0.005 0.001] color:[.9 .9 .9] mass:0.015}\n"
            " (drone){Q:<d(-45 0 0 1)> shape:ssBox size:[0.094 0.005 0.005 0.001] color:[.9 .9 .9] mass:0.015}\n"
            "\n"
            "m1(drone){ Q:[ 0.0325 -0.0325 0.0] shape:cylinder size:[.02 .005] color:[.9 .9 .9] }\n"
            "m2(drone){ Q:[-0.0325 -0.0325 0.0] shape:cylinder size:[.02 .005] color:[.9 .9 .9] }\n"
            "m3(drone){ Q:[-0.0325  0.0325 0.0] shape:cylinder size:[.02 .005] color:[.9 .9 .9] }\n"
            "m4(drone){ Q:[ 0.0325  0.0325 0.0] shape:cylinder size:[.02 .005] color:[.9 .9 .9] }\n"
            "\n"
            # "target (world) { shape:marker size:[.03] color:[.9 .9 .9 .5] X:[$gx $gy $gz] }\n"
            "start (world) { shape:marker size:[.03] color:[.9 .9 .9 .5] X:<t($sx $sy $sz) d($sangle $saxisx $saxisy $saxisz)> }\n"
            "target (world) { shape:marker size:[.03] color:[.9 .9 .9 .5] X:<t($gx $gy $gz) d($gangle $gaxisx $gaxisy $gaxisz)> }\n"
        ))
        # states are: x,y,z, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz
        start_q = np.array([self.start[6], self.start[3], self.start[4], self.start[5]])
        start_q = rowan.normalize(start_q)
        saxis, sangle  = rowan.to_axis_angle(start_q)

        goal_q = np.array([self.goal[6], self.goal[3], self.goal[4], self.goal[5]])
        goal_q = rowan.normalize(goal_q)
        gaxis, gangle  = rowan.to_axis_angle(goal_q)

        result = t.substitute(
            sx=self.start[0], sy=self.start[1], sz=self.start[2],
            sangle=math.degrees(sangle[0]), saxisx=saxis[0,0], saxisy=saxis[0,1], saxisz=saxis[0,2],
            gx=self.goal[0], gy=self.goal[1], gz=self.goal[2],
            gangle=math.degrees(gangle[0]), gaxisx=gaxis[0,0], gaxisy=gaxis[0,1], gaxisz=gaxis[0,2])
        # print(result)
        return result


class Box:
    def __init__(self, center: List[float], size: List[float], ID: int):
        self.size = size
        self.center = center
        self.color = [.2, .2, .2]
        self.ID = ID

    def to_g(self) -> str:
        size3 = self.size + [1]
        center3 = self.center + [0.5]
        out = ("obs{} {{X: <t({})>, shape:ssBox, size:[{}  0.05],"
               "color:[{}], contact}}\n").format(
            self.ID,
            ' '.join(map(str, center3)),
            ' '.join(map(str, [v+0.05 for v in size3])), # increase the size to "correct" for rounded corners
            ' '.join(map(str, self.color)))

        return out


def write(file_in: str, file_out: str) -> None:

    with open(file_in) as env_file:
        env = yaml.safe_load(env_file)

    boxes: List[Box] = []
    robots: List[Robot] = []
    i = 0
    for obstacle in env["environment"]["obstacles"]:
        if obstacle["type"] == "box":
            boxes.append(Box(obstacle["center"], obstacle["size"], i))
            i += 1
    ID_robot = 0
    assert len(env["robots"]) == 1
    for robot in env["robots"]:
        if "unicycle" in robot["type"]:
            robots.append(RobotUnicycle(robot["start"][0:3], robot["goal"][0:3], ID_robot))
            offset_z = 0.5
        elif "trailer" in robot["type"]:
            robots.append(RobotTrailer(robot["start"], robot["goal"], ID_robot))
            offset_z = 0.5
        elif "quadrotor" in robot["type"]:
            robots.append(RobotQuadrotor(robot["start"], robot["goal"], ID_robot))
            offset_z = 0
        else:
            raise Exception("Unknown robot_type! {}".format(robot["type"]))
        ID_robot += 1

    with open(file_out, "w") as g_file:
        g_file.write("world{ X:<t(0 0 " + str(offset_z) + ")>}\n")
        for i in boxes:
            g_file.write(i.to_g())
        for i in robots:
            g_file.write(i.to_g())


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--fin", required=True, help="yaml file")
    parser.add_argument("--fout", required=True, help="g file")
    args = parser.parse_args()

    write(args.fin, args.fout)


if __name__ == "__main__":
    main()
