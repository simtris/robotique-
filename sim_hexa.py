#!/usr/bin/env python
import math
import sys
import os
import time
import argparse
import pybullet as p
from onshape_to_robot.simulation import Simulation
import numpy as np
import robot_moves

import kinematics
from constants import *

# from squaternion import Quaternion
from scipy.spatial.transform import Rotation


class Parameters:
    def __init__(
        self,
        z=-0.06,
    ):
        self.z = z

        self.coords = []
        
        self.coords = []  # INIT_LEG_POSITIONS
        self.coords.append([0.170, 0, 0])
        self.coords.append([0.170, 0, 0])
        self.coords.append([0.170, 0, 0])
        self.coords.append([0.170, 0, 0])
        self.coords.append([0.170, 0, 0])
        self.coords.append([0.170, 0, 0])
        
        
        # Angle between the X axis of the leg and the X axis of the robot for each leg
        self.legAngles = LEG_ANGLES
        # Initial leg positions in the coordinates of each leg.
        self.initLeg = []  # INIT_LEG_POSITIONS
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])

        # Motor re-united by joint name for the simulation
        self.legs = {}
        self.legs[1] = ["j_c1_rf", "j_thigh_rf", "j_tibia_rf"]
        self.legs[6] = ["j_c1_rm", "j_thigh_rm", "j_tibia_rm"]
        self.legs[5] = ["j_c1_rr", "j_thigh_rr", "j_tibia_rr"]
        self.legs[2] = ["j_c1_lf", "j_thigh_lf", "j_tibia_lf"]
        self.legs[3] = ["j_c1_lm", "j_thigh_lm", "j_tibia_lm"]
        self.legs[4] = ["j_c1_lr", "j_thigh_lr", "j_tibia_lr"]


def to_pybullet_quaternion(roll, pitch, yaw, degrees=False):
    # q = Quaternion.from_euler(roll, pitch, yaw, degrees=degrees)
    # return [q[1], q[2], q[3], q[0]]

    # Create a rotation object from Euler angles specifying axes of rotation
    rot = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=degrees)

    # Convert to quaternions and print
    rot_quat = rot.as_quat()
    # print(rot_quat)
    return rot_quat


# Updates the values of the dictionnary targets to set 3 angles to a given leg
def set_leg_angles(alphas, leg_id, targets, params):
    leg = params.legs[leg_id]
    i = -1
    for name in leg:
        i += 1
        targets[name] = alphas[i]


# m_friction
parser = argparse.ArgumentParser()
parser.add_argument("--mode", "-m", type=str, default="direct", help="test")
args = parser.parse_args()
controls = {}
robotPath = "phantomx_description/urdf/phantomx.urdf"
sim = Simulation(robotPath, gui=True, panels=True, useUrdfInertia=False)
# sim.setFloorFrictions(lateral=0, spinning=0, rolling=0)
pos, rpy = sim.getRobotPose()
sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

leg_center_pos = [0.1248, -0.06164, 0.001116 + 0.5]
leg_angle = -math.pi / 4
params = Parameters()


def inverseUpdate(controls):
    x = p.readUserDebugParameter(controls[0])
    y = p.readUserDebugParameter(controls[1])
    z = p.readUserDebugParameter(controls[2])
    p.resetBasePositionAndOrientation(
        controls[3], [x, y, z+0.1], p.getQuaternionFromEuler([0, 0, 0]))
    return x, y, z

if args.mode == "frozen-direct":
    crosses = []
    for i in range(4):
        crosses.append(p.loadURDF("target2/robot.urdf"))
    for name in sim.getJoints():
        print(name)
        
        if "c1" in name or "thigh" in name or "tibia" in name:
            controls[name] = p.addUserDebugParameter(name, -math.pi, math.pi, 0)
elif args.mode == "direct":
    for name in sim.getJoints():
        print(name)
        if "c1" in name or "thigh" in name or "tibia" in name:
            controls[name] = p.addUserDebugParameter(name, -math.pi, math.pi, 0)
elif args.mode == "inverse":
    cross = p.loadURDF("target2/robot.urdf")

    print("\n cross :", cross, "\n") # -> 2
    # Use your own DK function
    alphas = kinematics.computeDK(0, 0, 0, use_rads=True)
    controls["target_x"] = p.addUserDebugParameter("target_x", -0.4, 0.4, alphas[0])
    controls["target_y"] = p.addUserDebugParameter("target_y", -0.4, 0.4, alphas[1])
    controls["target_z"] = p.addUserDebugParameter("target_z", -0.4, 0.4, alphas[2])
    

elif args.mode == "walk":
    controls["teta"] = p.addUserDebugParameter("orientation", -math.pi, math.pi, 0)
    controls["freq"] = p.addUserDebugParameter("speed", 0, 5, 1)


elif args.mode == "walk-advanced":
    controls["teta"] = p.addUserDebugParameter("orientation", -math.pi, math.pi, 0)
    controls["freq"] = p.addUserDebugParameter("speed", 0, 5, 1)
    controls["dist"] = p.addUserDebugParameter("dist", 0.04, 0.08, 0.01)
    controls["hauteur"] = p.addUserDebugParameter("hauteur", 0.0001, 0.065, 0.01)
    
    
elif args.mode == "holonomic":
    controls["teta"] = p.addUserDebugParameter("orientation", -math.pi, math.pi, 0)
    controls["freq"] = p.addUserDebugParameter("speed", 0, 5, 1)
    controls["dist_x"] = p.addUserDebugParameter("dist_x", 0.000000001, 0.2, 0.02)
    controls["dist_y"] = p.addUserDebugParameter("dist_y", 0.0, 0.15, 0.01)
    

        
elif args.mode == "center-follow":
    alphas = kinematics.computeDK(0, 0, 0, use_rads=True)
    controls["target_x"] = p.addUserDebugParameter("target_x", -0.2, 0.2, alphas[0])
    controls["target_y"] = p.addUserDebugParameter("target_y", -0.15, 0.15, alphas[1])
    controls["target_z"] = p.addUserDebugParameter("target_z", -0.15, 0.09, alphas[2])

elif args.mode == "control-legs":
    alphas = kinematics.computeDK(0, 0, 0, use_rads=True)
    controls["target_x1"] = p.addUserDebugParameter("target_x1", -0.4, 0.4, alphas[0])
    controls["target_y1"] = p.addUserDebugParameter("target_y1", -0.4, 0.4, alphas[1])
    controls["target_z1"] = p.addUserDebugParameter("target_z1", -0.4, 0.4, alphas[2])
    controls["target_x2"] = p.addUserDebugParameter("target_x2", -0.4, 0.4, alphas[0])
    controls["target_y2"] = p.addUserDebugParameter("target_y2", -0.4, 0.4, alphas[1])
    controls["target_z2"] = p.addUserDebugParameter("target_z2", -0.4, 0.4, alphas[2])
    controls["target_x3"] = p.addUserDebugParameter("target_x3", -0.4, 0.4, alphas[0])
    controls["target_y3"] = p.addUserDebugParameter("target_y3", -0.4, 0.4, alphas[1])
    controls["target_z3"] = p.addUserDebugParameter("target_z3", -0.4, 0.4, alphas[2])
    controls["target_x4"] = p.addUserDebugParameter("target_x4", -0.4, 0.4, alphas[0])
    controls["target_y4"] = p.addUserDebugParameter("target_y4", -0.4, 0.4, alphas[1])
    controls["target_z4"] = p.addUserDebugParameter("target_z4", -0.4, 0.4, alphas[2])
    controls["target_x5"] = p.addUserDebugParameter("target_x5", -0.4, 0.4, alphas[0])
    controls["target_y5"] = p.addUserDebugParameter("target_y5", -0.4, 0.4, alphas[1])
    controls["target_z5"] = p.addUserDebugParameter("target_z5", -0.4, 0.4, alphas[2])
    controls["target_x6"] = p.addUserDebugParameter("target_x6", -0.4, 0.4, alphas[0])
    controls["target_y6"] = p.addUserDebugParameter("target_y6", -0.4, 0.4, alphas[1])
    controls["target_z6"] = p.addUserDebugParameter("target_z6", -0.4, 0.4, alphas[2])
    
elif args.mode == "static_rotation":
    controls["theta"] = p.addUserDebugParameter("theta", math.pi/200, math.pi/5, 50)
    controls["rayon"] = p.addUserDebugParameter("rayon", 0.25, 0.326, 0.05)
    
    
elif args.mode == "dynamic-rotation":
    controls["freq"] = p.addUserDebugParameter("speed", 0, 5, 1)
    controls["hauteur"] = p.addUserDebugParameter("hauteur", 0.01, 0.1, 0.02)
    controls["dist_x"] = p.addUserDebugParameter("dist_x", 0.15, 0.2, 0.02)
    controls["dist_y"] = p.addUserDebugParameter("dist_y", 0.001, 0.12, 0.01)
    
    

while True:
    targets = {}
    for name in sim.getJoints():
        if "c1" in name or "thigh" in name or "tibia" in name:
            targets[name] = 0
    if args.mode == "frozen-direct":
        for name in controls.keys():
            targets[name] = p.readUserDebugParameter(controls[name])
        # Use your own DK function, the result should be: https://www.youtube.com/watch?v=w3psAbh3AoM
        points = kinematics.computeDKDetailed(
            targets["j_c1_rf"],
            targets["j_thigh_rf"],
            targets["j_tibia_rf"],
            use_rads=True,
        )
        
        # points = LEG_CENTER_POS # --> met une croix sur les 6 épaules (mettre leg_angle à 0)
        i = -1
        T = []
        for pt in points:
            # Drawing each step of the DK calculation
            i += 1
            T.append(kinematics.rotaton_2D(pt[0], pt[1] , pt[2], leg_angle))
            T[-1][0] += leg_center_pos[0] # Ajout l'offset de l'épaule
            T[-1][1] += leg_center_pos[1]
            T[-1][2] += leg_center_pos[2]
            #print("Drawing cross {} at {}".format(i, T[-1]))
            p.resetBasePositionAndOrientation(
                crosses[i], T[-1], to_pybullet_quaternion(0, 0, leg_angle)
            )
        sim.setRobotPose(
            [0, 0, 0.5],
            to_pybullet_quaternion(0, 0, 0),
        )
        state = sim.setJoints(targets)

    elif args.mode == "direct":

        for name in controls.keys():
            targets[name] = p.readUserDebugParameter(controls[name])
            print("name:", name, ", valeur:", targets[name])
        state = sim.setJoints(targets)
    
    elif args.mode == "inverse":
        x = p.readUserDebugParameter(controls["target_x"])
        y = p.readUserDebugParameter(controls["target_y"])
        z = p.readUserDebugParameter(controls["target_z"])
        # Use your own IK function
        alphas = kinematics.computeIK(x, y, z, verbose=True, use_rads=True)

        print("alphas :", alphas)
        
        targets["j_c1_rf"] = alphas[0]
        targets["j_thigh_rf"] = alphas[1]
        targets["j_tibia_rf"] = alphas[2]

        state = sim.setJoints(targets)

        #fixé dans une position haute
        sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

        T = kinematics.rotaton_2D(x, y, z, leg_angle)
        T[0] += leg_center_pos[0]
        T[1] += leg_center_pos[1]
        T[2] += leg_center_pos[2]
        # print("Drawing cross {} at {}".format(i, T))
        p.resetBasePositionAndOrientation(
            cross, T, to_pybullet_quaternion(0, 0, leg_angle)
        )
    elif args.mode == "robot-ik":
        #None
        #Use your own IK function
        for leg_id in range(1, 7):
            alphas = kinematics.computeIKOriented(
                0.01 * math.sin(2 * math.pi * 0.5 * time.time()),
                0.02 * math.cos(2 * math.pi * 0.5 * time.time()),
                0.03 * math.sin(2 * math.pi * 0.2 * time.time()),
                leg_id,
                params,
                0,
                verbose=True
            )
            set_leg_angles(alphas, leg_id, targets, params)
        state = sim.setJoints(targets)


    elif args.mode == "walk":
        None
        # Use your own IK function
        #sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])
        t = time.time()
        teta = p.readUserDebugParameter(controls["teta"])
        freq = p.readUserDebugParameter(controls["freq"])

        first_step, next_step = kinematics.walk(t, freq, params, targets, teta)

        for leg_id in [1,3,5]:
            alphas = kinematics.computeIKOriented(first_step[0], first_step[1], first_step[2], leg_id, params, teta, verbose=True)
            set_leg_angles(alphas, leg_id, targets, params)

        for leg_id in [2,4,6]:
            alphas = kinematics.computeIKOriented(next_step[0], next_step[1], next_step[2], leg_id, params, teta, verbose=True)
            set_leg_angles(alphas, leg_id, targets, params)
        
        
        
        state = sim.setJoints(targets)


    elif args.mode == "walk-advanced":
        None
        # Use your own IK function
        #sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])
        t = time.time()
        teta = p.readUserDebugParameter(controls["teta"])
        freq = p.readUserDebugParameter(controls["freq"])
        hauteur = p.readUserDebugParameter(controls["hauteur"])
        dist = p.readUserDebugParameter(controls["dist"])

        
        first_step, next_step = kinematics.walk_advanced(t, freq, dist, hauteur, params, targets, teta)

        for leg_id in [1,3,5]:
            alphas = kinematics.computeIKOriented(first_step[0], first_step[1], first_step[2], leg_id, params, teta, verbose=True)
            set_leg_angles(alphas, leg_id, targets, params)

        for leg_id in [2,4,6]:
            alphas = kinematics.computeIKOriented(next_step[0], next_step[1], next_step[2], leg_id, params, teta, verbose=True)
            set_leg_angles(alphas, leg_id, targets, params)
    
        state = sim.setJoints(targets)

        
    elif args.mode == "holonomic":
        None
        # Use your own IK function
        #sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])
        t = time.time()
        teta = p.readUserDebugParameter(controls["teta"])
        freq = p.readUserDebugParameter(controls["freq"])
        dist_x = p.readUserDebugParameter(controls["dist_x"])
        dist_y = p.readUserDebugParameter(controls["dist_y"])    
        
        
        first_step, next_step = kinematics.walk_advanced(t, freq, dist_x, 0.05, params, targets, teta)
        first_step2, next_step2 = kinematics.dynamic_rotation(t, freq, dist_x, dist_y, 0.001, params, targets)

        for leg_id in [1,3,5]:
            alphas = kinematics.computeIKOriented(first_step[0], first_step[1], first_step[2], leg_id, params, teta, verbose=True)

            alphas2 = kinematics.computeIK(first_step2[0], first_step2[1], first_step2[2])

            set_leg_angles(np.array(alphas) + np.array(alphas2), leg_id, targets, params)

        for leg_id in [2,4,6]:
            alphas = kinematics.computeIKOriented(next_step[0], next_step[1], next_step[2], leg_id, params, teta, verbose=True)

            alphas2 = kinematics.computeIK(next_step2[0], next_step2[1], next_step2[2])
            
            set_leg_angles(alphas + alphas2, leg_id, targets, params)
        
        
        
        state = sim.setJoints(targets)    

    elif args.mode == "dynamic-rotation":
        None
        t = time.time()
        freq = p.readUserDebugParameter(controls["freq"])
        hauteur = p.readUserDebugParameter(controls["hauteur"])
        dist_x = p.readUserDebugParameter(controls["dist_x"])
        dist_y = p.readUserDebugParameter(controls["dist_y"])
        

        first_step, next_step = kinematics.dynamic_rotation(t, freq, dist_x, dist_y, hauteur, params, targets)

        for leg_id in [1,3,5]:
            alphas = kinematics.computeIK(first_step[0], first_step[1], first_step[2])

            
            
            set_leg_angles(alphas, leg_id, targets, params)

        for leg_id in [2,4,6]:
            alphas = kinematics.computeIK(next_step[0], next_step[1], next_step[2])
            set_leg_angles(alphas, leg_id, targets, params)
        

        state = sim.setJoints(targets)    

    elif args.mode == "center-follow":
        x = p.readUserDebugParameter(controls["target_x"])
        y = p.readUserDebugParameter(controls["target_y"])
        z = p.readUserDebugParameter(controls["target_z"]) 
        for leg_id in range(1, 7):
            alphas = kinematics.computeIKOriented(
                x,
                y,
                z,
                leg_id,
                params,
                0,
                verbose=False,
            )
            set_leg_angles(alphas, leg_id, targets, params)
        
        state = sim.setJoints(targets)

    elif args.mode == "control-legs":
        #1
        x = p.readUserDebugParameter(controls["target_x1"])
        y = p.readUserDebugParameter(controls["target_y1"])
        z = p.readUserDebugParameter(controls["target_z1"])
        alphas = kinematics.computeIK(x, y, z, verbose=True, use_rads=True)

        targets["j_c1_rf"] = alphas[0]
        targets["j_thigh_rf"] = alphas[1]
        targets["j_tibia_rf"] = alphas[2]
        

        
        #2
        x = p.readUserDebugParameter(controls["target_x2"])
        y = p.readUserDebugParameter(controls["target_y2"])
        z = p.readUserDebugParameter(controls["target_z2"])
        alphas = kinematics.computeIK(x, y, z, verbose=True, use_rads=True)

        targets["j_c1_lf"] = alphas[0]
        targets["j_thigh_lf"] = alphas[1]
        targets["j_tibia_lf"] = alphas[2]
        

        
        #3
        x = p.readUserDebugParameter(controls["target_x3"])
        y = p.readUserDebugParameter(controls["target_y3"])
        z = p.readUserDebugParameter(controls["target_z3"])
        alphas = kinematics.computeIK(x, y, z, verbose=True, use_rads=True)

        
        targets["j_c1_lm"] = alphas[0]
        targets["j_thigh_lm"] = alphas[1]
        targets["j_tibia_lm"] = alphas[2]
        

        #4
        x = p.readUserDebugParameter(controls["target_x4"])
        y = p.readUserDebugParameter(controls["target_y4"])
        z = p.readUserDebugParameter(controls["target_z4"])
        alphas = kinematics.computeIK(x, y, z, verbose=True, use_rads=True)

        
        targets["j_c1_lr"] = alphas[0]
        targets["j_thigh_lr"] = alphas[1]
        targets["j_tibia_lr"] = alphas[2]
        

        #5
        x = p.readUserDebugParameter(controls["target_x5"])
        y = p.readUserDebugParameter(controls["target_y5"])
        z = p.readUserDebugParameter(controls["target_z5"])
        alphas = kinematics.computeIK(x, y, z, verbose=True, use_rads=True)

        
        targets["j_c1_rr"] = alphas[0]
        targets["j_thigh_rr"] = alphas[1]
        targets["j_tibia_rr"] = alphas[2]
        

        #6
        x = p.readUserDebugParameter(controls["target_x6"])
        y = p.readUserDebugParameter(controls["target_y6"])
        z = p.readUserDebugParameter(controls["target_z6"])
        alphas = kinematics.computeIK(x, y, z, verbose=True, use_rads=True)
        
        targets["j_c1_rm"] = alphas[0]
        targets["j_thigh_rm"] = alphas[1]
        targets["j_tibia_rm"] = alphas[2]


        state = sim.setJoints(targets)

        #fixé dans une position haute
        sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])


    elif args.mode == "static_rotation" :
        h = -0.1
        theta = p.readUserDebugParameter(controls["theta"])

        rayon = p.readUserDebugParameter(controls["rayon"])
        for leg_id in range (1,7):
            
            angle = theta - LEG_ANGLES[leg_id - 1]
            
            px = rayon * math.cos(angle)
            py = rayon * math.sin(angle)
            
            alphas = kinematics.centerTarget(px, py, h, leg_id)
            
            set_leg_angles(alphas, leg_id, targets, params)

        state = sim.setJoints(targets)

        

    sim.tick()


