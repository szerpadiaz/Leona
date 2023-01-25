#!/usr/bin/env python
## Example to setup the frameworks for the canvas

from pickle import TRUE
import rospy
import os
import numpy as np

import almath
from naoqi import ALProxy
import math
import motion

from inv_kinematic import *

from leonao.srv import Nao_RArm_chain_get_angles, Nao_RArm_chain_get_transform

# Naming convention:
# - BASE-FRAME    : the frame attached to the NAO's Torso (in our case is fixed, the robot is not moving)
# - WRIST-FRAME   : the frame attached to the NAO's RArm end-effector
# - TOOL-FRAME    : the frame attached to the tip of the marker
# - STATION-FRAME : the frame attached to the Table where Leonao is painting (this is our workstation universal reference)
# - GOAL-FRAME    : the frame indicating where the tip of the marker should move to

TIP_POSITION_WITH_RESPECT_TO_W = [0.08,0.0,0.035]
BASE_FRAME_ID = motion.FRAME_TORSO

class Canvas():
    def __init__(self):
        robot_ip=str(os.getenv("NAO_IP"))
        robot_port=int(9559)
        self.motion_proxy = ALProxy("ALMotion", robot_ip, robot_port)

        # T_bs: Transformation of the STATION-FRAME (s) with respect to the BASE-FRAME (b)
        self.T_bs, plane_point_1, plane_point_2, plane_point_3 = self.get_configuration()
        self.calculate_drawing_plane(plane_point_1, plane_point_2, plane_point_3)

        # max distance between two consecutive points
        self.max_distance = 0.01

        self.enable_arm_stiffness()
        rospy.sleep(2.0)

    def get_configuration(self):
        # Read initial position
        raw_input("Move to pen to initial position. Press enter to start.")
        T_bs = self.get_transform_bs()
        initial_position = self.motion_proxy.getPosition("RArm", motion.FRAME_TORSO, True)
        print("initial_position", initial_position)

        # Read 1st point
        raw_input("Move to 1st point of the plane. Press enter if done.")
        plane_point_1 = self.get_position()
        # plane_point_1 = ?
        print("1st point read. ", plane_point_1)

        # Read 2nd point
        raw_input("Move to 2nd point of the plane. Press enter if done.")
        plane_point_2 = self.get_position()
        # plane_point_2 = ?
        print("2nd point read. ", plane_point_2)

        # Read 3rd point
        raw_input("Move to 3rd point of the plane. Press enter if done.")
        plane_point_3 = self.get_position()
        # plane_point_3 = ?
        print("3rd point read. ", plane_point_3)

        return T_bs, plane_point_1, plane_point_2, plane_point_3

    def calculate_drawing_plane(self, plane_point_1, plane_point_2, plane_point_3):
        plane_point_1 = np.array(plane_point_1)
        plane_point_2 = np.array(plane_point_2)
        plane_point_3 = np.array(plane_point_3)

        # Compute the vectors from point 1 to point 2 and point 3
        vector_12 = np.array(plane_point_2 - plane_point_1)
        vector_13 = np.array(plane_point_3 - plane_point_1)
        print("vector_12: ", vector_12)
        print("vector_13", vector_13)
        # Compute the plane normal from with the cross product
        self.plane_normal = np.cross(vector_12, vector_13)
        print("self.plane_normal: ", self.plane_normal)
        # Solve for one point to get offset of the equation
        self.plane_offset = -plane_point_1[0]*self.plane_normal[0] - plane_point_1[1]*self.plane_normal[1] - plane_point_1[2]*self.plane_normal[2]
        print("self.plane_offset: ", self.plane_offset)

    def get_x_in_drawing_plane(self, y, z):
        x = (-self.plane_normal[1]*y - self.plane_normal[2]*z - self.plane_offset) / self.plane_normal[0]
        return x

    def calculate_angles(self, y, z):
        x = self.get_x_in_drawing_plane(y, z)
        self.calculate_angles(self, x, y, z)

    def calculate_angles(self, x, y, z):
        # T_sg: Transformation of the GOAL-FRAME (g) with respect to the STATION-FRAME (s)
        T_sg = almath.Transform(x,y,z)

        # T_bg: Transformation of the GOAL-FRAME (g) with respect to the BASE-FRAME (b)
        T_bg = self.T_bs * T_sg
        position6D = [T_bg.r1_c4, T_bg.r2_c4, T_bg.r3_c4, 0, 0, 0]

        # get angles (using service)
        rospy.wait_for_service('Nao_RArm_chain_get_angles')
        try:
            service_get_angles = rospy.ServiceProxy('Nao_RArm_chain_get_angles', Nao_RArm_chain_get_angles)
            resp = service_get_angles(position6D)
            return resp.angles
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return []

    def get_position(self):
        T_temp = self.get_transform_bs()
        point = [T_temp.r1_c4, T_temp.r2_c4, T_temp.r3_c4]
        T_bg = almath.Transform().fromPosition(point[0], point[1], point[2])
        T_sg = self.T_bs.inverse() * T_bg
        return [T_sg.r1_c4, T_sg.r2_c4, T_sg.r3_c4]

    def get_transform_bs(self):
        joints_names = self.motion_proxy.getBodyNames("RArm")
        joints_angles = self.motion_proxy.getAngles(joints_names, True)
        rospy.wait_for_service('Nao_RArm_chain_get_transform')
        try:
            service_get_transform = rospy.ServiceProxy('Nao_RArm_chain_get_transform', Nao_RArm_chain_get_transform)
            resp = service_get_transform(joints_angles)
            T = almath.Transform(resp.transform)   
            return T
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return []

    def move_joints(self, joints_angles_list):
        joint_names = self.motion_proxy.getBodyNames("RArm")
        for target_angles in joints_angles_list:
            self.motion_proxy.angleInterpolationWithSpeed(joint_names[:-1], target_angles, 0.25)
            #rospy.sleep(0.5)

    def disable_arm_stiffness(self):
        self.motion_proxy.stiffnessInterpolation('RArm', 0.0, 1.0)

    def enable_arm_stiffness(self):
        self.motion_proxy.stiffnessInterpolation('RArm', 1.0, 1.0)

    def go_to_point(self, point):
        # x in the moving-plane (parallel to the drawing plane)
        y = point[0]
        z = point[1]

        # go to a position in the drawing plane without drawing anything
        # we should move along a plane parallell to the drawing plane
        pass

    def draw_line(self, start_point, end_point):
        y0 = start_point[0]
        z0 = start_point[1]
        y = end_point[0]
        z = end_point[1]

        joints_angles_list = []
        length = math.sqrt((x-x0)**2 + (y-y0)**2 + (z-z0)**2)
        for i in range(1, int(length) + 1):
             yi = (y - y0) /length * i + y0
             zi = (z - z0) /length * i + z0
             angles_i = self.calculate_angles(yi, zi)
             if(angles_i):
                joints_angles_list.append(angles_i)

        angles = self.calculate_angles(y, z)
        if(angles):
            joints_angles_list.append(angles)
        
        # Move by setting a list of angles.
        # self.move_joints(joints_angles_list)

    def draw_ellipse(self, a, b):
        ellipse_path = self.calculate_ellipse_path(a, b)
        joints_angles_list = []
        for point in ellipse_path:
            angles_i = self.calculate_angles(point[0], point[1])
            if(angles_i):
                joints_angles_list.append(angles_i)

        #print(joints_angles_list)

        # Move by setting a list of angles.
        self.move_joints(joints_angles_list)

    def calculate_ellipse_path(self, a, b):
        path = []
        i = 0
        final_i = 0
        while i <= 360:
            z = a * math.cos(math.radians(i))
            y = b * math.sin(math.radians(i))
            path.append([y, z])
            final_i = i
            i = i + math.degrees(math.acos(1- (self.max_distance**2)/(2*(a**2)) ))
        if final_i < 360:
            final_i = 360 
            z = a * math.cos(math.radians(final_i))
            y = b * math.sin(math.radians(final_i))
            path.append([y, z])
        return path

    def draw_bezier_curve(self, control_point_1, control_point_2):
        # at current point
        pass

    def draw_rectangle(self, l, w):
        # at current point
        pass

    def draw_point(self):
        # at current point
        pass