#!/usr/bin/env python
## Example to setup the frameworks for the canvas

from locale import normalize
from pickle import TRUE
from re import T
import rospy
import os
import numpy as np

import almath
from naoqi import ALProxy
import math
import motion
import tf

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
        plane_point_1, plane_point_2, plane_point_3 = self.get_configuration()
        self.calculate_drawing_plane(plane_point_1, plane_point_2, plane_point_3)

        self.enable_arm_stiffness()
        rospy.sleep(2.0)

        # Get current point and move to new origin?
        raw_input("To go to origin, press enter.")
        self.go_to_point([0, 0])



    def get_configuration(self):
        # Read 1st point/origin
        raw_input("Move to 1st point of the plane (origin). Press enter if done.")
        plane_point_1 = self.get_position_bt()
        # plane_point_1 = ?
        print("1st point read. ", plane_point_1)

        # Read 2nd point
        raw_input("Move to 2nd point of the plane. Press enter if done.")
        plane_point_2 = self.get_position_bt()
        # plane_point_2 = ?
        print("2nd point read. ", plane_point_2)

        # Read 3rd point
        raw_input("Move to 3rd point of the plane. Press enter if done.")
        plane_point_3 = self.get_position_bt()
        # plane_point_3 = ?
        print("3rd point read. ", plane_point_3)

        return plane_point_1, plane_point_2, plane_point_3

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
        self.plane_normal = self.plane_normal / np.linalg.norm(self.plane_normal)
        print("self.plane_normal: ", self.plane_normal)
        # Solve for one point to get offset of the equation
        # self.plane_offset = -plane_point_1[0]*self.plane_normal[0] - plane_point_1[1]*self.plane_normal[1] - plane_point_1[2]*self.plane_normal[2]
        # print("self.plane_offset: ", self.plane_offset)
        v1 = vector_12 / np.linalg.norm(vector_12) #- np.dot(vector_12, self.plane_normal) * self.plane_normal
        v2 = np.cross(v1, self.plane_normal)
        v2 = v2 / np.linalg.norm(v2)
        print("normal: ", self.plane_normal)
        print("v2: ", v2)
        print("v1: ", v1)

        R = np.column_stack((self.plane_normal, v2, v1))
        print("R: ", R)
        print("plane_point_1", plane_point_1)

        #euler_angles = np.array(np.rad2deg(np.array(np.matrix(R).flat)), dtype=np.float32)
        euler_angles = tf.transformations.euler_from_matrix(R, 'rxyz')
        print("euler_angles", np.degrees(euler_angles))

        T_bs = almath.Transform()
        T_bs.r1_c1 = R[0,0]
        T_bs.r1_c2 = R[0,1]
        T_bs.r1_c3 = R[0,2]
        T_bs.r1_c4 = plane_point_1[0]
        T_bs.r2_c1 = R[1,0]
        T_bs.r2_c2 = R[1,1]
        T_bs.r2_c3 = R[1,2]
        T_bs.r2_c4 = plane_point_1[1]
        T_bs.r3_c1 = R[2,0]
        T_bs.r3_c2 = R[2,1]
        T_bs.r3_c3 = R[2,2]
        T_bs.r3_c4 = plane_point_1[2]
        self.T_bs = T_bs


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

    def get_transform_bt(self):
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

    def get_position_bt(self):
        T_temp = self.get_transform_bt()
        point = [T_temp.r1_c4, T_temp.r2_c4, T_temp.r3_c4]
        return point

    def move_joints(self, joints_angles_list):
        joint_names = self.motion_proxy.getBodyNames("RArm")
        for target_angles in joints_angles_list:
            self.motion_proxy.angleInterpolationWithSpeed(joint_names[:-1], target_angles, 0.25)
            #rospy.sleep(0.5)

    def disable_arm_stiffness(self):
        self.motion_proxy.stiffnessInterpolation('RArm', 0.0, 1.0)

    def enable_arm_stiffness(self):
        self.motion_proxy.stiffnessInterpolation('RArm', 1.0, 1.0)

    def go_to_point(self, end_point):
        T_bt = self.get_transform_bt()
        T_sb = self.T_bs.inverse()
        T_st = T_sb * T_bt
        start_point = [T_st.r2_c4, T_st.r3_c4]

        # Move in a line parallel to the drawing plane
        print("GO TO: start_point: ", start_point, " end_point: ", end_point)
        line_path = self.calculate_line_path(start_point, end_point)
        x = 0.04
        self.move_along_path(x, line_path)

    def move_along_path(self, x, path):
        joints_angles_list = []
        for point in path:
            angles_i = self.calculate_angles(x, point[0], point[1])
            if(angles_i):
                joints_angles_list.append(angles_i)

        #print(joints_angles_list)

        if(joints_angles_list):
            self.move_joints(joints_angles_list)

    def draw_path(self, path):
        # Move to the first point in the path (without drawing anything)
        self.go_to_point(path[0])
        # Draw (move in drawing plane)
        x = 0.0
        self.move_along_path(x, path)

    def draw_line(self, start_point, end_point):
        line_path = self.calculate_line_path(start_point, end_point)
        self.draw_path(line_path)

    def calculate_line_path(self, start_point, end_point):
        line_path = []
        y0 = start_point[0]
        z0 = start_point[1]
        line_path.append([y0, z0])

        yn = end_point[0]
        zn = end_point[1]
        length = math.sqrt((yn-y0)**2 + (zn-z0)**2) * 100
        yi = y0
        zi = z0
        for i in range(1, int(length) + 1):
             yi = (yn - y0) /length * i + y0
             zi = (zn - z0) /length * i + z0
             line_path.append([yi,zi])

        line_path.append([yn,zn])

        print("line_path: ", line_path)

        return line_path

    def draw_ellipse(self, start_point, a, b):
        ellipse_path = self.calculate_ellipse_path(start_point, a, b)
        self.draw_path(ellipse_path)

    def calculate_ellipse_path(self, start_point, a, b):
        # max distance between two consecutive points
        max_distance = 0.01
        path = []
        i = 0
        final_i = 0
        while i <= 360:
            z = a * math.cos(math.radians(i))
            y = b * math.sin(math.radians(i))
            path.append([y + start_point[0], z + start_point[1]])
            final_i = i
            i = i + math.degrees(math.acos(1- (max_distance**2)/(2*(a**2)) ))
        if final_i < 360:
            final_i = 360 
            z = a * math.cos(math.radians(final_i))
            y = b * math.sin(math.radians(final_i))
            path.append([y + start_point[0], z + start_point[1]])
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