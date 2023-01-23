#!/usr/bin/env python
## Example to setup the frameworks for the canvas

from pickle import TRUE
import rospy
import os

import almath
from naoqi import ALProxy
from naoqi_bridge_msgs.msg import HeadTouch
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

class Drawing_setup_tester():
    def __init__(self):
        robot_ip=str(os.getenv("NAO_IP"))
        robot_port=int(9559)
        self.motion_proxy = ALProxy("ALMotion", robot_ip, robot_port)

        self.head_sub = rospy.Subscriber('/tactile_touch', HeadTouch, self.head_touch_callback)
        self.front_button_pressed = False

        self.init_station_frame()

        self.previous_point = [0,0,0]

    def init_station_frame(self):
        # Set a initial position
        #initial_position =  [0.1349065601825714, -0.20778782665729523, 0.08519446104764938, -0.3972117304801941, 0.07472652941942215, -0.013175832107663155]
        #self.motion_proxy.setPosition("RArm", motion.FRAME_TORSO, initial_position, 0.5, motion.AXIS_MASK_ALL)
        #rospy.sleep(1.0)

        # T_bs: Transformation of the STATION-FRAME (s) with respect to the BASE-FRAME (b)
        joints_names = self.motion_proxy.getBodyNames("RArm")
        joints_angles = self.motion_proxy.getAngles(joints_names, True)
        self.T_bs = self.get_transform_client(joints_angles)

        initial_position = self.motion_proxy.getPosition("RArm", motion.FRAME_TORSO, True)
        print("initial_position", initial_position)
        
    def disable_arm_stiffness(self):
        self.motion_proxy.stiffnessInterpolation('RArm', 0.0, 1.0)

    def enable_arm_stiffness(self):
        self.motion_proxy.stiffnessInterpolation('RArm', 1.0, 1.0)

    def head_touch_callback(self, head_touch_event):
        self.front_button_pressed = head_touch_event.button == HeadTouch.buttonFront and head_touch_event.state == HeadTouch.statePressed
        #self.rear_button_pressed = head_touch_event.button == HeadTouch.buttonRear and head_touch_event.state == HeadTouch.statePressed
    
    def get_joints_angles(self, x, y, z):
        # T_sg: Transformation of the GOAL-FRAME (g) with respect to the STATION-FRAME (s)
        T_sg = almath.Transform(x,y,z)

        # T_bg: Transformation of the GOAL-FRAME (g) with respect to the BASE-FRAME (b)
        T_bg = self.T_bs * T_sg
        position6D = [T_bg.r1_c4, T_bg.r2_c4, T_bg.r3_c4, 0, 0, 0]
        joints_angles = self.get_angles_client(position6D)
        return joints_angles

    def get_angles_client(self, position6D):
        rospy.wait_for_service('Nao_RArm_chain_get_angles')
        try:
            service_get_angles = rospy.ServiceProxy('Nao_RArm_chain_get_angles', Nao_RArm_chain_get_angles)
            resp = service_get_angles(position6D)
            return resp.angles
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return []

    def get_transform_client(self, joint_angles):
        rospy.wait_for_service('Nao_RArm_chain_get_transform')
        try:
            service_get_transform = rospy.ServiceProxy('Nao_RArm_chain_get_transform', Nao_RArm_chain_get_transform)
            resp = service_get_transform(joint_angles)
            T = almath.Transform(resp.transform)   
            return T
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return []

    def move(self, T_bw_as_vector_list):
        time_per_move = 1
        times = [time_per_move * (i+1) for i in range(len(T_bw_as_vector_list))]
        
        print(times)
        #fractionMaxSpeed = 0.2
        # axisMask = almath.AXIS_MASK_ALL # we want to set both the position and the orientation
        axisMask = almath.AXIS_MASK_VEL
        self.motion_proxy.transformInterpolations("RArm", BASE_FRAME_ID, T_bw_as_vector_list, axisMask, times)

    def move_angles(self, joints_angles_list):
        joint_names = self.motion_proxy.getBodyNames("RArm")
        for target_angles in joints_angles_list:
            self.motion_proxy.angleInterpolationWithSpeed(joint_names[:-1], target_angles, 0.25)
            #rospy.sleep(0.5)

    def main_loop(self):
        x = float(input("Enter x value in cm")) / 100
        y = float(input("Enter y value in cm")) / 100
        z = float(input("Enter z value in cm")) / 100

        joints_angles_list = []
        length = math.sqrt((x-self.previous_point[0])**2 + (y-self.previous_point[1])**2 + (z-self.previous_point[2])**2)*100
        for i in range(1, int(length) + 1):
             xi = (x - self.previous_point[0]) /length * i + self.previous_point[0]
             yi = (y - self.previous_point[1]) /length * i + self.previous_point[1]
             zi = (z - self.previous_point[2]) /length * i + self.previous_point[2]
             angles_i = self.get_joints_angles(xi, yi, zi)
             if(angles_i):
                joints_angles_list.append(angles_i)

        self.previous_point = [x,y,z]

        angles = self.get_joints_angles(x, y, z)
        if(angles):
            joints_angles_list.append(angles)
        
        # Move by setting a list of angles.
        self.move_angles(joints_angles_list)
        
if __name__ == '__main__':
    rospy.init_node('drawing_setup_example', anonymous=True)
    tester = Drawing_setup_tester()
    tester.enable_arm_stiffness()
    rospy.sleep(1.0)

    try:
        while not rospy.is_shutdown():
            tester.main_loop()
            pass
    except rospy.ROSInterruptException:
        pass

    tester.disable_arm_stiffness()     