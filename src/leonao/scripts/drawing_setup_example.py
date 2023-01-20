#!/usr/bin/env python
## Example to setup the frameworks for the canvas

import rospy
import os

import almath
from naoqi import ALProxy
from naoqi_bridge_msgs.msg import HeadTouch
import math
import motion

from inv_kinematic import *

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
        self.arm_chain = Nao_RArm_chain()
        robot_ip=str(os.getenv("NAO_IP"))
        robot_port=int(9559)
        self.motion_proxy = ALProxy("ALMotion", robot_ip, robot_port)

        self.head_sub = rospy.Subscriber('/tactile_touch', HeadTouch, self.head_touch_callback)
        self.front_button_pressed = False

        self.init_station_frame()

        self.previous_point = [0,0,0]

    def init_station_frame(self):
        # T_bs: Transformation of the STATION-FRAME (s) with respect to the BASE-FRAME (b)
        joints_names = self.motion_proxy.getBodyNames("RArm")
        joints_angles = self.motion_proxy.getAngles(joints_names, True)
        self.T_bs = self.arm_chain.get_transform(joints_angles)
        joints_angles2 = self.arm_chain.get_angles_from_transform(self.T_bs)
        print("joint_angles alproxy:", joints_angles)
        print("joint_angles inv kin:", joints_angles2)
        #print("T_bs", self.T_bs)


    def get_joints_angles(self, x, y, z):
        # T_sg: Transformation of the GOAL-FRAME (g) with respect to the STATION-FRAME (s)
        T_sg = almath.Transform(x,y,z)

        # T_bg: Transformation of the GOAL-FRAME (g) with respect to the BASE-FRAME (b)
        T_bg = self.T_bs * T_sg
        joints_angles = self.arm_chain.get_angles_from_transform(T_bg)
        return joints_angles

    def move(self, T_bw_as_vector_list):
        time_per_move = 1
        times = [time_per_move * (i+1) for i in range(len(T_bw_as_vector_list))]
        
        print(times)
        #fractionMaxSpeed = 0.2
        # axisMask = almath.AXIS_MASK_ALL # we want to set both the position and the orientation
        axisMask = almath.AXIS_MASK_VEL
        self.motion_proxy.transformInterpolations("RArm", BASE_FRAME_ID, T_bw_as_vector_list, axisMask, times)

    def disable_arm_stiffness(self):
        self.motion_proxy.stiffnessInterpolation('RArm', 0.0, 1.0)

    def enable_arm_stiffness(self):
        self.motion_proxy.stiffnessInterpolation('RArm', 1.0, 1.0)

    def head_touch_callback(self, head_touch_event):
        self.front_button_pressed = head_touch_event.button == HeadTouch.buttonFront and head_touch_event.state == HeadTouch.statePressed
        #self.rear_button_pressed = head_touch_event.button == HeadTouch.buttonRear and head_touch_event.state == HeadTouch.statePressed

    def main_loop(self):
        x = float(input("Enter x value in cm")) / 100
        y = float(input("Enter y value in cm")) / 100
        z = float(input("Enter z value in cm")) / 100

        joins_angles_list = []
        length = math.sqrt((x-self.previous_point[0])**2 + (y-self.previous_point[1])**2 + (z-self.previous_point[2])**2)*100
        for i in range(1, int(length) + 1):
            xi = (x - self.previous_point[0]) /length * i + self.previous_point[0]
            yi = (y - self.previous_point[1]) /length * i + self.previous_point[1]
            zi = (z - self.previous_point[2]) /length * i + self.previous_point[2]
            joins_angles_list.append(self.get_joints_angles(xi, yi, zi))
        self.previous_point = [x,y,z]

        joins_angles_list.append(self.get_joints_angles(x, y, z))
        
        print(joins_angles_list)
        #self.move(T_bw_as_vector_list)

if __name__ == '__main__':
    rospy.init_node('drawing_setup_example', anonymous=True)
    tester = Drawing_setup_tester()
    #tester.enable_arm_stiffness()
    #rospy.sleep(2.0)

    try:
        while not rospy.is_shutdown():
            tester.main_loop()
            pass
    except rospy.ROSInterruptException:
        pass

    #self.disable_arm_stiffness()     