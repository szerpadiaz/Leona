#!/usr/bin/env python
## Example to setup the frameworks for the canvas

from turtle import position
import rospy
import os
import numpy as np

import almath
from naoqi import ALProxy
from naoqi_bridge_msgs.msg import HeadTouch
import math
import motion

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

        self.init_tool_and_station_transformations()

        self.previous_point = [0,0,0]

    def init_tool_and_station_transformations(self):
        # T_wt: transformation of the TOOL-FRAME (t) with respect to the WRIST-FRAME (w)
        x = TIP_POSITION_WITH_RESPECT_TO_W[0]
        y = TIP_POSITION_WITH_RESPECT_TO_W[1]
        z = TIP_POSITION_WITH_RESPECT_TO_W[2]
        self.T_wt = almath.Transform(x,y,z)

        # T_tw: transformation of the WRIST-FRAME (w) with respect to the TOOL-FRAME (t)
        self.T_tw = almath.transformInverse(self.T_wt)

        # T_bs: Transformation of the STATION-FRAME (s) with respect to the BASE-FRAME (b)
        T_bw = almath.Transform(self.motion_proxy.getTransform("RArm", BASE_FRAME_ID, False))
        # When this function is called we assume that the tip of the TOOL-FRAME coincides with the STATION-FRAME
        # {TOOL-FRAME} = {STATION-FRAME}
        T_ws = self.T_wt
        self.T_bs = T_bw * T_ws
        

    def get_goal_transformation_with_respect_to_the_station(self, x, y, z):
        # T_sg: Transformation of the GOAL-FRAME (g) with respect to the STATION-FRAME (s)
        T_sg = almath.Transform(x,y,z)

        return T_sg

    def get_wrist_transformation_with_respect_to_base_to_reach_the_goal(self, T_sg):
        # T_bg: Transformation of the GOAL-FRAME (g) with respect to the BASE-FRAME (b)
        T_bg = self.T_bs * T_sg

        # To move the tool to the GOAL-FRAME, we have to make the {TOOL-FRAME} = {GOAL-FRAME}
        T_bt = T_bg

        # T_bw: Transformation of the WRIST-FRAME (w) with respect to the BASE-FRAME (b)
        T_bw = T_bt * self.T_tw

        # print("T_sg = ", T_sg)
        # print("T_bg = ", T_bg)
        # print("T_bt = ", T_bt)
        # print("T_bw = ", T_bw)
        return T_bw

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

        T_bw_as_vector_list = []
        length = math.sqrt((x-self.previous_point[0])**2 + (y-self.previous_point[1])**2 + (z-self.previous_point[2])**2)*100
        for i in range(1, int(length) + 1):
            xi = (x - self.previous_point[0]) /length * i + self.previous_point[0]
            yi = (y - self.previous_point[1]) /length * i + self.previous_point[1]
            zi = (z - self.previous_point[2]) /length * i + self.previous_point[2]
            print(xi,yi,zi)
            T_sg = self.get_goal_transformation_with_respect_to_the_station(xi, yi, zi)
            T_bw = self.get_wrist_transformation_with_respect_to_base_to_reach_the_goal(T_sg)
            T_bw_as_vector_list.append(list(T_bw.toVector()))
        self.previous_point = [x,y,z]

        T_sg = self.get_goal_transformation_with_respect_to_the_station(x, y, z)
        T_bw = self.get_wrist_transformation_with_respect_to_base_to_reach_the_goal(T_sg)
        T_bw_as_vector_list.append(list(T_bw.toVector()))
        
        print(T_bw_as_vector_list)
        self.move(T_bw_as_vector_list)

if __name__ == '__main__':
    rospy.init_node('drawing_setup_example', anonymous=True)
    tester = Drawing_setup_tester()
    tester.enable_arm_stiffness()
    rospy.sleep(2.0)

    try:
        while not rospy.is_shutdown():
            tester.main_loop()
            
    except rospy.ROSInterruptException:
        pass

    #self.disable_arm_stiffness()     