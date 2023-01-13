#!/usr/bin/env python
## Example to setup the frameworks for the canvas

from turtle import position
import rospy
import os
import numpy as np

import almath
from naoqi import ALProxy
from naoqi_bridge_msgs.msg import HeadTouch

DIST_TO_TIP = [0.08,0.0,0.035, 1]
FRAME_TORSO = 0

class Drawing_setup_tester():
    def __init__(self):
        robot_ip=str(os.getenv("NAO_IP"))
        robot_port=int(9559)
        self.motion_proxy = ALProxy("ALMotion", robot_ip, robot_port)

        self.head_sub = rospy.Subscriber('/tactile_touch', HeadTouch, self.head_touch_callback)
        self.front_button_pressed = False

        #tf_t_w = almath.Transform(DIST_TO_TIP[0], DIST_TO_TIP[1], DIST_TO_TIP[2])
        #self.tf_w_t = almath.transformInverse(tf_t_w)
        
        self.t_w_t = DIST_TO_TIP
        self.tf_b_w = almath.Transform(self.motion_proxy.getTransform("RArm", FRAME_TORSO, False))
        self.initial_t_b_t = self.get_point_b_t(self.t_w_t)

    def get_point_b_t(self, point_w_t):
        tf_b_w = np.reshape(self.tf_b_w.toVector(), (4,4))
        point_b_t = tf_b_w.dot(point_w_t)
        return point_b_t[:3]

    def get_target_point(self):
        y = float(input("Enter y value in cm"))
        z = float(input("Enter z value in cm"))
        target_point = [y/100.0, z/100.0]
        print("target_point", target_point)
        return target_point

    def convert_target_point_to_base_frame(self, p_t):

        tf_b_w = np.reshape(self.frame_w_origin.toVector(), (4,4))
        #tf_b_w.r2_c4 += target_point[0]
        #tf_b_w.r3_c4 += target_point[1]

        target_w = np.ones(4)
        target_w[0] = 0
        target_w[1] = target_point[0] 
        target_w[2] = target_point[1]
        target_b = tf_b_w.dot(target_w)
        #print("target_t", target_t)
        #print("target_b", target_b)
        return target_b[:3]

    def move_to_target_point(self, target_delta):
        target_point_b_t =  self.initial_t_b_t
        target_point_b_t[1] = target_delta[0]
        target_point_b_t[2] = target_delta[1]

        point_b_wt = self.tf_b_w * self.t_w_t

        ###### TODO ######
        target_point_b_w = target_point_b_t - self.t_w_t # not correct
        x = target_point_b_w[0]
        y = target_point_b_w[1]
        z = target_point_b_w[2]
        
        next_tf_b_w = almath.Transform().fromPosition(x, y, z, rx, ry, rz)
        
        target_point_b_w =  target_point_b_t - next_tf_b_w * self.t_w_t
        ###### TODO ######

        path_to_p_b, times = self.get_path(p_t)
        self.enable_arm_stiffness()
        #self.move_to_origin()
        # target_b_6D = list(target_b) + [0,0,0]
        # self.motion_proxy.setPositions('RArm', FRAME_TORSO, target_b_6D, 0.2, 7)
        self.motion_proxy.positionInterpolation('RArm', FRAME_TORSO, target_path_b, 7, times, True)
        rospy.sleep(5.0)
        self.disable_arm_stiffness()

    def move_to_origin(self):
        #position = [val for val in self.frame_tip_origin.toVector()]
        #self.motion_proxy.setTransforms('RArm', FRAME_TORSO, position, 0.2, 63)
        postion = list(self.convert_target_point_to_base_frame([0,0])) + [0,0,0]
        print(postion)
        self.motion_proxy.setPositions('RArm', FRAME_TORSO, postion, 0.2, 7)
        rospy.sleep(5.0)

    def get_path(self, target_point):
        path = []
        times = []
        for i in range(1,11):
            times.append(0.5*i)
            target_point_i = [p*0.1*i for p in target_point]
            target_b = list(self.convert_target_point_to_base_frame(target_point_i)) + [0,0,0]
            path.append(target_b)
        print(path)
        print(times)
        return [path, times]

    def disable_arm_stiffness(self):
        self.motion_proxy.stiffnessInterpolation('RArm', 0.0, 1.0)

    def enable_arm_stiffness(self):
        self.motion_proxy.stiffnessInterpolation('RArm', 1.0, 1.0)


    def head_touch_callback(self, head_touch_event):
        self.front_button_pressed = head_touch_event.button == HeadTouch.buttonFront and head_touch_event.state == HeadTouch.statePressed
        #self.rear_button_pressed = head_touch_event.button == HeadTouch.buttonRear and head_touch_event.state == HeadTouch.statePressed

    def main_loop(self):
        target_delta = tester.get_target_point()
        self.move_to_target_point(target_delta)

if __name__ == '__main__':
    rospy.init_node('drawing_setup_tester', anonymous=True)
    tester = Drawing_setup_tester()
    try:
        while not rospy.is_shutdown():
            tester.main_loop()
            
    except rospy.ROSInterruptException:
        pass        