#!/usr/bin/env python
import rospy
import os

from naoqi_bridge_msgs.msg import HeadTouch
import pickle
from picture_taker import *


class Test_picture_taker:
    def __init__(self, local = False):
        self.picture_taker = pictureTaker(local)
        self.front_button_pressed = False
        self.head_sub = rospy.Subscriber('/tactile_touch', HeadTouch, self.head_touch_callback)

    def main_loop(self):
        #while not (self.front_button_pressed):
        #    rospy.sleep(0.2)
        raw_input("Press enter to take picture")
        success = self.picture_taker.take_stylish_picture()
        if(success):
            self.picture_taker.speak("Your sketch is done!") 
            print("SKETCH_FACE_PATHS_FILE: ", SKETCH_FACE_PATHS_FILE)   
        else:
            self.picture_taker.speak("Try again!")

    ################ Running Callbacks ################

    def head_touch_callback(self, head_touch_event):
        self.front_button_pressed = head_touch_event.button == HeadTouch.buttonFront and head_touch_event.state == HeadTouch.statePressed

if __name__ == '__main__':
    rospy.init_node('test_picture_taker', anonymous=False)
    tester = Test_picture_taker(local = False)
    try:
        while not rospy.is_shutdown():
            tester.main_loop()
            pass     
    except rospy.ROSInterruptException:
        pass