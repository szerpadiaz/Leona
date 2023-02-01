#!/usr/bin/env python
import rospy
import os
import cv2

from naoqi_bridge_msgs.msg import HeadTouch
import pickle
from picture_taker import *

SKETCH_FACE_FILE = WATCHFOLDER_PATH + "sketch_face.jpg"
SKETCH_FACE_PATHS_FILE = WATCHFOLDER_PATH + "sketcher_result.pkl"

class Test_picture_taker:
    def __init__(self, local = False):
        self.picture_taker = pictureTaker(local)
        self.front_button_pressed = False
        self.head_sub = rospy.Subscriber('/tactile_touch', HeadTouch, self.head_touch_callback)

    def speak(self, text):
        print(text)
        if self.local:
            os.system(str("say " + text))
        else:
            self.tts.say(text)

    def take_stylish_picture(self):
        success = False
        img, conv_img = self.picture_taker.takePicture("detect_face.jpg")
        analyzePictureResponse, _ = self.picture_taker.analyzePicture(conv_img, showAnalysis= True)
        if analyzePictureResponse == "Success":
            cv2.imwrite(SKETCH_FACE_FILE, img) 
            paths = "Still processing"
            print("looking for sketcher result")
            while paths == "Still processing":
                with open(SKETCH_FACE_PATHS_FILE, "rb") as f:    
                    paths = pickle.load(f)
                    print(paths)
                    if paths == "Still processing":
                        rospy.sleep(1)
                        print("Still procesing here")
                        continue
                    print("Sketcher result:", paths)
        return success     

    def main_loop(self):
        while not (self.front_button_pressed):
            rospy.sleep(0.2)
        self.speak("Taking a picture in 3, 2, 1. Smile!")
        success = self.take_stylish_picture()
        if(success):
            self.speak("Your sketch is ready under the Watch folder") 
            print("SKETCH_FACE_PATHS_FILE: ", SKETCH_FACE_PATHS_FILE)   
        else:
            self.speak("Let's try again!")

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