#!/usr/bin/env python
## Main application controller

import rospy
import cv2

from naoqi_bridge_msgs.msg import HeadTouch
from sensor_msgs.msg import Image
from naoqi import ALProxy

import random

from picture_taker import *
from picture_painter import *

INTRO_MSG_1 = ["Hello! I am Leonao, Leonao Davinci. Welcome to my studio!", "Step right up, step right up! I, the great Leonardo Da Vinci, am here to paint your portrait!"]
INTRO_MSG_2 = ["If you want a beautiful picture, please press the button on the front of my head", "I can draw a masterpiece for you. Just come here and touch my head to start."]
TAKING_PICTURE_INSTRUCTIONS_1 = ["OK! lets get started", "Alright, let's do this!", "Here we go!"]
TAKING_PICTURE_INSTRUCTIONS_2 = ["Please position yourself in front of me and get ready for the picture.", "Hold still, my friend! I don't want to capture your nerves, just your beauty."]

MSG_AFTER_SUCCESS_PICTURE_TAKEN = "Well done! please relax and enjoy while I paint you. Don't worry, I'll make you look better than you do in real life."
MSG_PICTURE_TAKEN_FAILED = "OH no, let's try again!"
MSG_PAINTING_IS_DONE = "I am done, here it is your beautiful picture! That would be 5 Euro please!"
MSG_THANKS = "Thanks for letting me sketching you."
MSG_BEFORE_SIESTA = "I am not going to take a Siesta. Wake me up!"

class Main_leonao_controller():
    def __init__(self):
        self.wake_up = False

        self.robot_ip=str(os.getenv("NAO_IP"))
        self.robot_port=int(9559)
        self.tts = ALProxy("ALTextToSpeech", self.robot_ip, 9559)
        self.head_sub = rospy.Subscriber('/tactile_touch', HeadTouch, self.head_touch_callback)

        self.picture_taker =  pictureTaker(useTestPicture = False)
        self.picture_painter = Picture_painter()
        # self.speak(INTRO_MSG_1)
        # self.speak(INTRO_MSG_2)
        self.wake_up =  False
        from cv_bridge import CvBridge
        self.bridge = CvBridge()

        imageTop = rospy.Subscriber("/nao_robot/camera/top/camera/image_raw", Image, self.showImageCallback)

    def showImageCallback(self, img_msg):
        img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        cv2.imshow("Current Image", img)
    
    def speak(self, text):
        if type(text) == str:
            self.tts.say(text)
        elif type(text) == list:
            self.speak(random.choice(text))
        else:
            print("Speech input: " + text + " of type: " + type(text) + " not recognized!")
            

    def head_touch_callback(self, head_touch_event):
        self.wake_up = head_touch_event.button == HeadTouch.buttonFront and head_touch_event.state == HeadTouch.statePressed

    def main_loop(self):
        #while not self.wake_up:
        #    rospy.sleep(0.1)
        
        raw_input("Press enter")
        self.wake_up = True
        if self.wake_up:
            self.wake_up = False
            # self.speak(TAKING_PICTURE_INSTRUCTIONS_1)
            # self.speak(TAKING_PICTURE_INSTRUCTIONS_2)
            #success = self.picture_taker.take_stylish_picture()
            #paths_file = SKETCH_FACE_PATHS_FILE
            paths_file = WATCHFOLDER_PATH + "ingo_face_paths.pkl"
            success = True
            if success:
                # self.speak(MSG_AFTER_SUCCESS_PICTURE_TAKEN)
                self.picture_painter.draw_face(paths_file)
                self.speak(MSG_PAINTING_IS_DONE)
                # self.speak(MSG_THANKS)
                # self.speak(MSG_BEFORE_SIESTA)
            else:
                self.speak(MSG_PICTURE_TAKEN_FAILED)

if __name__ == '__main__':

    rospy.init_node('main_controller', anonymous=True)

    try:
        main_controller = Main_leonao_controller()
        #rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            main_controller.main_loop()
            #rate.sleep()
            #ros::spin()?

    except rospy.ROSInterruptException:
        pass
