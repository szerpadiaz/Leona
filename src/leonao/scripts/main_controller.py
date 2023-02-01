#!/usr/bin/env python
## Main application controller

import rospy
#from move_controller import Move_controller

from naoqi_bridge_msgs.msg import HeadTouch
from naoqi import ALProxy

from picture_taker import *
from picture_painter import *

INTRO_MSG_1 = "Hello! I am Leonao, Leonao Davinci. Welcome to my studio!"
INTRO_MSG_2 = "If you want a beautiful picture, please press the button on the front of my head"
TAKING_PICTURE_INSTRUCTIONS_1 = "OK! lets get started"
TAKING_PICTURE_INSTRUCTIONS_2 = "Please position yourself in front of me and get ready for the picture"
MSG_AFTER_SUCCESS_PICTURE_TAKEN = "Well done! please relax and enjoy while I paint you"
MSG_PICTURE_TAKEN_FAILED = "OH no, let's try again!"
MSG_PAINTING_IS_DONE = "I am done, here it is your beautiful picture"
MSG_THANKS = "Thanks for letting me sketching you."
MSG_BEFORE_SIESTA = "I am not going to take a Siesta. Wake me up!"

class Main_leonao_controller():
    def __init__(self):
        self.front_button_pressed = False

        self.robot_ip=str(os.getenv("NAO_IP"))
        self.robot_port=int(9559)
        self.tts = ALProxy("ALTextToSpeech", self.robot_ip, 9559)
        self.head_sub = rospy.Subscriber('/tactile_touch', HeadTouch, self.head_touch_callback)

        self.picture_taker =  pictureTaker()
        self.picture_painter = Picture_painter()
        self.speak(INTRO_MSG_1)
        self.speak(INTRO_MSG_2)
    
    def speak(self, text):
        print(text)
        if self.local:
            os.system(str("say " + text))
        else:
            self.tts.say(text)

    def head_touch_callback(self, head_touch_event):
        self.front_button_pressed = head_touch_event.button == HeadTouch.buttonFront and head_touch_event.state == HeadTouch.statePressed


    def drawing_loop(self):
        if self.front_button_pressed:
            self.front_button_pressed = False
            self.speak(TAKING_PICTURE_INSTRUCTIONS_1)
            self.speak(TAKING_PICTURE_INSTRUCTIONS_2)
            success = self.take_stylish_picture()
            if success:
                self.speak(MSG_AFTER_SUCCESS_PICTURE_TAKEN)
                self.picture_painter.draw_face(SKETCH_FACE_PATHS_FILE)
                self.speak(MSG_PAINTING_IS_DONE)
            else:
                self.speak(MSG_PICTURE_TAKEN_FAILED)
            self.speak(MSG_THANKS)
            self.speak(MSG_BEFORE_SIESTA)

if __name__ == '__main__':

    rospy.init_node('main_controller', anonymous=True)

    try:
        main_controller = Main_leonao_controller()
        #rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            main_controller.drawing_loop()
            #rate.sleep()
            #ros::spin()?

    except rospy.ROSInterruptException:
        pass
