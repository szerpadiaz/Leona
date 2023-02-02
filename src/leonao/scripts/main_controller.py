#!/usr/bin/env python
## Main application controller

import rospy

from naoqi_bridge_msgs.msg import HeadTouch
from naoqi import ALProxy

import random

from picture_taker import *
from picture_painter import *



MSG_THANKS = "Thank you for visiting my studio, come back soon!"
MSG_BEFORE_SIESTA = "I am going to take a nap now. See you later!"

INTRO_MSG_1 = [
"Greetings! I am LeoNao Davinci, the mechanical artist.",
"Welcome to my workshop, where I create beautiful portraits just like Leonardo Da Vinci.",
"How do you do? I am LeoNao, a robot that paints portraits in the style of Leonardo Da Vinci.",
"Good day! I am LeoNao, a robot that emulates the famous Leonardo Da Vinci.",
"Salutations! I am LeoNao Davinci, a robot with the talent of Leonardo Da Vinci.",
"Step right up, step right up! I, the great Leonardo Da Vinci, am here to paint your portrait!"
]
INTRO_MSG_2 = [
"If you desire a stunning portrait, just press the button on my head",
"I would be honored to paint your portrait. Just touch my head to start.",
"Let's create a masterpiece together! Just press the button on my head.",
"Are you ready for a beautiful portrait? Simply touch the button on my head to begin.",
"It would be my pleasure to paint your portrait. Please touch the button on my head to start.",
"I can draw a masterpiece for you. Just come here and touch my head to start."
]
TAKING_PICTURE_INSTRUCTIONS_1 = [
"Perfect! Let's get started.",
"Excellent! Here we go!",
"Fantastic! Let's begin.",
"Great! I'm ready when you are.",
"Awesome! Let's make a masterpiece."
]
TAKING_PICTURE_INSTRUCTIONS_2 = [
"Please stand in front of me and hold still for the picture.",
"Get ready for your close-up! Stand in front of me and hold still."
"Just stand in front of me and stay still for the picture."
"Position yourself in front of me and hold still. I don't want to capture any movement."
"Take your place in front of me and hold still. I want to capture your true beauty.",
"Hold still, my friend! I don't want to capture your nerves, just your beauty."
]
MSG_AFTER_SUCCESS_PICTURE_TAKEN = [
"Wonderful! Now relax and let me paint your portrait.",
"Marvelous! I'll make sure you look even more stunning in the portrait.",
"Excellent! Just sit back and enjoy while I create your portrait.",
"Fantastic! I'll make sure to capture your best features in the portrait.",
"Superb! I'll bring out your beauty in the portrait.",
"Well done! please relax and enjoy while I paint you. Don't worry, I'll make you look better than you do in real life."
]
MSG_PICTURE_TAKEN_FAILED = [
"Oh dear, let's try again.",
"Darn, let's take the picture again.",
"Too bad, let's try taking the picture again.",
"Oh no, let's retry the picture.",
"Bummer, let's take the picture one more time."
]
MSG_PAINTING_IS_DONE = [
"It's finished! Here is your beautiful portrait! That will be 5 Euro, please.",
"Ta-da! Here is your stunning portrait! That will be 5 Euro, if you please.",
"Voila! Here is your gorgeous portrait! May I have 5 Euro, please?",
"Et voila! Here is your amazing portrait! That will be 5 Euro, thank you.",
"Behold! Here is your breathtaking portrait!"]


class Main_leonao_controller():
    def __init__(self):
        self.wake_up = False

        self.robot_ip=str(os.getenv("NAO_IP"))
        self.robot_port=int(9559)
        self.tts = ALProxy("ALTextToSpeech", self.robot_ip, 9559)
        self.head_sub = rospy.Subscriber('/tactile_touch', HeadTouch, self.head_touch_callback)

        self.picture_taker =  pictureTaker()
        self.picture_painter = Picture_painter()
        self.speak(INTRO_MSG_1)
        self.speak(INTRO_MSG_2)
    
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
        if self.wake_up:
            self.wake_up = False
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
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            main_controller.main_loop()
            rate.sleep()
            #ros::spin()?

    except rospy.ROSInterruptException:
        pass
