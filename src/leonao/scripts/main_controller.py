#!/usr/bin/env python
## Main application controller

import rospy
#from move_controller import Move_controller

from naoqi_bridge_msgs.msg import HeadTouch

class Main_leonao_controller():
    def __init__(self):
        self.front_button_pressed = False
        self.rear_button_pressed = False
        self.head_sub = rospy.Subscriber('/tactile_touch', HeadTouch, self.head_touch_callback)

        #self.move_controller = Move_controller()

        self.record_timer = None
        self.recording_started = False
        self.drawing_points = []

    def head_touch_callback(self, head_touch_event):
        self.front_button_pressed = head_touch_event.button == HeadTouch.buttonFront and head_touch_event.state == HeadTouch.statePressed
        self.rear_button_pressed = head_touch_event.button == HeadTouch.buttonRear and head_touch_event.state == HeadTouch.statePressed

    def start_record_timer(self):
        self.record_timer = rospy.Timer(rospy.Duration(0.125), self.record_drawing_points, oneshot=False)

    def stop_record_timer(self):
        if self.record_timer:
            self.record_timer.shutdown()

    def record_drawing_points(self, timer_event):
    	pass
        #self.drawing_points.append(self.move_controller.get_position('RArm'))

    def disable_arm_stiffness(self):
        effector_names = ["RShoulderPitch", "RShoulderRoll", "ElbowYaw", "RElbowRoll", "RWristYaw"]
        stiffness_values = [0.0, 0.0, 0.0, 0.0, 0.0]
        #self.move_controller.set_stiffness(effector_names, stiffness_values)
        pass

    def enable_arm_stiffness(self):
        #self.move_controller.set_stiffness(["RArm"], [1.0])
        pass

    def draw(self):
        #self.move_controller.move_end_effector_with_speed('RArm', self.drawing_points, 0.2)
        #self.move_controller.move_end_effector_with_time('RArm', self.drawing_points, 0.1)
        print("Sending MOVE ENDED *********    i = ", len(self.drawing_points)) 

    def drawing_loop(self):
        if self.front_button_pressed:
            self.front_button_pressed = False
            self.stop_record_timer()
            if (self.recording_started):
                self.recording_started = False
                print(self.drawing_points)
                print("###### RECORDING STOPPED ######")
            else:
                self.disable_arm_stiffness()
                print("Move to initial position")
                rospy.sleep(1.0)
                print("RECORDING STARTED")
                self.drawing_points = []
                self.start_record_timer()
                self.recording_started = True

        if self.rear_button_pressed and not self.recording_started:
            self.rear_button_pressed = False
            self.enable_arm_stiffness()
            self.draw()

if __name__ == '__main__':

    rospy.init_node('main_controller', anonymous=True)

    try:
        main_controller = Main_leonao_controller()
        #rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            main_controller.drawing_loop()
            #rate.sleep()

    except rospy.ROSInterruptException:
        pass
