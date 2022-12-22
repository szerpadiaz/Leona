#!/usr/bin/env python
## Main application controller

import rospy
from naoqi_bridge_msgs.msg import HeadTouch
from leonao.srv import GetCartesianCoordinates, Stiffness, MoveJoints

class Main_leonao_controller():
    def __init__(self):
        self.front_button_pressed = False
        self.rear_button_pressed = False
        self.recording_started = False
        self.recorded_positions = []

        self.head_sub = rospy.Subscriber('/tactile_touch', HeadTouch, self.head_touch_callback)
        self.record_timer = None

    def head_touch_callback(self, head_touch_event):
        self.front_button_pressed = head_touch_event.button == HeadTouch.buttonFront and head_touch_event.state == HeadTouch.statePressed
        self.rear_button_pressed = head_touch_event.button == HeadTouch.buttonRear and head_touch_event.state == HeadTouch.statePressed

    def start_record_timer(self):
        self.record_timer = rospy.Timer(rospy.Duration(0.125), self.record_data, oneshot=False)

    def stop_record_timer(self):
        if self.record_timer:
            self.record_timer.shutdown()

    def record_data(self, timer_event):
        rospy.wait_for_service('get_cartesian_coordinates')
        try:
            get_position = rospy.ServiceProxy('get_cartesian_coordinates', GetCartesianCoordinates)
            resp = get_position("RArm")
            self.recorded_positions.append(list(resp.position))
        except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

    def disable_arm_stiffness(self):
        rospy.wait_for_service('set_stiffness')
        try:
            set_stiffness = rospy.ServiceProxy('set_stiffness', Stiffness)
            effector_names = ["RShoulderPitch", "RShoulderRoll", "ElbowYaw", "RElbowRoll", "RWristYaw"]
            stiffness_values = [0.0, 0.0, 0.0, 0.0, 0.0]
            set_stiffness(effector_names, stiffness_values)
        except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

    def enable_arm_stiffness(self):
        rospy.wait_for_service('set_stiffness')
        try:
            set_stiffness = rospy.ServiceProxy('set_stiffness', Stiffness)
            effector_names = ["RArm"]
            stiffness_values = [1.0]
            set_stiffness(effector_names, stiffness_values)
        except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

    def move_arm(self):
        rospy.wait_for_service('move_end_effector')
        try:
            move_end_effector=rospy.ServiceProxy('move_end_effector', MoveJoints)
            time = 0.0 #0.1
            speed = 0.2 #0.2
            for pos6D in self.recorded_positions:
                position = pos6D[:3]
                orientation = pos6D[3:]
                move_end_effector('RArm', position, orientation, speed, time)
                #rospy.sleep(0.5)
            print("Sending MOVE ENDED *********    i = ", len(self.recorded_positions)) 
                
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def recording_loop(self):
        if self.front_button_pressed:
            self.front_button_pressed = False
            self.stop_record_timer()
            if (self.recording_started):
                self.recording_started = False
                print(self.recorded_positions)
                print("###### RECORDING STOPPED ######")
            else:
                self.disable_arm_stiffness()
                print("Move to initial position")
                rospy.sleep(1.0)
                print("RECORDING STARTED")
                self.recorded_positions = []
                self.start_record_timer()
                self.recording_started = True

        if self.rear_button_pressed and not self.recording_started:
            self.rear_button_pressed = False
            self.enable_arm_stiffness()
            self.move_arm()

if __name__ == '__main__':

    rospy.init_node('main_controller', anonymous=True)

    try:
        main_controller = Main_leonao_controller()
        #rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            main_controller.recording_loop()
            #rate.sleep()

    except rospy.ROSInterruptException:
        pass