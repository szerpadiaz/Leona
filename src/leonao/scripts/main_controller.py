#!/usr/bin/env python
## Main application controller

import rospy
from naoqi_bridge_msgs.msg import HeadTouch
from leonao.srv import GetCartesianCoordinates, Stiffness

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

    def recording_loop(self):
        if self.front_button_pressed:
            self.front_button_pressed = False
            print("Front button pressed down")
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

        if self.rear_button_pressed:
            self.rear_button_pressed = False
            print("Rear button pressed down")
            self.enable_arm_stiffness()
            
        #     std::cout << "Sending MOVE !!!!!!!!!" << std::endl;
        #     int i = 0;
        #     for(const auto& pos6D : recorded_positions){
        #         i++;
        #         nao_control_tutorial_2::MoveJoints srvMove;
        #         srvMove.request.effector_name = "RArm";
        #         std::vector<float> pos{pos6D[0], pos6D[1], pos6D[2]};
        #         srvMove.request.position = pos;
        #         std::vector<float> orient{pos6D[3], pos6D[4], pos6D[5]};
        #         srvMove.request.orientation = orient;
        #         //srvMove.request.speed = 0.2;
        #          srvMove.request.time = 0.1;
        #         move_client.call(srvMove);
        #         //ros::Duration(0.5).sleep();
        #     }

        #     std::cout << "Sending MOVE ENDED *********    i = " << i << std::endl;  
        # }

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