#!/usr/bin/env python
## Main application controller

import rospy
from naoqi_bridge_msgs.msg import HeadTouch
from leonao.srv import GetCartesianCoordinates

class Main_leonao_controller():
    def __init__(self):
        self.front_button_pressed = False
        self.rear_button_pressed = False
        self.recording_started = False
        self.recorded_positions = []

        self.head_sub = rospy.Subscriber('/tactile_touch', HeadTouch, self.head_touch_callback)
        self.record_timer = None

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

    def head_touch_callback(self, head_touch_event):
        self.front_button_pressed = head_touch_event.button == HeadTouch.buttonFront and head_touch_event.state == HeadTouch.statePressed
        self.rear_button_pressed = head_touch_event.button == HeadTouch.buttonRear and head_touch_event.state == HeadTouch.statePressed

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
            #     nao_control_tutorial_2::Stiffness service;
            #     // service.request.stiffness = 0.0;
            #     // service.request.effector_name = "RArm";
            #     service.request.effector_names.push_back("RShoulderPitch");
            #     service.request.effector_names.push_back("RShoulderRoll");
            #     service.request.effector_names.push_back("RElbowYaw");
            #     service.request.effector_names.push_back("RElbowRoll");
            #     service.request.effector_names.push_back("RWristYaw");
            #     // service.request.effector_names.push_back("RHand");
            #     service.request.stiffness.push_back(0.0);
            #     service.request.stiffness.push_back(0.0);
            #     service.request.stiffness.push_back(0.0);
            #     service.request.stiffness.push_back(0.0);
            #     service.request.stiffness.push_back(0.0);
            #     stiffness_client.call(service);
                print("Move to initial position")
                rospy.sleep(1.0)
                print("RECORDING STARTED")
                self.recorded_positions = []
                self.start_record_timer()
                self.recording_started = True

        if self.rear_button_pressed:
            self.rear_button_pressed = False
            print("Rear button pressed down")

        #     nao_control_tutorial_2::Stiffness srvStiffness;
        #     srvStiffness.request.effector_names.push_back("RArm");
        #     srvStiffness.request.stiffness.push_back(1.0);
        #     stiffness_client.call(srvStiffness);
            
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
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            #hello_str = "hello world %s" % rospy.get_time()
            #rospy.loginfo(hello_str)
            main_controller.recording_loop()
            rate.sleep()
            #rospy.spin_once()

    except rospy.ROSInterruptException:
        pass