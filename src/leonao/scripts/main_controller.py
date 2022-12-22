#!/usr/bin/env python
## Main application controller

import rospy

class Main_leonao_controller():
    def __init__(self):
        self.record_timer = rospy.Timer(rospy.Duration(0.125), self.record_timer_callback, oneshot=False)
        self.record_timer.shutdown()

        self.recording_started = False
        #self.timer_record.run()

    def record_timer_callback(self, timer_event):
        print("Entered record_timer_callback")



if __name__ == '__main__':

    rospy.init_node('main_controller', anonymous=True)

    try:
        Main_leonao_controller()
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            hello_str = "hello world %s" % rospy.get_time()
            rospy.loginfo(hello_str)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass