#!/usr/bin/env python
## Main application controller

import rospy

def main_controller():
    rospy.init_node('main_controller', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        main_controller()
    except rospy.ROSInterruptException:
        pass