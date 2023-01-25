#!/usr/bin/env python
## Example to setup the frameworks for the canvas

import ssl
import rospy
import os

from canvas import * 

class Drawing_setup_tester():
    def __init__(self):

        self.canvas = Canvas()

        self.start = [0, 0]

    def main_loop(self):
        y = float(input("Enter y value in cm")) / 100
        z = float(input("Enter z value in cm")) / 100
        print("y: ", y)
        print("z: ", z)
        
        end = [y, z]
        self.canvas.draw_line(self.start, end)

        self.start = end

        
if __name__ == '__main__':
    rospy.init_node('drawing_setup_example', anonymous=True)
    tester = Drawing_setup_tester()
    
    try:
        while not rospy.is_shutdown():
            tester.main_loop()
            pass
    except rospy.ROSInterruptException:
        pass

    tester.disable_arm_stiffness()     