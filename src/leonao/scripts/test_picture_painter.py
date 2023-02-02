#!/usr/bin/env python
## Test picture_painter and canvas

import rospy
import os

from picture_painter import * 

class Test_picture_painter():
    def __init__(self):

        self.painter = Picture_painter()
        
        self.canvas = self.painter.canvas
        self.start = [0, 0]

    def line_loop(self):
        y = float(input("Enter y value in cm")) / 100
        z = float(input("Enter z value in cm")) / 100
        print("y: ", y)
        print("z: ", z)
        
        end = [y, z]
        self.canvas.draw_line(self.start, end)

        self.start = end

    def ellipse_loop(self):
        y = float(input("Enter y center value in cm")) / 100
        z = float(input("Enter z center value in cm")) / 100
        a = float(input("Enter a value in cm")) / 100
        b = float(input("Enter b value in cm")) / 100
        print("y: ", y)
        print("z: ", z)
        print("a: ", a)
        print("b: ", b)
        
        center = [y, z]
        self.canvas.draw_ellipse(center, a, b)
    
    def rectangle_loop(self):
        y = float(input("Enter y center value in cm")) / 100
        z = float(input("Enter z center value in cm")) / 100
        width = float(input("Enter width value in cm")) / 100
        height = float(input("Enter height value in cm")) / 100
        print("y: ", y)
        print("z: ", z)
        print("width: ", width)
        print("height: ", height)
        
        center = [y, z]
        self.canvas.draw_rectangle(center, width, height)
        
    def face_loop(self):
        #filename_face = "ingo_face_paths.pkl"
        #raw_input("Press enter to start drawing: ", filename_face)
        #self.painter.draw_face(filename_face)
        pass

if __name__ == '__main__':
    rospy.init_node('test_picture_painter', anonymous=True)
    tester = Test_picture_painter()
    
    try:
        while not rospy.is_shutdown():
            tester.face_loop()

    except rospy.ROSInterruptException:
        pass