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
        filename_face = "face_paths_david_4.pkl"
        self.face_path = self.load_face_path_from_pkl(filename_face)
        
        hight_scale_factor = 0.20
        width_scale_factor = 0.15
        y_offset = 0.0
        
        print("Size inner path",len(self.face_path['inner']))
        raw_input("Press enter to start drawing.")
        paths = [self.face_path['inner'], self.face_path['outer']]
        for group in paths:
           for i, path in enumerate(group):
               scaled_path = []
               for point in path:
                   scaled_path.append([-point[0]* width_scale_factor - y_offset, point[1] * hight_scale_factor])
                   # scaled_path.append([-point[0]* width_scale_factor, point[1] * hight_scale_factor])

               # print(scaled_path)
               # print("draw path %d/%d", i+1, len(group))
               # raw_input("Press enter to draw")
               
               self.canvas.draw_path(scaled_path)

        self.canvas.go_to_point([0,0])
        
    def load_face_path_from_pkl(self, filename):
        abs_path = os.path.dirname(os.path.abspath(__file__)) + "/../watchfolder/" + filename
        # Open the file for reading
        config_file = open(abs_path, "rb")
        # Load the tuple of points from the file
        face_path = pickle.load(config_file)
        # Close the file
        config_file.close()
        return face_path

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
        pass

if __name__ == '__main__':
    rospy.init_node('drawing_setup_example', anonymous=True)
    tester = Drawing_setup_tester()
    
    try:
        while not rospy.is_shutdown():
            tester.face_loop()
            #x_drawing_plane = float(input("Enter the x_drawing_plane")) / 100
            #tester.canvas.x_drawing_plane = x_drawing_plane
            #tester.rectangle_loop()
            #tester.canvas.draw_rectangle([-0.075, 0.08], 0.15, 0.20)

    except rospy.ROSInterruptException:
        pass