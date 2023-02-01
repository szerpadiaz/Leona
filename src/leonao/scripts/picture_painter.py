#!/usr/bin/env python
## Picture painter class, it should use the canvas to provide painting services to main-controller

import rospy

from canvas import * 

WATCHFOLDER_PATH = "/home/hrsa/leonao/src/leonao/watchfolder/"


class Picture_painter():
    def __init__(self):

        self.canvas = Canvas()
        
    def load_face_path_from_pkl(self, filename):
        #abs_path = os.path.dirname(os.path.abspath(__file__)) + "/../watchfolder/" + filename
        abs_path = WATCHFOLDER_PATH + filename
        # Open the file for reading
        config_file = open(abs_path, "rb")
        # Load the tuple of points from the file
        face_path = pickle.load(config_file)
        # Close the file
        config_file.close()
        return face_path
        
    def draw_face(self, filename_face):

        hight_scale_factor = 0.20
        width_scale_factor = 0.15
        y_offset = 0.025

        self.canvas.go_to_point([0,0])

        self.face_path = self.load_face_path_from_pkl(filename_face)
        paths = [self.face_path['inner'], self.face_path['outer']]
        for group in paths:
           for i, path in enumerate(group):
               scaled_path = []
               for point in path:
                   scaled_path.append([-point[0]* width_scale_factor - y_offset, point[1] * hight_scale_factor])
               
               self.canvas.draw_path(scaled_path)

        self.canvas.go_to_point([0,0])
