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
        abs_path = filename
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
        y_offset = 0.01

        self.canvas.go_to_point([0,0])

        self.face_path = self.load_face_path_from_pkl(filename_face)
        paths = [self.face_path['inner'], self.face_path['outer']]
        distances = [0.0005, 0.001]
        for j, group in enumerate(paths):
           for i, path in enumerate(group):
                scaled_path = []
                for point in path:
                    scaled_path.append([-point[0]* width_scale_factor - y_offset, point[1] * hight_scale_factor])
                scaled_path = self.reduceClosePoints(scaled_path, distances[j])
                if scaled_path:
                    self.canvas.draw_path(scaled_path)

        self.canvas.go_to_point([0,0])

    def reduceClosePoints(self, path, distance):
        print("Reducing close points, closer than " + str(distance) + "m")
        print("Number of points before: Inside: " + str(len(path))) #+ "points outside: " + str(sum(len(s) for s in paths[1])))
        #for group in paths:
        new_path = []
        first_point = True
        for j, point in enumerate(path):
            if (point[0] < -0.13 and point[1] > 0.17) or point[1] > 18:
                print("deleted point", point)
                continue
            if first_point == True:
                first_point = False
                new_path.append(point)
            else:
                if self.distance(point, new_path[-1]) > distance:
                    new_path.append(point)
        
        print("Number of points after: Inside: " + str(len(new_path))) #+ "points outside: " + str(sum(len(s) for s in paths[1])))
        return new_path
    
    def distance(self, p1, p2):
        return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
    