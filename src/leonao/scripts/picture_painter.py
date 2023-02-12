#!/usr/bin/env python

"""
ROS node providing painting services to the main-controller. 
- subscribes to the /draw_path topic and listens for a message to draw a face (by using a canvas object)
- publishes to the painting_done topic once the face has been drawn.
"""

import rospy
from std_msgs.msg import String, Empty
from canvas import * 

WATCHFOLDER_PATH = "/home/hrsa/leonao/src/leonao/watchfolder/"


class Picture_painter():
    def __init__(self):
        """
        Initialize the Picture_painter class. It creates an instance of the 
        Canvas class and sets up the subscribers and publishers for the 
        draw_path topic and painting_done topic respectively.
        """
        self.canvas = Canvas()
        self.draw_path_sub = rospy.Subscriber('/draw_path', String, self.draw_path_callback, queue_size=1)
        self.painting_done_pub = rospy.Publisher('painting_done', Empty, queue_size=1)

    def draw_path_callback(self, data):
        """
        Callback function for /draw_path topic
        Calls draw_face and publishes on the painting_done topic when painting is done
        :param data: message received which contains the name of pickle file of the face paths to be drawn
        """
        filename_face = data.data
        self.draw_face(filename_face)
        self.painting_done_pub.publish()

    def load_face_path_from_pkl(self, filename):
        """
        Loads face path data from the specified pickle file
        :param filename: name of the pickle file containing the face path data
        Returns: a dict with the face path data
        """
        abs_path = filename
        config_file = open(abs_path, "rb")
        face_path = pickle.load(config_file)
        config_file.close()
        return face_path
        
    def draw_face(self, filename_face):
        """
        Draws the face specified in the file. It loads both the inner and outer face paths, scales the path, 
        and reduces close points. The final path is then drawn using the canvas.
        :param filename_face: name of the pickle file containing the face path data
        """
        hight_scale_factor = 0.20
        width_scale_factor = 0.15
        y_offset = 0.00

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
        """
        Reduce the number of points in a given path based on the given distance.
        :param path: a list of points, where each point is a tuple with two values (x, y).
        :param distance: the minimum distance (in meters) between two consecutive points. 
        Returns the new path with the reduced number of points.
        """
        print("Reducing close points, closer than " + str(distance) + "m")
        print("Number of points before: " + str(len(path)))
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
        
        print("Number of points after: " + str(len(new_path)))
        return new_path
    
    def distance(self, p1, p2):
        """
        Calculates the Euclidean distance between two points.
        :param p1: a tuple representing a point with two values (x, y).
        :param p2: a tuple representing a point with two values (x, y).
        Returns the Euclidean distance between the two points.
        """
        return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
    

if __name__ == '__main__':
    rospy.init_node('picture_painter', anonymous=True)
    try:
        picture_painter = Picture_painter()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("picture_painter: FAILED")
