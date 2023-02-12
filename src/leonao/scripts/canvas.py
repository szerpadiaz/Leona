#!/usr/bin/env python
"""
The canvas class configures and moves the right arm of a NAO robot to draw on a canvas.

- Uses the ALMotion, ALProxy and numpy modules, as well as a service to compute the inverse kinematics (leonao.srv)
- Initializes the robot, enables arm stiffness, and sets the speed of motion.
- Allows the configuration of the drawing plane, either by collecting three points in space to calculate the plane, or 
  by loading the plane data from a saved file. 
- Allows drawing geometric shapes such as lines, rectangles and ellipses
- Allows drawing a customised shaped given as a path of line segments (list of points specified as x,y tuples)
"""

from curses.textpad import rectangle
from locale import normalize
from pickle import TRUE
from re import T
import rospy
import os
import numpy as np

import almath
from naoqi import ALProxy
import math
import tf
import pickle

from leonao.srv import Nao_RArm_chain_get_angles, Nao_RArm_chain_get_transform

# Physical setup reference frames:
# - BASE-FRAME    : the frame attached to the NAO's Torso (in our case is fixed, the robot is not moving)
# - TOOL-FRAME    : the frame attached to the tip of the marker
# - STATION-FRAME : the frame attached to the Table where Leonao is painting (this is our workstation universal reference)

class Canvas():
    def __init__(self):
        """
        Sets up the canvas for drawing.
        It initializes the necessary variables, proxies, drawing plane configuration 
        and the robot's joints for drawing.
        """
        robot_ip=str(os.getenv("NAO_IP"))
        robot_port=int(9559)
        self.motion_proxy = ALProxy("ALMotion", robot_ip, robot_port)
        
        # while 1:
        #     joints_names = self.motion_proxy.getBodyNames("RArm")
        #     joints_angles = self.motion_proxy.getAngles(joints_names, True)
        #     print(joints_angles)
        #     input = raw_input("Press c to break.")
        #     if input == "c":
        #         break

        self.enable_arm_stiffness()
        rospy.sleep(2.0)

        self.x_drawing_plane = 0.0 #0.005 v2_turned
        self.x_go_to_point = 0.03
        self.speed = 0.4
        self.configure_drawing_plane()

        # Get current point and move to new origin?
        #raw_input("To go to origin, press enter.")
        self.go_to_point([0, 0])

    def configure_drawing_plane(self):
        """
        Configures the drawing plane by either loading pre-defined plane points from a file or getting new plane points from the user.
        
        If the user chooses to configure the plane, the get_configuration method is called to get 3 points that define the plane. 
        The points are then saved to a file for future use.
        If the user chooses not to configure the plane, the method tries to load the plane points from a file.
        """
        choice = raw_input("Configure drawing plane? (y,n) ")[0].lower()
        #choice = 'n'
        if(choice == 'y'):
            plane_point_1, plane_point_2, plane_point_3 = self.get_configuration()
            self.save_config([plane_point_1, plane_point_2, plane_point_3])
        else:
            plane_points = self.load_config()
            if(plane_points):
                plane_point_1 = plane_points[0]
                plane_point_2 = plane_points[1]
                plane_point_3 = plane_points[2]
            else:
                print("There are not values to load, we  have to do the configuration")
                plane_point_1, plane_point_2, plane_point_3 = self.get_configuration()

        self.calculate_drawing_plane(plane_point_1, plane_point_2, plane_point_3)

    def save_config(self, plane_points):
        """
        Saves the plane points to a configuration file.
        :param plane_points: list of the three points in the plane
        """
        # Path to the plane config file (3 points that define the drawing plane)
        abs_path = os.path.dirname(os.path.abspath(__file__)) + "/../config/plane_config.pkl"
        # Open a file for writing
        config_file = open(abs_path, "wb")
        # Dump the tuple of points to the file
        pickle.dump(plane_points, config_file)
        # Close the file
        config_file.close()

    def load_config(self):
        """
        Loads the plane points from the configuration file.
        Return the plane_points
        """
        # Path to the plane config file (3 points that define the drawing plane)
        abs_path = os.path.dirname(os.path.abspath(__file__)) + "/../config/plane_config.pkl"
        # Open the file for reading
        config_file = open(abs_path, "rb")
        # Load the tuple of points from the file
        plane_points = pickle.load(config_file)
        # Close the file
        config_file.close()
        return plane_points

    def move_to_plane_point(self, i, start_point, end_point):
        """
        Move the robot to the i-th point on the plane.
        param: i: the index of the point on the plane.
        param: start_point: a list of 3 float values representing the starting point in 3D space.
        param: end_point: a list of 3 float values representing the ending point in 3D space.
        """
        # Read 1st point/origin
        print("Moving to point #", i ," of the plane.")
        line_path = self.calculate_line_path_3D(start_point, end_point)
        angles_list = []
        for point in line_path:
            angles = self.get_angles(point)
            angles_list.append(angles)
        self.move_joints(angles_list)
        raw_input("Press enter to continue.")
        

    def get_configuration(self):
        """
        Get 3D points from the user for plane configuration
        Return the points provided by the user 
        """
        success = False
        x_offset = 0.0

        plane_point_1 = [0.247 - x_offset, -0.17, 0.05]
        plane_point_2 = [0.242 - x_offset, -0.125, 0.23]
        plane_point_3 = [0.248 - x_offset, 0.045, 0.14]
        plane_point_0 = plane_point_1    
        while not success:
            # Read 1st point/origin
            self.move_to_plane_point(1, plane_point_0, plane_point_1)

            # Read 2nd point
            self.move_to_plane_point(2, plane_point_1, plane_point_2)

            # Read 3rd point
            self.move_to_plane_point(3, plane_point_2, plane_point_3)
            
            plane_point_0 = plane_point_3
            success = 'y' != raw_input("Move again? (y,n) ")[0].lower()

        return plane_point_1, plane_point_2, plane_point_3

    # def get_configuration(self):
    #     """
    #     Get 3D points from the user for plane configuration, by manually
    #     moving the pen-tip the 3 plane points.
    #     Return the points provided by the user 
    #     """
    #     raw_input("Move to pen to initial position. Press enter to start.")
    #     
    #     # T_bs: Transformation from the STATION-FRAME (s) to the BASE-FRAME (b)
    #     joints_names = self.motion_proxy.getBodyNames("RArm")
    #     joints_angles = self.motion_proxy.getAngles(joints_names, True)
    #     self.T_bs = self.get_transform_client(joints_angles)
    
    #     # Read 1st point
    #     raw_input("Move to 1st point of the plane. Press enter if done.")
    #     plane_point_1 = self.get_current_point()
    #     print("1st point read. ", plane_point_1)
        
    #     # Read 2nd point
    #     raw_input("Move to 2nd point of the plane. Press enter if done.")
    #     plane_point_2 = self.get_current_point()
    #     print("2nd point read. ", plane_point_2)
        
    #     # Read 3rd point
    #     raw_input("Move to 3rd point of the plane. Press enter if done.")
    #     plane_point_3 = self.get_current_point()
    #     print("3rd point read. ", plane_point_3)

    #     return plane_point_1, plane_point_2, plane_point_3

    # def get_current_point(self):
    #     """
    #     Returns the current point of the pen-tip w.r.t. the base frame.
    #     """
    #     joint_names = self.motion_proxy.getBodyNames("RArm")
    #     joint_angles = self.motion_proxy.getAngles(joint_names, True)
    #     T_temp = self.get_transform_client(joint_angles)
    #     plane_point = [T_temp.r1_c4, T_temp.r2_c4, T_temp.r3_c4]
    #     plane_point = self.convert_from_base_to_station(plane_point)
    #     return plane_point

    def calculate_line_path_3D(self, start_point, end_point):
        """
        Calculates a path in 3D between the start point and end point.
        :param start_point: lists with x, y, and z coordinates.
        :param end_point: lists with x, y, and z coordinates.
        Returns a list of 3D points along the path which gradually goes from the start to the end point.
        """
        line_path = []
        x0 = start_point[0]
        y0 = start_point[1]
        z0 = start_point[2]
        line_path.append([x0, y0, z0])

        xn = end_point[0]
        yn = end_point[1]
        zn = end_point[2]
        length = math.sqrt((xn-x0)**2 + (yn-y0)**2 + (zn-z0)**2) * 100
        xi = x0
        yi = y0
        zi = z0
        for i in range(1, int(length) + 1):
             xi = (xn - x0) /length * i + x0
             yi = (yn - y0) /length * i + y0
             zi = (zn - z0) /length * i + z0
             line_path.append([xi, yi, zi])

        line_path.append([xn, yn, zn])

        #print("line_path: ", line_path)

        return line_path

    def calculate_drawing_plane(self, plane_point_1, plane_point_2, plane_point_3):
        """
        calculate_drawing_plane - computes the normal and transform matrix of a 3D plane defined by three points
        :param plane_point_1: first point on the plane (list of x, y, and z coordinates)
        :param plane_point_2: second point on the plane (list of x, y, and z coordinates)
        :param plane_point_3: third point on the plane, (list of x, y, and z coordinates)
        """
        plane_point_1 = np.array(plane_point_1)
        plane_point_2 = np.array(plane_point_2)
        plane_point_3 = np.array(plane_point_3)

        # Compute the vectors from point 1 to point 2 and point 3
        vector_12 = np.array(plane_point_2 - plane_point_1)
        vector_13 = np.array(plane_point_3 - plane_point_1)
        #print("vector_12: ", vector_12)
        #print("vector_13", vector_13)
        # Compute the plane normal from with the cross product
        self.plane_normal = np.cross(vector_12, vector_13)
        self.plane_normal = self.plane_normal / np.linalg.norm(self.plane_normal)
        #print("self.plane_normal: ", self.plane_normal)
        # Solve for one point to get offset of the equation
        # self.plane_offset = -plane_point_1[0]*self.plane_normal[0] - plane_point_1[1]*self.plane_normal[1] - plane_point_1[2]*self.plane_normal[2]
        # print("self.plane_offset: ", self.plane_offset)
        v1 = vector_12 / np.linalg.norm(vector_12) #- np.dot(vector_12, self.plane_normal) * self.plane_normal
        v2 = np.cross(v1, self.plane_normal)
        v2 = v2 / np.linalg.norm(v2)
        #print("normal: ", self.plane_normal)
        #print("v2: ", v2)
        #print("v1: ", v1)

        R = np.column_stack((self.plane_normal, v2, v1))
        #print("R: ", R)
        #print("plane_point_1", plane_point_1)

        #euler_angles = np.array(np.rad2deg(np.array(np.matrix(R).flat)), dtype=np.float32)
        euler_angles = tf.transformations.euler_from_matrix(R, 'rxyz')
        #print("euler_angles", np.degrees(euler_angles))

        T_bs = almath.Transform()
        T_bs.r1_c1 = R[0,0]
        T_bs.r1_c2 = R[0,1]
        T_bs.r1_c3 = R[0,2]
        T_bs.r1_c4 = plane_point_1[0]
        T_bs.r2_c1 = R[1,0]
        T_bs.r2_c2 = R[1,1]
        T_bs.r2_c3 = R[1,2]
        T_bs.r2_c4 = plane_point_1[1]
        T_bs.r3_c1 = R[2,0]
        T_bs.r3_c2 = R[2,1]
        T_bs.r3_c3 = R[2,2]
        T_bs.r3_c4 = plane_point_1[2]
        self.T_bs = T_bs


    def calculate_angles(self, x, y, z):
        """
        Calculates the joint angles for the given position of the end-effector
        in the tool frame (goal) with respect to the station frame.
        :param x, y, z: position of the end-effector in the tool frame
        Returns a list of joint angles for the desired position of the end-effector
        """
        # T_sg: Transformation of the GOAL-FRAME (g) with respect to the STATION-FRAME (s)
        T_sg = almath.Transform(x,y,z)

        # T_bg: Transformation of the GOAL-FRAME (g) with respect to the BASE-FRAME (b)
        T_bg = self.T_bs * T_sg
        point = [T_bg.r1_c4, T_bg.r2_c4, T_bg.r3_c4]

        return self.get_angles(point)

    def get_angles(self, point):
        """
        Calculates the joint angles for the given position of the end-effector
        in the tool frame (goal) with respect to the base frame.
        :param point: goal position of the end-effector
        Returns a list of joint angles for the desired position of the end-effector
        """
        position6D = [point[0], point[1], point[2], 0, 0, 0]

        # get angles (using service)
        rospy.wait_for_service('Nao_RArm_chain_get_angles')
        try:
            service_get_angles = rospy.ServiceProxy('Nao_RArm_chain_get_angles', Nao_RArm_chain_get_angles)
            resp = service_get_angles(position6D)
            return resp.angles
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return []

    def get_transform_bt(self):
        """
        Returns the current transformation of the right arm in the base frame (as ALMath.Transform)
        """
        joints_names = self.motion_proxy.getBodyNames("RArm")
        joints_angles = self.motion_proxy.getAngles(joints_names, True)
        rospy.wait_for_service('Nao_RArm_chain_get_transform')
        try:
            service_get_transform = rospy.ServiceProxy('Nao_RArm_chain_get_transform', Nao_RArm_chain_get_transform)
            resp = service_get_transform(joints_angles)
            T = almath.Transform(resp.transform)   
            return T
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return []

    def get_position_bt(self):
        """
        Returns the current position of the robot's right arm end-effector in the base frame.
        """
        T_temp = self.get_transform_bt()
        point = [T_temp.r1_c4, T_temp.r2_c4, T_temp.r3_c4]
        return point

    def move_joints(self, joints_angles_list):
        """
        Moves the robot's right arm to a series of target joint angles.
        :param joints_angles_list: a list of lists.
        Each sublist contains the target angles for the joints in the robot's right arm.
        """
        joint_names = self.motion_proxy.getBodyNames("RArm")
        for target_angles in joints_angles_list:
            # print(target_angles[4])
            self.motion_proxy.angleInterpolationWithSpeed(joint_names[:-1], target_angles, self.speed)
            #rospy.sleep(0.5)

    def disable_arm_stiffness(self):
        self.motion_proxy.stiffnessInterpolation('RArm', 0.0, 1.0)

    def enable_arm_stiffness(self):
        self.motion_proxy.stiffnessInterpolation('RArm', 1.0, 1.0)

    def go_to_point(self, end_point):
        """
        Moves the robot arm to the given point on the canvas.
        It moves along a line from current position to the given end point
        (parallel to the drawing plane, without touching it)
        :param end_point: a 2D list with x and y coordinates
        """
        T_bt = self.get_transform_bt()
        T_sb = self.T_bs.inverse()
        T_st = T_sb * T_bt
        start_point = [T_st.r2_c4, T_st.r3_c4]

        # Move in a line parallel to the drawing plane
        print("GO TO: start_point: ", start_point, " end_point: ", end_point)
        line_path = self.calculate_line_path(start_point, end_point)
        self.move_along_path(self.x_go_to_point, line_path)

    def move_along_path(self, x, path):
        """
        Calculates the joint angles required to move to each point in the path and moves the joints accordingly.
        :param x: the x coordinate of the plane in which to draw.
        :param path: a list of points in the form of [y,z] to be drawn in a line.
        """

        joints_angles_list = []
        for point in path:
            angles_i = self.calculate_angles(x, point[0], point[1])
            if(angles_i):
                joints_angles_list.append(angles_i)

        #print(joints_angles_list)

        if(joints_angles_list):
            self.move_joints(joints_angles_list)

    def draw_path(self, path):
        """
        Moves the arm to the first point in the path, then moves along the path.
        :param path: a list of points in the form of [y,z] to be drawn.
        """

        # Move to the first point in the path (without drawing anything)
        self.go_to_point(path[0])
        # Draw (move in drawing plane)
        self.move_along_path(self.x_drawing_plane, path)

    def draw_line(self, start_point, end_point):
        """
        Draws a line on the canvas.
        :param start_point: The starting point of the line.
        :param end_point: The ending point of the line.
        """
        line_path = self.calculate_line_path(start_point, end_point)
        self.draw_path(line_path)

    def calculate_line_path(self, start_point, end_point):
        """
        Calculates a list of points along the a line from start to end points.
        :param start_point: the starting point of the line in the form of [y,z].
        :param end_point: the ending point of the line in the form of [y,z].
        Returns a list of points in the form of [y,z] to be drawn.
        """
        line_path = []
        y0 = start_point[0]
        z0 = start_point[1]
        line_path.append([y0, z0])

        yn = end_point[0]
        zn = end_point[1]
        length = math.sqrt((yn-y0)**2 + (zn-z0)**2) * 100
        yi = y0
        zi = z0
        for i in range(1, int(length) + 1):
             yi = (yn - y0) /length * i + y0
             zi = (zn - z0) /length * i + z0
             line_path.append([yi,zi])

        line_path.append([yn,zn])

        #print("line_path: ", line_path)

        return line_path

    def draw_ellipse(self, center, a, b):
        """
        Draws an ellipse on the canvas.
        :param center: the center of the ellipse.
        :param a: the semi-major axis of the ellipse.
        :param b: the semi-minor axis of the ellipse.
        """
        ellipse_path = self.calculate_ellipse_path(center, a, b)
        self.draw_path(ellipse_path)

    def calculate_ellipse_path(self, center, a, b):
        """
        Calculate the path of an ellipse.
        :param center: the center of the ellipse, in the form of [y,z].
        :param a: the semi-major axis of the ellipse.
        :param b: the semi-minor axis of the ellipse.
        Returns a list of points in the form of [y,z] to be drawn.
        """

        # max distance between two consecutive points
        max_distance = 0.01
        path = []
        i = 0
        final_i = 0
        while i <= 360:
            z = a * math.cos(math.radians(i))
            y = b * math.sin(math.radians(i))
            path.append([y + center[0], z + center[1]])
            final_i = i
            i = i + math.degrees(math.acos(1- (max_distance**2)/(2*(a**2)) ))
        if final_i < 360:
            final_i = 360 
            z = a * math.cos(math.radians(final_i))
            y = b * math.sin(math.radians(final_i))
            path.append([y + center[0], z + center[1]])
        return path

    def draw_rectangle(self, center, width, height):
        """
        Draws a rectangle on the canvas.
        :param center: the center of the rectangle.
        :param width: the width of the rectangle.
        :param height: the height of the rectangle.
        """
        rectangle_path = self.calculate_rectangle_path(center, width, height)
        self.draw_path(rectangle_path)

    def calculate_rectangle_path(self, center, width, height):
        """
        Calculate the path of a rectangle.
        :param center: the center of the rectangle, in the form of [y,z].
        :param width: the width of the rectangle.
        :param height: the height of the rectangle.
        Returns a list of points in the form of [y,z] to be drawn.
        """
        rectangle_path = []
        y1 = center[0] - width/2.0
        y2 = center[0] + width/2.0
        z1 = center[1] - height/2.0
        z2 = center[1] + height/2.0
        
        point_1 = [y1, z1] 
        point_2 = [y1, z2] 
        point_3 = [y2, z2]
        point_4 = [y2, z1]
        line_1 = self.calculate_line_path(point_1, point_2)
        line_2 = self.calculate_line_path(point_2, point_3)
        line_3 = self.calculate_line_path(point_3, point_4)
        line_4 = self.calculate_line_path(point_4, point_1)
        
        rectangle_path.extend(line_1)
        rectangle_path.extend(line_2)
        rectangle_path.extend(line_3)
        rectangle_path.extend(line_4)
        
        return rectangle_path