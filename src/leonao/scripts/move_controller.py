#!/usr/bin/env python
## Move controller: wrap access to move server


import rospy
from leonao.srv import GetCartesianCoordinates, Stiffness, MoveJoints

class Move_controller():
    def __init__(self):
        pass

    def set_stiffness(self, effector_names, stiffness_values):
        rospy.wait_for_service('set_stiffness')
        try:
            _set_stiffness = rospy.ServiceProxy('set_stiffness', Stiffness)
            _set_stiffness(effector_names, stiffness_values)
        except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

    def get_position(self, end_effector):
        rospy.wait_for_service('get_cartesian_coordinates')
        try:
            _get_position = rospy.ServiceProxy('get_cartesian_coordinates', GetCartesianCoordinates)
            resp = _get_position(end_effector)
            return list(resp.position)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return []

    def move_end_effector_with_time(self, effector_name, positions, time): 
        rospy.wait_for_service('move_end_effector')
        try:
            _move_end_effector=rospy.ServiceProxy('move_end_effector', MoveJoints)
            speed = 0.0
            for pos6D in positions:
                position = pos6D[:3]
                orientation = pos6D[3:]
                _move_end_effector(effector_name, position, orientation, speed, time)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def move_end_effector_with_speed(self, effector_name, positions, speed): 
        rospy.wait_for_service('move_end_effector')
        try:
            _move_end_effector=rospy.ServiceProxy('move_end_effector', MoveJoints)
            time = 0.0
            for pos6D in positions:
                position = pos6D[:3]
                orientation = pos6D[3:]
                _move_end_effector(effector_name, position, orientation, speed, time)
                rospy.sleep(0.5)            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
