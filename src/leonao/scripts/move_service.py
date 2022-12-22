#!/usr/bin/env python
import os
import rospy
import time
import numpy as np
import sys
from leonao.srv import MoveJoints, GetCartesianCoordinates, GetCartesianCoordinatesResponse, Stiffness
from leonao.srv import ConvertFromCameraBottomToTorso, ConvertFromCameraBottomToTorsoResponse
from leonao.srv import RestEndEffector
import roslib
import tf

import almath
from naoqi import ALProxy

motionProxy = 0
FRAME_TORSO = 0 # Task frame FRAME_TORSO = 0, FRAME_WORLD = 1, FRAME_ROBOT = 2
left_arm_rest_position  = [0.09748069196939468, 0.13762640953063965, -0.060107216238975525, -1.1941713094711304, 0.5784283876419067, 0.02633906528353691]
right_arm_rest_position =  [0.11440207064151764, -0.10276402533054352, -0.05344542860984802, 1.4171783924102783, 0.468814879655838, 0.13231989741325378]
fake_position = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]

def MotionProxy_init():
    robotIP=str(os.getenv("NAO_IP"))
    PORT=int(9559)
    global motionProxy
    #print("Initializing ALMotion with (ip, port)", robotIP, str(PORT))
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    MotionProxy_stiffnessInterpolation(['Body'], [1.0], [1.0])

    #postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
    #postureProxy.goToPosture("StandInit", 1.0)    
    #postureProxy.goToPosture("Sit", 1.0)
    #time.sleep(0.2)

    #left_arm_rest_position = motionProxy.getPosition('LArm', FRAME_TORSO, True)
    #right_arm_rest_position = motionProxy.getPosition('RArm', FRAME_TORSO, True)
    #print('left_arm_rest_position', left_arm_rest_position)
    #print('right_arm_rest_position', right_arm_rest_position)

def MotionProxy_setPositions(effector_name, frame_id, position, speed, axis_mask):
    print("Setting position to " + str(effector_name) + " Position" + str(position))
    motionProxy.setPositions(effector_name, frame_id, position, speed, axis_mask)

def MotionProxy_positionInterpolations(effector_name, frame_id, position, axis_mask, time):
    motionProxy.positionInterpolations(req.efector_name, frame_id, position, axis_mask, time)
    print("Setting position (interpolation) to " + str(effector_name) + " Position" + str(position))

def MotionProxy_getPosition(effector_name, frame_id):
    position = motionProxy.getPosition(effector_name, frame_id, True)
    #position = fake_position 
    #fake_position += 1
    return position

def MotionProxy_stiffnessInterpolation(effector_name, stiffness, time):
    #motionProxy.stiffnessInterpolation(effector_name, stiffness, time)
    print("MotionProxy_stiffnessInterpolation: " + str(effector_name) + " - " + str(stiffness))

def MotionProxy_getTransform(from_joint_name, to_ref_frame_id):
    return almath.Transform(motionProxy.getTransform(from_joint_name, to_ref_frame_id, False))
    #return None

def Almath_transform_fromPosition(x,y,z,rx,ry,rz):
    return almath.Transform().fromPosition(x, y, z, rx, ry, rz)
    #return None

def Almath_transformInverse(tf):
    return almath.transformInverse(tf)
    # return None


def rest_end_effector(req):
    axis_mask = 7  # use mask 63 if you want to set the orientation also.
    speed = 0.7
    if req.effector_name == "LArm":
        MotionProxy_setPositions(req.effector_name, FRAME_TORSO, left_arm_rest_position, speed, axis_mask)
    elif req.effector_name == "RArm":
        MotionProxy_setPositions(req.effector_name, FRAME_TORSO, right_arm_rest_position, speed, axis_mask)
    else:
        print("Cannot rest the specified end-effector")
    return True

def convert_from_camera_bottom_to_torso(req):
    # Nomenclature: tf_ToFrom
    # Get transform from CameraTop_optical_frame to Head.
    #   Using: tf tf_echo /CameraBottom_optical_frame /Head
    #     - Translation:              [-0.000, 0.065, -0.057]
    #     - Rotation in RPY (radian): [3.142, -1.550, -1.571]
    tf_ch = Almath_transform_fromPosition(0.000, 0.065, -0.057, 3.142, -1.550, -1.571) # Head /CameraTop_optical_frame 
    tf_hc = Almath_transformInverse(tf_ch)

    # Get transform from Head to torso
    tf_th = MotionProxy_getTransform("Head", FRAME_TORSO) # Head to Torso
    tf_tc = tf_th * tf_hc
    tf_tc = np.reshape(tf_tc.toVector(), (4,4))

    marker_c = np.ones(4)
    marker_c[:3] = req.position_from_camera[:3]
    marker_t = tf_tc.dot(marker_c)

    res = ConvertFromCameraBottomToTorsoResponse()
    res.position_from_torso = marker_t[0:3]
    return res

def get_cartesian_coordinates(req):
    res = GetCartesianCoordinatesResponse()
    res.position = MotionProxy_getPosition(req.effector_name, FRAME_TORSO)
    return res 

def move_end_effector(req):
    print(req)
    axis_mask = 63
    if not req.orientation:
        # If no Orientation specified, use current orientation
        axis_mask = 7
        position = list(req.position) + [0,0,0]
    else:
        position = list(req.position) + list(req.orientation)
    
    position[:3] = getClippedPositionVector(position[:3], req.effector_name)
    br = tf.TransformBroadcaster()
    br.sendTransform(position[:3],[0, 0 , 0, 1.0], rospy.Time.now(), "target_position", "torso")

    print("TARGET POSITION", req.effector_name, position)

    if req.speed > 0.0: # Speed is given
        speed = np.min([req.speed, 1])
        MotionProxy_setPositions(req.effector_name, FRAME_TORSO, position, speed, axis_mask)
    elif req.time > 0.0: # Time is given
        MotionProxy_positionInterpolations(req.effector_name, FRAME_TORSO, position, axis_mask, req.time)
        
    return True

def set_stiffness(req):
    MotionProxy_stiffnessInterpolation(req.effector_name, req.stiffness, [1.0])
    return True

def move_server():
    get_server = rospy.Service("get_cartesian_coordinates", GetCartesianCoordinates, get_cartesian_coordinates)
    move_joint_server = rospy.Service("move_end_effector", MoveJoints, move_end_effector)
    convert_position_server = rospy.Service("convert_from_camera_bottom_to_torso",ConvertFromCameraBottomToTorso, convert_from_camera_bottom_to_torso)
    rest_server = rospy.Service("rest_end_effector", RestEndEffector, rest_end_effector)
    move_joint_server = rospy.Service("set_stiffness", Stiffness, set_stiffness)


def getClippedPositionVector(position, endeffector):
    # If position is outside of reach of the endeffector, calculate a vector that is pointing into that direction (but with maximum possible length)
    arm_length = 0.22 # in m (max. 0.25 m, which is the approx. physical length)

    position = np.array(position)
    if endeffector == "LArm":
        translation = np.array([0.0, 0.095, 0.095])
    elif endeffector == "RArm":
        translation = np.array([0.0, -0.095, 0.095])
    else:
        return position
    position = position - translation
    if np.linalg.norm(position) < arm_length:
        return position + translation    
    else:
        position = position / np.linalg.norm(position) * arm_length + translation
        #print("Target out of range, adjusting to point", position)
        return position.tolist()

if __name__ == '__main__':

    MotionProxy_init()

    rospy.init_node('move_server')
    
    try:
        move_server()
    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupt.")
    
    rospy.spin()
			
		
# Left-Arm extreme positions
#                                 x                     y                     z
# Sitting-rest:          [0.12505659461021423, 0.06373628973960876, -0.012352260760962963, -1.8494623899459839,  0.11859912425279617, -0.35565486550331116]

# extended-to-his-Front: [0.21234837174415588, 0.09861649572849274,  0.04985637962818146,  -1.6371146440505981,  0.23856692016124725, -0.027712734416127205]
# extended-to-his-Right: [0.20623324811458588, 0.035095904022455215, 0.06672549992799759,  -1.6818329095840454,  0.15852829813957214, -0.327205628156662]
# extended-to-his-Left:  [0.19027532637119293, 0.19335415959358215,  0.05194295570254326,  -1.5260448455810547,  0.2318994551897049,   0.43493470549583435]
# extended-down:         [0.07539370656013489, 0.14203672111034393, -0.09998910129070282,  -1.1943954229354858,  1.178049921989441,    0.4702848196029663]
# extended-up:           [0.07621075212955475, 0.07642205059528351,  0.3032306432723999,   -1.3731063604354858, -1.180416226387024,   -0.35208749771118164]

# inwards-middle:        [0.10702996701002121, 0.008671395480632782, 0.08108305931091309,  -1.8121334314346313,  0.03588167950510979, -1.4182108640670776]
# inwards-up:            [0.1142091304063797,  0.015760257840156555, 0.11925718188285828,  -1.8445631265640259, -0.30224984884262085, -1.3431559801101685]
# inwards-down:          [0.08642135560512543, 0.13544097542762756, -0.023914147168397903,  1.6841381788253784,  1.4406911134719849,   1.8346441984176636]