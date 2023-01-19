#!/usr/bin/env python

import os 
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from urdf_parser_py.urdf import URDF
import PyKDL
import rospy

import motion
import almath
from naoqi import ALProxy
import numpy as np

#https://www.orocos.org/book/export/html/800.html

# Xarcro

# path to URDF file
#path_to_urdf_file = "/opt/ros/kinetic/share/nao_description/urdf/nao_robot_v4_structure.urdf.xacro"


class InverseKinematics:
    def __init__(self):
        self.create_tree()

    def create_tree(self):
        # get URDF model from parameter server
        urdf_str = rospy.get_param("robot_description")
        # create kdl tree from URDF
        robot = URDF.from_xml_string(urdf_str)
        self.kdl_tree = kdl_tree_from_urdf_model(robot)
        # check if the tree is created successfully
        if self.kdl_tree is None:
            rospy.logerr("Failed to create KDL tree.")
        else:
            rospy.loginfo("Chain created successfully.")

    def create_chain(self, base_link, end_link):
        chain = self.kdl_tree.getChain(base_link, end_link)

        print("Root link: " , base_link, "End link:", end_link)
        
        self.print_chain(chain)

        return chain
    
    def print_chain(self, chain):
        print(chain.getNrOfSegments())
        print(chain.getNrOfJoints())
            
        for i in range(chain.getNrOfSegments()):
            joint_i = chain.getSegment(i).getJoint()
            print(joint_i.getName())

    def get_end_link_position(self, chain, joint_angles):
        # set the values of the joints
        pykdl_joint_array = PyKDL.JntArray(chain.getNrOfJoints())

        for i in range(chain.getNrOfJoints()):
            pykdl_joint_array[i] = joint_angles[i]

        # compute end_effector_pose
        solver = PyKDL.ChainFkSolverPos_recursive(chain) #PyKDL.ChainFkSolverVel_recursive ?
        end_effector_pose = PyKDL.Frame()
        solver.JntToCart(pykdl_joint_array, end_effector_pose)

        # convert end_effector_pose into a Positon6D array
        x, y, z = end_effector_pose.p
        R = end_effector_pose.M
        wx, wy, wz = R.GetRPY()
        
        position6d = [x, y, z, wx, wy, wz]

        return position6d

    def get_joints_angles(self, chain, position6d):
        
        # Define the desired end-effector position
        end_effector_position = PyKDL.Frame()
        end_effector_position.p[0] = position6d[0]
        end_effector_position.p[1] = position6d[1]
        end_effector_position.p[2] = position6d[2]
        end_effector_position.M = PyKDL.Rotation.RPY(position6d[3],position6d[4],position6d[5])

        # use initial guess from previous value? or all zeros?
        # joints_angles_initial_guess
        joints_array_initial_guess = PyKDL.JntArray(chain.getNrOfJoints())

        # Create an FK solver for the chain
        fk_p_solver = PyKDL.ChainFkSolverPos_recursive(chain)

        # Create an IK solver for the chain
        ik_v_solver = PyKDL.ChainIkSolverVel_pinv(chain)
        ik_p_solver = PyKDL.ChainIkSolverPos_NR(chain, fk_p_solver, ik_v_solver)

        # ChainIkSolverPos_NR iksolver1(chain1,fksolver1,iksolver1v,100,1e-6) ?
        # Maximum100 iterations, stop at accuracy 1e-6 ?

        # Compute the inverse kinematics
        joints_array = PyKDL.JntArray(chain.getNrOfJoints())
        ik_status = ik_p_solver.CartToJnt(joints_array_initial_guess, end_effector_position, joints_array)
        # ik_p_solver.CartToJnt(q_init,F_dest,q) ?

        if  ik_status >= 0:
            # The IK solution is stored in the joints_array variable
            print(joints_array)
        else:
            print("IK solution not found")

        #convert joint array to list?
        joints_angles = []
        for i in range(chain.getNrOfJoints()):
            joints_angles.append(joints_array[i])

        return joints_angles
            

class Motion_tester():
    def __init__(self):
        robot_ip=str(os.getenv("NAO_IP"))
        robot_port=int(9559)
        self.motion_proxy = ALProxy("ALMotion", robot_ip, robot_port)
        self.arm_name = "RArm"
        self.arm_joints_names = self.motion_proxy.getBodyNames(self.arm_name) # should be equal to ARM_JOINT_NAMES
        self.arm_joints_limits = self.motion_proxy.getLimits(self.arm_name)
        
        print("arm_joints", self.arm_joints_names)
        self.print_joints_limits(self.arm_joints_names, self.arm_joints_limits)

    def disable_arm_stiffness(self):
        self.motion_proxy.stiffnessInterpolation(self.arm_name, 0.0, 1.0)

    def enable_arm_stiffness(self):
        self.motion_proxy.stiffnessInterpolation(self.arm_name, 1.0, 1.0)

    def get_arm_end_position(self):
        position6d = self.motion_proxy.getPosition(self.arm_name, motion.FRAME_TORSO, True)
        return position6d
    
    def get_arm_joints(self):
        sensorAngles = self.motion_proxy.getAngles(self.arm_joints_names, True)
        return sensorAngles

    def print_joints_limits(self, names, limits):
        for i in range(0,len(limits)):
            print(names[i], ":", \
                "minAngle", limits[i][0],\
                "maxAngle", limits[i][1],\
                "maxVelocity", limits[i][2],\
                "maxTorque", limits[i][3])


def main():
    rospy.init_node('nao_kdl', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    ik = InverseKinematics()
    base_link = 'torso'
    end_link = 'r_gripper'
    arm_chain = ik.create_chain(base_link, end_link)

    joint = PyKDL.Joint('PenTip')
    frame = PyKDL.Frame(PyKDL.Vector(0.08,0.0,0.035))
    segment = PyKDL.Segment('PenTip',joint,frame)
    arm_chain.addSegment(segment)

    ik.print_chain(arm_chain)

    motion_tester = Motion_tester()
    measured_position6D = motion_tester.get_arm_end_position()
    measured_joints = motion_tester.get_arm_joints()
    print("measured_joints: ", measured_joints)
    print("measured_position6D: ", measured_position6D)

    calculated_position6D = ik.get_end_link_position(arm_chain, measured_joints)
    print("calculated_position6D: ", calculated_position6D)

    new_joints = ik.get_joints_angles(arm_chain, calculated_position6D)
    new_position6D = ik.get_end_link_position(arm_chain, new_joints)
    print("new_position6D: ", new_position6D)

    while not rospy.is_shutdown():
        # do any other calculations here
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass