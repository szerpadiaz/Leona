#!/usr/bin/env python
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from urdf_parser_py.urdf import URDF
from PyKDL import *#Chain, Tree
import rospy

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

        print("Root link: %s; end link: %s" ,base_link, end_link)
        print(chain.getNrOfSegments())
        print(chain.getNrOfJoints())
            
        for i in range(chain.getNrOfSegments()):
            print(chain.getSegment(i).getJoint())
            #print(chain.getJoint(i).getName())

        return chain
    
    def get_end_link_position(self, chain, joint_angles):
        assert len(joint_angles) >= chain.getNrOfJoints()

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
        _, _, _, quaternion = np.linalg.svd(R)
        wx, wy, wz, ww = quaternion
        position6d = [x, y, z, wx, wy, wz]
        print(position6d)

        return position6d

    def get_joints_angles(self, chain, position6d, joints_angles_initial_guess):
        
        # Define the desired end-effector position
        end_effector_position = PyKDL.Vector(position6d[0], position6d[1], position6d[2])

        # use initial guess from previous value? or all zeros?
        joints_array_initial_guess = PyKDL.JntArray(chain.getNrOfJoints())

        # Create an FK solver for the chain
        fk_p_solver = PyKDL.ChainFkSolverPos_recursive(chain)

        # Create an IK solver for the chain
        ik_v_solver = PyKDL.ChainIkSolverVel_pinv(chain) #,ik_solver_init_pos?
        ik_p_solver = PyKDL.ChainIkSolverPos_NR(chain, fk_p_solver, ik_v_solver)

        # ChainIkSolverPos_NR iksolver1(chain1,fksolver1,iksolver1v,100,1e-6) ?
        # Maximum100 iterations, stop at accuracy 1e-6 ?

        # Compute the inverse kinematics
        joints_array = PyKDL.JntArray(chain.getNrOfJoints())
        ik_status = ik_p_solver.CartToJnt(joints_array, end_effector_position, joints_array_initial_guess)
        # ik_p_solver.CartToJnt(q_init,F_dest,q) ?

        if  ik_status >= 0:
            # The IK solution is stored in the joints_array variable
            print(joints_array)
        else:
            print("IK solution not found")

        #convert joint array to list?
        joints_angles = joints_array
        return joints_angles
            
        
def main():
    rospy.init_node('nao_kdl', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    ik = InverseKinematics()
    base_link = 'torso'
    end_link = 'r_wrist'
    arm_chain = ik.create_chain(base_link, end_link)
    while not rospy.is_shutdown():
        # do any other calculations here
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass