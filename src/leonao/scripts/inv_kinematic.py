#!/usr/bin/env python
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from urdf_parser_py.urdf import URDF
from PyKDL import *#Chain, Tree
import rospy

# path to URDF file
#path_to_urdf_file = "/opt/ros/kinetic/share/nao_description/urdf/nao_robot_v4_structure.urdf.xacro"

class InverseKinematics:
    def __init__(self):
        pass





def main():
    rospy.init_node('nao_kdl', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    # get URDF model from parameter server
    urdf_str = rospy.get_param("robot_description")

    # create kdl tree from URDF
    robot = URDF.from_xml_string(urdf_str)
    #print(robot)
    kdl_tree = kdl_tree_from_urdf_model(robot)

    # check if the tree is created successfully
    if kdl_tree is None:
        rospy.logerr("Failed to create KDL tree.")
    else:
        print(kdl_tree.getNrOfSegments())
        print(kdl_tree.getNrOfJoints())
        base_link = robot.get_root()
        end_link = 'r_wrist'
        chain = kdl_tree.getChain(base_link, 'r_wrist')
        print("Root link: %s; Random end link: %s" ,base_link, end_link)
        print(chain.getNrOfSegments())
        print(chain.getNrOfJoints())
        for i in range(chain.getNrOfSegments()):
            print(chain.getSegment(i).getJoint())
            #print(chain.getJoint(i).getName())
        rospy.loginfo("KDL tree created successfully.")
        
    while not rospy.is_shutdown():
        # do any other calculations here
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass