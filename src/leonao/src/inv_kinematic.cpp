#include <iostream>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <ros/ros.h>
#include <urdf/model.h>
// #include <almath/tools/almath.h>
// #include <almath/tools/almathio.h>
// #include <almath/tools/altrigonometry.h>

// Include the service message header
#include <leonao/Nao_RArm_chain_get_angles.h>  
#include <leonao/Nao_RArm_chain_get_transform.h>

class InverseKinematics {
    public:
        KDL::Tree kdl_tree_;
        InverseKinematics() {
            create_tree();
        }
        void create_tree() {
            urdf::Model robot_model;
            std::string xml_string;
            ros::param::get("robot_description", xml_string);
            if (!robot_model.initString(xml_string)) {
                ROS_ERROR("Failed to parse URDF");
            }
            if (!kdl_parser::treeFromUrdfModel(robot_model, kdl_tree_)) {
                ROS_ERROR("Failed to create KDL tree.");
            } else {
                ROS_INFO("Chain created successfully.");
            }
        }
        KDL::Chain create_chain(const std::string& base_link, const std::string& end_link) {
            KDL::Chain chain;
            if (!kdl_tree_.getChain(base_link, end_link, chain)) {
                ROS_ERROR("Failed to create KDL chain.");
            } else {
                ROS_INFO("Root link: %s, End link: %s", base_link.c_str(), end_link.c_str());
                print_chain(chain);
            }
            return chain;
        }
        void print_chain(const KDL::Chain& chain) {
            std::cout << "Number of segments: " << chain.getNrOfSegments() << std::endl;
            std::cout << "Number of joints: " << chain.getNrOfJoints() << std::endl;
            for (unsigned int i = 0; i < chain.getNrOfSegments(); i++) {
                std::cout << chain.getSegment(i).getJoint().getName() << std::endl;
            }
        }

        std::vector<double> get_end_link_position(const KDL::Chain& chain, const std::vector<double>& joint_angles) {
             // set the values of the joints
            KDL::JntArray pykdl_joint_array(chain.getNrOfJoints());

            for (unsigned int i = 0; i < chain.getNrOfJoints(); i++)
            {
                pykdl_joint_array(i) = joint_angles[i];
            }

            // compute end_effector_pose
            KDL::ChainFkSolverPos_recursive solver(chain);
            KDL::Frame end_effector_pose;
            solver.JntToCart(pykdl_joint_array, end_effector_pose);

            std::vector<double> position6D;
            std::cout << "end_effector_pose.p[0]" << end_effector_pose.p[0] << std::endl;
            position6D.push_back(end_effector_pose.p[0]);
            position6D.push_back(end_effector_pose.p[1]);
            position6D.push_back(end_effector_pose.p[2]);
            double wx, wy, wz;
            end_effector_pose.M.GetRPY(wx, wy, wz);
            position6D.push_back(wx);
            std::cout << "wx" << wx << std::endl;
            position6D.push_back(wy);
            position6D.push_back(wz);
            return position6D;
        }
        std::vector<double> get_end_link_transform(const KDL::Chain& chain, std::vector<double>& joint_angles) {
            // set the values of the joints
            KDL::JntArray pykdl_joint_array(chain.getNrOfJoints());

            for (unsigned int i = 0; i < chain.getNrOfJoints(); i++)
            {
                pykdl_joint_array(i) = joint_angles[i];
            }

            // compute end_effector_pose
            KDL::ChainFkSolverPos_recursive solver(chain);
            KDL::Frame end_effector_pose;
            solver.JntToCart(pykdl_joint_array, end_effector_pose);

            std::vector<double> transform;
            transform.push_back(end_effector_pose.M[0])
            transform.push_back(end_effector_pose.M[1])
            transform.push_back(end_effector_pose.M[2])
            transform.push_back(end_effector_pose.p[0])
            transform.push_back(end_effector_pose.M[3])
            transform.push_back(end_effector_pose.M[4])
            transform.push_back(end_effector_pose.M[5])
            transform.push_back(end_effector_pose.p[1])
            transform.push_back(end_effector_pose.M[6])
            transform.push_back(end_effector_pose.M[7])
            transform.push_back(end_effector_pose.M[8])
            transform.push_back(end_effector_pose.p[2])
             return transform;
        }

        // void get_pose(const KDL::Chain& chain, const KDL::JntArray& q_init, const AL::Transform& target, KDL::JntArray& q_out) {
        //     // create solver
        //     KDL::ChainIkSolverPos_LMA ik_solver(chain);
        //     // set solver parameters
        //     ik_solver.setLambda(0.1);
        //     ik_solver.setEpsilon(1e-6);
        //     ik_solver.setMaxIterations(100);
        //     // create target frame
        //     KDL::Frame frame_target;
        //     frame_target.p.x(target.x);
        //     frame_target.p.y(target.y);
        //     frame_target.p.z(target.z);
        //     frame_target.M = KDL::Rotation::Quaternion(target.q0, target.q1, target.q2, target.q3);
        //     // solve
        //     int ret = ik_solver.CartToJnt(q_init, frame_target, q_out);
        //     if (ret < 0) {
        //         ROS_ERROR("IK failed.");
        //     }
        // }

        KDL::Frame get_pose(std::vector<double> position6d)
        {
            KDL::Frame pose;
            pose.p = KDL::Vector(position6d[0], position6d[1], position6d[2]);
            pose.M = KDL::Rotation::RPY(position6d[3], position6d[4], position6d[5]);
            return pose;
        }
        // KDL::Frame get_end_link_pose(const KDL::Chain& chain, const std::vector<double>& joint_angles)
        // {
        //     // set the values of the joints
        //     KDL::JntArray pykdl_joint_array(chain.getNrOfJoints());

        //     for (unsigned int i = 0; i < chain.getNrOfJoints(); i++)
        //     {
        //         pykdl_joint_array(i) = joint_angles[i];
        //     }

        //     // compute end_effector_pose
        //     KDL::ChainFkSolverPos_recursive solver(chain);
        //     KDL::Frame end_effector_pose;
        //     solver.JntToCart(pykdl_joint_array, end_effector_pose);

        //     return end_effector_pose;
        // }

        std::vector<double> get_joints_angles(const KDL::Chain& chain, const KDL::Frame& end_effector_pose, std::vector<double> joints_angles_initial_guess) {
            // convert initial guess to array
            KDL::JntArray joints_array_initial_guess = KDL::JntArray(chain.getNrOfJoints());
            for (int i = 0; i < chain.getNrOfJoints(); i++) {
                joints_array_initial_guess(i) = joints_angles_initial_guess[i];
            }

            // Create an FK solver for the chain
            // KDL::ChainFkSolverPos_recursive fk_p_solver(chain);

            // Create an IK solver for the chain
            // KDL::ChainIkSolverVel_pinv ik_v_solver(chain);
            // KDL::ChainIkSolverPos_NR ik_p_solver(chain, fk_p_solver, ik_v_solver);
            Eigen::Matrix<double,6,1> mask;
            mask << 1, 1, 1, 0, 0, 0;
            auto ik_p_solver = KDL::ChainIkSolverPos_LMA(chain, mask);

            // Compute the inverse kinematics
            KDL::JntArray joints_array = KDL::JntArray(chain.getNrOfJoints());
            int ik_status = ik_p_solver.CartToJnt(joints_array_initial_guess, end_effector_pose, joints_array);

            if (ik_status < 0) {
                std::cout << "IK solution not found" << std::endl;
            }

            // convert joint array to list
            std::vector<double> joints_angles;
            for (int i = 0; i < chain.getNrOfJoints(); i++) {
                joints_angles.push_back(joints_array(i));
                std::cout << "joints_angles: i="<< i << "   " <<joints_angles[i] << std::endl;
            }

            return joints_angles;
        }
};

class Nao_RArm_chain {
    public:
        Nao_RArm_chain() {
            ik = new InverseKinematics();
            auto base_link = "torso";
            auto end_link = "r_gripper";
            arm_chain = ik->create_chain(base_link, end_link);

            auto joint = KDL::Joint("PenTip");
            auto frame = KDL::Frame(KDL::Vector(0.08,0.0,0.035));
            auto segment = KDL::Segment("PenTip",joint,frame);
            arm_chain.addSegment(segment);
            ik->print_chain(arm_chain);
            for(int i = 0; i < arm_chain.getNrOfJoints(); i++) {
                prev_joints_angles.push_back(0.0);
            }
        }

        std::vector<double> get_joint_angles_from_pose(KDL::Frame &pose) {
            std::vector<double> joints_angles = ik->get_joints_angles(arm_chain, pose, prev_joints_angles);
            for(int i = 0; i < arm_chain.getNrOfJoints(); i++) {
                prev_joints_angles[i] = joints_angles[i];
                //std::cout << "prev_joints_angles: i="<< i << "   " <<prev_joints_angles[i] << std::endl;
            }
            return joints_angles;
        }

        std::vector<double> get_angles(std::vector<double> position6D) {
            // Define the desired end-effector pose
            KDL::Frame end_effector_pose = ik->get_pose(position6D);
            std::vector<double> joints_angles = get_joint_angles_from_pose(end_effector_pose);
            return joints_angles;
        }

        // double* get_angles_from_transform(AL::Transform transform) {
        //     // Define the desired end-effector pose
        //     PyKDL::Frame pose;
        //     pose.p[0] = transform.r1_c4;
        //     pose.p[1] = transform.r2_c4;
        //     pose.p[2] = transform.r3_c4;

        //     PyKDL::Rotation R(transform.r1_c1, transform.r1_c2, transform.r1_c3, \
        //         transform.r2_c1, transform.r2_c2, transform.r2_c3, \
        //         transform.r3_c1, transform.r3_c2, transform.r3_c3);
        //     pose.M = R;

        //     std::cout << "get_angles_from_transform: " << pose << std::endl;
        //     return get_joint_angles_from_pose(pose);
        // }

        std::vector<double> get_position(std::vector<double> angles) {
            return ik->get_end_link_position(arm_chain, angles);
        }

        AL::Transform get_transform(std::vector<double> angles) {
             return ik->get_end_link_transform(arm_chain, angles);
        }

        InverseKinematics* ik;
        KDL::Chain arm_chain;
        std::vector<double> prev_joints_angles;
};

Nao_RArm_chain* arm_chain_p;

bool getAnglesCallback(leonao::Nao_RArm_chain_get_angles::Request& req,
                       leonao::Nao_RArm_chain_get_angles::Response& res) {
    // Convert the request position vector to a KDL::Frame
    std::vector<double> position6D;
    for (int i = 0; i < req.position6D.size(); i++)
    {
        position6D.push_back(req.position6D[i]);
        // std::cout << "req.position6D: i="<< i << "   " <<req.position6D[i] << std::endl;
    }
    
    std::vector<double> angles = arm_chain_p->get_angles(position6D);
    std::vector<double> new_position6D = arm_chain_p->ik->get_end_link_position(arm_chain_p->arm_chain, angles);
    
    for (int i = 0; i < new_position6D.size(); i++)
    {
        std::cout << "new_position6D "<< new_position6D[i] << std::endl;
    }
    for (int i = 0; i < angles.size(); i++)
    {
        res.angles.push_back(angles[i]);  // Set the response angles
    }
    
    return true;
}

bool getTransformCallback(leonao::Nao_RArm_chain_get_transform::Request& req,
                       leonao::Nao_RArm_chain_get_transform::Response& res) {
    
    std::vector<double> angles;
    for (int i = 0; i < req.angles.size(); i++)
    {
        angles.push_back(req.angles[i]);
    }
    
    std::vector<double> transform = arm_chain_p->get_end_link_transform(angles);
    
    for (int i = 0; i < transform.size(); i++)
    {
        res.angles.push_back(angles[i]);
    }
    
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "nao_kdl");
    ros::NodeHandle nh;
    ros::Rate rate(10); // 10hz

    arm_chain_p = new Nao_RArm_chain();

    ros::ServiceServer service = nh.advertiseService("Nao_RArm_chain_get_angles", &getAnglesCallback);
    ros::ServiceServer service = nh.advertiseService("Nao_RArm_chain_get_transform", &getTransformCallback);
    // Nao_RArm_motion_proxy arm_motion_proxy;

    // std::vector<float> measured_position6D = arm_motion_proxy.get_position();
    // std::vector<float> measured_joints = arm_motion_proxy.get_angles();
    // std::vector<float> calculated_position6D = arm_chain.get_position(measured_joints);
    // std::cout << "measured_joints: " << measured_joints << std::endl;
    // std::cout << "measured_position6D: " << measured_position6D << std::endl;
    // std::cout << "calculated_position6D: " << calculated_position6D << std::endl;

    // std::vector<float> new_joints = arm_chain.get_angles(calculated_position6D);
    // std::vector<float> new_position6D = arm_chain.get_position(new_joints);
    // std::cout << "new_joints" << new_joints << std::endl;
    // std::cout << "new_position6D: " << new_position6D << std::endl;

    ros::spin();
    return 0;
}