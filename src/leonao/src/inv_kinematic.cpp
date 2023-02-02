#include <iostream>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <ros/ros.h>
#include <urdf/model.h>

// Include the service message header
#include <leonao/Nao_RArm_chain_get_angles.h>  
#include <leonao/Nao_RArm_chain_get_transform.h>

class Nao_RArm_chain {
public:
    Nao_RArm_chain() {
        // Get chain from model
        arm_chain = create_chain("torso", "r_gripper");

        // Add Pen to the end of the chain
        auto joint = KDL::Joint("PenTip");
        auto frame = KDL::Frame(KDL::Vector(0.08,0.0,0.04));
        auto segment = KDL::Segment("PenTip",joint,frame);
        arm_chain.addSegment(segment);
        print_chain_info(arm_chain);

        this->num_of_joints = arm_chain.getNrOfJoints();
        this->prev_joints_angles = KDL::JntArray(this->num_of_joints);

        // this->prev_joints_angles(0) = 0.658128023147583;
        // this->prev_joints_angles(1) = -0.6918759346008301;
        // this->prev_joints_angles(2) = 1.0722241401672363;
        // this->prev_joints_angles(3) = 1.112192153930664;
        // this->prev_joints_angles(4) = 1.4679961204528809;
        // this->prev_joints_angles(5) = 0.23720002174377441;

        // Construct forward and inverse kinematics solvers
        this->fk_p_solver = new KDL::ChainFkSolverPos_recursive(arm_chain);
        //Eigen::Matrix<double,6,1> mask;
        //mask << 1, 1, 1, 0, 0, 0;
        //this->ik_p_solver = KDL::ChainIkSolverPos_LMA(arm_chain, mask);

        // Joint-limits:         ['RShoulderPitch',    'RShoulderRoll',    'RElbowYaw',         'RElbowRoll',        'RWristYaw',         RHand']
        // this->joint_min_limits = {-2.0856685638427734, -1.326450228691101, -2.0856685638427734, 0.03490658476948738, -1.8238691091537476, 0.0};
        // this->joint_max_limits = {2.0856685638427734,  0.3141592741012573,  2.0856685638427734, 1.5446163415908813,   1.8238691091537476, 1.};
        // this->joint_min_limits = {-2.0856685638427734, -1.326450228691101, -2.0856685638427734, 0.03490658476948738, -0.2, 0.0};// 1.1, 0.0};
        // this->joint_max_limits = {2.0856685638427734,  0.3141592741012573,  2.0856685638427734, 1.5446163415908813, -0.07, 1.0}; // 1.4, 1.};
        this->joint_min_limits = {-2.0856685638427734, -1.326450228691101, -2.0856685638427734, 0.03490658476948738, -0.2, 0.0};
        this->joint_max_limits = {2.0856685638427734,  0.3141592741012573,  2.0856685638427734, 1.5446163415908813, 1.82, 1.};
    }

    KDL::Chain create_chain(const std::string& base_link, const std::string& end_link) {
        // Load chain from model
        urdf::Model robot_model;
        std::string xml_string;
        KDL::Tree kdl_tree;
        ros::param::get("robot_description", xml_string);
        if (!robot_model.initString(xml_string)) {
            ROS_ERROR("Failed to parse URDF");
        }
        if (!kdl_parser::treeFromUrdfModel(robot_model, kdl_tree)) {
            ROS_ERROR("Failed to create KDL tree.");
        } else {
            ROS_INFO("Chain created successfully.");
        }

        // Get chain from model
        KDL::Chain chain;
        if (!kdl_tree.getChain(base_link, end_link, chain)) {
            ROS_ERROR("Failed to create KDL chain.");
        } else {
            ROS_INFO("Root link: %s, End link: %s", base_link.c_str(), end_link.c_str());
        }

        return chain;
    }

    void print_chain_info(const KDL::Chain& chain) {
        std::cout << "Number of segments: " << chain.getNrOfSegments() << std::endl;
        std::cout << "Number of joints: " << chain.getNrOfJoints() << std::endl;
        for (unsigned int i = 0; i < chain.getNrOfSegments(); i++) {
             std::cout << chain.getSegment(i).getJoint().getName() << std::endl;
        }
    }

    KDL::JntArray get_angles(const KDL::Frame& end_effector_transform) {
        Eigen::Matrix<double,6,1> mask;
        mask << 1, 1, 1, 0, 0, 0;
        auto ik_p_solver = KDL::ChainIkSolverPos_LMA(arm_chain, mask);

        // Get first angles, using initial-guess as the prev_joints_angles
        auto angles_initial_guess = this->prev_joints_angles;
        KDL::JntArray angles = KDL::JntArray(this->num_of_joints);
        int ik_status = ik_p_solver.CartToJnt(angles_initial_guess, end_effector_transform, angles);

        // Check for angles limits repeat with different inital-guess until convergance (within limits)
        auto iteration = 0;
        const auto max_iter_for_range_check = 100;
        auto num_joints_to_check = this->num_of_joints - 1; //all but the last one
        do {
            auto out_of_range = false;
            for(unsigned int i = 0; i < num_joints_to_check; i++) {
                if(angles(i) < this->joint_min_limits[i] || angles(i) > this->joint_max_limits[i]) {
                    out_of_range = true;
                }
            }

            // for(unsigned int i = 0; i < ik_p_solver.lastSV.size() - 1; i++) {
            //     std::cout << "Singular value: " << i << " - " << ik_p_solver.lastSV[i] << std::endl;
            //     if(ik_p_solver.lastSV[i] < 1e-15){
            //         out_of_range = true;
            //     }
            // }

            if(out_of_range) {
                for(unsigned int i = 0; i < num_joints_to_check; i++) {
                    // Change initial wuess to a random value in between min/2 and max/2 (to aviod singularities at the limits of the joints)
                    angles_initial_guess(i) = this->joint_min_limits[i]/2 + (this->joint_max_limits[i]/2 - this->joint_min_limits[i]/2) * ((double)rand() / (double)RAND_MAX);
                }
            }

            if(out_of_range) {
                ik_status = ik_p_solver.CartToJnt(angles_initial_guess, end_effector_transform, angles);
                iteration++;
            }
            else{
                break;
            }
        }while(iteration < max_iter_for_range_check);

        // if solution is found update the prev_joints_angles
        if (ik_status < 0 || iteration >= max_iter_for_range_check) {
            ROS_INFO("Inverse kinematics failed.");
            return KDL::JntArray();
        } else {
            this->prev_joints_angles = angles;
            return angles;
        }
    }

    KDL::Frame get_transform(const KDL::JntArray& joints) {
        KDL::Frame end_effector_transform;
        int fk_status = this->fk_p_solver->JntToCart(joints, end_effector_transform);
        if (fk_status < 0) {
            ROS_INFO("Forward kinematics failed.");
        }
        return end_effector_transform;
    }

private:
    unsigned int num_of_joints{};
    KDL::JntArray prev_joints_angles;
    KDL::ChainFkSolverPos_recursive * fk_p_solver;
    //KDL::ChainIkSolverPos_LMA *ik_p_solver;

    std::vector<double> joint_min_limits;
    std::vector<double> joint_max_limits;
    
    KDL::Chain arm_chain;
};

Nao_RArm_chain *arm_chain_p;

bool get_angles_callback(leonao::Nao_RArm_chain_get_angles::Request& req,
                       leonao::Nao_RArm_chain_get_angles::Response& res) {
    // Parse request
    KDL::Frame end_effector_transform;
    end_effector_transform.p = KDL::Vector(req.position6D[0], req.position6D[1], req.position6D[2]);
    end_effector_transform.M = KDL::Rotation::RPY(req.position6D[3], req.position6D[4], req.position6D[5]);

    KDL::JntArray angles = arm_chain_p->get_angles(end_effector_transform);

    // Parse response
    for (int i = 0; i < angles.data.size(); i++)
    {
        res.angles.push_back(angles(i));
        //Make sure to check n the client if the response.angles is empty!
    }

    if(angles.data.size() != 0){
         // Debug: calculate position with forward kinematics and print it as position6D
        KDL::Frame transform = arm_chain_p->get_transform(angles);
        auto x = transform.p[0];
        auto y = transform.p[1];
        auto z = transform.p[2];
        double wx, wy, wz;
        transform.M.GetRPY(wx, wy, wz);
        // std::cout << "fk->new_position6D: "<< x  << ", " << y << ", " << z << ", " << wx << ", " << wy << ", " << wz << std::endl;     
    }

    return true;
}

bool get_transform_callback(leonao::Nao_RArm_chain_get_transform::Request& req,
                       leonao::Nao_RArm_chain_get_transform::Response& res) {
    // Parse request
    auto angles = KDL::JntArray(req.angles.size());
    for (int i = 0; i < req.angles.size(); i++)
    {
        angles(i) = req.angles[i];
    }



    KDL::Frame transform = arm_chain_p->get_transform(angles);

    // Parse response
    res.transform.push_back(transform.M(0,0));
    res.transform.push_back(transform.M(0,1));
    res.transform.push_back(transform.M(0,2));
    res.transform.push_back(transform.p[0]);
    res.transform.push_back(transform.M(1,0));
    res.transform.push_back(transform.M(1,1));
    res.transform.push_back(transform.M(1,2));
    res.transform.push_back(transform.p[1]);
    res.transform.push_back(transform.M(2,0));
    res.transform.push_back(transform.M(2,1));
    res.transform.push_back(transform.M(2,2));
    res.transform.push_back(transform.p[2]);

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "nao_kdl");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    arm_chain_p = new Nao_RArm_chain();

    ros::ServiceServer get_angles_service = nh.advertiseService("Nao_RArm_chain_get_angles", &get_angles_callback);
    ros::ServiceServer get_transform_service = nh.advertiseService("Nao_RArm_chain_get_transform", &get_transform_callback);
    ros::spin();
    return 0;
}