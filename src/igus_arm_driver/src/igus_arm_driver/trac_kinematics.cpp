#include "igus_arm_driver/trac_kinematics.h"

namespace igus_arm_driver{

  TracKinematics::TracKinematics(std::string robot_description, std::string base_link, std::string tool_link): base_link(base_link), tool_link(tool_link), robot_description(robot_description){
  
  };

  TracKinematics::~TracKinematics(){};

  void TracKinematics::vectorToKDLJntArray(std::vector<double> joint_values, KDL::JntArray &JntArray){
    for (int i = 0; i < joint_number; i++)
    {
      JntArray(i) = joint_values.at(i);
    }
  }

  bool TracKinematics::inicitalizeKDL(){
    if (!kdl_parser::treeFromString(robot_description, arm_tree)){
      return false;
    }

    if(!arm_tree.getChain(base_link, tool_link, arm_chain)){
      return false;
    }

    joint_number = arm_chain.getNrOfJoints();
    fk_solver = std::make_unique<KDL::ChainFkSolverPos_recursive>(arm_chain);

    lower_limits.resize(joint_number);    
    upper_limits.resize(joint_number);   

    lower_limits(0)= -0.2;
    upper_limits(0)= 3.3; 

    lower_limits(1)= -0.3;
    upper_limits(1)= 3.5;

    lower_limits(2)= -3.14;
    upper_limits(2)= 3.14;

    ik_solver = std::make_unique<TRAC_IK::TRAC_IK>(arm_chain, lower_limits, upper_limits, 0.005, 1e-5, TRAC_IK::Distance);

    bool valid = ik_solver->getKDLChain(trac_chain);
    bool limits = ik_solver->getKDLLimits(lower_limits, upper_limits);

    if (!valid || !limits) {
      return false;
    }

    jk_solver = std::make_unique<KDL::ChainJntToJacSolver>(arm_chain);

    return true;
  }

  geometry_msgs::Point TracKinematics::getToolPosition(std::vector<double> joint_values){
    geometry_msgs::Point tool_pos;
    KDL::JntArray joint_positions(joint_number);
    KDL::Frame cartesian_position;

    this->vectorToKDLJntArray(joint_values, joint_positions);

    fk_solver->JntToCart(joint_positions, cartesian_position); 

    tool_pos.x = cartesian_position.p.x();   
    tool_pos.y = cartesian_position.p.y();   
    tool_pos.z = cartesian_position.p.z();   

    return tool_pos;
  }

  std::vector<double> TracKinematics::getJointValues(geometry_msgs::Point desired_point, std::vector<double> current_joint_values){
    std::vector<double> joint_values;
    KDL::Frame desired_pose(KDL::Vector(desired_point.x, desired_point.y, desired_point.z));
    KDL::JntArray result(joint_number);
    KDL::JntArray init_guess(joint_number);
    KDL::Vector position_tolerance(0, 0, 0);
    KDL::Vector orientation_tolerance(1e6, 1e6, 1e6);
    KDL::Twist twist(position_tolerance, orientation_tolerance);

    this->vectorToKDLJntArray(current_joint_values, init_guess);

    int rc = ik_solver->CartToJnt(init_guess, desired_pose, result, twist);

    int attempt = 0;
    while (attempt < max_attempts) {
        int rc = ik_solver->CartToJnt(init_guess, desired_pose, result, twist);

        if (rc >= 0) {
            bool within_limits = true;
            for (int i = 0; i < joint_number; ++i) {
                if (result(i) < lower_limits(i) || result(i) > upper_limits(i)) {
                    within_limits = false;
                    std::cout << "Outside limits" << std::endl;
                    break;
                }
            }

            if (within_limits) {
                for (int i = 0; i < joint_number; ++i) {
                    joint_values.push_back(result(i));
                }
                return joint_values;
            }
        }

        std::random_device rd;
        std::mt19937 gen(rd());
        for (int i = 0; i < joint_number; ++i) {
            std::uniform_real_distribution<> dis(lower_limits(i), upper_limits(i));
            init_guess(i) = dis(gen);
        }

        attempt++;
    }
  }

  std::vector<double> TracKinematics::getJointSpeed(std::vector<double> current_joint_pos, geometry_msgs::Point velocity_vector){
    std::vector<double> joint_speeds;
    KDL::JntArray current_q(joint_number);
    KDL::Jacobian jacobian_matrix(joint_number);
    Eigen::Matrix3d jacobian;
    Eigen::Vector3d joint_velocity, cartesian_velocity;

    this->vectorToKDLJntArray(current_joint_pos, current_q);

    jk_solver->JntToJac(current_q, jacobian_matrix);
    
    jacobian = jacobian_matrix.data.topRows(3);

    cartesian_velocity << velocity_vector.x, velocity_vector.y, velocity_vector.z;

    joint_velocity = jacobian.inverse() * cartesian_velocity;

    for (int i = 0; i < joint_number; i++)
    {
      joint_speeds.push_back(joint_velocity(i));
    }
    
    return joint_speeds;
  }

  geometry_msgs::Point TracKinematics::getToolSpeed(std::vector<double> current_joint_pos, std::vector<double> joint_velocities){
    KDL::JntArray current_q(joint_number);
    KDL::Jacobian jacobian_matrix(joint_number);
    Eigen::Matrix3d jacobian;
    Eigen::Vector3d joint_velocity, cartesian_velocity;
    geometry_msgs::Point tool_speed;

    this->vectorToKDLJntArray(current_joint_pos, current_q);

    jk_solver->JntToJac(current_q, jacobian_matrix);

    jacobian = jacobian_matrix.data.topRows(3);

    joint_velocity << joint_velocities.at(0), joint_velocities.at(1), joint_velocities.at(2);
    
    cartesian_velocity = jacobian * joint_velocity;

    tool_speed.x = cartesian_velocity(0);
    tool_speed.y = cartesian_velocity(1);
    tool_speed.z = cartesian_velocity(2);
  }
}