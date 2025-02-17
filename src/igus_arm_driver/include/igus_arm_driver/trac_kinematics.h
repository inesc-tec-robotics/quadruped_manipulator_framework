#pragma once

#include <geometry_msgs/Point.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <trac_ik/trac_ik.hpp>
#include <random>
#include <Eigen/Dense>

namespace igus_arm_driver{

  class TracKinematics {
    private:
      std::string robot_description, base_link, tool_link;
      KDL::Tree arm_tree;
      KDL::Chain arm_chain, trac_chain;
      KDL::JntArray lower_limits, upper_limits;
      int joint_number, max_attempts=5;
      std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
      std::unique_ptr<TRAC_IK::TRAC_IK> ik_solver;
      std::unique_ptr<KDL::ChainJntToJacSolver> jk_solver;
     
      void vectorToKDLJntArray(std::vector<double> joint_values, KDL::JntArray &JntArray);

    public:
      TracKinematics(std::string robot_description, std::string base_link, std::string tool_link);
      ~TracKinematics();
      bool inicitalizeKDL();

      geometry_msgs::Point getToolPosition(std::vector<double> joint_values);

      std::vector<double> getJointValues(geometry_msgs::Point desired_point, std::vector<double> current_joint_values);

      std::vector<double> getJointSpeed(std::vector<double> current_joint_pos, geometry_msgs::Point velocity_vector);

      geometry_msgs::Point getToolSpeed(std::vector<double> current_joint_pos, std::vector<double>  joint_velocities);
  };

}