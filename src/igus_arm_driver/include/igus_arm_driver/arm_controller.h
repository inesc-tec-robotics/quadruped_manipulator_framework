#pragma once

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>

#include <thread>
#include <mutex>
#include <cmath> 
#include <vector>
#include <sstream> 

#include "igus_arm_driver/trac_kinematics.h"

namespace igus_arm_driver{

  enum class moveStage{
    kAcel,
    kMax,
    kDAcel,
    kStopping,
    kStopped
  };

  struct joint{
    int dir;
    double current_pos;
    double current_vel;
    double des_vel;
    double acel_max;
    double max_vel;
    double error_thresh;
    moveStage current_stage;
  };

  struct move{
    int type; //0 - movej, 1 - movejjoints, 2 - movel, 3 - moveljoints
    double blend;
    double speed;
    double acceleration;
    geometry_msgs::Point goal;
    std::vector<double> goal_joints;
  };

  class ArmController{
    private:
      double k_correction;
      geometry_msgs::Point pi, u;
      moveStage current_phase;
      double prev_norm = 0;
      double linear_vel_step;
      bool limit_speed = false;

    protected:
      std::unique_ptr<TracKinematics> kin_lib;
      ros::Publisher goal_pub, max_spped;
      bool joints_enabled_;
      int joint_num_;
      std::vector<joint> joints_group_;
      double w_min, w_max, a_max, error_threshold_joints, error_threshold, error_deacel, k_deacel;

      double normalizeAngle(double angle);
      geometry_msgs::Point dist2Line(geometry_msgs::Point current_pos);
      double calculateDistance(geometry_msgs::Point goal);
      std::vector<double> calculateJointError(std::vector<double> goal);
      std::vector<double> getCurrentPos();

    public:
      ArmController();
      virtual ~ArmController() = default;
      
      void jointSpaceSetup(move &current, bool joints);
      void taskSpaceSetup(move &current, bool joints);
      void calculateJointVelocity(geometry_msgs::Point goal, double max_speed, double dist, bool deacel);
      void calculatePorpotionalVel(int joint, double dist);
      bool moveJointsSpaceIteration(move current);
      bool moveTaskSpaceIteration(move current);
      virtual void updateJointStatus(bool enabled) = 0;
      int getJointNumber();
      void applyVelocityRamp(std::vector<std_msgs::Float64> &cmd_vel);
      void stop();
  };
}