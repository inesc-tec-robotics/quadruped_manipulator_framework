#pragma once

#include "igus_arm_driver/arm_controller.h"

namespace igus_arm_driver{

  class SimArmController : public ArmController{
    private:
      ros::NodeHandle nh_;
      ros::Subscriber joint_states_sub_;
      std::vector<ros::Publisher> cmd_vel_pub_;

      std::mutex vel_mutex, pos_mutex;
      std::thread write_thread_;

    public:
      SimArmController(ros::NodeHandle &nh);
      ~SimArmController();
      
      void jointStatesCb(const sensor_msgs::JointState::ConstPtr& msg);
      void writeThread();
      void getParameters();
      void updateJointStatus(bool enabled) override;
  };
}