#pragma once

#include <std_msgs/String.h>
#include "igus_arm_driver/arm_controller.h"
#include <actionlib/server/simple_action_server.h>
#include "igus_arm_driver/IgusCommandAction.h"

namespace igus_arm_driver{

    class IgusArmDriver{
      private:
        ros::NodeHandle nh_;
        ros::Subscriber goal_sub_;
        std::list<move> trajectory;
        ArmController *arm_interface_;
        std::string action_name_ = "igus_arm_driver_skill";
        actionlib::SimpleActionServer<igus_arm_driver::IgusCommandAction> command_server_;
        igus_arm_driver::IgusCommandFeedback feedback_;
        igus_arm_driver::IgusCommandResult result_;

        void commandHandle(std::string command);

      public:
        IgusArmDriver(ros::NodeHandle& node, ArmController *interface);
        ~IgusArmDriver();
        bool is_goal_running;

        void executeCB(const igus_arm_driver::IgusCommandGoalConstPtr &goal);
    };
}