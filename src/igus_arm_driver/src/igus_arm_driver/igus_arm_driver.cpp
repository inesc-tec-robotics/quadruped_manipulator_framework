#include "igus_arm_driver/igus_arm_driver.h"

namespace igus_arm_driver{
  IgusArmDriver::IgusArmDriver(ros::NodeHandle &node, ArmController *interface): nh_(node), command_server_(nh_, action_name_,boost::bind(&IgusArmDriver::executeCB, this, _1), false){

    arm_interface_ = interface;

    command_server_.start();

    is_goal_running = false;
  }

  IgusArmDriver::~IgusArmDriver(){

  }

  void IgusArmDriver::commandHandle(std::string command){
    move instance;

    //Type
    if (command.find("movelj") != std::string::npos) {
      instance.type = 3;
    } else if (command.find("movejj") != std::string::npos) {
      instance.type = 1;
    } else if (command.find("movel") != std::string::npos) {
      instance.type = 2;
    } else if (command.find("movej") != std::string::npos) {
      instance.type = 0;
    }

    //Goal
    std::size_t p_start = command.find("p={");
    std::size_t p_end = command.find("}", p_start);
    if (p_start != std::string::npos && p_end != std::string::npos) {
      std::string p_values = command.substr(p_start + 3, p_end - (p_start + 3));
      std::stringstream ss(p_values);
      std::string value;

      if(instance.type == 0 || instance.type == 2){
        std::getline(ss, value, ',');
        instance.goal.x = std::stod(value); 
        std::getline(ss, value, ',');
        instance.goal.y = std::stod(value);
        std::getline(ss, value, ',');
        instance.goal.z = std::stod(value); 
      }
      else{
        for (int i = 0; i < arm_interface_->getJointNumber(); ++i) {
          std::getline(ss, value, ',');
          instance.goal_joints.push_back(std::stod(value)); 
        }
      }
    }

    //Velocity
    std::size_t v_start = command.find("v=");
    std::size_t v_end = command.find(",", v_start);
    
    if (v_start != std::string::npos && v_end != std::string::npos) {
        std::string v_value = command.substr(v_start + 2, v_end - (v_start + 2));
        instance.speed = std::stod(v_value);
    }

    //Acceleration
    std::size_t a_start = command.find("a=");
    std::size_t a_end = command.find(",", a_start);
    
    if (a_start != std::string::npos && a_end != std::string::npos) {
        std::string a_value = command.substr(a_start + 2, a_end - (a_start + 2));
        instance.acceleration = std::stod(a_value);
    }

    //Blend
    std::size_t b_start = command.find("b=");
    if (b_start != std::string::npos) {
        std::string b_value = command.substr(b_start + 2);
        instance.blend = std::stod(b_value);
    }

    trajectory.push_back(instance);
  }

  void IgusArmDriver::executeCB(const igus_arm_driver::IgusCommandGoalConstPtr &goal){
    bool success;
    std::vector<std::string> commands;
    std::string token;
    std::string delimiter = "),";
    ros::Rate loop_rate(100);

    std::size_t pos = 0;
    std::string current_command = goal->target;

    if(is_goal_running){
      command_server_.setPreempted();
      arm_interface_->stop();
      return;
    }

    trajectory.clear();

    while ((pos = current_command.find(delimiter)) != std::string::npos) {
      token = current_command.substr(0, pos + 1);
      commands.push_back(token);
      current_command.erase(0, pos + delimiter.length());
    }

    if (!current_command.empty()) {
      current_command.erase(std::remove(current_command.begin(), current_command.end(), ';'), current_command.end());
      commands.push_back(current_command);
    }

    for (int i = 0; i < commands.size(); i++)
    {
      commandHandle(commands.at(i));
    }

    arm_interface_->updateJointStatus(true);


    while (!trajectory.empty())
    {
      move current = trajectory.front();

      switch (current.type) {
        case 0: {// MOVEJ
            ROS_INFO("MOVEJ GOAL: %f %f %f", current.goal.x, current.goal.y, current.goal.z);
            arm_interface_->jointSpaceSetup(current, false);

            while(!command_server_.isPreemptRequested()){
              success = arm_interface_->moveJointsSpaceIteration(current);
              if (success) break;
              loop_rate.sleep();
            }

            break;
        }
        case 1: { // MOVEJJOINTS
            ROS_INFO("MOVEJJOINTS GOAL: %f %f %f", current.goal_joints[0], current.goal_joints[1], current.goal_joints[2]);

            arm_interface_->jointSpaceSetup(current, true);

            while(!command_server_.isPreemptRequested()){
              success = arm_interface_->moveJointsSpaceIteration(current);
              if (success) break;
              loop_rate.sleep();
            }

            break;
        }
        case 2:{ // MOVEL
          ROS_INFO("MOVEL GOAL: %f %f %f", current.goal.x, current.goal.y, current.goal.z);
          arm_interface_->taskSpaceSetup(current, false);

          while(!command_server_.isPreemptRequested()){
            success = arm_interface_->moveTaskSpaceIteration(current);
            if (success) break;
            loop_rate.sleep();
          }

          break;
        }
            
        case 3: {// MOVELJOINTS
          ROS_INFO("MOVELJOINTS GOAL: %f %f %f", current.goal_joints[0], current.goal_joints[1], current.goal_joints[2]);
          arm_interface_->taskSpaceSetup(current, true);

          while(!command_server_.isPreemptRequested()){
            success = arm_interface_->moveTaskSpaceIteration(current);
            if (success) break;
            loop_rate.sleep();
          }

          break;
        }
      }

      if (success) {
        trajectory.pop_front();
        feedback_.progress = 1.0 - (double)trajectory.size() / commands.size();
        command_server_.publishFeedback(feedback_);
      } else {
          result_.success = false;
          result_.message = "Trajectory execution failed.";
          command_server_.setAborted(result_);
          trajectory.clear();
          return;
      }
    }
    
    
    trajectory.clear();
    arm_interface_->updateJointStatus(false);
    ROS_INFO("Trajectory executed successfully.");
    result_.success = true;
    result_.message = "Trajectory completed.";
    command_server_.setSucceeded(result_);
  }
}