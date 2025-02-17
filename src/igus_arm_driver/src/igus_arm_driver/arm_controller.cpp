#include "igus_arm_driver/arm_controller.h"

namespace igus_arm_driver{
  ArmController::ArmController(){
    joints_enabled_ = false;
  }

  double ArmController::normalizeAngle(double angle) {
    angle = fmod(angle, 2 * M_PI);

    if (angle > M_PI) {
        angle -= 2 * M_PI;
    } else if (angle < -M_PI) {
        angle += 2 * M_PI;
    }

    return angle;
  }

  geometry_msgs::Point ArmController::dist2Line(geometry_msgs::Point current_pos){
    double k;
    geometry_msgs::Point intersection;

    k = (u.x*(current_pos.x-pi.x)+u.y*(current_pos.y-pi.y))/(pow(u.x,2)+pow(u.y,2));

    intersection.x = pi.x+k*u.x;
    intersection.y = pi.y+k*u.y;
    intersection.z = pi.z+k*u.z;

    return intersection;
  }

  std::vector<double> ArmController::getCurrentPos(){
    std::vector<double> positions;

    for (int i = 0; i < joint_num_; i++)
    {
      positions.push_back(joints_group_.at(i).current_pos);
    }

    return positions;
  }

  double ArmController::calculateDistance(geometry_msgs::Point goal){
    double dist = 0;
    std::vector<double> current_joint_pos;
    geometry_msgs::Point current_pos;

    current_joint_pos = this->getCurrentPos();
    current_pos = kin_lib->getToolPosition(current_joint_pos);

    dist = std::sqrt(std::pow(goal.x - current_pos.x, 2) + 
                     std::pow(goal.y - current_pos.y, 2) + 
                     std::pow(goal.z - current_pos.z, 2));

    return dist;
  }

  std::vector<double> ArmController::calculateJointError(std::vector<double> goal){
    std::vector<double> current_pos;
    std::vector<double> errors;
    double aux;

    current_pos = this->getCurrentPos();

    for (int i = 0; i < joint_num_; i++)
    {
      aux = normalizeAngle(goal.at(i)- current_pos.at(i));
      if(abs(aux) > M_PI && i == 1){
        aux = normalizeAngle(2*M_PI - aux);
      }
      errors.push_back(aux);
    }

    return errors;
  }

  void ArmController::jointSpaceSetup(move &current, bool joints){
    std::vector<double> errors, cur;
    double t_ac=0, t_total=0;
    double aux_tac, aux_thresh;
    
    cur = this->getCurrentPos();

    ROS_DEBUG("CURRENT POS: %f %f %f", cur.at(0), cur.at(1), cur.at(2));

    if(!joints){
      current.goal_joints = kin_lib->getJointValues(current.goal, cur);
      ROS_DEBUG("CURRENT GOAL: %f %f %f", current.goal_joints.at(0), current.goal_joints.at(1), current.goal_joints.at(2));
    }

    errors = this->calculateJointError(current.goal_joints);

    ROS_DEBUG("ERROR: %f %f %f", errors.at(0), errors.at(1), errors.at(2));

    auto max_error = std::max_element(errors.begin(), errors.end(),
      [](double a, double b) {
          return std::abs(a) < std::abs(b);
      });
    size_t index = std::distance(errors.begin(), max_error);

    if(current.speed > w_max) {
      current.speed = w_max;
      ROS_WARN("Maximum speed was limited to: %f", w_max);
    }
    if(current.acceleration > a_max) {
      current.acceleration = a_max;
      ROS_WARN("Maximum acceleration was limited to: %f", a_max);
    }

    do{
      t_ac = current.speed/current.acceleration;
      t_total = t_ac + abs(*max_error)/(current.acceleration*t_ac);
      aux_thresh = 0.5*current.acceleration*pow(t_ac,2);
      current.speed = current.speed  -0.1;
    }
    while(aux_thresh > abs(*max_error));

    ROS_DEBUG("MOVE SETUP 1: %f %f", t_ac, t_total);
  
    for (int i = 0; i < joint_num_; i++)
    {
      joints_group_.at(i).dir = (errors.at(i) > 0) ? 1 : -1;
      joints_group_.at(i).acel_max = current.acceleration;

      if(abs(errors.at(i)) < error_threshold_joints){
        joints_group_.at(i).des_vel = 0;
        joints_group_.at(i).error_thresh = 0; 
        joints_group_.at(i).current_stage = moveStage::kStopped;
        ROS_DEBUG("Joint %d: Too close vel = %f", i+1, joints_group_.at(i).des_vel);
      }
      else{
        if(i == index){
          aux_tac = t_ac;
        }
        else{
          aux_tac = (t_total/2) - (sqrt(pow(current.acceleration,2)*pow(t_total,2)-4*current.acceleration*abs(errors.at(i)))/(2*current.acceleration));
        }
        ROS_DEBUG("Joint %d: Aux_tac = %f", i+1, aux_tac);

        joints_group_.at(i).des_vel = joints_group_.at(i).dir*(current.acceleration*aux_tac);
        joints_group_.at(i).max_vel = current.acceleration*aux_tac;
        joints_group_.at(i).current_stage = moveStage::kAcel;
        joints_group_.at(i).error_thresh = 0.5*current.acceleration*pow(aux_tac,2);

        if(abs(joints_group_.at(i).des_vel) < w_min){
          joints_group_.at(i).current_stage = moveStage::kDAcel;
          joints_group_.at(i).des_vel = joints_group_.at(i).dir*w_min;
          joints_group_.at(i).max_vel = w_min;
        }
        ROS_DEBUG("Joint %d: Far vel = %f", i+1, joints_group_.at(i).des_vel);
      }
    }

    visualization_msgs::Marker marker;
    marker.header.frame_id = "joint1";
    marker.header.stamp = ros::Time::now();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = kin_lib->getToolPosition(current.goal_joints);
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.015;
    marker.scale.y = 0.015;
    marker.scale.z = 0.015;
    marker.color.a = 1.0;
    marker.color.r = 0.675;
    marker.color.g = 0;
    marker.color.b = 0.98;

    goal_pub.publish(marker);
  }

  void ArmController::taskSpaceSetup(move &current, bool joints){
    double distance;

    pi = kin_lib->getToolPosition(this->getCurrentPos());

    if(joints){
      current.goal = kin_lib->getToolPosition(current.goal_joints);
    }

    distance = this->calculateDistance(current.goal);

    u.x = (current.goal.x-pi.x)/distance;
    u.y = (current.goal.y-pi.y)/distance;
    u.z = (current.goal.z-pi.z)/distance;

    if(current.speed > w_max) current.speed = current.speed;
    if(current.acceleration > a_max) current.acceleration = a_max;

    for (size_t i = 0; i < joint_num_; i++)
    {
      joints_group_.at(i).acel_max = current.acceleration;
    }
    
    error_deacel = 1.5*current.speed;
    k_correction = 100*current.speed-2.5;
    linear_vel_step = 0.0001/current.speed;

    if(distance < error_deacel) {
      ROS_DEBUG("STAGE: DeACel");
      ROS_DEBUG("Dist: %f | Error: %f", error_deacel, distance);
      current_phase = moveStage::kDAcel;
      prev_norm = 0.01;
    }
    else{
      ROS_DEBUG("STAGE: Max");
      current_phase = moveStage::kMax;
      prev_norm = 0;
    } 

    visualization_msgs::Marker marker;
    marker.header.frame_id = "joint1";
    marker.header.stamp = ros::Time::now();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.points.push_back(pi);
    marker.points.push_back(current.goal);
    marker.scale.x = 0.005;
    marker.scale.y = 0.005;
    marker.scale.z = 0.01;
    marker.color.a = 1.0;
    marker.color.r = 0.675;
    marker.color.g = 0;
    marker.color.b = 0.98;

    goal_pub.publish(marker);
  }

  void ArmController::calculateJointVelocity(geometry_msgs::Point goal, double max_speed, double dist, bool deacel){
    geometry_msgs::Point tool_speed, current_pos, p_intersection, e_tool_speed, vel_dir;
    std::vector<double> joints_vel, joints_vel2, current_joint_pos;
    double max_value, factor, desired_speed, max_norm_e = 0, max_norm, vel_norm;
    std_msgs::Float64 msg;

    current_joint_pos = this->getCurrentPos();
    current_pos = kin_lib->getToolPosition(current_joint_pos);
    p_intersection = this->dist2Line(current_pos);

    if(deacel){
      factor = (1/error_deacel)*dist*max_speed;

      if(factor > prev_norm) factor = prev_norm;
      if(factor < 0.008) factor = 0.008;
    }
    else{
      factor = prev_norm + linear_vel_step;
      if(factor > max_speed) factor = max_speed;
    }

    vel_dir.x = u.x + k_correction*(p_intersection.x-current_pos.x);
    vel_dir.y = u.y + k_correction*(p_intersection.y-current_pos.y);
    vel_dir.z = u.z + k_correction*(p_intersection.z-current_pos.z);

    vel_norm = sqrt(pow(vel_dir.x,2)+pow(vel_dir.y,2)+pow(vel_dir.z,2));

    tool_speed.x = factor*(vel_dir.x)/vel_norm;
    tool_speed.y = factor*(vel_dir.y)/vel_norm;
    tool_speed.z = factor*(vel_dir.z)/vel_norm;

    max_norm = sqrt(pow(tool_speed.x,2)+pow(tool_speed.y,2)+pow(tool_speed.z,2));

    joints_vel = kin_lib->getJointSpeed(current_joint_pos, tool_speed);

    // ROS_DEBUG("Desired joint %f %f %f", joints_vel.at(0), joints_vel.at(1), joints_vel.at(2));

    max_value = *std::max_element(joints_vel.begin(), joints_vel.end(), 
                             [](double a, double b) { return std::abs(a) < std::abs(b); });

    for (int i = 0; i < joint_num_; i++)
    {
      joints_group_.at(i).dir = (joints_vel.at(i) > 0) ? 1 : -1;

      if(abs(max_value) > w_max){        
        joints_group_.at(i).des_vel = joints_vel.at(i)*w_max/abs(max_value);
        msg.data = prev_norm;
      }
      else{
        joints_group_.at(i).des_vel = joints_vel.at(i);
        msg.data = max_norm;
      }
    }

    prev_norm = msg.data;
    max_spped.publish(msg);
  }

  void ArmController::calculatePorpotionalVel(int joint, double dist){
    double k, b, speed;;

    speed = dist*joints_group_.at(joint).max_vel/(2.5*joints_group_.at(joint).error_thresh);

    if(abs(speed) > joints_group_.at(joint).max_vel) joints_group_.at(joint).des_vel = joints_group_.at(joint).dir*joints_group_.at(joint).max_vel;
    else if(abs(speed) < w_min) joints_group_.at(joint).des_vel = joints_group_.at(joint).dir*w_min;
    else{
      joints_group_.at(joint).des_vel = joints_group_.at(joint).dir*speed;
    }
  }

  bool ArmController::moveJointsSpaceIteration(move current){
    int stop, allStopped;
    std::vector<double> distance;

    distance = this->calculateJointError(current.goal_joints);

    for (int i = 0; i < joint_num_; i++)
    {
      switch (joints_group_.at(i).current_stage)
      {
        case moveStage::kAcel:{
          if(joints_group_.at(i).current_vel == joints_group_.at(i).des_vel){
            joints_group_.at(i).current_stage = moveStage::kMax;
            ROS_DEBUG("STAGE %d: Max", i+1);
          }
          break;
        }
        case moveStage::kMax:{
          if(abs(distance.at(i)) <= 2.5*joints_group_.at(i).error_thresh){
            joints_group_.at(i).current_stage = moveStage::kDAcel;
            calculatePorpotionalVel(i, abs(distance.at(i)));
            ROS_DEBUG("STAGE %d: Deacel", i+1);
          }
          break;
        }
        case moveStage::kDAcel:{
          calculatePorpotionalVel(i, abs(distance.at(i)));

          if(abs(distance.at(i)) <= error_threshold_joints){
            joints_group_.at(i).des_vel = 0;
            joints_group_.at(i).current_stage = moveStage::kStopping;
            ROS_DEBUG("STAGE %d: Stopping", i+1);
          }
          break;
        }
        case moveStage::kStopping:{
          if(joints_group_.at(i).current_vel == 0){
            joints_group_.at(i).current_stage = moveStage::kStopped;
            ROS_DEBUG("STAGE %d: Stopped", i+1);
          }
          break;
        }
        default:
        break;
      }
    }

    allStopped = true;
    for (const auto& joint : joints_group_) {
      if (joint.current_stage != moveStage::kStopped) {
          allStopped = false;
          break; 
      }
    }

    if(allStopped){
      return true;
    }

    return false;
  }

  bool ArmController::moveTaskSpaceIteration(move current){
    int stop, allStopped;
    double distance;

    distance = this->calculateDistance(current.goal);

    switch (current_phase)
    {
      case moveStage::kMax:{
        this->calculateJointVelocity(current.goal, current.speed, distance, false);
        if(distance < (error_deacel)){
          current_phase = moveStage::kDAcel;
          ROS_DEBUG("STAGE: DeACel");
        }
        break;
      }
      case moveStage::kDAcel:{
        this->calculateJointVelocity(current.goal, current.speed, distance, true);

        if(distance < error_threshold){
          for (int i = 0; i < joint_num_; i++)
          {
            joints_group_.at(i).des_vel = 0;
          }
          current_phase = moveStage::kStopping;
          ROS_DEBUG("STAGE: Stopping");
        }
        break;
      }
      case moveStage::kStopping:{
        allStopped = true;
        for (int i = 0; i < joint_num_; i++)
        {
          if(joints_group_.at(i).current_vel != 0){
            allStopped = false;
            break;
          }
        }
        
        if(allStopped) {
          current_phase = moveStage::kStopped;
          ROS_DEBUG("STAGE: Stopped");
          limit_speed = false;
          return true;
        }
        break;
      }
      default:
      break;
    }

    return false;
  } 

  int ArmController::getJointNumber(){
    return joint_num_;
  }

  void ArmController::applyVelocityRamp(std::vector<std_msgs::Float64> &cmd_vel){
    for (int i = 0; i < joint_num_; i++)
    {
      if(joints_group_.at(i).des_vel == 0 && abs(joints_group_.at(i).current_vel) <= w_min){
        cmd_vel.at(i).data = 0;
      }
      else{
          if(joints_group_.at(i).current_vel > joints_group_.at(i).des_vel)
          {
              cmd_vel.at(i).data = joints_group_.at(i).current_vel - joints_group_.at(i).acel_max*0.05;

              if(cmd_vel.at(i).data < joints_group_.at(i).des_vel) cmd_vel.at(i).data = joints_group_.at(i).des_vel;
          }  
          else if (joints_group_.at(i).current_vel < joints_group_.at(i).des_vel)
          {
              cmd_vel.at(i).data= joints_group_.at(i).current_vel + joints_group_.at(i).acel_max*0.05;

              if(cmd_vel.at(i).data > joints_group_.at(i).des_vel) cmd_vel.at(i).data = joints_group_.at(i).des_vel;
          }
          else{
              cmd_vel.at(i).data = joints_group_.at(i).current_vel;
          }
     }
    }
  }

  void ArmController::stop(){
    for (int i = 0; i < joint_num_; i++)
    {
      joints_group_.at(i).des_vel = 0;
    }
  }
}