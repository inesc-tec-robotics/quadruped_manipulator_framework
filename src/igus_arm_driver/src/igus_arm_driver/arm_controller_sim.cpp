#include "igus_arm_driver/arm_controller_sim.h"

namespace igus_arm_driver{ 
  SimArmController::SimArmController(ros::NodeHandle &nh) : nh_(nh){
    std::string topic_name, robot_description;
    this->getParameters();

    nh_.getParam("/robot_description", robot_description);
    kin_lib = std::make_unique<TracKinematics>(robot_description, "joint1", "tool");
    kin_lib->inicitalizeKDL();

    goal_pub = nh_.advertise<visualization_msgs::Marker>( "line_marker", 0);
    max_spped = nh_.advertise<std_msgs::Float64>( "max_speed", 0);

    joints_enabled_ = false;
    k_deacel = 1/(error_deacel);

    joint_states_sub_ = nh_.subscribe("/joint_states", 10, &SimArmController::jointStatesCb, this);

    for (int i = 1; i <= joint_num_; i++)
    {
      ros::Publisher joint_pub_;
      topic_name = "/igus_controller/art"+std::to_string(i)+"_vel/command";
      joint_pub_ = nh_.advertise<std_msgs::Float64>(topic_name, 1);
      cmd_vel_pub_.push_back(joint_pub_);
    }
    
    write_thread_ = std::thread(&SimArmController::writeThread, this);
  }

  SimArmController::~SimArmController(){
    if(write_thread_.joinable()){
      write_thread_.join();
    }
  }

  void SimArmController::getParameters(){
    if (!nh_.hasParam("/joint_num")) {
        ROS_ERROR("Parameter '/joint_num' not found. Shutting down node.");
        ros::shutdown();  
        return;           
    }
    
    nh_.getParam("/joint_num", joint_num_);

    for (int i = 0; i < joint_num_; i++)
    {
      igus_arm_driver::joint aux;
      aux.current_pos = 0;
      aux.current_vel = 0;
      aux.des_vel = 0;
      aux.current_stage = moveStage::kStopped;
      joints_group_.push_back(aux);
    }

    nh_.getParam("/w_min", w_min);
    nh_.getParam("/w_max", w_max);
    nh_.getParam("/a_max", a_max);
    nh_.getParam("/error_threshold_joints", error_threshold_joints);
    nh_.getParam("/error_threshold", error_threshold);
  }

  void SimArmController::jointStatesCb(const sensor_msgs::JointState::ConstPtr& msg){
    std::lock_guard<std::mutex> lock(pos_mutex);
    joints_group_.at(0).current_pos = normalizeAngle(msg->position.at(12));
    joints_group_.at(1).current_pos = normalizeAngle(msg->position.at(13));
    joints_group_.at(2).current_pos = normalizeAngle(msg->position.at(14));
  }

  void SimArmController::updateJointStatus(bool enabled){
    if(enabled){
      joints_enabled_ = true;
    }
    else{
      joints_enabled_ = false;
    }
  }

  void SimArmController::writeThread(){
    std::vector<std_msgs::Float64> cmd_vel;
    std_msgs::Float64 zeros;
    ros::Rate rate(20);
    int finish_count;

    zeros.data = 0;
    cmd_vel.assign(joint_num_, zeros);
    
    while (ros::ok())
    { 
      if(joints_enabled_){
        std::lock_guard<std::mutex> lock(vel_mutex);
        // ROS_DEBUG("CYCLE COUNT: %d", cycle_count_);

        applyVelocityRamp(cmd_vel);

        for (int j = 0; j < joint_num_; j++)
        {
          joints_group_.at(j).current_vel = cmd_vel.at(j).data;
          cmd_vel_pub_.at(j).publish(cmd_vel.at(j));
        }
        // ROS_DEBUG("VEL: %f %f %f", cmd_vel.at(0),  cmd_vel.at(1), cmd_vel.at(2));
      }

      rate.sleep();
    }
  }

}