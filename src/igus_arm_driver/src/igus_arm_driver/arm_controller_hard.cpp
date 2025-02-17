#include "igus_arm_driver/arm_controller_hard.h"

namespace igus_arm_driver{

  HardArmController::HardArmController(ros::NodeHandle &nh) : nh_(nh), can_bus_("can0"){
    std::string robot_description;
    this->getParameters();

    nh_.getParam("/robot_description", robot_description);
    kin_lib = std::make_unique<TracKinematics>(robot_description, "joint1", "tool");
    kin_lib->inicitalizeKDL();

    goal_pub = nh_.advertise<visualization_msgs::Marker>( "line_marker", 0);
    max_spped = nh_.advertise<std_msgs::Float64>( "max_speed", 0);

    joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states",1);

    joints_enabled_ = false;
    k_deacel = 1/(error_deacel);

    try{
        can_bus_.connect();
    }
    catch (const char* msg){
        ROS_ERROR(msg);
        ros::shutdown();
    }

    write_thread_ = std::thread(&HardArmController::writeThread, this);
    read_thread_ = std::thread(&HardArmController::readThread, this);
  }

  HardArmController::~HardArmController(){
    if(write_thread_.joinable()){
        write_thread_.join();
    }
    if(read_thread_.joinable()){
        read_thread_.join();
    }
  }

  void HardArmController::errorHandling(int error_state, int joint){
    std::bitset<8> error_code(error_state);

    if(error_code[0]){
        ROS_WARN("Joint %d over temperature", joint);
    }
    if(error_code[1]){
        ROS_WARN("Joint %d no voltage / emergency stop", joint);
    }
    if(error_code[2]){
        //ROS_WARN("Joint %d motor not active", joint);
    }
    if(error_code[3]){
        //ROS_WARN("Joint %d communication failure", joint);
    }
    if(error_code[4]){
        //ROS_WARN("Joint %d following error", joint);
    }
    if(error_code[5]){
        ROS_WARN("Joint %d encoder error", joint);
    }
    if(error_code[6]){
        ROS_WARN("Joint %d driver error", joint);
    }
    if(error_code[7]){
        ROS_WARN("Joint %d over current", joint);
    }
  }

  float HardArmController::ticks2rad(int32_t ticks, int joint){
    return ticks*(2*M_PI)/joint_params_.at(joint).tpr;
  }

  double HardArmController::convert2JointSpeed(double speed_rad_s, int joint_id){
    return joint_params_.at(joint_id).orientation * joint_params_.at(joint_id).gr * 60/(2*M_PI)*speed_rad_s;
  }

  void HardArmController::timerWatchdog(const ros::TimerEvent& event){
    ROS_ERROR("No info about joints for 5 seconds");
    ros::shutdown();
  }

  void HardArmController::checkArmStatus(){
    bool aux_enable = true;
    bool aux_disable = true;

    for (int i = 0; i < joint_num_; i++)
    {
        aux_enable = aux_enable && (joint_params_.at(i).state == states::kEnabled); 
        aux_disable = aux_disable && (joint_params_.at(i).state == states::kDisabled);
    }

    if(aux_enable){
        joints_enabled_ = true;
        ROS_DEBUG("All joints enabled");
        check_joint_status = false;
    }
    else if (aux_disable){
        joints_enabled_ = false;
        ROS_DEBUG("All joints disabled");
        check_joint_status = false;
    }
  }

  void HardArmController::getParameters(){
    int aux_can_id;

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

      igus_arm_driver::joint_info aux_info;
      nh_.getParam("/joint" + std::to_string(i+1) + "/gear_ratio", aux_info.gr);
      nh_.getParam("/joint" + std::to_string(i+1) + "/ticks_per_rotation", aux_info.tpr);
      nh_.getParam("/joint" + std::to_string(i+1) + "/offset", aux_info.offset);
      nh_.getParam("/joint" + std::to_string(i+1) + "/can_id", aux_can_id);
      aux_info.can_id = aux_can_id;
      nh_.getParam("/joint" + std::to_string(i+1) + "/direction", aux_info.orientation);
      aux_info.state = states::kInit;
      joint_params_.push_back(aux_info);
    }   

    nh_.getParam("/w_min", w_min);
    nh_.getParam("/w_max", w_max);
    nh_.getParam("/a_max", a_max);
    nh_.getParam("/error_threshold_joints", error_threshold_joints);
    nh_.getParam("/error_threshold", error_threshold); 
  }

  void HardArmController::publishJointStates(){
    sensor_msgs::JointState states;

    states.header.frame_id = "joint1";
    states.header.stamp = ros::Time::now();
    
    for(int i=0; i < joint_num_; i++){
        states.name.push_back("art"+std::to_string(i+1));
        states.position.push_back(joints_group_.at(i).current_pos);
        states.velocity.push_back(joints_group_.at(i).current_vel);
    }

    joint_states_pub_.publish(states);
  }

  void HardArmController::updateJointStatus(bool enabled){
    if(enabled){
        enable_joint_.fill(true);
    }
    else{
        disable_joint_.fill(true);
    }
  }

  void HardArmController::readThread(){
    struct can_frame rcv_frame;
    float temp_m, temp_b, voltage;
    int i;
    std::vector<ros::Timer> joint_timers;

    for(int i =0; i < joint_num_; i++){
        joint_timers.push_back(nh_.createTimer(ros::Duration(5.0), &HardArmController::timerWatchdog, this));
    }

    ROS_INFO("CANBUS: Read thread started.");
    while(ros::ok()){
        try{
            rcv_frame = can_bus_.readMessage();

            //SELECT JOINT
            if(rcv_frame.can_id < joint_params_.at(0).can_id + 4){
                i = 0;
            }
            else if(rcv_frame.can_id >= joint_params_.at(0).can_id + 4 && rcv_frame.can_id < joint_params_.at(1).can_id + 4){
                i = 1;
            }
            else if(rcv_frame.can_id >= joint_params_.at(2).can_id){
                i = 2;
            }

            joint_timers.at(i).stop();
            joint_timers.at(i).start();

            //HANDLE MESSAGES
            if(rcv_frame.can_id == joint_params_.at(i).can_id+1){
                joint_params_.at(i).error_state = rcv_frame.data[0];
                joints_group_.at(i).current_pos = joint_params_.at(i).offset + joint_params_.at(i).orientation*ticks2rad((int)(((((uint32_t)rcv_frame.data[1]) << 24) | (((uint32_t)rcv_frame.data[2]) << 16) | (((uint32_t)rcv_frame.data[3]) << 8) | ((uint32_t)rcv_frame.data[4]))), i);
                this->errorHandling(rcv_frame.data[0], i);
            }
            else if(rcv_frame.can_id == joint_params_.at(i).can_id+2){
                if(rcv_frame.data[0] == 0xE0){
                    if(rcv_frame.data[1] !=0 || rcv_frame.data[2] != 0 || rcv_frame.data[3] !=0 || rcv_frame.data[4] !=0){
                        ROS_ERROR("Internal errors on motor %d", i+1);
                    }
                }
            }
            else if(rcv_frame.can_id == joint_params_.at(i).can_id+3){
                if(rcv_frame.data[0] == 0x12 && rcv_frame.data[1] == 0x00){
                    voltage = (int)((((uint32_t)rcv_frame.data[3]) << 8) | ((uint32_t)rcv_frame.data[2]));
                    temp_m = (int)((((uint32_t)rcv_frame.data[5]) << 8) | ((uint32_t)rcv_frame.data[4]));
                    temp_b = (int)((((uint32_t)rcv_frame.data[7]) << 8) | ((uint32_t)rcv_frame.data[6]));
                    // ROS_DEBUG("Joint %d: %f mV M: %f mC B: %f mC", i+1, voltage, temp_m, temp_b);
                }
            }
        }
        catch(const char *msg){
            ROS_WARN("Reading error: %s", msg);
        }
    }
    ROS_INFO("CANBUS: Read thread terminating."); 
  }

  void HardArmController::writeThread(){
    std::lock_guard<std::mutex> guard(mutex);
    float reset_timer, enable_timer, disable_timer;
    std_msgs::Float64 zeros;
    std::vector<std_msgs::Float64> cmd_vel;
    ros::Rate rate(20);

    zeros.data = 0;
    cmd_vel.assign(joint_num_, zeros);

    while (ros::ok())
    {
        try{
            for(int i = 0; i < joint_num_; i++){
                switch (joint_params_.at(i).state){
                    case states::kInit:{
                        if(enable_joint_.at(i)){
                            joint_params_.at(i).state = states::kError;
                            ROS_DEBUG("Joint %d state %d", i+1, joint_params_.at(i).state);
                            can_bus_.resetErrors(joint_params_.at(i).can_id);
                            reset_timer = ros::Time::now().toSec();
                        }
                        break;
                    }
                    case states::kError:{
                        if(joint_params_.at(i).error_state == 0x04){
                            joint_params_.at(i).state = states::kEnabling;
                            check_joint_status = true;
                            ROS_DEBUG("Joint %d state %d", i+1, joint_params_.at(i).state);
                            can_bus_.enableMotor(joint_params_.at(i).can_id);
                            enable_timer = ros::Time::now().toSec();
                            ROS_INFO("Errors reseted at joint %d", i+1);
                        }
                        else if(ros::Time::now().toSec() - reset_timer > timeout){
                            ROS_INFO("Timeout in reset joint %d", i+1);
                            joint_params_.at(i).state = states::kInit;
                            ROS_DEBUG("Joint %d state %d", i+1, joint_params_.at(i).state);
                        }
                        break;
                    }
                    case states::kEnabling:{
                        if(joint_params_.at(i).error_state == 0x00){
                            ROS_INFO("Motor %d enabled", i+1);
                            joint_params_.at(i).state = states::kEnabled;
                            ROS_DEBUG("Joint %d state %d", i+1, joint_params_.at(i).state);
                            enable_joint_.at(i) = false;
                        }
                        else if(ros::Time::now().toSec() - enable_timer > timeout){
                            ROS_INFO("Timeout in enable joint %d", i+1);
                            joint_params_.at(i).state = states::kDisabled;
                            ROS_DEBUG("Joint %d state %d", i+1, joint_params_.at(i).state);
                        }
                        break;
                    }      
                    case states::kEnabled:{
                        // Calculate velocity
                        if(joints_enabled_){
                            applyVelocityRamp(cmd_vel);

                            // double joint_speed = convert2JointSpeed(cmd_vel.at(i).data, i);
                            ROS_DEBUG("Velocity %d: %f | %f", i, cmd_vel.at(i).data, joints_group_.at(i).des_vel);
                            // can_bus_.velocityMsg(0x40+i, joint_speed, joint_params_.at(i).message_counter);
                        }

                        if(joint_params_.at(i).error_state != 0x00){
                            joint_params_.at(i).state = states::kInit;
                            ROS_DEBUG("Joint %d state %d", i+1, joint_params_.at(i).state);
                            ROS_INFO("Motor %d disabled", i+1);
                        }
                        else if(disable_joint_.at(i)){
                            joint_params_.at(i).state = states::kDisabling;
                            check_joint_status = true;
                            ROS_DEBUG("Joint %d state %d", i+1, joint_params_.at(i).state);
                            can_bus_.disableMotor(joint_params_.at(i).can_id);
                            disable_timer = ros::Time::now().toSec();
                            joints_group_.at(i).des_vel = 0;
                        }
                        break;
                    }      
                    case states::kDisabling:{
                        cmd_vel.assign(joint_num_, zeros);
                        if(joint_params_.at(i).error_state == 0x04){
                            disable_joint_.at(i) = false;
                            joint_params_.at(i).state = states::kDisabled;
                            ROS_DEBUG("Joint %d state %d", i+1, joint_params_.at(i).state);
                            ROS_INFO("Motor %d disabled", i+1);
                        }
                        else if(ros::Time::now().toSec() - disable_timer > timeout){
                            ROS_INFO("Timeout in disable joint %d", i+1);
                            joint_params_.at(i).state = states::kEnabled;
                            ROS_DEBUG("Joint %d state %d", i+1, joint_params_.at(i).state);
                        }
                        break;
                    }      
                    case states::kDisabled:{
                        if(joint_params_.at(i).error_state != 0x04){
                            joint_params_.at(i).state = states::kInit;
                            ROS_DEBUG("Joint %d state %d", i+1, joint_params_.at(i).state);
                        }
                        else if(enable_joint_.at(i)){
                            joint_params_.at(i).state = states::kEnabling;
                            check_joint_status = true;
                            ROS_DEBUG("Joint %d state %d", i+1, joint_params_.at(i).state);
                            can_bus_.enableMotor(joint_params_.at(i).can_id);
                            enable_timer = ros::Time::now().toSec();
                        }
                        break;
                    }                  
                
                    default:
                        break;
                }

                can_bus_.velocityMsg(joint_params_.at(i).can_id, convert2JointSpeed(cmd_vel.at(i).data, i), joint_params_.at(i).message_counter);
                joint_params_.at(i).message_counter++;
                joints_group_.at(i).current_vel = cmd_vel.at(i).data;
            }

            this->publishJointStates();
            if(check_joint_status) checkArmStatus();
            rate.sleep();
        }
        catch(const char *msg){
            ROS_ERROR("ExecutionLoop Error: %s", msg);
        }
    }  
 }
}