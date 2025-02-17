#include <ros/ros.h>
#include "igus_arm_driver/trac_kinematics.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "test_trac_lib_node");
    ros::NodeHandle nh;

    std::string robot_description;
    std::string base_link_name = "joint1", tool_link_name="tool";

    nh.getParam("/robot_description", robot_description);
    
    igus_arm_driver::TracKinematics kin_lin(robot_description, base_link_name, tool_link_name);
    kin_lin.inicitalizeKDL();

    std::vector<double> joint_values = {1.57, 0.65, 2.5};
    geometry_msgs::Point tool_position = kin_lin.getToolPosition(joint_values);
    ROS_INFO("TOOL POSITION: %f %f %f", tool_position.x, tool_position.y, tool_position.z);

    std::vector<double> ik_current_joint_values = {1.57, 0.65, 2.5};
    geometry_msgs::Point desired_point;
    desired_point.x = 0.35;
    desired_point.y = -0.15;
    desired_point.z = 0.35;
    std::vector<double> ik_result = kin_lin.getJointValues(desired_point, ik_current_joint_values);
    ROS_INFO("JOINT ANGLES: %f %f %f", ik_result.at(0), ik_result.at(1), ik_result.at(2));

    std::vector<double> current_joint_values = {1.57, 0.65, 2.5};
    geometry_msgs::Point desired_speed;
    desired_speed.x = 0.0;
    desired_speed.y = 0.0;
    desired_speed.z = 0.2;
    std::vector<double> result = kin_lin.getJointSpeed(current_joint_values, desired_speed);
    ROS_INFO("JOINT SPEED: %f %f %f", result.at(0), result.at(1), result.at(2));

    ros::spin();
}