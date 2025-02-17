#include "igus_arm_driver/igus_arm_driver.h"
#include "igus_arm_driver/arm_controller_hard.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "igus_arm_driver_hard_node");
    ros::NodeHandle nh;
    
    igus_arm_driver::HardArmController hard_interface(nh);
    igus_arm_driver::IgusArmDriver igus_driver(nh, &hard_interface);

    ros::spin();
}