#include "igus_arm_driver/igus_arm_driver.h"
#include "igus_arm_driver/arm_controller_sim.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "igus_arm_driver_sim_node");
    ros::NodeHandle nh;
    
    ros::AsyncSpinner spinner(2); 
    
    igus_arm_driver::SimArmController sim_interface(nh);
    igus_arm_driver::IgusArmDriver igus_driver(nh, &sim_interface);

    spinner.start();
    ros::waitForShutdown();
}