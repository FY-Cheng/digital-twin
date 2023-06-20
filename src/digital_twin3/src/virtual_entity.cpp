#include "ve_configure.h"

// control my_robot built in Webots
int main(int argc, char **argv)
{
    ros::init(argc, argv, "virtual_entity", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    // Create "dt_ROS" class
    RosBasic dt_ROS(nh);

    signal(SIGINT, quit);
    
    // sleep(3);
    dt_ROS.Webots_robot();

    // Exit
    dt_ROS.ros_quit();
    return (0);
}