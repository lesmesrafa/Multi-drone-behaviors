#include "ros/ros.h"
#include <string>
#include "thrust_controller.hpp"
#include "ros_utils_lib/ros_utils.hpp"


int main(int argc, char **argv)
{
    ros::init(argc, argv, ros_utils_lib::getNodeName("thrust_controller"));
    std::cout << ros::this_node::getName()+" Node starting "<< std::endl;
    ThrustController thrust_controller;
    thrust_controller.setUp();
    thrust_controller.start();
    ros::Rate r(200);
    while (ros::ok())
    {
        thrust_controller.run();
        ros::spinOnce();
        r.sleep();
    }
    thrust_controller.stop();
    return 0;
}
