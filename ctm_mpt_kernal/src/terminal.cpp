#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>
#include "ctm_mpt2.h"

int main(int argc, char** argv)
{
    // ROS initialisation.
    // *************************************************
	ros::init(argc, argv, "motor_terminal_node");
    ros::NodeHandle nh;
    // *************************************************
    


    // Tpye of manipulator.
    // 
    // cd /dev
    // ls -al ttyUSB*
    // sudo chmod a+rw ttyUSB*
    // *************************************************
    // CtmMpt2 m;
    //        "sensor 1",     "sensor 2",     "motor",       "ros handle".
    CtmMpt2 m("/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB0",        &nh);
    // *************************************************

    // m.print();
    // m.zero();
    // m.print();
    // m.init();
    // m.reset();
    m.relax(10);
    ros::Duration(5).sleep();
    std::cout << "Start?";
    std::getchar();

    while(ros::ok())
    {
        float dist;
        size_t id;

        std::cout << "Motor ID: ";
        std::cin >> id;
        std::cout << "Distance: ";
        std::cin >> dist;
        m.move(id, dist, true);
        ROS_INFO("Motor %d travels %.2f mm.\n", (int)id, dist);
        m.printPose();
    }

    return 0;
}
