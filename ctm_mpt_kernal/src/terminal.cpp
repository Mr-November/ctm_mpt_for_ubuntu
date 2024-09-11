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
    // CtmMpt2 m("/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB0",        &nh);
    CtmMpt2 m("/dev/ttyUSB0", "/dev/ttyUSB1",                        &nh);
    // *************************************************
    ros::Rate loop(5);
    while(ros::ok())
    {
        m.readTorque();
        loop.sleep();
    }

    m.init();
    m.print();
    // m.zero();
    // m.print();
    // m.init();
    // m.reset();
    // m.move(10, true);
    // m.print();
    ros::Duration(5).sleep();
    std::cout << "Start?";
    std::getchar();
    
    ros::Rate loop_rate(5);
    ros::Time time_start = ros::Time::now();
    float time_cur = 0.0;
    while(ros::ok())
    {
        float dist;
        size_t id;

        std::cout << "Motor ID: ";
        std::cin >> id;
        std::cout << "Distance: ";
        std::cin >> dist;

        time_start = ros::Time::now();
        m.move(id, dist, false);
        // m.run(id, dist);
        time_cur = (ros::Time::now()-time_start).toSec();
        ROS_INFO("Motor %d travels %.2f mm.\n", (int)id, dist);
        ROS_INFO("Execution time of writing motors %.4f ms.\n", time_cur*1000);



        // ros::Duration(3.0).sleep();
        // m.stop();
        // m.run(id, 0.0);

        // time_start = ros::Time::now();
        m.print();
        // time_cur = (ros::Time::now()-time_start).toSec();
        // ROS_INFO("Execution time of print status %.4f ms.\n", time_cur*1000);

        // time_start = ros::Time::now();
        // m.printPose();
        // time_cur = (ros::Time::now()-time_start).toSec();
        // ROS_INFO("Execution time of print pose %.4f ms.\n", time_cur*1000);

        // time_start = ros::Time::now();
        // m.readTorque();
        // time_cur = (ros::Time::now()-time_start).toSec();
        // ROS_INFO("Execution time reading torque %.4f ms.\n", time_cur*1000);

        // // When setting the initial pose, uncomment this block.
        // ros::spinOnce();
        // if(!loop_rate.sleep())
        // {
        //     ROS_WARN("The terminal did not meet the desired rate.");
        // }
    }

    return 0;
}
