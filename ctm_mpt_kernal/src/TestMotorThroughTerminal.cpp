#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>
#include "ctm_mpt2.h"

int main(int argc, char** argv)
{
    // ROS initialisation.
    // *************************************************
	ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    ros::Publisher trq_pub = nh.advertise<std_msgs::Float32MultiArray>("trq", 100);
    
    const size_t N = 9;
    std_msgs::Float32MultiArray msg;
    std_msgs::MultiArrayDimension dim;
    dim.label = "tau";
    dim.size = N;
    dim.stride = N;
    msg.layout.dim.push_back(dim);
    msg.layout.data_offset = 0;
    msg.data = std::vector<float>(N, 0.0);
    // *************************************************
    


    // Tpye of manipulator.
    // 
    // cd /dev
    // ls -al ttyUSB*
    // sudo chmod a+rw ttyUSB*
    // *************************************************
    // CtmMpt2 m;
    CtmMpt2 m("/dev/ttyUSB0");
    // CtmMpt2 m("/dev/ttyUSB2", "/dev/ttyUSB1");
    //            "sensor 1",     "sensor 2",        "motor".
    // CtmMpt2 m("/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB0");
    // *************************************************


    
    ros::Rate loop_rate(2);
    m.print();
    std::getchar();

    while(ros::ok())
    {
        float angle;
        uint8_t index;
        float dist[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        std::cout << "Motor ID: ";
        std::cin >> index;
        std::cout << "Angle: ";
        std::cin >> angle;
        dist[index-1] = angle;
        ROS_INFO("Motor %d rotates %.1f degree.\n", index, angle);

        // m.move(dist);
    }

    return 0;
}
