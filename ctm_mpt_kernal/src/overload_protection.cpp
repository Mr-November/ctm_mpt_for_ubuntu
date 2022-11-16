#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>
#include "ctm_mpt2.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "overload_protection");
    ros::NodeHandle nh;
    ros::Publisher trq_pub = nh.advertise<std_msgs::Float32MultiArray>("trq", 100);
    
    // cd /dev
    // ls -al ttyUSB*
    // sudo chmod a+rw ttyUSB*
    // 
    //            "sensor 1",     "sensor 2",        "motor".
    CtmMpt2 m("/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB0");
    const size_t N = 9;
    float trq[N] = {0.0};
    size_t i = 0;

    std_msgs::Float32MultiArray msg;
    std_msgs::MultiArrayDimension dim;
    dim.label = "tau";
    dim.size = N;
    dim.stride = N;
    msg.layout.dim.push_back(dim);
    msg.layout.data_offset = 0;
    msg.data = std::vector<float>(N, 0.0);

    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        bool is_overloaded = m.read(trq);
        if (is_overloaded)
        {
            m.stop();
            m.reset();

            return 0;
        }

        while (i < N)
        {
            msg.data.at(i) = trq[i];
            i++;
        }
        trq_pub.publish(msg);
        loop_rate.sleep();
    }

    return 0;
}