#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "ctm_mpt.h"

#include <iostream>
#include <cmath>

static const float PERMITTED_TRQ[9] = { 40.0, 40.0, 40.0,
                                        40.0, 40.0, 40.0,
                                        40.0, 40.0, 40.0 };
static const uint8_t ID_ALL[9] = { 1, 2, 3, 4, 5, 6, 7, 8, 9 };
static const size_t N_ALL = 9;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ctm_mpt_torque_monitor");
    ros::NodeHandle nh;
    ros::Publisher trq_pub = nh.advertise<std_msgs::Float32MultiArray>("torque", 100);
    
    // cd /dev
    // ls -al ttyUSB*
    // sudo chmod a+rw ttyUSB*
    // 
    // ctm_mpt::CtmMpt m;
    // ctm_mpt::CtmMpt m("/dev/ttyUSB0");
    // ctm_mpt::CtmMpt m("/dev/ttyUSB0", "/dev/ttyUSB1");
    ctm_mpt::CtmMpt m("/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2");

    std_msgs::Float32MultiArray msg;
    std_msgs::MultiArrayDimension dim;
    size_t i = 0;

    m.snsrInit();
    dim.label = "tau";
    dim.size = 9;
    dim.stride = 9;
    msg.layout.dim.push_back(dim);
    msg.layout.data_offset = 0;
    msg.data = std::vector<float>(9, 0.0);
    ros::Rate loop_rate(20);

    while (ros::ok())
    {
        float trq[9] = { 0.0 };
        bool is_overloaded = false;

        m.snsrRead(trq);
        for (i = 0; i < 9; i++)
        {
            msg.data.at(i) = trq[i];
            if (std::fabs(trq[i]) >= PERMITTED_TRQ[i])
            {
                is_overloaded = true;
                ROS_ERROR("(id %d) Overloaded. Suffer %.4f with %.4f permitted.", i + 1, trq[i], PERMITTED_TRQ[i]);
            }
        }

        trq_pub.publish(msg);

        if (is_overloaded)
        {
            m.mtrStop(ID_ALL, N_ALL);
            m.mtrReset(ID_ALL, N_ALL);
            ROS_ERROR_STREAM("RESET AND EXIT.");

            return 0;
        }

        loop_rate.sleep();
    }

    return 0;
}