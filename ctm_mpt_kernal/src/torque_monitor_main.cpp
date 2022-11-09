#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "ctm_mpt2.h"

#include <iostream>
#include <cmath>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "torque_monitor");
    ros::NodeHandle nh;
    ros::Publisher trq_pub = nh.advertise<std_msgs::Float32MultiArray>("torque", 100);
    
    // cd /dev
    // ls -al ttyUSB*
    // sudo chmod a+rw ttyUSB*
    // 
    // CtmMpt2 m("/dev/ttyUSB0");
    // CtmMpt2 m("/dev/ttyUSB2", "/dev/ttyUSB1");
    //                   "sensor 1",     "sensor 2",     "motor"
    CtmMpt2 m("/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB0");

    std_msgs::Float32MultiArray msg;
    std_msgs::MultiArrayDimension dim;
    dim.label = "tau";
    dim.size = 9;
    dim.stride = 9;
    msg.layout.dim.push_back(dim);
    msg.layout.data_offset = 0;
    msg.data = std::vector<float>(9, 0.0);

    m.snsrInit();
    m.snsrGetCfg();
    m.snsrGetMat();

    const uint8_t ID_ALL[9] = { 1, 2, 3, 4, 5, 6, 7, 8, 9 };
    const size_t N_ALL = 9;


    m.mtrInit(ID_ALL, N_ALL);
    m.mtrGetPos(ID_ALL, N_ALL);
    m.mtrSetPosAbs(4, 50000, 10000, 5000, 5000, "UNTIL_ARRIVED");
    m.mtrGetPos(ID_ALL, N_ALL);

    // m.mtrInit(ID_ALL, N_ALL);
    // m.mtrSetPosAbs(4, 100000);
    // m.mtrGetVel(ID_ALL, N_ALL);
    // m.mtrGetTemp(ID_ALL, N_ALL);
    // m.mtrGetVolt(ID_ALL, N_ALL);

    // init pos: 26100, 10200, 74700, 29800, -2500, 267200, 42460, 10900, 82300
    // m.mtrSetPosRel(5, -10000, 5000, 5000, 5000, "UNTIL_ARRIVED");
    // m.mtrZero(6);

    // m.mtrSetPosAbs(1,  46100, 5000, 5000, 5000, "UNTIL_ARRIVED");
    // m.mtrSetPosAbs(2,  10200, 5000, 5000, 5000, "UNTIL_ARRIVED");
    // m.mtrSetPosAbs(3,  74700, 5000, 5000, 5000, "UNTIL_ARRIVED");
    // m.mtrSetPosAbs(4,  29800, 5000, 5000, 5000, "UNTIL_ARRIVED");
    // m.mtrSetPosAbs(5,  -2500, 5000, 5000, 5000, "UNTIL_ARRIVED");
    // m.mtrSetPosAbs(6, 187200, 5000, 5000, 5000, "UNTIL_ARRIVED");
    // m.mtrSetPosAbs(7,  52460, 5000, 5000, 5000, "UNTIL_ARRIVED");
    // m.mtrSetPosAbs(8,  50900, 5000, 5000, 5000, "UNTIL_ARRIVED");
    // m.mtrSetPosAbs(9,  82300, 5000, 5000, 5000, "UNTIL_ARRIVED");

    // m.mtrSetPosRel(1,   86047, 5000, 5000, 5000, "UNTIL_ARRIVED");
    // m.mtrSetPosRel(2, -795845, 5000, 5000, 5000, "UNTIL_ARRIVED");
    // m.mtrSetPosRel(3,  748569, 5000, 5000, 5000, "UNTIL_ARRIVED");
    // m.mtrSetPosRel(4, -189498, 5000, 5000, 5000, "UNTIL_ARRIVED");
    // m.mtrSetPosRel(5, -184441, 5000, 5000, 5000, "UNTIL_ARRIVED");
    // m.mtrSetPosRel(6,  326152, 5000, 5000, 5000, "UNTIL_ARRIVED");
    // m.mtrSetPosRel(7, -277073, 5000, 5000, 5000, "UNTIL_ARRIVED");
    // m.mtrSetPosRel(8,   44471, 5000, 5000, 5000, "UNTIL_ARRIVED");
    // m.mtrSetPosRel(9,  238334, 5000, 5000, 5000, "UNTIL_ARRIVED");

    // // Continuum manipulator preload.
    // while (ros::ok())
    // {
    //     float trq[9] = { 0.0 };
    //     // bool is_preloaded = true;
    //     size_t k = 0;

    //     m.snsrRead(trq);
    //     // // Proximal.
    //     // k = (trq[0]-INIT_TRQ[0]) < (trq[1]-INIT_TRQ[1]) ? 0 : 1;
    //     // k = (trq[k]-INIT_TRQ[k]) < (trq[2]-INIT_TRQ[2]) ? k : 2;
    //     // if (trq[k] < INIT_TRQ[k])
    //     // {
    //     //     is_preloaded = false;
    //     //     m.mtrSetPosRel(k+1, 100, 500, 1000, 1000, "UNTIL_ARRIVED");
    //     // }
    //     // // Middle.
    //     // k = (trq[6]-INIT_TRQ[6]) < (trq[7]-INIT_TRQ[7]) ? 6 : 7;
    //     // k = (trq[k]-INIT_TRQ[k]) < (trq[8]-INIT_TRQ[8]) ? k : 8;
    //     // if (trq[k] < INIT_TRQ[k])
    //     // {
    //     //     is_preloaded = false;
    //     //     m.mtrSetPosRel(k+1, 100, 500, 1000, 1000, "UNTIL_ARRIVED");
    //     // }
    //     // // Distal.
    //     // k = (trq[3]-INIT_TRQ[3]) < (trq[4]-INIT_TRQ[4]) ? 3 : 4;
    //     // k = (trq[k]-INIT_TRQ[k]) < (trq[5]-INIT_TRQ[5]) ? k : 5;
    //     // if (trq[k] < INIT_TRQ[k])
    //     // {
    //     //     is_preloaded = false;
    //     //     m.mtrSetPosRel(k+1, 100, 500, 1000, 1000, "UNTIL_ARRIVED");
    //     // }

    //     for (k = 0; k < 9; k++)
    //     {
    //         msg.data.at(k) = trq[k] - INIT_TRQ[k];
    //     }
    //     trq_pub.publish(msg);

    //     // if (is_preloaded)
    //     // {
    //     //     m.mtrGetPos(ID_ALL, N_ALL);
    //     //     ROS_INFO_STREAM("Preloaded.\n");

    //     //     // return 0;
    //     // }
    // }

    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        size_t k = 0;
        float trq[9] = { 0.0 };
        bool is_overloaded = m.readTrq(trq);

        if (is_overloaded)
        {
            m.stopAll();
            m.resetAll();

            return 0;
        }

        for (k = 0; k < 9; k++)
        {
            msg.data.at(k) = trq[k];
        }
        trq_pub.publish(msg);
        loop_rate.sleep();
    }

    return 0;
}