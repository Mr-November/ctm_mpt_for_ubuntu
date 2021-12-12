#include <ros/ros.h>
#include "ctm_mpt.h"

#include <stdio.h>
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_kernal");
    ros::NodeHandle nh;
    
    // cd /dev
    // ls -al ttyUSB*
    // sudo chmod a+rw ttyUSB*
    ctm_mpt::CtmMpt m("/dev/ttyUSB0");

    int i = 0, j = 3;

    // For single motor debugging.
    m.mtrInit(j);
    m.mtrSetPos(j, 100000, 20000, 5000, 5000);
    m.mtrReset(j);
    m.mtrSetVel(j, -20000, 3, 5000, 5000);

    // // For multiple motors debugging.
    // for (i = 1; i < 10; i++)
    // {
    //     m.mtrInit(i);
    //     m.mtrSetPos(i, 100000, 20000, 5000, 5000);
    //     m.mtrReset(i);
    //     m.mtrSetVel(i, -20000, 3, 5000, 5000);
    // }

    // const char* str = "AT+UARTCFG=115200,8,1.00,N\r\n";
    // std::cout << str;
    // unsigned char cmd[64] = { 0x00 };
    // for (i = 0; i < 30; i++)
    // {
    //    cmd[i] = *(str + i);
    // }
    // for (i = 0; i < 64; i++)
    // {
    //    printf("0x%02x, ", cmd[i]);
    // }

    // m.SenInit();
    // m.SenRec();


    return 0;
}