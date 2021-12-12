#include <ros/ros.h>
#include "ctm_mpt.h"

#include <stdio.h>
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_kernal");
    ros::NodeHandle nh;

    ctm_mpt::CtmMpt m("/dev/ttyUSB-", "/dev/ttyUSB-", "/dev/ttyUSB0");

    int i = 0, j = 1;

    // for (i = 1; i < 10; i++) { m.MotInit(i); }
    m.mtrInit(j);
    m.mtrSetPos(j, 100000, 20000, 5000, 5000);
    m.mtrReset(j);
    m.mtrSetVel(j, -20000, 3, 5000, 5000);
    // for (i = 1; i < 10; i++)
    // {
    //    m.MotPos(i, -60000, 15000, 5000, 5000);
    //    m.MotVel(i, -20000, 2, 8000, 8000);
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