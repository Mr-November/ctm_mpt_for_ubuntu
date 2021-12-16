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
    // ctm_mpt::CtmMpt m;
    // ctm_mpt::CtmMpt m("/dev/ttyUSB0");
    ctm_mpt::CtmMpt m("/dev/ttyUSB0", "/dev/ttyUSB1");
    // ctm_mpt::CtmMpt m("/dev/ttyUSB0", "/dev/ttyUSB0", "/dev/ttyUSB0");

    int i = 0, j = 3;

    // // For single motor debugging.
    // m.mtrInit(j);
    // m.mtrSetPos(j, 100000, 20000, 5000, 5000);
    // m.mtrReset(j);
    // m.mtrSetVel(j, -20000, 3, 5000, 5000);

    // // For multiple motors debugging.
    // for (i = 1; i < 10; i++)
    // {
    //     m.mtrInit(i);
    //     m.mtrSetPos(i, 100000, 20000, 5000, 5000);
    //     m.mtrReset(i);
    //     m.mtrSetVel(i, -20000, 3, 5000, 5000);
    // }

    // // For sensor info parser debugging.
    // uint8_t g[] = { 0xAA, 0x55,
	// 						0x00, 0x1B,
	// 						0xC4, 0xC7,
	// 						0x01, 0x6A, 0xF4, 0xC0,//6
	// 						0xEF, 0x7D, 0x33, 0xC0,//10
	// 						0x49, 0x62, 0xC9, 0xC0,
	// 						0xA2, 0x5C, 0xC6, 0xBD,
	// 						0xA6, 0x19, 0x8F, 0xBD,
	// 						0xAF, 0xDA, 0x69, 0x3E,
	// 						0x6E};
    // uint8_t h[] = { 0xAA, 0x55,
	// 						0x00, 0x1B,
	// 						0x00, 0x00,
	// 						0xC1, 0x48, 0xF2, 0xC1,//6
	// 						0x71, 0x71, 0x2F, 0xC2,//10
	// 						0x9D, 0xE1, 0x94, 0xC1,
	// 						0x78, 0xA9, 0x6B, 0xC1,
	// 						0x70, 0x57, 0xE5, 0xC1,
	// 						0x9B, 0x8C, 0xE3, 0xC1,
	// 						0xE7 };
    // m.snsrInfoAnalyse_(g, "Test group 1");
	// m.snsrInfoAnalyse_(h, "Test group 2");

    // For sensors debugging.
    m.snsrInit();
    m.snsrRead();


    return 0;
}