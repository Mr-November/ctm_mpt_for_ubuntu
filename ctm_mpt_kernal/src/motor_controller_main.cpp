#include <ros/ros.h>
#include "ctm_mpt.h"

#include <stdio.h>
#include <iostream>
#include <string>

static const uint8_t ID_ALL[9] = { 1, 2, 3, 4, 5, 6, 7, 8, 9 };
static const uint8_t ID_DISTAL[3] = { 4, 5, 6 };
static const uint8_t ID_MIDDLE[3] = { 7, 8, 9 };
static const uint8_t ID_PROXIMAL[3] = { 1, 2, 3 };

static const size_t N_ALL = 9;
static const size_t N_SEG = 3;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ctm_mpt_motor_controller");
    ros::NodeHandle nh;

    // cd /dev
    // ls -al ttyUSB*
    // sudo chmod a+rw ttyUSB*
    // 
    // ctm_mpt::CtmMpt m;
    ctm_mpt::CtmMpt m("/dev/ttyUSB2");
    // ctm_mpt::CtmMpt m("/dev/ttyUSB0", "/dev/ttyUSB1");
    // ctm_mpt::CtmMpt m("/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2");

    uint8_t i = 0, j = 6;

    // For single motor debugging.
    // m.mtrInit(j);
    // m.mtrSetPosRel(j, -250000, 20000, 5000, 5000, "UNTIL_ARRIVED");
    // m.mtrReset(j);
    // m.mtrSetVel(j, -10000, 2.0, 6000, 6000);
    // m.mtrSetPosRel(j, 20000, 20000, 5000, 5000, "UNTIL_ARRIVED");
    // m.mtrSetPosAbs(j, 10000, 20000, 5000, 5000, "EXIT_DIRECTLY");

    m.mtrInit(ID_ALL, N_ALL);
    m.mtrGetPos(ID_ALL, N_ALL);
    m.mtrGetVel(ID_ALL, N_ALL);
    m.mtrGetTemp(ID_ALL, N_ALL);
    m.mtrGetVolt(ID_ALL, N_ALL);

    // m.mtrInit(ID_ALL, N_ALL);
    // m.mtrZero(ID_ALL, N_ALL);
    // m.mtrStop(ID_ALL, N_ALL);
    // m.mtrGetPos(ID_ALL, N_ALL);
    // m.mtrGetVel(ID_ALL, N_ALL);
    // m.mtrGetTemp(ID_ALL, N_ALL);
    // m.mtrGetVolt(ID_ALL, N_ALL);

    // // For multiple motors debugging.
    // for (i = 1; i < 10; i++)
    // {
    //     m.mtrInit(i);
    //     m.mtrSetPosRel(i, 100000, 20000, 5000, 5000, "UNTIL_ARRIVED");
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

    // // For sensors debugging.
    // m.snsrInit();
    // m.snsrRead();

    return 0;
}