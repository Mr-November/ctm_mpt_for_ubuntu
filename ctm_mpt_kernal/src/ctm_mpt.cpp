#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include "ctm_mpt.h"
#include "utils.h"

ctm_mpt::CtmMpt::CtmMpt()
{
	return;
}

ctm_mpt::CtmMpt::CtmMpt(const std::string& snsr_port_1,
			   			const std::string& snsr_port_2,
			   			const std::string& mtr_port)
: snsr_serial_1_(snsr_port_1,
				 115200,
				 serial::Timeout::simpleTimeout(1000),
				 serial::eightbits,
				 serial::parity_none,
				 serial::stopbits_one),
  snsr_serial_2_(snsr_port_2,
				 115200,
				 serial::Timeout::simpleTimeout(1000),
				 serial::eightbits,
				 serial::parity_none,
				 serial::stopbits_one),
  mtr_serial_(mtr_port,
			  115200,
			  serial::Timeout::simpleTimeout(1000),
			  serial::eightbits,
			  serial::parity_none,
			  serial::stopbits_one)
{
	if ( this->snsr_serial_1_.isOpen() )
	{
		ROS_INFO("The first serial port of sensors is opened.");
	}
	else
	{
		ROS_WARN("Cannot open the first serial port of sensors.");
	}

	if ( this->snsr_serial_2_.isOpen() )
	{
		ROS_INFO("The second serial port of sensors is opened.");
	}
	else
	{
		ROS_WARN("Cannot open the second serial port of sensors.");
	}

	if ( this->mtr_serial_.isOpen() )
	{
		ROS_INFO("The serial port of motors is opened.");
	}
	else
	{
		ROS_WARN("Cannot open the serial port of motors.");
	}

	return;
}

ctm_mpt::CtmMpt::~CtmMpt()
{
	return;
}

void ctm_mpt::CtmMpt::mtrInit(uint8_t id)
{
	uint8_t cmd_reso[] = { 0x00, 0x10, 0x03, 0x80, 0x00, 0x04, 0x08,
						   0x00, 0x00, 0x00, 0x01, // Electronic gear A = 1.
						   0x00, 0x00, 0x00, 0x01 }; // Electronic gear B = 1, resolution = 30,000Hz.
	uint8_t cmd_rst_paras[] = { 0x00, 0x10, 0x02, 0xb0, 0x00, 0x06, 0x0c,
								0x00, 0x00, 0x1f, 0x40, // Operation speed = 8000Hz.
								0x00, 0x00, 0x07, 0xd0, // Acceleration = 2000Hz.
								0x00, 0x00, 0x00, 0x00 }; // Starting speed = 0Hz.

	cmd_reso[0] = id;
	cmd_rst_paras[0] = id;

	this->mtrWrite_(cmd_reso, 15);
	this->mtrWrite_(cmd_rst_paras, 19);

	this->mtrReset(id);

	return;
}

void ctm_mpt::CtmMpt::mtrInit(uint8_t* id)
{
	return;
}

void ctm_mpt::CtmMpt::mtrReset(uint8_t id)
{
	uint8_t cmd_rst[] = { 0x00, 0x06, 0x00, 0x7d, 0x00, 0x10 };

	cmd_rst[0] = id;
	this->mtrWrite_(cmd_rst, 6);

	while (!this->mtrAtHome_(id));
	ROS_INFO("Motor (id = %d) is now at home.", id);

	return;
}

void ctm_mpt::CtmMpt::mtrReset(uint8_t* id)
{
	return;
}

void ctm_mpt::CtmMpt::mtrSetPos(uint8_t id,
								int32_t pos, int32_t vel,
								uint32_t k_i, uint32_t k_f)
{
	uint8_t cmd_pos_paras[] = { 0x00, 0x10, 0x18, 0x00, 0x00, 0x0a, 0x14,
								0x00, 0x00, 0x00, 0x02, // Incremental positioning (based on command position).
								0x00, 0x00, 0x00, 0x00, // Index 11-14, position.
								0x00, 0x00, 0x00, 0x00, // Index 15-18, speed.
								0x00, 0x00, 0x00, 0x00, // Index 19-22, starting rate.
								0x00, 0x00, 0x00, 0x00 }; // Index 23-26, stopping deceleration.
	uint8_t cmd_pos_on[] = { 0x00, 0x06, 0x00, 0x7d, 0x00, 0x08 };
	uint8_t cmd_pos_off[] = { 0x00, 0x06, 0x00, 0x7d, 0x00, 0x00 };

	cmd_pos_paras[0] = id;
	cmd_pos_on[0] = id;
	cmd_pos_off[0] = id;

	utils::loadInt32ToUint8Array(&pos, cmd_pos_paras + 11);
	utils::loadInt32ToUint8Array(&vel, cmd_pos_paras + 15);
	utils::loadUint32ToUint8Array(&k_i, cmd_pos_paras + 19);
	utils::loadUint32ToUint8Array(&k_f, cmd_pos_paras + 23);

	this->mtrWrite_(cmd_pos_paras, 27);
	this->mtrWrite_(cmd_pos_on, 6);
	this->mtrWrite_(cmd_pos_off, 6);

	while (!this->mtrAtPos_(id));
	ROS_INFO("Motor (id = %d) has arrived.", id);

	return;
}

void ctm_mpt::CtmMpt::mtrSetVel(uint8_t id,
								int32_t vel, float dur,
								uint32_t k_i, uint32_t k_f)
{
	uint8_t cmd_vel_paras[] = { 0x00, 0x10, 0x18, 0x00, 0x00, 0x0a, 0x14,
								0x00, 0x00, 0x00, 0x10, // Continuous (speed control).
								0x00, 0x00, 0x00, 0x00, // Do not change.
								0x00, 0x00, 0x00, 0x00, // Index 15-18, speed.
								0x00, 0x00, 0x00, 0x00, // Index 19-22, starting rate.
								0x00, 0x00, 0x00, 0x00 }; // Index 23-26, stopping deceleration.
	uint8_t cmd_vel_on[] = { 0x00, 0x06, 0x00, 0x7d, 0x00, 0x08 };
	uint8_t cmd_vel_off[] = { 0x00, 0x06, 0x00, 0x7d, 0x00, 0x20 };

	cmd_vel_paras[0] = id;
	cmd_vel_on[0] = id;
	cmd_vel_off[0] = id;

	utils::loadInt32ToUint8Array(&vel, cmd_vel_paras + 15);
	utils::loadUint32ToUint8Array(&k_i, cmd_vel_paras + 19);
	utils::loadUint32ToUint8Array(&k_f, cmd_vel_paras + 23);

	this->mtrWrite_(cmd_vel_paras, 27);
	this->mtrWrite_(cmd_vel_on, 6);

	ros::Duration(dur).sleep(); // sleep for a certain amount of time.
	
	this->mtrWrite_(cmd_vel_off, 6);

	return;
}

void ctm_mpt::CtmMpt::snsrInit(void)
{
	std::string cmd_stop_gsd("AT+GSD=STOP");
	std::string cmd_uart_paras("AT+UARTCFG=115200,8,1.00,N");
	std::string cmd_smpf("AT+SMPF=50");

	this->snsrWrite_(cmd_stop_gsd);
	this->snsrWrite_(cmd_uart_paras);
	this->snsrWrite_(cmd_smpf);

	return;
}

void ctm_mpt::CtmMpt::snsrRead(void)
{
	std::string cmd_read("AT+GOD");
	size_t i = 0, bytes_read = 0;
	uint8_t data_1[31] = {}, data_2[31] = {};

	// Write command of reading data.
	this->snsrWrite_(cmd_read);

	// Collect data from the first sensors.
	bytes_read = this->snsr_serial_1_.read(data_1, 31);
	ROS_INFO("Bytes read from (the first) sensors: %d.", bytes_read);
	std::cout << "Full response: ";
	for (i = 0; i < bytes_read; i++)
	{
		printf("%02x ", data_1[i]);
	}
	std::cout << "." << std::endl;

	// Collect data from the second sensors.
	bytes_read = this->snsr_serial_2_.read(data_2, 31);
	ROS_INFO("Bytes read from (the second) sensors: %d.", bytes_read);
	std::cout << "Full response: ";
	for (i = 0; i < bytes_read; i++)
	{
		printf("%02x ", data_2[i]);
	}
	std::cout << "." << std::endl;

	this->snsrInfoAnalyse_(data_1);
	this->snsrInfoAnalyse_(data_2);

	return;
}

void ctm_mpt::CtmMpt::mtrWrite_(const uint8_t* cmd, const size_t len)
{
	uint8_t* new_cmd = new uint8_t[len + 2]; // Create a new array.
	size_t i = 0, bytes_wrote = 0, bytes_read = 0;
	uint8_t rsp[64] = {};

	// Add crc16 correct codes.
	utils::addCRC16(cmd, new_cmd, len);

	// Write to motors.
	bytes_wrote = this->mtr_serial_.write(new_cmd, len + 2);
	ROS_INFO("Bytes wrote to motors: %d.", bytes_wrote);
	std::cout << "Full command: ";
	for (i = 0; i < len + 2; i++)
	{
		printf("%02x ", new_cmd[i]);
	}
	std::cout << "." << std::endl;

	if (new_cmd[1] != 0x03) // If the command is not to read the motor register.
	{
		bytes_read = this->mtr_serial_.read(rsp, 8);
		ROS_INFO("Bytes read from motors: %d.", bytes_read);
		std::cout << "Full response: ";
		for (i = 0; i < bytes_read; i++)
		{
			printf("%02x ", rsp[i]);
		}
		std::cout << "." << std::endl;
	}
	else
	{
		;
	}

	// Delete the new array.
	delete[] new_cmd;

	return;
}

void ctm_mpt::CtmMpt::snsrWrite_(const std::string& cmd)
{
	std::string new_cmd(cmd + "\r\n");
	size_t bytes_wrote = 0, bytes_read = 0;
	std::string rsp;

	// Write the first serial port of sensors.
	bytes_wrote = this->snsr_serial_1_.write(new_cmd);
	ROS_INFO("Bytes wrote to (the first) sensors: %d.", bytes_wrote);
	std::cout << "Full command: " << new_cmd;
	// Get response.
	if (cmd != "AT+GOD" && cmd != "AT+GSD" && cmd != "AT+GSD=STOP")
	{
		bytes_read = this->snsr_serial_1_.readline(rsp, 64, "\n");
		ROS_INFO("Bytes read from (the first) sensors: %d.", bytes_read);
		std::cout << "Full response: " << rsp;
	}
	else if (cmd == "AT+GOD")
	{
		;
	}
	else if (cmd == "AT+GSD")
	{
		ROS_WARN("Caution! Overflow!");
	}
	else // In this case, cmd == "AT+GSD=STOP".
	{
		ROS_WARN("Caution! Overflow!");
	}

	// Write the second serial port of sensors.
	bytes_wrote = this->snsr_serial_2_.write(new_cmd);
	ROS_INFO("Bytes wrote to (the second) sensors: %d.", bytes_wrote);
	std::cout << "Full command: " << new_cmd;
	// Get response.
	if (cmd != "AT+GOD" && cmd != "AT+GSD" && cmd != "AT+GSD=STOP")
	{
		bytes_read = this->snsr_serial_2_.readline(rsp, 64, "\n");
		ROS_INFO("Bytes read from (the second) sensors: %d.", bytes_read);
		std::cout << "Full response: " << rsp;
	}
	else if (cmd == "AT+GOD")
	{
		;
	}
	else if (cmd == "AT+GSD")
	{
		ROS_WARN("Caution! Overflow!");
	}
	else // In this case, cmd == "AT+GSD=STOP".
	{
		ROS_WARN("Caution! Overflow!");
	}

	return;
}

bool ctm_mpt::CtmMpt::mtrAtPos_(uint8_t id)
{
	bool at_pos = false;
	uint8_t BIT_AT_POS = 0x40;
	uint8_t cmd_read[] = { 0x00, 0x03, 0x00, 0x7f, 0x00, 0x01 };
	uint8_t rsp[64] = {};

	cmd_read[0] = id;

	this->mtrWrite_(cmd_read, 6);
	this->mtr_serial_.read(rsp, 7);
	at_pos = ( (*(rsp + 3) & BIT_AT_POS) == BIT_AT_POS );

	return at_pos;
}

bool mtrAtPos_(uint8_t* id)
{
	return false;
}

bool ctm_mpt::CtmMpt::mtrAtHome_(uint8_t id)
{
	bool at_home = false;
	uint8_t BIT_AT_HOME = 0x10;
	uint8_t cmd_read[] = { 0x00, 0x03, 0x00, 0x7f, 0x00, 0x01 };
	uint8_t rsp[64] = {};

	cmd_read[0] = id;

	this->mtrWrite_(cmd_read, 6);
	this->mtr_serial_.read(rsp, 7);
	at_home = ( (*(rsp + 3) & BIT_AT_HOME) == BIT_AT_HOME );

	return at_home;
}

bool mtrAtHome_(uint8_t* id)
{
	return false;
}

void ctm_mpt::CtmMpt::snsrInfoAnalyse_(const uint8_t* data)
{
	// For debugging.
	//uint8_t f[] = { 0xAA, 0x55,
	//						0x00, 0x1B,
	//						0x00, 0x00,
	//						0xC1, 0x48, 0xF2, 0xC1,//6
	//						0x71, 0x71, 0x2F, 0xC2,//10
	//						0x9D, 0xE1, 0x94, 0xC1,
	//						0x78, 0xA9, 0x6B, 0xC1,
	//						0x70, 0x57, 0xE5, 0xC1,
	//						0x9B, 0x8C, 0xE3, 0xC1,
	//						0xE7,
	//						0xAA, 0x55,
	//						0x00, 0x1B,
	//						0x00, 0x00,
	//						0xC1, 0x48, 0xF2, 0xC1,
	//						0x71, 0x71, 0x2F, 0xC2,
	//						0x9D, 0xE1, 0x94, 0xC1,
	//						0x78, 0xA9, 0x6B, 0xC1,
	//						0x70, 0x57, 0xE5, 0xC1,
	//						0x9B, 0x8C, 0xE3, 0xC1,
	//						0xE7 };
	//uint8_t g[] = { 0xAA, 0x55,
	//						0x00, 0x1B,
	//						0xC4, 0xC7,
	//						0x01, 0x6A, 0xF4, 0xC0,//6
	//						0xEF, 0x7D, 0x33, 0xC0,//10
	//						0x49, 0x62, 0xC9, 0xC0,
	//						0xA2, 0x5C, 0xC6, 0xBD,
	//						0xA6, 0x19, 0x8F, 0xBD,
	//						0xAF, 0xDA, 0x69, 0x3E,
	//						0x6E,
	//						0xAA, 0x55,
	//						0x00, 0x1B,
	//						0xC4, 0xC7,
	//						0x01, 0x6A, 0xF4, 0xC0,//6
	//						0xEF, 0x7D, 0x33, 0xC0,//10
	//						0x49, 0x62, 0xC9, 0xC0,
	//						0xA2, 0x5C, 0xC6, 0xBD,
	//						0xA6, 0x19, 0x8F, 0xBD,
	//						0xAF, 0xDA, 0x69, 0x3E,
	//						0x6E };
	uint16_t pkg_len = 0;
	uint16_t pkg_num = 0;
	float ch_val[6] = {};
	const size_t ch_size = 6;
	size_t i = 0;

	if (data[0] != 0xaa || data[1] != 0x55)
	{
		ROS_WARN("Cannot parse data from sensors.");
		
		return;
	}
	else
	{
		std::cout << "Here is the result." << std::endl;
	}

	// Package length.
	utils::loadUint8ArrayToUint16(data + 2, &pkg_len);
	printf("Package length:  %d Bytes.\n", pkg_len);

	// Package number.
	utils::loadUint8ArrayToUint16(data + 4, &pkg_num);
	printf("Package number:  No.%d.\n", pkg_num);

	// Sensor value.
	for (i = 0; i < ch_size; i++)
	{
		utils::loadUint8ArrayToFloat32(data + 4 * i + 6, ch_val + i);
	}
	std::cout << "Group 1 channel: ( ";
	for (i = 0; i < ch_size; i++)
	{
		printf("%.4f ", ch_val[i]);
	}
	std::cout << ")." << std::endl;

	// Correction code.
	printf("Correction code: %02x.\n", data[30]);

	return;
}
