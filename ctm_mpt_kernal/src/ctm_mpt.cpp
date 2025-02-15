#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include "ctm_mpt.h"

CtmMpt::CtmMpt()
{
	return;
}

CtmMpt::CtmMpt(const std::string& port_name)
: mtr_serial_(port_name,
			  115200,
			  serial::Timeout::simpleTimeout(500),
			  serial::eightbits,
			  serial::parity_none,
			  serial::stopbits_one)
{
	if ( this->mtr_serial_.isOpen() )
	{
		ROS_INFO("Open the motor port.");
	}
	else
	{
		ROS_WARN("Cannot open the motor port.");
	}

	return;
}

CtmMpt::CtmMpt(const std::string& snsr_port_1,
			   			const std::string& snsr_port_2)
: snsr_serial_1_(snsr_port_1,
				 115200,
				 serial::Timeout::simpleTimeout(500),
				 serial::eightbits,
				 serial::parity_none,
				 serial::stopbits_one),
  snsr_serial_2_(snsr_port_2,
				 115200,
				 serial::Timeout::simpleTimeout(500),
				 serial::eightbits,
				 serial::parity_none,
				 serial::stopbits_one)
{
	if ( this->snsr_serial_1_.isOpen() )
	{
		ROS_INFO("Open the first sensor port.");
	}
	else
	{
		ROS_WARN("Cannot open the first sensor port.");
	}

	if ( this->snsr_serial_2_.isOpen() )
	{
		ROS_INFO("Open the second sensor port.");
	}
	else
	{
		ROS_WARN("Cannot open the second sensor port.");
	}

	return;
}

CtmMpt::CtmMpt(const std::string& snsr_port_1,
			   			const std::string& snsr_port_2,
			   			const std::string& mtr_port)
: snsr_serial_1_(snsr_port_1,
				 115200,
				 serial::Timeout::simpleTimeout(500),
				 serial::eightbits,
				 serial::parity_none,
				 serial::stopbits_one),
  snsr_serial_2_(snsr_port_2,
				 115200,
				 serial::Timeout::simpleTimeout(500),
				 serial::eightbits,
				 serial::parity_none,
				 serial::stopbits_one),
  mtr_serial_(mtr_port,
			  115200,
			  serial::Timeout::simpleTimeout(500),
			  serial::eightbits,
			  serial::parity_none,
			  serial::stopbits_one)
{
	if ( this->snsr_serial_1_.isOpen() )
	{
		ROS_INFO("Open the first sensor port.");
	}
	else
	{
		ROS_WARN("Cannot open the first sensor port.");
	}

	if ( this->snsr_serial_2_.isOpen() )
	{
		ROS_INFO("Open the second sensor port.");
	}
	else
	{
		ROS_WARN("Cannot open the second sensor port.");
	}

	if ( this->mtr_serial_.isOpen() )
	{
		ROS_INFO("Open the motor port.");
	}
	else
	{
		ROS_WARN("Cannot open the motor port.");
	}

	return;
}

CtmMpt::~CtmMpt()
{
	return;
}

void CtmMpt::mtrInit(const uint8_t id)
{
	uint8_t cmd_reso[] = { 0x00, 0x10, 0x03, 0x80, 0x00, 0x04, 0x08,
						   0x00, 0x00, 0x00, 0x01, // Electronic gear A = 1.
						   0x00, 0x00, 0x00, 0x01 }; // Electronic gear B = 1.
	uint8_t cmd_rst_paras[] = { 0x00, 0x10, 0x02, 0xb0, 0x00, 0x06, 0x0c,
								0x00, 0x00, 0x75, 0x30, // Operation speed = 30kHz.
								0x00, 0x00, 0x4e, 0x20, // Acceleration = 20kHz.
								0x00, 0x00, 0x00, 0x00 }; // Starting speed = 0Hz.
	
	cmd_reso[0] = id;
	cmd_rst_paras[0] = id;

	this->mtrWrite_(cmd_reso, 15);
	this->mtrWrite_(cmd_rst_paras, 19);
	this->mtrReset(id);

	ROS_INFO("(id %d) Motor is initialised.", id);

	return;
}

void CtmMpt::mtrInit(const uint8_t *const id, const size_t n)
{
	uint8_t cmd_reso[] = { 0x00, 0x10, 0x03, 0x80, 0x00, 0x04, 0x08,
						   0x00, 0x00, 0x00, 0x01, // Electronic gear A = 1.
						   0x00, 0x00, 0x00, 0x01 }; // Electronic gear B = 1.
						   						     // Resolution = A / B * 1000 * FC?UA * (Reducer)80 Hz.
	uint8_t cmd_rst_paras[] = { 0x00, 0x10, 0x02, 0xb0, 0x00, 0x06, 0x0c,
								0x00, 0x00, 0x75, 0x30, // Operation speed = 30kHz.
								0x00, 0x00, 0x4e, 0x20, // Acceleration = 20kHz.
								0x00, 0x00, 0x00, 0x00 }; // Starting speed = 0Hz.
	size_t k = 0;
	std::string str("id");
	char id_k[4];

	for (k = 0; k < n; k++)
	{
		cmd_reso[0] = id[k];
		cmd_rst_paras[0] = id[k];

		this->mtrWrite_(cmd_reso, 15);
		this->mtrWrite_(cmd_rst_paras, 19);

		sprintf(id_k, " %d", id[k]);
		str += id_k;
	}
	this->mtrReset(id, n);
	
	ROS_INFO_STREAM("(" << str << ") Motors are initialised.");

	return;
}

void CtmMpt::mtrReset(const uint8_t id)
{
	uint8_t cmd_rst[] = { 0x00, 0x06, 0x00, 0x7d, 0x00, 0x10 }; // ZHOME operation.

	cmd_rst[0] = id;
	this->mtrWrite_(cmd_rst, 6);

	while (!this->mtrAtHome_(id));
	ROS_INFO("(id %d) Motor is home.", id);

	return;
}

void CtmMpt::mtrReset(const uint8_t *const id, const size_t n)
{
	uint8_t cmd_rst[] = { 0x00, 0x06, 0x00, 0x7d, 0x00, 0x10 };
	size_t k = 0;
	std::string str("id");
	char id_k[4];

	for (k = 0; k < n; k++)
	{
		cmd_rst[0] = id[k];
		this->mtrWrite_(cmd_rst, 6);

		sprintf(id_k, " %d", id[k]);
		str += id_k;
	}

	while (!this->mtrAtHome_(id, n));

	ROS_INFO_STREAM("(" << str << ") Motors are home.");

	return;
}

void CtmMpt::mtrStop(const uint8_t id)
{
	uint8_t cmd_stp[] = { 0x00, 0x06, 0x00, 0x7d, 0x00, 0x20 };

	cmd_stp[0] = id;
	this->mtrWrite_(cmd_stp, 6);

	ROS_INFO("(id %d) Stop.", id);

	return;
}

void CtmMpt::mtrStop(const uint8_t *const id, const size_t n)
{
	uint8_t cmd_stp[] = { 0x00, 0x06, 0x00, 0x7d, 0x00, 0x20 };
	size_t k = 0;
	std::string str("id");
	char id_k[4];

	for (k = 0; k < n; k++)
	{
		cmd_stp[0] = id[k];
		this->mtrWrite_(cmd_stp, 6);

		sprintf(id_k, " %d", id[k]);
		str += id_k;
	}

	ROS_INFO_STREAM("(" << str << ") Stop.");

	return;
}

void CtmMpt::mtrZero(const uint8_t id)
{
	uint8_t cmd_zero_on[] = { 0x00, 0x06, 0x01, 0x8b, 0x00, 0x01 };
	uint8_t cmd_zero_off[] = { 0x00, 0x06, 0x01, 0x8b, 0x00, 0x00 };

	cmd_zero_on[0] = id;
	cmd_zero_off[0] = id;
	this->mtrWrite_(cmd_zero_on, 6);
	this->mtrWrite_(cmd_zero_off, 6);
	ROS_INFO("(id %d) Zero out.", id);

	return;
}

void CtmMpt::mtrZero(const uint8_t *const id, const size_t n)
{
	uint8_t cmd_zero_on[] = { 0x00, 0x06, 0x01, 0x8b, 0x00, 0x01 };
	uint8_t cmd_zero_off[] = { 0x00, 0x06, 0x01, 0x8b, 0x00, 0x00 };
	size_t k = 0;
	std::string str("id");
	char id_k[4];

	for (k = 0; k < n; k++)
	{
		cmd_zero_on[0] = id[k];
		this->mtrWrite_(cmd_zero_on, 6);

		sprintf(id_k, " %d", id[k]);
		str += id_k;
	}
	for (k = 0; k < n; k++)
	{
		cmd_zero_off[0] = id[k];
		this->mtrWrite_(cmd_zero_off, 6);
	}
	ROS_INFO_STREAM("(" << str << ") Zero out.");

	return;
}

// Not recommended. Because wrap setting is enabled, with \pm 900 rev (900,000 step).
void CtmMpt::mtrSetPosAbs(const uint8_t id,
							const int32_t pos, const int32_t vel,
							const uint32_t k_i, const uint32_t k_f)
{
	uint8_t cmd_pos_paras[] = { 0x00, 0x10, 0x18, 0x00, 0x00, 0x0a, 0x14,
								0x00, 0x00, 0x00, 0x01, // Absolute positioning.
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

	return;
}
// void CtmMpt::mtrSetPosAbs(const uint8_t id,
// 							const int32_t pos, const int32_t vel,
// 							const uint32_t k_i, const uint32_t k_f)
// {
// 	uint8_t cmd_pos_paras[] = { 0x00, 0x10, 0x00, 0x58, 0x00, 0x10, 0x20,
// 								0x00, 0x00, 0x00, 0x00, // Operation data number=0.
// 								0x00, 0x00, 0x00, 0x01, // Absolute positioning.
// 								0x00, 0x00, 0x00, 0x00, // Index 15-18, position.
// 								0x00, 0x00, 0x00, 0x00, // Index 19-22, operating speed.
// 								0x00, 0x00, 0x00, 0x00, // Index 23-26, starting speed rate.
// 								0x00, 0x00, 0x00, 0x00, // Index 27-30, stopping deceleration.
// 								0x00, 0x00, 0x03, 0xe8, // Operating current=100.0%.
// 								0x00, 0x00, 0x00, 0x01 }; // Trigger=1: All data updated.

// 	cmd_pos_paras[0] = id;
// 	utils::loadInt32ToUint8Array(&pos, cmd_pos_paras + 15);
// 	utils::loadInt32ToUint8Array(&vel, cmd_pos_paras + 19);
// 	utils::loadUint32ToUint8Array(&k_i, cmd_pos_paras + 23);
// 	utils::loadUint32ToUint8Array(&k_f, cmd_pos_paras + 27);

// 	this->mtrWrite_(cmd_pos_paras, 39);

// 	return;
// }

void CtmMpt::mtrSetPosRel(const uint8_t id,
						  const int32_t pos, const int32_t vel,
						  const uint32_t k_i, const uint32_t k_f)
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

	return;
}
// void CtmMpt::mtrSetPosRel(const uint8_t id,
// 						  const int32_t pos, const int32_t vel,
// 						  const uint32_t k_i, const uint32_t k_f)
// {
// 	uint8_t cmd_pos_paras[] = { 0x00, 0x10, 0x00, 0x58, 0x00, 0x10, 0x20,
// 								0x00, 0x00, 0x00, 0x00, // Operation data number=0.
// 								0x00, 0x00, 0x00, 0x02, // Incremental positioning (based on command position).
// 								0x00, 0x00, 0x00, 0x00, // Index 15-18, position.
// 								0x00, 0x00, 0x00, 0x00, // Index 19-22, operating speed.
// 								0x00, 0x00, 0x00, 0x00, // Index 23-26, starting speed rate.
// 								0x00, 0x00, 0x00, 0x00, // Index 27-30, stopping deceleration.
// 								0x00, 0x00, 0x03, 0xe8, // Operating current=100.0%.
// 								0x00, 0x00, 0x00, 0x01 }; // Trigger=1: All data updated.

// 	cmd_pos_paras[0] = id;
// 	utils::loadInt32ToUint8Array(&pos, cmd_pos_paras + 15);
// 	utils::loadInt32ToUint8Array(&vel, cmd_pos_paras + 19);
// 	utils::loadUint32ToUint8Array(&k_i, cmd_pos_paras + 23);
// 	utils::loadUint32ToUint8Array(&k_f, cmd_pos_paras + 27);

// 	this->mtrWrite_(cmd_pos_paras, 39);

// 	return;
// }

void CtmMpt::mtrSetVel(const uint8_t id,
						const int32_t vel,
						const uint32_t k_i, const uint32_t k_f)
{
	uint8_t cmd_vel_paras[] = { 0x00, 0x10, 0x18, 0x00, 0x00, 0x0a, 0x14,
								0x00, 0x00, 0x00, 0x10, // Continuous (speed control).
								0x00, 0x00, 0x00, 0x00, // Do not change.
								0x00, 0x00, 0x00, 0x00, // Index 15-18, speed.
								0x00, 0x00, 0x00, 0x00, // Index 19-22, starting rate.
								0x00, 0x00, 0x00, 0x00 }; // Index 23-26, stopping deceleration.
	uint8_t cmd_vel_fw_on[] = { 0x00, 0x06, 0x00, 0x7d, 0x40, 0x00 };

	cmd_vel_paras[0] = id;
	cmd_vel_fw_on[0] = id;
	utils::loadInt32ToUint8Array(&vel, cmd_vel_paras + 15);
	utils::loadUint32ToUint8Array(&k_i, cmd_vel_paras + 19);
	utils::loadUint32ToUint8Array(&k_f, cmd_vel_paras + 23);

	this->mtrWrite_(cmd_vel_paras, 27);
	this->mtrWrite_(cmd_vel_fw_on, 6);

	return;
}
// void CtmMpt::mtrSetVel(const uint8_t id,
// 					   const int32_t vel,
// 					   const uint32_t k_i, const uint32_t k_f)
// {
// 	uint8_t cmd_vel_paras[] = { 0x00, 0x10, 0x00, 0x58, 0x00, 0x10, 0x20,
// 								0x00, 0x00, 0x00, 0x00, // Operation data number=0.
// 								0x00, 0x00, 0x00, 0x10, // Continuous (speed control).
// 								0x00, 0x00, 0x00, 0x00, // Do not change.
// 								0x00, 0x00, 0x00, 0x00, // Index 19-22, operating speed.
// 								0x00, 0x00, 0x00, 0x00, // Index 23-26, starting speed rate.
// 								0x00, 0x00, 0x00, 0x00, // Index 27-30, stopping deceleration.
// 								0x00, 0x00, 0x03, 0xe8, // Operating current=100.0%.
// 								0x00, 0x00, 0x00, 0x01 }; // Trigger=1: All data updated.

// 	// It is dangerous. Do not forget to write 0 speed thereafter.
// 	cmd_vel_paras[0] = id;
// 	utils::loadInt32ToUint8Array(&vel, cmd_vel_paras + 19);
// 	utils::loadUint32ToUint8Array(&k_i, cmd_vel_paras + 23);
// 	utils::loadUint32ToUint8Array(&k_f, cmd_vel_paras + 27);

// 	this->mtrWrite_(cmd_vel_paras, 39);

// 	return;
// }

void CtmMpt::mtrGetPos(const uint8_t id, int32_t *const pos)
{
	uint8_t cmd_read_pos[] = { 0x00, 0x03, 0x00, 0xc6, 0x00, 0x02 };
	size_t bytes_read = 0;
	uint8_t rsp[9] = {};

	cmd_read_pos[0] = id;
	this->mtrWrite_(cmd_read_pos, 6);
	bytes_read = this->mtr_serial_.read(rsp, 9);
	utils::loadUint8ArrayToInt32(rsp + 3, pos);

	return;
}

void CtmMpt::mtrGetPos(const uint8_t *const id, const size_t n, int32_t *const pos)
{
	// ros::Time tt = ros::Time::now();
	uint8_t cmd_read_pos[] = { 0x00, 0x03, 0x00, 0xc6, 0x00, 0x02 };
	size_t bytes_read = 0;
	uint8_t rsp[9] = {};
	size_t k = 0;

	for (k = 0; k < n; k++)
	{
		cmd_read_pos[0] = id[k];
		this->mtrWrite_(cmd_read_pos, 6);
		bytes_read = this->mtr_serial_.read(rsp, 9);
		utils::loadUint8ArrayToInt32(rsp + 3, pos + k);
	}
	// std::cout << "Time for position reading: "
	// 	<< (ros::Time::now()-tt).toSec() << " sec" << std::endl;

	return;
}

void CtmMpt::mtrGetVel(const uint8_t id, int32_t *const vel_in_rpm, int32_t *const vel_in_hz)
{
	uint8_t cmd_read_vel[] = { 0x00, 0x03, 0x00, 0xc8, 0x00, 0x04 };
	size_t bytes_read = 0;
	uint8_t rsp[13] = {};

	cmd_read_vel[0] = id;
	this->mtrWrite_(cmd_read_vel, 6);
	bytes_read = this->mtr_serial_.read(rsp, 13);
	utils::loadUint8ArrayToInt32(rsp + 3, vel_in_rpm);
	utils::loadUint8ArrayToInt32(rsp + 7, vel_in_hz);

	return;
}

void CtmMpt::mtrGetVel(const uint8_t *const id, const size_t n, int32_t *const vel_in_rpm, int32_t *const vel_in_hz)
{
	uint8_t cmd_read_vel[] = { 0x00, 0x03, 0x00, 0xc8, 0x00, 0x04 };
	size_t bytes_read = 0;
	uint8_t rsp[13] = {};

	for (size_t k = 0; k < n; k++)
	{
		cmd_read_vel[0] = id[k];
		this->mtrWrite_(cmd_read_vel, 6);
		bytes_read = this->mtr_serial_.read(rsp, 13);
		utils::loadUint8ArrayToInt32(rsp + 3, vel_in_rpm + k);
		utils::loadUint8ArrayToInt32(rsp + 7, vel_in_hz + k);
	}

	return;
}

void CtmMpt::mtrGetTemp(const uint8_t id, int32_t *const drv_temp, int32_t *const mtr_temp)
{
	uint8_t cmd_read_temp[] = { 0x00, 0x03, 0x00, 0xf8, 0x00, 0x04 };
	size_t bytes_read = 0;
	uint8_t rsp[13] = {};

	cmd_read_temp[0] = id;
	this->mtrWrite_(cmd_read_temp, 6);
	bytes_read = this->mtr_serial_.read(rsp, 13);
	utils::loadUint8ArrayToInt32(rsp + 3, drv_temp);
	utils::loadUint8ArrayToInt32(rsp + 7, mtr_temp);

	return;
}

void CtmMpt::mtrGetTemp(const uint8_t *const id, const size_t n, int32_t *const drv_temp, int32_t *const mtr_temp)
{
	uint8_t cmd_read_temp[] = { 0x00, 0x03, 0x00, 0xf8, 0x00, 0x04 };
	size_t bytes_read = 0;
	uint8_t rsp[13] = {};

	for (size_t k = 0; k < n; k++)
	{
		cmd_read_temp[0] = id[k];
		this->mtrWrite_(cmd_read_temp, 6);
		bytes_read = this->mtr_serial_.read(rsp, 13);
		utils::loadUint8ArrayToInt32(rsp + 3, drv_temp + k);
		utils::loadUint8ArrayToInt32(rsp + 7, mtr_temp + k);
	}

	return;
}

void CtmMpt::mtrGetVolt(const uint8_t id, int32_t *const inv_volt, int32_t *const pwr_volt)
{
	uint8_t cmd_read_volt[] = { 0x00, 0x03, 0x01, 0x46, 0x00, 0x04 };
	size_t bytes_read = 0;
	uint8_t rsp[13] = {};

	cmd_read_volt[0] = id;
	this->mtrWrite_(cmd_read_volt, 6);
	bytes_read = this->mtr_serial_.read(rsp, 13);
	utils::loadUint8ArrayToInt32(rsp + 3, inv_volt);
	utils::loadUint8ArrayToInt32(rsp + 7, pwr_volt);
	
	return;
}

void CtmMpt::mtrGetVolt(const uint8_t *const id, const size_t n, int32_t *const inv_volt, int32_t *const pwr_volt)
{
	uint8_t cmd_read_volt[] = { 0x00, 0x03, 0x01, 0x46, 0x00, 0x04 };
	size_t bytes_read = 0;
	uint8_t rsp[13] = {};

	for (size_t k = 0; k < n; k++)
	{
		cmd_read_volt[0] = id[k];
		this->mtrWrite_(cmd_read_volt, 6);
		bytes_read = this->mtr_serial_.read(rsp, 13);
		utils::loadUint8ArrayToInt32(rsp + 3, inv_volt + k);
		utils::loadUint8ArrayToInt32(rsp + 7, pwr_volt + k);
	}

	return;
}

void CtmMpt::snsrInit(void)
{
	std::string cmd_stop_gsd("AT+GSD=STOP");
	std::string cmd_paras("AT+UARTCFG=115200,8,1.00,N");
	std::string cmd_smpf("AT+SMPF=");
	std::string rsp;

	this->snsrWrite_(cmd_stop_gsd);
	this->snsr_serial_1_.readline(rsp, 64, "\r\n");
	this->snsr_serial_2_.readline(rsp, 64, "\r\n");

	this->snsrWrite_(cmd_paras);
	this->snsr_serial_1_.readline(rsp, 64, "\r\n");
	this->snsr_serial_2_.readline(rsp, 64, "\r\n");

	cmd_smpf += std::to_string(this->sampling_rate_);
	this->snsrWrite_(cmd_smpf);
	this->snsr_serial_1_.readline(rsp, 64, "\r\n");
	this->snsr_serial_2_.readline(rsp, 64, "\r\n");

	ROS_INFO_STREAM("Sensors are initialised.");

	return;
}

void CtmMpt::snsrRead(float *const dst)
{
	std::string cmd_read("AT+GOD");
	size_t bytes_read = 0;
	uint8_t data_1[31] = {0};
	uint8_t data_2[31] = {0};

	// Write command of reading data.
	this->snsrWrite_(cmd_read);

	// Collect and parse data from the first sensors.
	bytes_read = this->snsr_serial_1_.read(data_1, 31);
	// printf("Bytes read from (the first) sensors: %d.\n", bytes_read);
	// utils::dispUint8Array(data_1, bytes_read, "Full response: ");

	// Collect and parse data from the second sensors.
	bytes_read = this->snsr_serial_2_.read(data_2, 31);
	// printf("Bytes read from (the second) sensors: %d.\n", bytes_read);
	// utils::dispUint8Array(data_2, bytes_read, "Full response: ");

	if (dst != NULL)
	{
		this->snsrInfoAnalyse_(data_1, dst, 6);
		this->snsrInfoAnalyse_(data_2, dst + 6, 3);
	}

	return;
}

void CtmMpt::snsrGetCfg(void)
{
	std::string cmd_read_paras("AT+UARTCFG=?");
	std::string cmd_read_smpf("AT+SMPF=?");
	std::string rsp_1_1, rsp_1_2, rsp_2_1, rsp_2_2;

	this->snsrWrite_(cmd_read_paras);
	this->snsr_serial_1_.readline(rsp_1_1, 64, "\r\n");
	this->snsr_serial_2_.readline(rsp_1_2, 64, "\r\n");
	if (rsp_1_1 == rsp_1_2)
	{
		rsp_1_1.pop_back();
		rsp_1_1.pop_back();
	}
	else
	{
		ROS_WARN("Inconsistency between two sensor groups.");
	}

	this->snsrWrite_(cmd_read_smpf);
	this->snsr_serial_1_.readline(rsp_2_1, 64, "\r\n");
	this->snsr_serial_2_.readline(rsp_2_2, 64, "\r\n");
	if (rsp_2_1 == rsp_2_2)
	{
		rsp_2_1.pop_back();
		rsp_2_1.pop_back();
	}
	else
	{
		ROS_WARN("Inconsistency between two sensor groups.");
	}

	ROS_INFO_STREAM(rsp_1_1 << ", " << rsp_2_1 << ".");

	return;
}

void CtmMpt::snsrGetMat(void)
{
	std::string cmd_read_mat("AT+DCPM=?");
	std::string rsp_1, rsp_2;

	this->snsrWrite_(cmd_read_mat);
	this->snsr_serial_1_.readline(rsp_1, 512, "\r\n");
	this->snsr_serial_2_.readline(rsp_2, 512, "\r\n");

	rsp_1.insert(9, "\n");
	rsp_1.insert(66, "\n");
	rsp_1.insert(123, "\n");
	rsp_1.insert(180, "\n");
	rsp_1.insert(237, "\n");
	rsp_1.insert(294, "\n");

	rsp_2.insert(9, "\n");
	rsp_2.insert(66, "\n");
	rsp_2.insert(123, "\n");
	rsp_2.insert(180, "\n");
	rsp_2.insert(237, "\n");
	rsp_2.insert(294, "\n");

	ROS_INFO("Sensor group 1 decoupling matrix:");
	std::cout << rsp_1 << std::endl;
	ROS_INFO("Sensor group 2 decoupling matrix:");
	std::cout << rsp_2 << std::endl;

	return;
}

void CtmMpt::mtrWrite_(const uint8_t *const cmd, const size_t len)
{
	uint8_t* new_cmd = new uint8_t[len + 2]; // Create a new array.
	size_t bytes_wrote = 0;
	size_t bytes_read = 0;
	uint8_t rsp[8];

	// Add crc16 correct codes.
	utils::addCRC16(cmd, new_cmd, len);

	// Write to motors.
	bytes_wrote = this->mtr_serial_.write(new_cmd, len + 2);

	// ros::Time tt = ros::Time::now();
	if (new_cmd[1] != 0x03)
	{
		while (this->mtr_serial_.available() != 8);
		this->mtr_serial_.flushInput();
		
	}
	// std::cout << "[" << ros::Time::now() << "] Motor write waiting: "
    //     	  << (ros::Time::now()-tt).toSec()*1000 << " ms." << std::endl;

	// if (new_cmd[1] != 0x03)
	// {
	// 	// while (this->mtr_serial_.available() != 8)
	// 	// {
	// 		// std::cout << "[" << ros::Time::now() << "] i am here."<< std::endl;
	// 		// ros::Duration(0.01).sleep();
	// 	// }
	// 	// this->mtr_serial_.flushInput();
	// 	ros::Duration(0.02).sleep();
	// 	bytes_read = this->mtr_serial_.read(rsp, 8);
	// }

	// Delete the new array.
	delete[] new_cmd;
	
	return;
}

void CtmMpt::snsrWrite_(const std::string& cmd)
{
	std::string new_cmd(cmd + "\r\n");
	size_t k = 0, bytes_wrote = 0, bytes_read = 0;
	std::string rsp_1, rsp_2;

	// Write to sensors.
	bytes_wrote = this->snsr_serial_1_.write(new_cmd);
	bytes_wrote = this->snsr_serial_2_.write(new_cmd);
	
	// for (k = 1; k <= 2; k++)
	// {
	// 	// Display what had been written to sensor group k.
	// 	// printf("Bytes wrote to sensor group %d: %d.\n", k, bytes_wrote);
	// 	// std::cout << "Full command: " << new_cmd << std::endl;

	// 	// Get response from group k.
	// 	if (cmd != "AT+GOD" && cmd.back() != '?')
	// 	{
	// 		if (k == 1)
	// 		{
	// 			bytes_read = this->snsr_serial_1_.readline(rsp_1, 64, "\r\n");
	// 			// printf("Bytes read from sensor group 1: %d.\n", bytes_read);
	// 			// std::cout << "Full response: " << rsp_1 << std::endl;
	// 		}
	// 		else // In this case k = 2.
	// 		{
	// 			bytes_read = this->snsr_serial_2_.readline(rsp_2, 64, "\r\n");
	// 			// printf("Bytes read from sensor group 2: %d.\n", bytes_read);
	// 			// std::cout << "Full response: " << rsp_2 << std::endl;
	// 		}
	// 	}
	// }

	return;
}

bool CtmMpt::mtrAtPos_(const uint8_t id)
{
	bool at_pos;
	uint8_t BIT_AT_POS = 0x40;
	uint8_t cmd_read[] = { 0x00, 0x03, 0x00, 0x7f, 0x00, 0x01 };
	uint8_t rsp[7] = {};

	cmd_read[0] = id;
	this->mtrWrite_(cmd_read, 6);
	this->mtr_serial_.read(rsp, 7);
	at_pos = ( (*(rsp + 3) & BIT_AT_POS) == BIT_AT_POS );

	return at_pos;
}

bool CtmMpt::mtrAtPos_(const uint8_t *const id, const size_t n)
{
	bool at_pos = true;
	uint8_t BIT_AT_POS = 0x40;
	uint8_t cmd_read[] = { 0x00, 0x03, 0x00, 0x7f, 0x00, 0x01 };
	uint8_t rsp[7] = {};
	size_t k = 0;

	for (k = 0; k < n; k++)
	{
		cmd_read[0] = id[k];
		this->mtrWrite_(cmd_read, 6);
		this->mtr_serial_.read(rsp, 7);
		at_pos &= ( (*(rsp + 3) & BIT_AT_POS) == BIT_AT_POS );
		if (!at_pos)
		{
			break;
		}
	}

	return at_pos;
}

bool CtmMpt::mtrAtHome_(const uint8_t id)
{
	bool at_home;
	uint8_t BIT_AT_HOME = 0x10;
	uint8_t cmd_read[] = { 0x00, 0x03, 0x00, 0x7f, 0x00, 0x01 };
	uint8_t rsp[7] = {};

	cmd_read[0] = id;
	this->mtrWrite_(cmd_read, 6);
	this->mtr_serial_.read(rsp, 7);
	at_home = ( (*(rsp + 4) & BIT_AT_HOME) == BIT_AT_HOME );

	return at_home;
}

bool CtmMpt::mtrAtHome_(const uint8_t *const id, const size_t n)
{
	bool at_home = true;
	uint8_t BIT_AT_HOME = 0x10;
	uint8_t cmd_read[] = { 0x00, 0x03, 0x00, 0x7f, 0x00, 0x01 };
	uint8_t rsp[7] = {};
	size_t k = 0;

	for (k = 0; k < n; k++)
	{
		cmd_read[0] = id[k];
		this->mtrWrite_(cmd_read, 6);
		this->mtr_serial_.read(rsp, 7);
		at_home &= ( (*(rsp + 4) & BIT_AT_HOME) == BIT_AT_HOME );
		if (!at_home)
		{
			break;
		}
	}
	
	return at_home;
}

void CtmMpt::snsrInfoAnalyse_(const uint8_t *const data,
								float *const dst, const size_t n)
{	
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

	// Package length.
	// utils::loadUint8ArrayToUint16(data + 2, &pkg_len);
	// printf("Package length:  %d Bytes.\n", pkg_len);

	// Package number.
	// utils::loadUint8ArrayToUint16(data + 4, &pkg_num);
	// printf("Package number:  No.%d.\n", pkg_num);

	// Sensor value.
	for (i = 0; i < ch_size; i++)
	{
		utils::loadUint8ArrayToFloat32(data + 4 * i + 6, ch_val + i);
		if (i < n)
		{
			*(dst + i) = ch_val[i];
		}
	}
	// printf("Correction code: %02x.\n\n", data[30]); // Correction code.

	return;
}

void utils::loadInt32ToUint8Array(const int32_t* src, uint8_t* dst)
{
    const size_t size = 4;
    size_t i = 0;

    for (i = 0; i < size; i++)
	{
		*(dst + i) = *((const uint8_t*)src + 3 - i);
	}

    return;
}

void utils::loadUint32ToUint8Array(const uint32_t* src, uint8_t* dst)
{
    const size_t size = 4;
    size_t i = 0;

    for (i = 0; i < size; i++)
	{
		*(dst + i) = *((const uint8_t*)src + 3 - i);
	}

    return;
}

void utils::loadUint8ArrayToUint16(const uint8_t* src, uint16_t* dst)
{
	*((uint8_t*)dst) = *(src + 1);
    *((uint8_t*)dst + 1) = *src;

    return;
}

void utils::loadUint8ArrayToInt32(const uint8_t* src, int32_t* dst)
{
	const size_t size = 4;
    size_t i = 0;

    for (i = 0; i < size; i++)
	{
		*((uint8_t*)dst + i) = *(src + 3 - i);
	}

	return;
}

void utils::loadUint8ArrayToFloat32(const uint8_t* src, float* dst)
{
    const size_t size = 4;
    size_t i = 4;

    for (i = 0; i < size; i++)
	{
		*((uint8_t*)dst + i) = *(src + i);
	}

    return;
}

void utils::addCRC16(const uint8_t* src, uint8_t* dst, const size_t size)
{
	uint16_t crc = 0xffff; // CRC16-MODBUS.
    size_t i = 0, j = 0;

	//Add check.
    for (i = 0; i < size; i++)
	{
		*(dst + i) = *(src + i);
	}

	for (i = 0; i < size; i++)
	{
		crc ^= (dst[i] & 0x00ff);
		for (j = 0; j < 8; j++)
		{
			if (crc & 0x01)
			{
				crc >>= 1;
				crc ^= 0xa001;
			}
			else
			{
				crc >>= 1;
			}
		}
	}
	*(dst + size) = crc;
	*(dst + size + 1) = crc >> 8;

    return;
}

void utils::dispUint8Array(const uint8_t* src, const size_t size,
						   const std::string& prefix)
{
	size_t i = 0;

	std::cout << prefix;
	for (i = 0; i < size; i++)
	{
		printf("%02x ", src[i]);
	}
	std::cout << "." << std::endl << std::endl;
}
