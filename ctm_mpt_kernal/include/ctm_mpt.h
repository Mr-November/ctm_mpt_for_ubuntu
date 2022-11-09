#ifndef CTM_MPT_H
#define CTM_MPT_H

#include <serial/serial.h>
#include <string>

class CtmMpt
{
public:
	CtmMpt(); // For debugging.

	CtmMpt(const std::string& port_name); // For debugging.

	CtmMpt(const std::string& snsr_port_1,
			const std::string& snsr_port_2); // For debugging.

	CtmMpt(const std::string& snsr_port_1,
			const std::string& snsr_port_2,
			const std::string& mtr_port);

	virtual ~CtmMpt();

public:
	// Motor.
	void mtrInit(const uint8_t id);

	void mtrInit(const uint8_t* id, const size_t n);

	void mtrReset(const uint8_t id);

	void mtrReset(const uint8_t* id, const size_t n);

	void mtrStop(const uint8_t id);

	void mtrStop(const uint8_t* id, const size_t n);

	void mtrZero(const uint8_t id);

	void mtrZero(const uint8_t* id, const size_t n);

	// Set motor position.
	void mtrSetPosAbs(const uint8_t id,
						const int32_t pos, const int32_t vel = 5000,
						const uint32_t k_i = 1500, const uint32_t k_f = 1500,
						const std::string mode = "EXIT_DIRECTLY");

	void mtrSetPosRel(const uint8_t id,
						const int32_t pos, const int32_t vel = 5000,
						const uint32_t k_i = 1500, const uint32_t k_f = 1500,
						const std::string mode = "EXIT_DIRECTLY");

	// Get command position.
	void mtrGetPos(const uint8_t id);

	void mtrGetPos(const uint8_t* id, const size_t n);

	// Set motor velocity.
	void mtrSetVel(const uint8_t id,
					const int32_t vel, float dur,
					const uint32_t k_i = 5000, const uint32_t k_f = 5000);

	void mtrSetVel(const uint8_t* id,
					const int32_t* vel, float* dur,
					const uint32_t* k_i, const uint32_t* k_f);

	// Get command velocity in rpm.
	void mtrGetVel(const uint8_t id);

	void mtrGetVel(const uint8_t* id, const size_t n);

	// Get temperature.
	void mtrGetTemp(const uint8_t id);

	void mtrGetTemp(const uint8_t* id, const size_t n);

	// Get voltage.
	void mtrGetVolt(const uint8_t id);

	void mtrGetVolt(const uint8_t* id, const size_t n);

	// Sensor.
	void snsrInit(void);

	void snsrRead(float* dst = NULL);

	void snsrGetCfg(void);

	void snsrGetMat(void);

private:
	serial::Serial snsr_serial_1_;

	serial::Serial snsr_serial_2_;
	
	serial::Serial mtr_serial_;

	const uint8_t sampling_rate_ = 50;

	const bool gossip_ = false;

private:
	void mtrWrite_(const uint8_t* cmd, const size_t len);
	
	void snsrWrite_(const std::string& cmd);

	bool mtrAtPos_(const uint8_t id);

	bool mtrAtHome_(const uint8_t id);
	
	void snsrInfoAnalyse_(const uint8_t* data, const std::string prefix, float* dst = NULL, const size_t len = 0);
};

#endif