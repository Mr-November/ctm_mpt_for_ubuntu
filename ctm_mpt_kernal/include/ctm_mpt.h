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
	void mtrDebug(void);

	void mtrInit(const uint8_t id);

	void mtrInit(const uint8_t* id, const size_t n);

	void mtrReset(const uint8_t id);

	void mtrReset(const uint8_t* id, const size_t n);

	void mtrStop(const uint8_t id);

	void mtrStop(const uint8_t* id, const size_t n);

	void mtrZero(const uint8_t id);

	void mtrZero(const uint8_t* id, const size_t n);

	// Set motor position.
	// Not recommended. Because wrap setting is enabled, with \pm 900 rev.
	void mtrSetPosAbs(const uint8_t id,
						const int32_t pos, const int32_t vel = 10000,
						const uint32_t k_i = 20000, const uint32_t k_f = 20000,
						const std::string mode = "EXIT_DIRECTLY");

	void mtrSetPosRel(const uint8_t id,
						const int32_t pos, const int32_t vel = 10000,
						const uint32_t k_i = 20000, const uint32_t k_f = 20000,
						const std::string mode = "EXIT_DIRECTLY");

	// Get command position.
	void mtrGetPos(const uint8_t id, int32_t* ps);

	// Set motor velocity.
	void mtrSetVel(const uint8_t id,
					const int32_t vel, float dur,
					const uint32_t k_i = 20000, const uint32_t k_f = 20000);

	void mtrSetVel(const uint8_t* id,
					const int32_t* vel, float* dur,
					const uint32_t* k_i, const uint32_t* k_f);

	// Get command velocity in rpm.
	void mtrGetVel(const uint8_t id, int32_t* vel_in_rpm, int32_t* vel_in_hz);

	// Get temperature.
	void mtrGetTemp(const uint8_t id, int32_t* drv_temp, int32_t* mtr_temp);

	// Get voltage.
	void mtrGetVolt(const uint8_t id, int32_t* inv_volt, int32_t* pwr_volt);

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

protected:
	bool mtrAtPos_(const uint8_t id);

	bool mtrAtHome_(const uint8_t id);

private:
	void mtrWrite_(const uint8_t* cmd, const size_t len);
	
	void snsrWrite_(const std::string& cmd);
	
	void snsrInfoAnalyse_(const uint8_t* data, const std::string prefix, float* dst = NULL, const size_t len = 0);
};

#endif