#ifndef CTM_MPT_H
#define CTM_MPT_H

#include <serial/serial.h>
#include <string>

class CtmMpt
{
public:
	CtmMpt(); // For debugging.

	CtmMpt(const std::string &port_name); // For debugging.

	CtmMpt(const std::string &snsr_port_1,
		   const std::string &snsr_port_2); // For debugging.

	CtmMpt(const std::string &snsr_port_1,
		   const std::string &snsr_port_2,
		   const std::string &mtr_port);

	virtual ~CtmMpt();

public:
	// Motor.
	void mtrInit(const uint8_t id);

	void mtrInit(const uint8_t *const id, const size_t n);

	void mtrReset(const uint8_t id);

	void mtrReset(const uint8_t *const id, const size_t n);

	void mtrStop(const uint8_t id);

	void mtrStop(const uint8_t *const id, const size_t n);

	void mtrZero(const uint8_t id);

	void mtrZero(const uint8_t *const id, const size_t n);

	// Set motor position.
	// Not recommended. Because wrap setting is enabled, with \pm 900 rev.
	// If wait = true, stay until arrived.
	// If wait = false, exit directly.
	void mtrSetPosAbs(const uint8_t id,
					  const int32_t pos, const int32_t vel = 1000,
					  const uint32_t k_i = 1000000, const uint32_t k_f = 1000000);

	void mtrSetPosRel(const uint8_t id,
					  const int32_t pos, const int32_t vel = 1000,
					  const uint32_t k_i = 1000000, const uint32_t k_f = 1000000);

	// Set motor velocity.
	void mtrSetVel(const uint8_t id,
				   const int32_t vel,
				   const uint32_t k_i = 1000000, const uint32_t k_f = 1000000);

	// Reading motor is very slow (approx 0.15 sec).
	// So I write all commands in one loop, then read feedbacks in the other loop.
	// Get command position.
	void mtrGetPos(const uint8_t id, int32_t *const pos);
	void mtrGetPos(const uint8_t *const id, const size_t n, int32_t *const pos);

	// Get command velocity in rpm.
	void mtrGetVel(const uint8_t id, int32_t *const vel_in_rpm, int32_t *const vel_in_hz);
	void mtrGetVel(const uint8_t *const id, const size_t n, int32_t *const vel_in_rpm, int32_t *const vel_in_hz);

	// Get temperature.
	void mtrGetTemp(const uint8_t id, int32_t *const drv_temp, int32_t *const mtr_temp);
	void mtrGetTemp(const uint8_t *const id, const size_t n, int32_t *const drv_temp, int32_t *const mtr_temp);

	// Get voltage.
	void mtrGetVolt(const uint8_t id, int32_t *const inv_volt, int32_t *const pwr_volt);
	void mtrGetVolt(const uint8_t *const id, const size_t n, int32_t *const inv_volt, int32_t *const pwr_volt);

	// Sensor.
	void snsrInit(void);

	void snsrRead(float *const dst = NULL);

	void snsrGetCfg(void);

	void snsrGetMat(void);

private:
	serial::Serial snsr_serial_1_;

	serial::Serial snsr_serial_2_;

	serial::Serial mtr_serial_;

	const uint8_t sampling_rate_ = 50;

protected:
	bool mtrAtPos_(const uint8_t id);

	bool mtrAtPos_(const uint8_t *const id, const size_t n);

	bool mtrAtHome_(const uint8_t id);
	
	bool mtrAtHome_(const uint8_t *const id, const size_t n);

private:
	void mtrWrite_(const uint8_t *const cmd, const size_t len);

	void snsrWrite_(const std::string& cmd);

	void snsrInfoAnalyse_(const uint8_t *const data,
							float *const dst = NULL, const size_t len = 0);
};

namespace utils
{
    void loadInt32ToUint8Array(const int32_t* src, uint8_t* dst);

    void loadUint32ToUint8Array(const uint32_t* src, uint8_t* dst);

    void loadUint8ArrayToUint16(const uint8_t* src, uint16_t* dst);

    void loadUint8ArrayToInt32(const uint8_t* src, int32_t* dst);

    void loadUint8ArrayToFloat32(const uint8_t* src, float* dst);

    void addCRC16(const uint8_t* src, uint8_t* dst, const size_t size);

    void dispUint8Array(const uint8_t* src, const size_t size,
                        const std::string& prefix);
}

#endif