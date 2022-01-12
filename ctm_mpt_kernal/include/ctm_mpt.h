#ifndef CTM_MPT_H
#define CTM_MPT_H

#include <serial/serial.h>
#include <string>

namespace ctm_mpt
{
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

		~CtmMpt();

	public:
		// Motor.
		void mtrInit(uint8_t id);

		void mtrInit(uint8_t* id, size_t n);

		void mtrReset(uint8_t id);

		void mtrReset(uint8_t* id, size_t n);

		void mtrStop(uint8_t id);

		void mtrStop(uint8_t* id, size_t n);

		void mtrZero(uint8_t id);

		void mtrZero(uint8_t* id, size_t n);

		// Set motor position.
		void mtrSetPosAbs(uint8_t id,
						  int32_t pos, int32_t vel = 5000,
						  uint32_t k_i = 1500, uint32_t k_f = 1500,
						  const std::string mode = "EXIT_DIRECTLY");

		void mtrSetPosRel(uint8_t id,
						  int32_t pos, int32_t vel = 5000,
						  uint32_t k_i = 1500, uint32_t k_f = 1500,
						  const std::string mode = "EXIT_DIRECTLY");

		// Get command position.
		void mtrGetPos(uint8_t id);

		void mtrGetPos(uint8_t* id, size_t n);

		// Set motor velocity.
		void mtrSetVel(uint8_t id,
					   int32_t vel, float dur,
					   uint32_t k_i = 5000, uint32_t k_f = 5000);

		void mtrSetVel(uint8_t* id,
					   int32_t* vel, float* dur,
					   uint32_t* k_i, uint32_t* k_f);

		// Get command velocity in rpm.
		void mtrGetVel(uint8_t id);

		void mtrGetVel(uint8_t* id, size_t n);

		// Get temperature.
		void mtrGetTemp(uint8_t id);

		void mtrGetTemp(uint8_t* id, size_t n);

		// Get voltage.
		void mtrGetVolt(uint8_t id);

		void mtrGetVolt(uint8_t* id, size_t n);

		// Sensor.
		void snsrInit(void);

		void snsrRead(void);

	private:
		serial::Serial snsr_serial_1_;

		serial::Serial snsr_serial_2_;
		
		serial::Serial mtr_serial_;

		const bool gossip_ = false;

	private:
		void mtrWrite_(const uint8_t* cmd, const size_t len);
		
		void snsrWrite_(const std::string& cmd);

		bool mtrAtPos_(uint8_t id);

		bool mtrAtHome_(uint8_t id);
		
		void snsrInfoAnalyse_(const uint8_t* data, const std::string prefix);
	};
}

#endif