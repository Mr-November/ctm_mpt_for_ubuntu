#ifndef CTM_MPT_H
#define CTM_MPT_H

#include <serial/serial.h>
#include <string>

namespace ctm_mpt
{
	class CtmMpt
	{
	public:
		CtmMpt();
		CtmMpt(const std::string& snsr_port_1,
			   const std::string& snsr_port_2,
			   const std::string& mtr_port);

		~CtmMpt();

	public:
		void mtrInit(uint8_t id);

		void mtrInit(uint8_t* id);

		void mtrReset(uint8_t id);

		void mtrReset(uint8_t* id);

		void mtrSetPos(uint8_t id,
					   int32_t pos, int32_t vel = 5000,
					   uint32_t k_i = 1500, uint32_t k_f = 1500);

		void mtrSetPos(uint8_t* id,
					   int32_t* pos, int32_t* vel,
					   uint32_t* k_i, uint32_t* k_f);

		void mtrGetPos(uint8_t id);

		void mtrGetPos(uint8_t* id);

		void mtrSetVel(uint8_t id,
					   int32_t vel, float dur,
					   uint32_t k_i = 5000, uint32_t k_f = 5000);

		void mtrSetVel(uint8_t* id,
					   int32_t* vel, float* dur,
					   uint32_t* k_i, uint32_t* k_f);

		void snsrInit(void);

		void snsrRead(void);

	private:
		serial::Serial snsr_serial_1_;

		serial::Serial snsr_serial_2_;
		
		serial::Serial mtr_serial_;

	private:
		void mtrWrite_(const uint8_t* cmd, const size_t len);
		
		void snsrWrite_(const std::string& cmd);

		bool mtrAtPos_(uint8_t id);

		bool mtrAtPos_(uint8_t* id);

		bool mtrAtHome_(uint8_t id);

		bool mtrAtHome_(uint8_t* id);

		void snsrInfoAnalyse_(const uint8_t* data);
	};
}

#endif