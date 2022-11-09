
#ifndef CTM_MPT2_H
#define CTM_MPT2_H

#include "ctm_mpt.h"

class CtmMpt2 : public CtmMpt
{
public:
    CtmMpt2(const std::string &port_name);

    CtmMpt2(const std::string &snsr_port_1,
            const std::string &snsr_port_2);

    CtmMpt2(const std::string &snsr_port_1,
            const std::string &snsr_port_2,
            const std::string &mtr_port);

    virtual ~CtmMpt2();

public:
    bool readTrq(float *trq);

    void stopAll();
    
    void resetAll();

private:
    const uint8_t ID_ALL[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};

    const uint8_t ID_DISTAL[3] = {4, 5, 6};

    const uint8_t ID_MIDDLE[3] = {7, 8, 9};

    const uint8_t ID_PROXIMAL[3] = {1, 2, 3};

    const size_t N_ALL = 9;

    const size_t N_SEG = 3;

    const float PERMITTED_TRQ[9] = {42, 42, 42, 42, 42, 42, 3.0, 42, 42};

    const float INIT_TRQ[9] = {0.3635, 0.7803, 0.3806,
                                0.1746, 0.6714, 1.2503,
                                1.2046, 0.4787, 0.2623};

    const int32_t RESOLUTION[9] = {2400000, 2400000, 2400000,
                                    800000, 800000, 800000,
                                    800000, 800000, 800000};
};

#endif