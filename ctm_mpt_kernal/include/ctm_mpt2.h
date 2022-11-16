
#ifndef CTM_MPT2_H
#define CTM_MPT2_H

#include "ctm_mpt.h"

class CtmMpt2 : protected CtmMpt
{
public:
    CtmMpt2(); // For debugging.

    CtmMpt2(const std::string &port_name);

    CtmMpt2(const std::string &snsr_port_1,
            const std::string &snsr_port_2);

    CtmMpt2(const std::string &snsr_port_1,
            const std::string &snsr_port_2,
            const std::string &mtr_port);

    virtual ~CtmMpt2();

public:
    void print(const uint8_t id); // Single motor status.

    void print(void); // A table of all motors.

    bool read(float* trq); // Read torque values.

    void stop(void); // Stop motors for emergency.

    void init(void); // Initialise motors and sensors.

    void reset(void); // Return to zero positions.

    void zero(void); // Adjust zero.
    
    void move(float* dist); // Relative angular distances. Unit: degree.

private:
    const uint8_t ID_ALL[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};

    const uint8_t ID_DISTAL[3] = {4, 5, 6};

    const uint8_t ID_MIDDLE[3] = {7, 8, 9};

    const uint8_t ID_PROXIMAL[3] = {1, 2, 3};

    const size_t N_ALL = 9;

    const size_t N_SEG = 3;

    const float TRQ_PERMITTED[9] = {10, 10, 10, 10, 10, 10, 10, 10, 10};

    const float TRQ_OFFSET[9] = {-0.3145, -0.7898, -0.3423,
                                 -0.2071, -0.7468, -0.5819,
                                 -1.1219, -0.4638, -0.2561};

    // Step per rev / 360 degrees per rev,
    // e.g. 240,0000 / 360 = 20000 / 3,
    // where 240,0000 = 1,000 * 30 * 80.
    const float RESOLUTION[9] = {20000/3, 20000/3, 20000/3,
                                 20000/9, 20000/9, 20000/9,
                                 20000/9, 20000/9, 20000/9};
};

class CntlrPID
{
public:
    CntlrPID();

    CntlrPID(float kp, float ki, float kd, float dt, size_t n);

    ~CntlrPID();

    void pid(float* err, float* out);
    
private:
    float dt_;
    float one_over_dt_;

    size_t n_;

    float* kp_ = NULL;
    float* ki_ = NULL;
    float* kd_ = NULL;
    
    float* err_sum_ = NULL;
    float* err_last_ = NULL;

    bool is_first_run_ = true;
};

#endif