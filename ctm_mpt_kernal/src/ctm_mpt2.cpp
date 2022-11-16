#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <algorithm>
#include "ctm_mpt2.h"
#include "utils.h"

CtmMpt2::CtmMpt2()
{
    return;
}

CtmMpt2::CtmMpt2(const std::string& port_name)
: CtmMpt::CtmMpt(port_name)
{
    return;
}

CtmMpt2::CtmMpt2(const std::string& snsr_port_1,
			   const std::string& snsr_port_2)
: CtmMpt::CtmMpt(snsr_port_1, snsr_port_2)
{
    return;
}

CtmMpt2::CtmMpt2(const std::string& snsr_port_1,
			   const std::string& snsr_port_2,
			   const std::string& mtr_port)
: CtmMpt::CtmMpt(snsr_port_1, snsr_port_2, mtr_port)
{
    return;
}

CtmMpt2::~CtmMpt2()
{
    return;
}

void CtmMpt2::print(const uint8_t id)
{
    int32_t pos = 0;
	int32_t vel_in_rpm = 0, vel_in_hz = 0;
	int32_t drv_temp = 0, mtr_temp = 0;
	int32_t inv_volt = 0, pwr_volt = 0;

    this->mtrGetPos(id, &pos);
    this->mtrGetVel(id, &vel_in_rpm, &vel_in_hz);
    this->mtrGetTemp(id, &drv_temp, &mtr_temp);
    this->mtrGetVolt(id, &inv_volt, &pwr_volt);

    ROS_INFO("\n -- id %d\
                \n  * pos  %6.1f (deg), or  %6d (step).\
                \n  * vel  %6d (rpm), or  %6d (Hz).\
                \n  * drv  %6.1f      , mtr  %5.1f (degs Celsius).\
                \n  * pwr  %6.1f      , inv  %5.1f (volt).\n",
                id,
                pos / this->RESOLUTION[id-1], pos,
                vel_in_rpm, vel_in_hz,
                drv_temp * 0.1, mtr_temp * 0.1,
                inv_volt * 0.1, pwr_volt * 0.1);

    return;
}

void CtmMpt2::print(void)
{
    int32_t pos = 0;
	int32_t vel_in_rpm = 0, vel_in_hz = 0;
	int32_t drv_temp = 0, mtr_temp = 0;
	int32_t inv_volt = 0, pwr_volt = 0;
    size_t i = 0;
    uint8_t id = 0;

    ROS_INFO("\n -- id  po(deg)  po(step)  ve(rpm)   ve(Hz)  tp(drv)  tp(mtr)  vt(inv)  vt(pwr)");
    while (i < this->N_ALL)
    {
        id = this->ID_ALL[i];
        this->mtrGetPos(id, &pos);
        this->mtrGetVel(id, &vel_in_rpm, &vel_in_hz);
        this->mtrGetTemp(id, &drv_temp, &mtr_temp);
        this->mtrGetVolt(id, &inv_volt, &pwr_volt);
        printf("  * %2d  %6.1f  %8d  %7d  %7d   %6.1f   %6.1f   %6.1f   %6.1f\n",
                id,
                pos / this->RESOLUTION[i], pos,
                vel_in_rpm, vel_in_hz,
                drv_temp * 0.1, mtr_temp * 0.1,
                inv_volt * 0.1, pwr_volt * 0.1);
        i++;
    }

    return;
}

bool CtmMpt2::read(float* trq)
{
    size_t k = 0;
    bool load = false;

    this->snsrRead(trq);

	while (k < this->N_ALL)
	{
		trq[k] -= this->TRQ_OFFSET[k];
        if (std::fabs(trq[k]) >= this->TRQ_PERMITTED[k])
        {
            load = true;
            ROS_ERROR("(id %d) Overloaded, suffering %.4f Nm.\n", k + 1, trq[k]);
        }
        k++;
	}

    return load;
}

void CtmMpt2::stop(void)
{
    this->mtrStop(this->ID_ALL, this->N_ALL);
    ROS_ERROR_STREAM("STOP.\n");

    return;
}

void CtmMpt2::init(void)
{
    this->mtrInit(this->ID_ALL, this->N_ALL);
    this->snsrInit();
    this->snsrGetCfg();
    this->snsrGetMat();
    ROS_INFO_STREAM("INITIALISE.\n");

    return;
}

void CtmMpt2::reset(void)
{
    this->mtrReset(this->ID_ALL, this->N_ALL);
    ROS_INFO_STREAM("RESET.\n");

    return;
}

void CtmMpt2::zero(void)
{
    this->mtrZero(this->ID_ALL, this->N_ALL);
    ROS_INFO_STREAM("ZERO.\n");

    return;
}

bool cmp(int16_t a, int16_t b)
{
    return a < b;
}

void CtmMpt2::move(float* dist)
{
    size_t* idcs = new size_t[this->N_ALL]();
    size_t idx = 0; // idx = idcs[i];
    size_t i = 0;
	bool all_at_pos = false;

    // Initialise with array indices.
    while (i < this->N_ALL)
    {
        idcs[i] = i;
        i++;
    }

    std::sort(idcs, idcs + this->N_ALL,
                [&](size_t j, size_t k)->bool{return dist[j] < dist[k];});

    // std::cout << "idcs: ";
    // for (i = 0; i < this->N_ALL; i++)
    // {
    //     std::cout << idcs[i] << "  ";
    // }
    // std::cout << std::endl << "id: ";
    // for (i = 0; i < this->N_ALL; i++)
    // {
    //     std::cout << int32_t(this->ID_ALL[idcs[i]]) << "  ";
    // }
    // std::cout << std::endl << "dist: ";
    // for (i = 0; i < this->N_ALL; i++)
    // {
    //     std::cout << dist[idcs[i]] << "  ";
    // }
    // std::cout << std::endl;

    i = 0;
    while (i < this->N_ALL)
    {
        idx = idcs[i++];
        this->mtrSetPosRel(this->ID_ALL[idx], dist[idx] * this->RESOLUTION[idx],
                            20000, 40000, 40000, "EXIT_DIRECTLY");
    }

	while (!all_at_pos)
	{
		all_at_pos = true;
        i = 0;
		while (i < this->N_ALL)
		{
			all_at_pos &= this->mtrAtPos_(this->ID_ALL[idcs[i++]]);
		}
	}

	// this->print();

    delete [] idcs;

    return;
}

CntlrPID::CntlrPID(float kp, float ki, float kd, float dt, size_t n)
{
    size_t i = 0;

    this->dt_ = dt;
    this->one_over_dt_ = 1 / dt;

    this->n_ = n;

    this->kp_ = new float[n]();
    this->ki_ = new float[n]();
    this->kd_ = new float[n]();

    this->err_sum_ = new float[n]();
    this->err_last_ = new float[n]();

    while (i < n)
    {
        this->kp_[i] = kp;
        this->ki_[i] = ki;
        this->kd_[i] = kd;
        i++;
    }

    return;
}

CntlrPID::~CntlrPID()
{
    delete [] this->kp_;
    delete [] this->ki_;
    delete [] this->kd_;
    
    delete [] this->err_sum_;
    delete [] this->err_last_;

    return;
}

void CntlrPID::pid(float* err, float* out)
{
    size_t i = 0;
    float derr = 0;

    while (i < this->n_)
    {
        if (this->is_first_run_)
        {
            derr = 0;
        }
        else
        {
            derr = (err[i] - this->err_last_[i]) * this->one_over_dt_;
        }

        this->err_sum_[i] += err[i] * this->dt_;
        this->err_last_[i] = err[i];
        out[i] = this->kp_[i] * err[i]
                    + this->ki_[i] * this->err_sum_[i]
                    + this->kd_[i] * derr;
        i++;
    }
    if (this->is_first_run_)
    {
        this->is_first_run_ = false;
    }

    return;
}
