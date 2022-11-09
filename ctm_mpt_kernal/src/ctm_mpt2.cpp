#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include "ctm_mpt2.h"
#include "utils.h"

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

bool CtmMpt2::readTrq(float* trq)
{
    size_t k = 0;
    bool load = false;

    this->snsrRead(trq);

	for (k = 0; k < this->N_ALL; k++)
	{
		trq[k] -= this->INIT_TRQ[k];
        if (std::fabs(trq[k]) >= this->PERMITTED_TRQ[k])
        {
            load = true;
            ROS_ERROR("(id %d) Overloaded, suffering %.4f Nm.\n", k + 1, trq[k]);
        }
	}

    return load;
}

void CtmMpt2::stopAll()
{
    this->mtrStop(this->ID_ALL, this->N_ALL);
    ROS_ERROR_STREAM("STOP ALL.");

    return;
}

void CtmMpt2::resetAll()
{
    this->mtrReset(this->ID_ALL, this->N_ALL);
    ROS_ERROR_STREAM("RESET ALL.");

    return;
}