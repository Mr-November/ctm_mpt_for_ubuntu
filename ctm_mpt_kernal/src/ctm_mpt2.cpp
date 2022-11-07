#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include "ctm_mpt2.h"
#include "utils.h"

ctm_mpt::CtmMpt2::CtmMpt2(const std::string& port_name)
: ctm_mpt::CtmMpt::CtmMpt(port_name)
{
    return;
}

ctm_mpt::CtmMpt2::CtmMpt2(const std::string& snsr_port_1,
			   const std::string& snsr_port_2)
: ctm_mpt::CtmMpt::CtmMpt(snsr_port_1, snsr_port_2)
{
    return;
}

ctm_mpt::CtmMpt2::CtmMpt2(const std::string& snsr_port_1,
			   const std::string& snsr_port_2,
			   const std::string& mtr_port)
: ctm_mpt::CtmMpt::CtmMpt(snsr_port_1, snsr_port_2, mtr_port)
{
    return;
}

ctm_mpt::CtmMpt2::~CtmMpt2()
{
    return;
}

bool ctm_mpt::CtmMpt2::readTrq(float* trq)
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

void ctm_mpt::CtmMpt2::stopAll()
{
    this->mtrStop(this->ID_ALL, this->N_ALL);
    ROS_ERROR_STREAM("STOP ALL.");

    return;
}

void ctm_mpt::CtmMpt2::resetAll()
{
    this->mtrReset(this->ID_ALL, this->N_ALL);
    ROS_ERROR_STREAM("RESET ALL.");

    return;
}