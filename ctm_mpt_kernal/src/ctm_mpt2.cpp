#include "ctm_mpt2.h"
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>

Eigen::Matrix3f uphat(const Eigen::Vector3f V);
Eigen::Matrix4f uphat(const Eigen::Matrix<float, N_DIM, 1> V);
Eigen::Vector3f upvee(const Eigen::Matrix3f M);
Eigen::Matrix<float, N_DIM, 1> upvee(const Eigen::Matrix4f M);
Eigen::Matrix4f invt(const Eigen::Matrix4f T);
Eigen::Matrix4f q2T(const Eigen::Quaternionf quat, Eigen::Vector3f orig);
Eigen::Matrix<float, N_DIM, 2> jaco_c12(const float w1, const float w2, const float L);

// Continuum manipulator.
CtmMpt2::CtmMpt2(const std::string& snsr_port_1,
			   const std::string& snsr_port_2,
			   const std::string& mtr_port,
               ros::NodeHandle* nh)
: CtmMpt::CtmMpt(snsr_port_1, snsr_port_2, mtr_port)
{
    this->torque_publisher = nh->advertise<std_msgs::Float32MultiArray>("torque", 100);
    this->tf_subscriber = nh->subscribe(
        "/ctm_mpt/vrpn_client_node/RigidBodyTest/pose", 1,
        &CtmMpt2::mcsCallback, this);
    this->invT_zero = q2T(this->quat_zero, this->orig_zero);

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
                \n  * pos  %6.1f  (mm), or  %6d (step).\
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

    ROS_INFO("\n -- id   po(mm)  po(step)  ve(rpm)   ve(Hz)  tp(drv)  tp(mtr)  vt(inv)  vt(pwr)");
    while (i < N_MOTOR)
    {
        id = this->ID_MOTOR[i];
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

bool CtmMpt2::readTorque() // and publish!
{
    size_t k = 0;
    bool load = false;
    std_msgs::Float32MultiArray msg;
    std_msgs::MultiArrayDimension dim;
    dim.label = "tau";
    dim.size = N_MOTOR;
    dim.stride = N_MOTOR;
    msg.layout.dim.push_back(dim);
    msg.layout.data_offset = 0;
    msg.data = std::vector<float>(N_MOTOR, 0.0);

    this->snsrRead(this->torque);

	while (k < N_MOTOR)
	{
		this->torque[k] -= this->TORQUE_OFFSET[k];
        msg.data.at(k) = this->torque[k];
        if (std::fabs(this->torque[k]) >= this->TORQUE_PERMITTED[k])
        {
            load = true;
            ROS_ERROR("(id %d) Overloaded, suffering %.4f Nm.", k + 1, this->torque[k]);
        }
        k++;
	}

    this->torque_publisher.publish(msg);

    if (load)
    {
        this->stop();
        this->reset();
        ROS_ERROR("Overload protection completed. Be careful!");
    }

    return load;
}

void CtmMpt2::stop(void)
{
    this->mtrStop(this->ID_MOTOR, N_MOTOR);
    ROS_ERROR_STREAM("STOPPED.");

    return;
}

void CtmMpt2::init(void)
{
    this->mtrInit(this->ID_MOTOR, N_MOTOR);
    this->snsrInit();
    // this->snsrGetCfg();
    // this->snsrGetMat();
    ROS_INFO_STREAM("INITIALISED.");

    // Adjust sensors to zero.
    // size_t N0 = 10000;
    // for(size_t i = 0; i < N0; i++)
    // {
    //     this->readTorque();
    //     for(size_t j = 0; j < N_MOTOR; j++)
    //     {
    //         this->torque_offset[j] += this->torque[j] / N0;
    //     }
    // }
    // ROS_INFO("!");
    // for(size_t k = 0; k < N_MOTOR; k++)
    // {
    //     printf("%.6f, ", this->torque_offset[k]);
    // }

    return;
}

void CtmMpt2::reset(void)
{
    this->mtrReset(this->ID_MOTOR, N_MOTOR);
    ROS_INFO_STREAM("RESET.");

    return;
}

void CtmMpt2::zero(void)
{
    this->mtrZero(this->ID_MOTOR, N_MOTOR);
    ROS_INFO_STREAM("ZERO.");

    return;
}

void CtmMpt2::getCableLengthDiff(float *const diff,
                                    const Eigen::Matrix<float, N_DIM, 1> xi_diff)
{
    Eigen::Matrix<float, N_CABLE, 1> lengths_diff =
        this->U_DELTA.transpose() * this->C_DELTA * xi_diff;
    for (size_t i = 0; i < N_CABLE; i++)
    {
        diff[i] = lengths_diff(i, 0);
        // std::cout << diff[i] << ", ";
    }
    // std::cout << std::endl;

    return;
}

void CtmMpt2::move(const float *const dist)
{
    size_t* idcs = new size_t[N_MOTOR]();
    size_t idx = 0; // idx = idcs[i];
    size_t i = 0;
	bool all_at_pos = false;

    // Initialise with array indices.
    while (i < N_MOTOR)
    {
        idcs[i] = i;
        i++;
    }

    std::sort(idcs, idcs + N_MOTOR,
                [&](size_t j, size_t k)->bool{return dist[j] < dist[k];});

    // std::cout << "idcs: ";
    // for (i = 0; i < N_MOTOR; i++)
    // {
    //     std::cout << idcs[i] << "  ";
    // }
    // std::cout << std::endl << "id: ";
    // for (i = 0; i < N_MOTOR; i++)
    // {
    //     std::cout << int32_t(this->ID_MOTOR[idcs[i]]) << "  ";
    // }
    // std::cout << std::endl << "dist: ";
    // for (i = 0; i < N_MOTOR; i++)
    // {
    //     std::cout << dist[idcs[i]] << "  ";
    // }
    // std::cout << std::endl;

    i = 0;
    while (i < N_MOTOR)
    {
        idx = idcs[i++];
        this->mtrSetPosRel(this->ID_MOTOR[idx], dist[idx] * this->RESOLUTION[idx],
                            this->VEL[idx], this->ACC_START[idx], this->ACC_BRAKE[idx],
                            "EXIT_DIRECTLY");
    }

	while (!all_at_pos)
	{
        this->readTorque();

		all_at_pos = true;
        i = 0;
		while (i < N_MOTOR)
		{
			all_at_pos &= this->mtrAtPos_(this->ID_MOTOR[idcs[i++]]);
		}
	}

	// this->print();

    delete [] idcs;

    return;
}

void CtmMpt2::move(const uint8_t id, const float dist)
{
    this->mtrSetPosRel(id, dist * this->RESOLUTION[id-1],
                        this->VEL[id-1], this->ACC_START[id-1], this->ACC_BRAKE[id-1],
                        "EXIT_DIRECTLY");
    while (!this->mtrAtPos_(id))
	{
        this->readTorque();
	}

    return;
}

void CtmMpt2::relax(const float dist)
{
    size_t i = 0;
    bool all_at_pos = false;

    while (i < N_MOTOR)
    {
        this->mtrSetPosRel(this->ID_MOTOR[i], dist * this->RESOLUTION[i],
                            this->VEL[i], this->ACC_START[i], this->ACC_BRAKE[i],
                            "EXIT_DIRECTLY");
        i++;
    }
	while (!all_at_pos)
	{
        size_t j = 0;

        this->readTorque();

		all_at_pos = true;
		while (j < N_MOTOR)
		{
			all_at_pos &= this->mtrAtPos_(this->ID_MOTOR[j++]);
		}
	}

    return;
}

// Motion capture system callback.
Eigen::Matrix3f uphat(const Eigen::Vector3f V)
{
    return Eigen::Matrix3f {
        {  0.0, -V(2),  V(1)},
        { V(2),   0.0, -V(0)},
        {-V(1),  V(0),   0.0},
    };
}
Eigen::Matrix4f uphat(const Eigen::Matrix<float, 6, 1> V)
{
    return Eigen::Matrix4f {
        {  0.0, -V(2),  V(1), V(3)},
        { V(2),   0.0, -V(0), V(4)},
        {-V(1),  V(0),   0.0, V(5)},
        {  0.0,   0.0,   0.0,  0.0},
    };
}
Eigen::Vector3f upvee(const Eigen::Matrix3f M)
{
	return Eigen::Vector3f {
        -M(1,2), M(0,2), -M(0,1)
    };
}
Eigen::Matrix<float, 6, 1> upvee(const Eigen::Matrix4f M)
{
    return Eigen::Matrix<float, 6, 1> {
        -M(1,2), M(0,2), -M(0,1), M(0,3), M(1,3), M(2,3)
    };
}
Eigen::Matrix4f invt(const Eigen::Matrix4f T)
{
	Eigen::Matrix4f invT = Eigen::Matrix4f::Identity(4,4);
	invT.block(0,0,3,3) = T.block(0,0,3,3).transpose();
	invT.block(0,3,3,1) = -T.block(0,0,3,3).transpose()*T.block(0,3,3,1);
	return invT;
}
Eigen::Matrix4f q2T(const Eigen::Quaternionf quat, const Eigen::Vector3f orig)
{
	Eigen::Matrix4f T = Eigen::Matrix4f::Identity(4,4);
    T.block(0,0,3,3) = quat.toRotationMatrix();
    T.block(0,3,3,1) = orig;
	return T;
}

void CtmMpt2::mcsCallback(const geometry_msgs::PoseStamped& tfmsg)
{ 
    Eigen::Vector3f orig {
        tfmsg.pose.position.x,
		tfmsg.pose.position.y,
		tfmsg.pose.position.z,
    };
    Eigen::Quaternionf quat {
        tfmsg.pose.orientation.w,
        tfmsg.pose.orientation.x,
	    tfmsg.pose.orientation.y,
	    tfmsg.pose.orientation.z,
    };

    // Calculate the pose in real world.
    this->Tb = this->invT_zero * q2T(quat, orig);
	
    return;
}

// Calculate the Jacobian matrix.
Eigen::Matrix<float, 6, 2> jaco_c12(const float w1, const float w2, const float L)
{
	Eigen::Vector3f w {w1, w2, 0};
	Eigen::Matrix<float, 6, 3> Jc = Eigen::Matrix<float, 6, 3>::Zero();
	float n = w.norm();

	if (n < 1e-6f)
	{
		Jc <<  1.0f,  0.0f,  0.0f,
               0.0f,  1.0f,  0.0f,
               0.0f,  0.0f,  1.0f,
               0.0f,  0.5f,  0.0f,
              -0.5f,  0.0f,  0.0f,
               0.0f,  0.0f,  0.0f;
	}
	else
	{
		float n2 = n * n;
		float n3 = n * n * n;
		float M = (1 - std::cos(n)) / n2;
		float N = (n - std::sin(n)) / n3;
		float p1M = w1/n2 - w1*N - 2*w1*M/n2;
		float p2M = w2/n2 - w2*N - 2*w2*M/n2;
		float p1N = w1*M/n2 - 3*w1*N/n2;
		float p2N = w2*M/n2 - 3*w2*N/n2;
        Eigen::Matrix3f pwJleftwv;

		pwJleftwv <<           p1M*w2,          p2M*w2 + M,    0,
                          -p1M*w1 - M,             -p2M*w1,    0,
                     -p1N*n2 - 2*N*w1,    -p2N*n2 - 2*N*w2,    0;
		Jc.block(0,0,3,3) = Eigen::Matrix3f::Identity(3,3) - M * uphat(w) + N * (uphat(w) * uphat(w));
		Jc.block(3,0,3,3) = uphat(w).exp().transpose() * L * pwJleftwv;
	}

	return Jc.block(0,0,6,2);
}

Eigen::Matrix<float, 6, 6> CtmMpt2::jacobian3cc(const float L1, const float L2, const float L3)
{
	Eigen::Matrix<float, 6, 1> xi_3 {this->xi(4),this->xi(5), 0, 0, 0, L3};
	Eigen::Matrix<float, 6, 1> xi_2 {this->xi(2),this->xi(3), 0, 0, 0, L2};

	Eigen::Matrix<float, 6, 2> J3 = jaco_c12(xi_3(0), xi_3(1), L3);
	Eigen::Matrix<float, 6, 2> J2 = jaco_c12(xi_2(0), xi_2(1), L2);
	Eigen::Matrix<float, 6, 2> J1 = jaco_c12(this->xi(0),this->xi(1), L1);

	Eigen::Matrix<float, 6, 1> J2c1 = J2.block(0,0,6,1);
	Eigen::Matrix<float, 6, 1> J2c2 = J2.block(0,1,6,1);
	Eigen::Matrix<float, 6, 1> J1c1 = J1.block(0,0,6,1);
	Eigen::Matrix<float, 6, 1> J1c2 = J1.block(0,1,6,1);

	Eigen::Matrix4f T3 = uphat(xi_3).exp();
	Eigen::Matrix4f T2 = uphat(xi_2).exp();
	Eigen::Matrix4f invT3 = invt(T3);
	Eigen::Matrix4f invT2 = invt(T2);

    Eigen::Matrix4f J2c1m = invT3 * uphat(J2c1) * T3;
	Eigen::Matrix4f J2c2m = invT3 * uphat(J2c2) * T3;
	Eigen::Matrix4f J1c1m = invT3 * invT2 * uphat(J1c1) * T2 * T3;
	Eigen::Matrix4f J1c2m = invT3 * invT2 * uphat(J1c2) * T2 * T3;

    Eigen::Matrix<float, 6, 6> J;
	
	J.block(0,0,6,1) = upvee(J1c1m);
	J.block(0,1,6,1) = upvee(J1c2m);

	J.block(0,2,6,1) = upvee(J2c1m);
	J.block(0,3,6,1) = upvee(J2c2m);
	
	J.block(0,4,6,2) = J3;

	return J;
}

void CtmMpt2::setTargetXi(const Eigen::Matrix<float, N_DIM, 1> Xi)
{
    this->xid = Xi;

    return;
}

void CtmMpt2::setTargetPose(const Eigen::Matrix4f P)
{
    this->Td = P;

    return;
}

void CtmMpt2::trackXi(void)
{
    float diff[N_CABLE] = {0.0};
    Eigen::Matrix<float, N_DIM, 1> xi_diff = this->xid - this->xi;

    // Get cable length difference.
    this->getCableLengthDiff(diff, xi_diff);
    std::cout << "diff = "
                << (Eigen::Map<Eigen::Matrix<float, N_CABLE, 1>> (diff)).transpose()
                << std::endl;

    // Move the motors!
    // this->move(diff);

    // Update xi.
    this->xi += xi_diff;

    return;
}

void CtmMpt2::trackPose(void)
{
    // Calculate the error.
    Eigen::Matrix4f V_uphat = (invt(this->Tb) * this->Td).log();
    Eigen::Matrix<float, 6, 1> V = upvee(V_uphat);
    std::cout << "V = " << V.transpose() << std::endl;

    // Calculate the Jacobian matrix, using this->xi.
    Eigen::Matrix<float, 6, 6> J = this->jacobian3cc();
    std::cout << "xi = " << this->xi.transpose() << std::endl;
    std::cout << "J = " << J << std::endl;

    // Calculate the xi difference.
    Eigen::Matrix<float, N_DIM, 1> xi_diff = J.transpose() * V;
    std::cout << "xi_diff = " << xi_diff.transpose() << std::endl;

    // Calculate the controller output.
    Eigen::Matrix<float, N_DIM, 1> xi_diff_out = this->position_controller.pid(xi_diff);
    std::cout << "xi_diff_out = " << xi_diff_out.transpose() << std::endl;

    // Get cable length difference.
    float diff[N_CABLE] = {0.0};
    this->getCableLengthDiff(diff, xi_diff_out);
    std::cout << "diff = "
                << (Eigen::Map<Eigen::Matrix<float, N_CABLE, 1>> (diff)).transpose()
                << std::endl;

    // Move the motors!
    // this->move(diff);

    // Update xi.
    this->xi += xi_diff_out;

    return;
}



CtrlPID::CtrlPID(const float kp, const float ki, const float kd,
                    const float dt)
{
    this->dt_ = dt;
    this->one_over_dt_ = 1.0 / dt;

    this->kp_ = kp;
    this->ki_ = ki;
    this->kd_ = kd;

    this->err_sum_ = Eigen::Matrix<float, N_DIM, 1>::Zero();
    this->err_last_ = Eigen::Matrix<float, N_DIM, 1>::Zero();

    return;
}

CtrlPID::~CtrlPID()
{
    return;
}

Eigen::Matrix<float, N_DIM, 1> CtrlPID::pid(const Eigen::Matrix<float, N_DIM, 1> err)
{
    Eigen::Matrix<float, N_DIM, 1> derr = Eigen::Matrix<float, N_DIM, 1>::Zero();
    Eigen::Matrix<float, N_DIM, 1> out = Eigen::Matrix<float, N_DIM, 1>::Zero();

        if (this->is_first_run_)
        {
            this->is_first_run_ = false;
        }
        else
        {
            derr = (err - this->err_last_) * this->one_over_dt_;
        }

        this->err_sum_ += err * this->dt_;
        this->err_last_ = err;
        out = this->kp_ * err + this->ki_ * this->err_sum_ + this->kd_ * derr;

    return out;
}
