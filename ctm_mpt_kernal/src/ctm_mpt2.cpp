#include "ctm_mpt2.h"
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
#include <unsupported/Eigen/Splines>
#include <chrono>
#include <ctime>

const Eigen::Matrix3f uphat(const Eigen::Vector3f& V);
const Eigen::Matrix4f uphat(const Eigen::Matrix<float, N_DIM, 1>& V);
const Eigen::Vector3f upvee(const Eigen::Matrix3f& M);
const Eigen::Matrix<float, N_DIM, 1> upvee(const Eigen::Matrix4f& M);
const Eigen::Matrix4f invt(const Eigen::Matrix4f& T);
const Eigen::Matrix4f q2T(const Eigen::Quaterniond& quat, const Eigen::Vector3d& orig);
const Eigen::Matrix<float, N_DIM, 2> jaco_c12(const float w1, const float w2, const float L);

// Continuum manipulator.
CtmMpt2::CtmMpt2()
{
    return;
}

CtmMpt2::CtmMpt2(const std::string& mtr_port,
               ros::NodeHandle* nh)
: CtmMpt::CtmMpt(mtr_port)
{
    // Define the time format.
    static const Eigen::IOFormat fmt(
        Eigen::FullPrecision,
        Eigen::DontAlignCols,
        ",", // coeffSeparator
        ",", // rowSeparator
        "", // rowPrefix
        "", // rowSuffix
        "", // matPrefix
        "," // matSuffix
    );

    // Initialise the publisher.
    this->torque_publisher = nh->advertise<std_msgs::Float32MultiArray>("torque", 1);
    std_msgs::MultiArrayDimension dim;
    dim.label = "tau";
    dim.size = N_MOTOR;
    dim.stride = N_MOTOR;
    this->torque_message.layout.dim.push_back(dim);
    this->torque_message.layout.data_offset = 0;
    this->torque_message.data = std::vector<float>(N_MOTOR, 0.0);

    // Initialise the subscribers.
    this->tf_subscriber = nh->subscribe(
        "/ctm_mpt/vrpn_client_node/RobotTip/pose", 1, &CtmMpt2::tfCallback, this
    );
    this->xi_subscriber = nh->subscribe("/ctm_mpt/xid", 1, &CtmMpt2::xiCallback, this);
    
    // Initialise the maximum cable velocity.
    this->L_DOT_MAX = ((Eigen::Map<const Eigen::Matrix<float, N_MOTOR, 1>> (this->VEL)).array() / (Eigen::Map<const Eigen::Matrix<float, N_MOTOR, 1>> (this->RESOLUTION)).array()).abs();

    // Initialise the logger.
    this->file_log.open(
        "/home/ellipse/Desktop/catkin_ws/src/ctm_mpt/ctm_mpt_kernal/log/log0001.csv",
        std::ios_base::app);
    if (!this->file_log.is_open())
    {
        ROS_ERROR_STREAM("Unable to open the log file.");
    }
    auto now = std::chrono::system_clock::now();
    std::time_t tt = std::chrono::system_clock::to_time_t(now);
    std::string tstr = ctime(&tt);
    this->file_log << "[ "
                   << Eigen::Matrix<double, 1, 1>(ros::Time::now().toSec()).format(fmt)
                   << " "
                   << tstr.substr(0, tstr.length()-1)
                   << " ]" << std::endl;

    return;
}

CtmMpt2::CtmMpt2(const std::string& snsr_port_1,
			   const std::string& snsr_port_2,
			   const std::string& mtr_port,
               ros::NodeHandle* nh)
: CtmMpt::CtmMpt(snsr_port_1, snsr_port_2, mtr_port)
{
    // Define the time format.
    static const Eigen::IOFormat fmt(
        Eigen::FullPrecision,
        Eigen::DontAlignCols,
        ",", // coeffSeparator
        ",", // rowSeparator
        "", // rowPrefix
        "", // rowSuffix
        "", // matPrefix
        "," // matSuffix
    );

    // Initialise the publisher.
    this->torque_publisher = nh->advertise<std_msgs::Float32MultiArray>("torque", 1);
    std_msgs::MultiArrayDimension dim;
    dim.label = "tau";
    dim.size = N_MOTOR;
    dim.stride = N_MOTOR;
    this->torque_message.layout.dim.push_back(dim);
    this->torque_message.layout.data_offset = 0;
    this->torque_message.data = std::vector<float>(N_MOTOR, 0.0);

    // Initialise the subscribers.
    this->tf_subscriber = nh->subscribe(
        "/ctm_mpt/vrpn_client_node/RobotTip/pose", 1, &CtmMpt2::tfCallback, this
    );
    this->xi_subscriber = nh->subscribe("/ctm_mpt/xid", 1, &CtmMpt2::xiCallback, this);
    
    // Initialise the maximum cable velocity.
    this->L_DOT_MAX = ((Eigen::Map<const Eigen::Matrix<float, N_MOTOR, 1>> (this->VEL)).array() / (Eigen::Map<const Eigen::Matrix<float, N_MOTOR, 1>> (this->RESOLUTION)).array()).abs();

    // Initialise the logger.
    this->file_log.open(
        "/home/ellipse/Desktop/catkin_ws/src/ctm_mpt/ctm_mpt_kernal/log/log0001.csv",
        std::ios_base::app);
    if (!this->file_log.is_open())
    {
        ROS_ERROR_STREAM("Unable to open the log file.");
    }
    auto now = std::chrono::system_clock::now();
    std::time_t tt = std::chrono::system_clock::to_time_t(now);
    std::string tstr = ctime(&tt);
    this->file_log << "[ "
                   << Eigen::Matrix<double, 1, 1>(ros::Time::now().toSec()).format(fmt)
                   << " "
                   << tstr.substr(0, tstr.length()-1)
                   << " ]" << std::endl;

    return;
}

CtmMpt2::~CtmMpt2()
{
    // Stop all motors when exit.
    this->stop();
    this->reset();
    // this->relax();
    this->file_log.close();

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
                \n  * pos  %6.1f mm,\
                \n         %6d step.\
                \n  * vel  %6d rpm,\
                \n         %6d Hz.\
                \n  * drv  %6.1f Deg Celsius.\
                \n  * mtr  %6.1f Deg Celsius.\
                \n  * pwr  %6.1f V,\
                \n  * inv  %6.1f V.\n",
                (int)id,
                pos / this->RESOLUTION[id-1], pos,
                vel_in_rpm, vel_in_hz,
                drv_temp * 0.1, mtr_temp * 0.1,
                inv_volt * 0.1, pwr_volt * 0.1);

    return;
}

void CtmMpt2::print(void)
{
    int32_t pos[N_MOTOR] = {0};
	int32_t vel_in_rpm[N_MOTOR] = {0};
    int32_t vel_in_hz[N_MOTOR] = {0};
	int32_t drv_temp[N_MOTOR] = {0};
    int32_t mtr_temp[N_MOTOR] = {0};
	int32_t inv_volt[N_MOTOR] = {0};
    int32_t pwr_volt[N_MOTOR] = {0};

    this->mtrGetPos(this->ID_MOTOR, N_MOTOR, pos);
    this->mtrGetVel(this->ID_MOTOR, N_MOTOR, vel_in_rpm, vel_in_hz);
    this->mtrGetTemp(this->ID_MOTOR, N_MOTOR, drv_temp, mtr_temp);
    this->mtrGetVolt(this->ID_MOTOR, N_MOTOR, inv_volt, pwr_volt);

    ROS_INFO("\n -- id   po(mm)  po(step)  ve(rpm)   ve(Hz)  tp(drv)  tp(mtr)  vt(inv)  vt(pwr)");
    for (size_t i = 0; i < N_MOTOR; i++)
    {
        printf("  * %2d  %6.1f  %8d  %7d  %7d   %6.1f   %6.1f   %6.1f   %6.1f\n",
                this->ID_MOTOR[i],
                pos[i] / this->RESOLUTION[i], pos[i],
                vel_in_rpm[i], vel_in_hz[i],
                drv_temp[i] * 0.1, mtr_temp[i] * 0.1,
                inv_volt[i] * 0.1, pwr_volt[i] * 0.1);
    }

    return;
}

bool CtmMpt2::readTorque() // and publish!
{
    bool load = false;

    this->snsrRead(this->tau);

	for (size_t k = 0; k < N_MOTOR; k++)
	{
		this->tau[k] -= this->TORQUE_OFFSET[k];
        this->torque_message.data.at(k) = this->tau[k];
        if (std::fabs(this->tau[k]) >= this->TORQUE_PERMITTED[k])
        {
            load = true;
            ROS_ERROR("(id %d) Overloaded, suffering %.4f Nm.", (int)k + 1, this->tau[k]);
        }
	}

    this->torque_publisher.publish(this->torque_message);

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
    ROS_WARN_STREAM("STOPPED.");

    return;
}

void CtmMpt2::init(void)
{
    this->mtrStop(this->ID_MOTOR, N_MOTOR);
    this->mtrInit(this->ID_MOTOR, N_MOTOR);
    this->snsrInit();
    // this->snsrGetCfg();
    // this->snsrGetMat();
    ROS_INFO_STREAM("INITIALISED.");
    this->printPose();

    // Read torque.
    this->readTorque();
    
    // Save.
    this->saveData(ros::Time::now().toSec());

    // Adjust sensors to zero.
    // float torque_offset[N_MOTOR] = {0.0};
    // size_t N0 = 10000;
    // for(size_t i = 0; i < N0; i++)
    // {
    //     this->readTorque();
    //     for(size_t j = 0; j < N_MOTOR; j++)
    //     {
    //         torque_offset[j] += this->tau[j] / N0;
    //     }
    // }
    // ROS_INFO("----------------------------");
    // for(size_t k = 0; k < N_MOTOR; k++)
    // {
    //     printf("%.6f, ", torque_offset[k]);
    // }
    // std::cout << std::endl;
    // ROS_INFO("----------------------------");

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

void CtmMpt2::move(const uint8_t id, const float dist, bool wait)
{
    this->mtrSetPosRel(id, dist * this->RESOLUTION[id-1],
                        this->VEL[id-1], this->ACC_START[id-1], this->ACC_BRAKE[id-1]);
    if (wait)
    {
        while (ros::ok() && !this->mtrAtPos_(id))
        {
            this->readTorque();
        }
    }

    return;
}

void CtmMpt2::move(const float *const dist, bool wait)
{
    size_t idcs[N_MOTOR] = {0};
    size_t idx = 0; // idx = idcs[i];
    size_t i = 0;

    // Initialise with array indices.
    while (i < N_MOTOR)
    {
        idcs[i] = i;
        i++;
    }

    // Largest first.
    std::sort(idcs, idcs + N_MOTOR,
                [&](size_t j, size_t k)->bool{return dist[j] > dist[k];});

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
                            this->VEL[idx], this->ACC_START[idx], this->ACC_BRAKE[idx]);
    }

    if (wait)
    {
        while (ros::ok() && !this->mtrAtPos_(this->ID_MOTOR, N_MOTOR))
        {
            this->readTorque();
        }
    }

    return;
}

void CtmMpt2::move(const float dist, bool wait)
{
    float ds[N_MOTOR] = {0.0};
    for (size_t i = 0; i < N_MOTOR; i++)
    {
        ds[i] = dist;
    }
    move(ds, wait);

    return;
}

void CtmMpt2::run(const uint8_t id, const float dist_dot)
{
    this->mtrSetVel(id, dist_dot * this->RESOLUTION[id-1],
                    this->ACC_START[id-1], this->ACC_BRAKE[id-1]);

    return;
}

void CtmMpt2::run(const float *const dist_dot)
{
    size_t idcs[N_MOTOR] = {0};
    size_t idx = 0; // idx = idcs[i];
    size_t i = 0;
	bool all_at_pos = false;

    // Initialise with array indices.
    while (i < N_MOTOR)
    {
        idcs[i] = i;
        i++;
    }

    // Largest first.
    std::sort(idcs, idcs + N_MOTOR,
                [&](size_t j, size_t k)->bool{return dist_dot[j] > dist_dot[k];});

    i = 0;
    while (i < N_MOTOR)
    {
        idx = idcs[i++];
        this->mtrSetVel(this->ID_MOTOR[idx], dist_dot[idx] * this->RESOLUTION[idx],
                        this->ACC_START[idx], this->ACC_BRAKE[idx]);
    }

    return;
}

void CtmMpt2::run(const float dist_dot)
{
    float ds[N_MOTOR] = {0.0};
    for (size_t i = 0; i < N_MOTOR; i++)
    {
        ds[i] = dist_dot;
    }
    run(ds);

    return;
}

// Motion capture system callback.
const Eigen::Matrix3f uphat(const Eigen::Vector3f& V)
{
    return Eigen::Matrix3f {
        {  0.0, -V(2),  V(1)},
        { V(2),   0.0, -V(0)},
        {-V(1),  V(0),   0.0},
    };
}
const Eigen::Matrix4f uphat(const Eigen::Matrix<float, 6, 1>& V)
{
    return Eigen::Matrix4f {
        {  0.0, -V(2),  V(1), V(3)},
        { V(2),   0.0, -V(0), V(4)},
        {-V(1),  V(0),   0.0, V(5)},
        {  0.0,   0.0,   0.0,  0.0},
    };
}
const Eigen::Vector3f upvee(const Eigen::Matrix3f& M)
{
	return Eigen::Vector3f {
        -M(1,2), M(0,2), -M(0,1)
    };
}
const Eigen::Matrix<float, 6, 1> upvee(const Eigen::Matrix4f& M)
{
    return Eigen::Matrix<float, 6, 1> {
        -M(1,2), M(0,2), -M(0,1), M(0,3), M(1,3), M(2,3)
    };
}
const Eigen::Matrix4f invt(const Eigen::Matrix4f& T)
{
	Eigen::Matrix4f invT = Eigen::Matrix4f::Identity(4,4);
	invT.block(0,0,3,3) = T.block(0,0,3,3).transpose();
	invT.block(0,3,3,1) = -T.block(0,0,3,3).transpose()*T.block(0,3,3,1);
	return invT;
}
const Eigen::Matrix4f q2T(const Eigen::Quaterniond& quat, const Eigen::Vector3d& orig)
{
	Eigen::Matrix4d T = Eigen::Matrix4d::Zero();
    T.block(0,0,3,3) = quat.toRotationMatrix();
    T.block(0,3,3,1) = orig;
    T(3,3) = 1.0;
	return T.cast<const float>();
}

void CtmMpt2::tfCallback(const geometry_msgs::PoseStamped& tfmsg)
{
    // Origin x, y, z.
    Eigen::Vector3d orig {
        tfmsg.pose.position.x,
		tfmsg.pose.position.y,
		tfmsg.pose.position.z,
    };
    // Quaternion w, x, y, z.
    Eigen::Quaterniond quat {
        tfmsg.pose.orientation.w,
        tfmsg.pose.orientation.x,
	    tfmsg.pose.orientation.y,
	    tfmsg.pose.orientation.z,
    };

    // Calculate the pose in real world.
    // Unit of translation: mm
    this->Tb = this->invT_zero * q2T(quat, 1000*orig);

    // // When setting the initial pose, uncomment this block.
    // this->saveData(ros::Time::now().toSec());
    // std::cout << "Tb = " << this->Tb << std::endl;
	
    return;
}

void CtmMpt2::xiCallback(const std_msgs::Float32MultiArray& xi_msg_array)
{
    const Eigen::Matrix<float, N_DIM*3+1, 1> xi_msg(xi_msg_array.data.data());
    this->xid = xi_msg.block(0,0,N_DIM,1);
    this->xi_dot_fw = xi_msg.block(N_DIM,0,N_DIM,1);
    this->xi_diff_fw = xi_msg.block(N_DIM*2,0,N_DIM,1);
    this->Td = this->fk3cc(this->L1, this->L2, this->L3, xid);

    // float vel_mode = xi_msg(N_DIM*3);
    // if (vel_mode < 1.2)
    // {
    //     this->VEL = this->VEL1;
    // }
    // else if (vel_mode < 2.2)
    // {
    //     this->VEL = this->VEL2;
    // }
    // else if (vel_mode < 3.2)
    // {
    //     this->VEL = this->VEL3;
    // }
    // else
    // {
    //     this->VEL = this->VEL4;
    // }
    // this->L_DOT_MAX = ((Eigen::Map<const Eigen::Matrix<float, N_MOTOR, 1>> (this->VEL)).array() / (Eigen::Map<const Eigen::Matrix<float, N_MOTOR, 1>> (this->RESOLUTION)).array()).abs();

    std::cout << "Callback ----------------------" << std::endl
        << "xid = " << this->xid.transpose() << std::endl
        << "xi_dot_fw = " << this->xi_dot_fw.transpose() << std::endl
        << "xi_diff_fw = " << this->xi_diff_fw.transpose() << std::endl
        << "Td = " << std::endl << this->Td << std::endl
        << "-------------------------------" << std::endl;

    return;
}

void CtmMpt2::printPose(void)
{
    ros::spinOnce();
    std::cout << "Print pose --------------------" << std::endl
        << "Tb = " << std::endl << this->Tb << std::endl
        << "-------------------------------" << std::endl;

    return;
}

// Forward kinematics.
const Eigen::Matrix4f CtmMpt2::fk3cc(
    const float L1, const float L2, const float L3,
    const Eigen::Matrix<float, N_DIM, 1>& Xi)
{
    Eigen::Matrix<float, 6, 1> x_3{Xi(4,0), Xi(5,0), 0, 0, 0, L3};
    Eigen::Matrix<float, 6, 1> x_2{Xi(2,0), Xi(3,0), 0, 0, 0, L2};
    Eigen::Matrix<float, 6, 1> x_1{Xi(0,0), Xi(1,0), 0, 0, 0, L1};

    Eigen::Matrix4f T3 = uphat(x_3).exp();
	Eigen::Matrix4f T2 = uphat(x_2).exp();
	Eigen::Matrix4f T1 = uphat(x_1).exp();

    return T1 * T2 * T3;
}

// Calculate the Jacobian matrix.
const Eigen::Matrix<float, 6, 2> jaco_c12(const float w1, const float w2, const float L)
{
	Eigen::Vector3f w {w1, w2, 0};
	Eigen::Matrix<float, 6, 3> Jc = Eigen::Matrix<float, 6, 3>::Zero();
	float n = w.norm();

	if (n < 1e-6f)
	{
		Jc <<  1.0f,  0.0f,  0.0f,
               0.0f,  1.0f,  0.0f,
               0.0f,  0.0f,  1.0f,
               0.0f, 0.5*L,  0.0f,
             -0.5*L,  0.0f,  0.0f,
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

const Eigen::Matrix<float, 6, 6> CtmMpt2::jacobian3cc(
    const float L1, const float L2, const float L3,
    const Eigen::Matrix<float, N_DIM, 1>& Xi)
{
	Eigen::Matrix<float, 6, 2> J3 = jaco_c12(Xi(4, 0), Xi(5, 0), L3);
	Eigen::Matrix<float, 6, 2> J2 = jaco_c12(Xi(2, 0), Xi(3, 0), L2);
	Eigen::Matrix<float, 6, 2> J1 = jaco_c12(Xi(0, 0), Xi(1, 0), L1); 

	Eigen::Matrix<float, 6, 1> J2c1 = J2.block(0,0,6,1);
	Eigen::Matrix<float, 6, 1> J2c2 = J2.block(0,1,6,1);
	Eigen::Matrix<float, 6, 1> J1c1 = J1.block(0,0,6,1);
	Eigen::Matrix<float, 6, 1> J1c2 = J1.block(0,1,6,1);

	Eigen::Matrix4f T3 = uphat(Eigen::Matrix<float, 6, 1> {Xi(4,0), Xi(5,0), 0, 0, 0, L3}).exp();
	Eigen::Matrix4f T2 = uphat(Eigen::Matrix<float, 6, 1> {Xi(2,0), Xi(3,0), 0, 0, 0, L2}).exp();
	Eigen::Matrix4f invT3 = invt(T3);
	Eigen::Matrix4f invT2 = invt(T2);

    Eigen::Matrix<float, 6, 6> J;
	J.block(0,0,6,1) = upvee((const Eigen::Matrix4f)(invT3 * invT2 * uphat(J1c1) * T2 * T3));
	J.block(0,1,6,1) = upvee((const Eigen::Matrix4f)(invT3 * invT2 * uphat(J1c2) * T2 * T3));
    J.block(0,2,6,1) = upvee((const Eigen::Matrix4f)(invT3 * uphat(J2c1) * T3));
	J.block(0,3,6,1) = upvee((const Eigen::Matrix4f)(invT3 * uphat(J2c2) * T3));
	J.block(0,4,6,2) = J3;

	return J;
}

const Eigen::Matrix<float, N_DIM, 1> CtmMpt2::getXi2(void)
{
    Eigen::Matrix<float, N_MOTOR, 1> pos_mm;
    int32_t pos[N_MOTOR] = {0};

    this->mtrGetPos(this->ID_MOTOR, N_MOTOR, pos);
    for (size_t i = 0; i < N_MOTOR; i++)
    {
        pos_mm(i,0) = pos[i] / this->RESOLUTION[i];
    }
    
    return (this->PUTC.transpose() * this->PUTC).inverse() * this->PUTC.transpose() * pos_mm;
}

const Eigen::Matrix<float, N_CABLE, 1> CtmMpt2::getCableLengthDiff(
    const Eigen::Matrix<float, N_DIM, 1>& xi_diff)
{
    return this->PUTC * xi_diff;
}

const Eigen::Matrix<float, N_CABLE, 1> CtmMpt2::getCableLengthDot(
    const Eigen::Matrix<float, N_DIM, 1>& xi_dot)
{
    return this->PUTC * xi_dot;
}

const Eigen::Matrix<float, 6, 1> CtmMpt2::getError(void)
{
    // std::cout << "Get error ---------------------" << std::endl
    //     << "Tb = " << std::endl << this->Tb << std::endl
    //     << "Td = " << std::endl << this->Td << std::endl
    //     << "-------------------------------" << std::endl;

    return upvee((const Eigen::Matrix4f)(invt(this->Tb) * this->Td).log());
}

void CtmMpt2::saveData(
    const double t,
    const Eigen::Matrix<float, N_DIM, 1>& mat_n_dim_by_1,
    const Eigen::Matrix<float, N_CABLE, 1>& mat_n_cable_by_1)
{
    static const Eigen::IOFormat fmt(
        Eigen::FullPrecision,
        Eigen::DontAlignCols,
        ",", // coeffSeparator
        ",", // rowSeparator
        "", // rowPrefix
        "", // rowSuffix
        "", // matPrefix
        "," // matSuffix
    );

    this->file_log << Eigen::Matrix<double, 1, 1>(t).format(fmt)
        << upvee((const Eigen::Matrix4f)this->Tb.log()).transpose().format(fmt)
        << upvee((const Eigen::Matrix4f)this->Td.log()).transpose().format(fmt)
        << mat_n_dim_by_1.transpose().format(fmt)
        << mat_n_cable_by_1.transpose().format(fmt)
        << (Eigen::Map<Eigen::Matrix<float, N_CABLE, 1>> (this->tau)).transpose().format(fmt)
        << std::endl;

    return;
}
/*
void CtmMpt2::setTargetTorque(const Eigen::Matrix<float, N_CABLE, 1>& Tau)
{
    this->taud = Tau;

    return;
}

void CtmMpt2::setTargetTorque(const float Tau)
{
    for (size_t i = 0; i < N_CABLE; i++)
    {
        this->taud(i, 0) = Tau;
    }

    return;
}

void CtmMpt2::setTargetPose(const Eigen::Matrix4f& P)
{
    this->Td = P;
    std::cout << "Set target pose ---------------" << std::endl;
    std::cout << "Td = " << std::endl << this->Td << std::endl;
    std::cout << "-------------------------------" << std::endl;

    return;
}

void CtmMpt2::setTargetPath(const Eigen::Matrix<float, N_DIM, Eigen::Dynamic>& Xis)
{
    this->xis = Xis;
    this->xis_cols = Xis.cols();
    this->xis_index = -1;
    this->Tds = Eigen::Matrix<float, 4, Eigen::Dynamic>::Zero(4, 4*this->xis_cols);
    for (size_t i = 0; i < this->xis_cols; i++)
    {
        this->Tds.block(0,4*i,4,4) = this->fk3cc(this->L1, this->L2, this->L3, this->xis.col(i));
    }
    std::cout << "Set target path ---------------" << std::endl;
    std::cout << "xis_cols = " << this->xis_cols << std::endl;
    std::cout << "-------------------------------" << std::endl;

    return;
}

void CtmMpt2::allocateTime(void)
{
    this->time_interval = Eigen::Matrix<float, 1, Eigen::Dynamic>::Zero(1, this->xis_cols);
    size_t i = 0;

    // For the last point, set t = 0.
    for (i = 0; i < this->xis_cols-1; i++)
    {
        Eigen::Matrix<float, N_DIM, 1> xi_diff = this->xis.col(i+1) - this->xis.col(i);
        Eigen::Matrix<float, N_CABLE, 1> length_diff = getCableLengthDiff(xi_diff);
        this->time_interval(0, i) = ((length_diff).array().abs() / this->L_DOT_MAX).maxCoeff();
    }

    std::cout << this->time_interval << std::endl;

    return;
}

void CtmMpt2::trackTorque(void)
{
    this->readTorque();

    // Calculate the torque difference.
    Eigen::Matrix<float, N_CABLE, 1> tau_diff;
    for (size_t i = 0; i < N_CABLE; i++)
    {
        tau_diff(i, 0) = this->tau[i] - this->taud[i];
    }
    std::cout << "tau_diff = " << std::endl << tau_diff.transpose() << std::endl;

    // Calculate the controller output.
    Eigen::Matrix<float, N_CABLE, 1> diff = this->ffbc.pid(tau_diff);
    std::cout << "diff = " << std::endl << diff.transpose() << std::endl;
    std::cout << "-------------------------------" << std::endl;

    // Move the motors!
    std::getchar();
    this->move(diff.data());

    return;
}

void CtmMpt2::trackPath(void)
{
    if (this->xis_index == this->xis_cols-1)
    {
        ROS_INFO("Path completed.");
        std::getchar();

        return;
    }
    else if (this->xis_index == -1)
    {
        this->setTargetXi(this->xis.col(0));
        this->trackXi();
        this->setTargetPose((const Eigen::Matrix4f)this->Tds.block(0,0,4,4));
        this->trackPose();

        this->xis_index += 1;

        return;
    }
    else
    {
        printf("xis_index = %d\n", (int)this->xis_index);

        // Calculate the error.
        Eigen::Matrix<float, 6, 1> V = getError();
        std::cout << "V = " << V.transpose() << std::endl;

        // Currently at the i-th target pose.
        // Want to track the (i+1)-th target pose.
        this->setTargetPose((const Eigen::Matrix4f)this->Tds.block(0,4*(this->xis_index+1),4,4));

        // Get the feedforward xi_diff_fw.
        Eigen::Matrix<float, N_DIM, 1> xi_diff_fw = this->xis.col(this->xis_index+1) - this->xis.col(this->xis_index);
        std::cout << "xi_diff_fw = " << xi_diff_fw.transpose() << std::endl;

        // Get cable length difference.
        Eigen::Matrix<float, N_CABLE, 1> diff = this->getCableLengthDiff(xi_diff_fw);
        std::cout << "diff = " << diff.transpose() << std::endl;
        std::cout << "-------------------------------" << std::endl;

        // Move the motors!
        std::getchar();
        this->move(diff.data());

        this->xi += xi_diff_fw;
        this->xis_index += 1;

        // Read torque.
        this->readTorque();
        
        // Save.
        this->saveData(ros::Time::now().toSec(), xi_diff_fw, diff);

        return;
    }
}

void CtmMpt2::trackPath2(void)
{
    if (this->xis_index == this->xis_cols-1)
    {
        ROS_INFO("Path completed.");
        std::getchar();

        return;
    }
    else if (this->xis_index == -1)
    {
        this->setTargetXi(this->xis.col(0));
        this->trackXi();
        this->setTargetPose((const Eigen::Matrix4f)this->Tds.block(0,0,4,4));
        this->trackPose();

        this->xis_index += 1;

        return;
    }
    else
    {
        printf("xis_index = %d\n", (int)this->xis_index);
        this->xis_index += 1;

        // Get the feedforward xi_diff_fw.
        Eigen::Matrix<float, N_DIM, 1> xi_diff_fw = (this->xis.col(this->xis_index) - this->xis.col(this->xis_index-1))/this->NILC;

        for (size_t i = 0; i < this->NILC; i++)
        {
            // Currently at the (i-1)-th target pose.
            // Want to track the i-th target pose.
            this->setTargetPose((const Eigen::Matrix4f)this->Tds.block(0,4*(this->xis_index),4,4));
            
            // Calculate the error towards the i-th target pose.
            Eigen::Matrix<float, 6, 1> Vi = getError();

            // Calculate the Jacobian and then xi_diff_fb,
            // provided with three different methods.
            // *************************************************
            // Gradient.
            // Eigen::Matrix<float, 6, 6> J = this->jacobian3cc(1/this->L1, 1/this->L2, 1/this->L3, this->xi);
            // Eigen::Matrix<float, N_DIM, 1> xi_diff_fb = J.transpose() * Vi;
            // Newton-Raphson.
            // This method is not stable.
            // Eigen::Matrix<float, 6, 6> J = this->jacobian3cc(this->L1, this->L2, this->L3, this->xi);
            // Eigen::Matrix<float, N_DIM, 1> xi_diff_fb = (J.transpose() * J).inverse() * J.transpose() * Vi;
            // Levenberg-Marquardt.
            Eigen::Matrix<float, 6, 6> J = this->jacobian3cc(this->L1, this->L2, this->L3, this->xi);
            Eigen::Matrix<float, 6, 6> M = J.transpose() * J;
            float damp = M.diagonal().maxCoeff();
            Eigen::Matrix<float, N_DIM, 1> xi_diff_fb = (M + damp*Eigen::Matrix<float, 6, 6>::Identity()).inverse() * J.transpose() * Vi;
            // *************************************************

            // Add up xi_diff_fb and xi_diff_fw.
            Eigen::Matrix<float, N_DIM, 1> xi_diff = xi_diff_fb + xi_diff_fw;
            std::cout << "xi_diff_fb = " << xi_diff_fb.transpose() << std::endl;
            std::cout << "xi_diff_fw = " << xi_diff_fw.transpose() << std::endl;

            // Get cable length difference.
            Eigen::Matrix<float, N_CABLE, 1> diff = this->getCableLengthDiff(xi_diff);
            std::cout << "diff = " << diff.transpose() << std::endl;
            std::cout << "-------------------------------" << std::endl;

            // Move the motors!
            this->move(diff.data());

            this->xi += xi_diff;

            // Read torque.
            this->readTorque();
            
            // Save.
            this->saveData(ros::Time::now().toSec(), xi_diff, diff);
        }

        return;
    }
}
*/
void CtmMpt2::trackX(void)
{
    Eigen::Matrix<float, N_DIM, 1> xi_diff = this->xid - this->xi;

    // Get cable length difference.
    Eigen::Matrix<float, N_CABLE, 1> diff = this->getCableLengthDiff(xi_diff);

    // Move the motors!
    this->move(diff.data(), true);

    // Calculate the error.
    Eigen::Matrix<float, 6, 1> V = getError();

    // Update xi.
    this->xi += xi_diff;

    // Read torque.
    this->readTorque();
    
    // Save.
    this->saveData(ros::Time::now().toSec(), xi_diff, diff);

    std::cout << "Track -------------------------" << std::endl;
    std::cout << "V = " << V.transpose() << std::endl;
    std::cout << "xi = " << this->xi.transpose() << std::endl;
    std::cout << "diff = " << diff.transpose() << std::endl;
    std::cout << "-------------------------------" << std::endl;

    return;
}

void CtmMpt2::trackPFB(const bool wait)
{
    ros::spinOnce();

    // Calculate the error.
    Eigen::Matrix<float, 6, 1> Vi = getError();

    // Calculate the Jacobian and then xi difference,
    // provided with three different methods.
    // *************************************************
    Eigen::Matrix<float, 6, 6> J = this->jacobian3cc(this->L1, this->L2, this->L3, this->xi);
    Eigen::Matrix<float, 6, 6> M = J.transpose() * this->W * J;
    float damp = M.diagonal().maxCoeff();
    Eigen::Matrix<float, N_DIM, 1> xi_diff = (M + damp*Eigen::Matrix<float, 6, 6>::Identity()).inverse() * J.transpose() * this->W * Vi;
    // *************************************************

    // Get cable length difference.
    Eigen::Matrix<float, N_CABLE, 1> diff = this->getCableLengthDiff(xi_diff);

    // Move the motors!
    this->move(diff.data(), wait);

    // Update xi.
    this->xi += xi_diff;

    // Read torque.
    this->readTorque();
    
    // Save.
    this->saveData(ros::Time::now().toSec(), xi_diff, diff);

    std::cout << "Track -------------------------" << std::endl;
    std::cout << "V = " << Vi.transpose() << std::endl;
    std::cout << "xi = " << this->xi.transpose() << std::endl;
    std::cout << "xi_diff = " << xi_diff.transpose() << std::endl;
    std::cout << "diff = " << diff.transpose() << std::endl;
    std::cout << "-------------------------------" << std::endl;

    return;
}

void CtmMpt2::trackPFBFW(const bool wait)
{
    ros::spinOnce();

    // Calculate the error between the current and the target transformation.
    Eigen::Matrix<float, 6, 1> Vi = getError();

    // Calculate the Jacobian and then xi_diff_fb,
    // provided with three different methods.
    // *************************************************
    Eigen::Matrix<float, 6, 6> J = this->jacobian3cc(this->L1, this->L2, this->L3, this->xi);
    Eigen::Matrix<float, 6, 6> M = J.transpose() * this->W * J;
    float damp = M.diagonal().maxCoeff();
    Eigen::Matrix<float, N_DIM, 1> xi_diff_fb = (M + damp*Eigen::Matrix<float, 6, 6>::Identity()).inverse() * J.transpose() * this->W * Vi;
    // *************************************************

    // Add up xi_diff_fb and xi_diff_fw.
    Eigen::Matrix<float, N_DIM, 1> xi_diff = xi_diff_fb + this->xi_diff_fw;

    // Get cable length difference.
    Eigen::Matrix<float, N_CABLE, 1> diff = this->getCableLengthDiff(xi_diff);
    
    // Move the motors!
    this->move(diff.data(), wait);

    // Update xi.
    this->xi += xi_diff;

    // Read torque.
    this->readTorque();
    
    // Save.
    this->saveData(ros::Time::now().toSec(), xi_diff, diff);

    std::cout << "Track -------------------------" << std::endl;
    std::cout << "V = " << Vi.transpose() << std::endl;
    std::cout << "xi = " << this->xi.transpose() << std::endl;
    std::cout << "xi_diff_fb = " << xi_diff_fb.transpose() << std::endl;
    std::cout << "xi_diff_fw = " << this->xi_diff_fw.transpose() << std::endl;
    std::cout << "xi_diff = " << xi_diff.transpose() << std::endl;
    std::cout << "diff = " << diff.transpose() << std::endl;
    std::cout << "-------------------------------" << std::endl;

    return;
}

void CtmMpt2::trackVFB(void)
{
    static ros::Time tt = ros::Time::now();
    static Eigen::Matrix<float, N_DIM, 1> xi_dot = Eigen::Matrix<float, N_DIM, 1>::Zero();

    Eigen::Matrix<float, N_DIM, 1> xi_diff = (ros::Time::now() - tt).toSec() * xi_dot;
    this->xi += xi_diff;

    ros::spinOnce();

    // Calculate the error between the current and the target transformation.
    Eigen::Matrix<float, 6, 1> Vi = getError();

    // this->xi2 = getXi2();
    // std::cout << "xi2 = " << std::endl << this->xi2.transpose() << std::endl;

    // Calculate the Jacobian and then xi_dot,
    // provided with three different methods.
    // *************************************************
    // Levenberg-Marquardt.
    Eigen::Matrix<float, 6, 6> J = this->jacobian3cc(this->L1, this->L2, this->L3, this->xi);
    Eigen::Matrix<float, 6, 6> M = J.transpose() * this->W * J;
    float damp = M.diagonal().maxCoeff();
    xi_dot = (M + damp*Eigen::Matrix<float, 6, 6>::Identity()).inverse() * J.transpose() * this->W * Vi;
    xi_dot *= CR;
    // *************************************************

    // Get cable length derivative.
    // v = v_fw + v_fb = A xi_dot_fw + A xi_dot_fb = A (xi_dot_fw + xi_dot_fb).
    Eigen::Matrix<float, N_CABLE, 1> vel = this->getCableLengthDot(xi_dot);

    // Normalise the velocity before write to motors.
    float norm = (vel.array().abs() / this->L_DOT_MAX).maxCoeff();
    if (norm > 1.0)
    {
        vel /= norm;
        xi_dot /= norm;
    }

    // Run the motors!
    tt = ros::Time::now();
    this->run(vel.data());
    
    // Read torque.
    this->readTorque();
    
    // Save.
    this->saveData(ros::Time::now().toSec(), xi_diff, this->getCableLengthDiff(xi_diff));

    // Update xi.
    // this->xi = this->xi2;

    std::cout << "Track -------------------------" << std::endl;
    std::cout << "V = " << Vi.transpose() << std::endl;
    std::cout << "xi = " << this->xi.transpose() << std::endl;
    // std::cout << "xi2 = " << this->getXi2().transpose() << std::endl;
    std::cout << "xi_dot = " << xi_dot.transpose() << std::endl;
    std::cout << "vel = " << vel.transpose() << std::endl;
    std::cout << "-------------------------------" << std::endl;

    return;
}

void CtmMpt2::trackVFBFW(void)
{
    ros::spinOnce();

    // Calculate the error between the current and the target transformation.
    Eigen::Matrix<float, 6, 1> Vi = getError();

    // Calculate the Jacobian and then xi_diff_fb,
    // provided with three different methods.
    // *************************************************
    // Levenberg-Marquardt.
    Eigen::Matrix<float, 6, 6> J = this->jacobian3cc(this->L1, this->L2, this->L3, this->xi);
    Eigen::Matrix<float, 6, 6> M = J.transpose() * this->W * J;
    float damp = M.diagonal().maxCoeff();
    Eigen::Matrix<float, N_DIM, 1> xi_diff_fb = (M + damp*Eigen::Matrix<float, 6, 6>::Identity()).inverse() * J.transpose() * this->W * Vi;
    // *************************************************

    // Add up the feedback velocity and the feedforward velocity.
    Eigen::Matrix<float, N_DIM, 1> xi_dot = CR * xi_diff_fb + this->xi_dot_fw;

    // Get cable length derivative.
    // v = v_fw + v_fb = A xi_dot_fw + A xi_dot_fb = A (xi_dot_fw + xi_dot_fb).
    Eigen::Matrix<float, N_CABLE, 1> vel = this->getCableLengthDot(xi_dot);
    
    // Normalise the velocity before write to motors.
    float norm = (vel.array().abs() / this->L_DOT_MAX).maxCoeff();
    if (norm > 1.0)
    {
        vel /= norm;
        xi_dot /= norm;
    }

    // Run the motors!
    this->run(vel.data());

    // Update xi.
    this->xi += xi_dot / CR;

    // Read torque.
    this->readTorque();
    
    // Save.
    this->saveData(ros::Time::now().toSec(), xi_dot / CR, vel / CR);

    std::cout << "Track -------------------------" << std::endl;
    std::cout << "V = " << Vi.transpose() << std::endl;
    std::cout << "xi = " << this->xi.transpose() << std::endl;
    std::cout << "xi_dot_fb = " << CR * xi_diff_fb.transpose() << std::endl;
    std::cout << "xi_dot_fw = " << this->xi_dot_fw.transpose() << std::endl;
    std::cout << "xi_dot = " << xi_dot.transpose() << std::endl;
    std::cout << "vel = " << vel.transpose() << std::endl;
    std::cout << "-------------------------------" << std::endl;

    return;
}
