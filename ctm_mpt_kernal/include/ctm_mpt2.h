#ifndef CTM_MPT2_H
#define CTM_MPT2_H

#include "ctm_mpt.h"
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <std_msgs/Float32MultiArray.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#define _PI 3.1415926535897932

// Number of motors.
const size_t N_MOTOR = 9;
// Number of cables.
const size_t N_CABLE = 9;
// Dimensionality of xi.
const size_t N_DIM = 6;
// Kinematics rate KR. Unit: Hz.
// Control rate CR. Unit: Hz.
const float KR = 1.0;
const float CR = 2.0;
const float KRDCR = KR/CR;

template <const size_t N_SF>
class XiSpFunc
{
public:
    XiSpFunc(const Eigen::Matrix<double, 1, Eigen::Dynamic>& t_vec,
        const Eigen::Matrix<double, N_SF, Eigen::Dynamic>& x_vec);

    ~XiSpFunc();

    const Eigen::Matrix<float, N_SF*3, 1> operator()(const double t);

private:
    Eigen::Spline<double, N_SF> spline_;
    
    Eigen::Matrix<double, N_SF, 1> xi_last_;
};

template <const size_t N_SF>
XiSpFunc<N_SF>::XiSpFunc(const Eigen::Matrix<double, 1, Eigen::Dynamic>& t_vec,
        const Eigen::Matrix<double, N_SF, Eigen::Dynamic>& x_vec)
  : spline_(Eigen::SplineFitting<Eigen::Spline<double,N_SF>>::Interpolate(
        x_vec,
        // No more than cubic spline, but accept short vectors.
        std::min<int>(x_vec.cols()-1, 3),
        t_vec
        )
    ),
    xi_last_(Eigen::Matrix<double, N_SF, 1>::Zero())
{
    return;
}

template <const size_t N_SF>
XiSpFunc<N_SF>::~XiSpFunc()
{
    return;
}

template <const size_t N_SF>
const Eigen::Matrix<float, N_SF*3, 1> XiSpFunc<N_SF>::operator()(const double t)
{
    Eigen::Matrix<double, N_SF*3, 1> xi_msg;
    xi_msg.block(0,0,N_SF*2,1) = this->spline_.derivatives(t, 1).reshaped(N_SF*2,1);
    xi_msg.block(N_SF*2,0,N_SF,1) = (xi_msg.block(0,0,N_SF,1) - this->xi_last_) * KRDCR;
    this->xi_last_ = xi_msg.block(0,0,N_SF,1);
    
    return xi_msg.template cast<const float>();
}

template <const size_t N_C>
class CtrlPID
{
public:
    CtrlPID(const float kp, const float ki, const float kd, const float dt);

    ~CtrlPID();

    const Eigen::Matrix<float, N_C, 1> pid(const Eigen::Matrix<float, N_C, 1>& err);
    
private:
    float dt_;
    float one_over_dt_;

    float kp_;
    float ki_;
    float kd_;
    
    Eigen::Matrix<float, N_C, 1> err_sum_;
    Eigen::Matrix<float, N_C, 1> err_last_;

    bool is_first_run_ = true;
};

template <const size_t N_C>
CtrlPID<N_C>::CtrlPID(const float kp, const float ki, const float kd, const float dt)
{
    this->dt_ = dt;
    this->one_over_dt_ = 1.0 / dt;

    this->kp_ = kp;
    this->ki_ = ki;
    this->kd_ = kd;

    this->err_sum_ = Eigen::Matrix<float, N_C, 1>::Zero();
    this->err_last_ = Eigen::Matrix<float, N_C, 1>::Zero();

    return;
}

template <const size_t N_C>
CtrlPID<N_C>::~CtrlPID()
{
    return;
}

template <const size_t N_C>
const Eigen::Matrix<float, N_C, 1> CtrlPID<N_C>::pid(
    const Eigen::Matrix<float, N_C, 1>& err)
{
    Eigen::Matrix<float, N_C, 1> derr = Eigen::Matrix<float, N_C, 1>::Zero();
    Eigen::Matrix<float, N_C, 1> out = Eigen::Matrix<float, N_C, 1>::Zero();

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

class CtmMpt2 : protected CtmMpt
{
public:
    CtmMpt2();
    CtmMpt2(const std::string& mtr_port,
            ros::NodeHandle* nh);
    CtmMpt2(const std::string& snsr_port_1,
            const std::string& snsr_port_2,
            ros::NodeHandle* nh);
    CtmMpt2(const std::string& snsr_port_1,
            const std::string& snsr_port_2,
            const std::string& mtr_port,
            ros::NodeHandle* nh);

    virtual ~CtmMpt2();

public:
    void printPose(void); // Pose feedback from the motion capture system.
    void print(const uint8_t id); // Single motor status.
    void print(void); // A table of all motors.

    bool readTorque(void); // Read torque values.

    void stop(void); // Stop all motors.
    void init(void); // Initialise motors and sensors.
    void reset(void); // Return to zero positions.
    void zero(void); // Adjust zero.
    
    void move(const float *const dist, bool wait); // Relative linear distances. Unit: mm.
    void move(const uint8_t id, const float dist, bool wait); // Move one single motor.
    void move(const float dist, bool wait); // Move all motors.
    
    // Function run() is dangerous.
    // Must call it together with stop().
    // Current maximum value: 1.31947 mm/s.
    void run(const float *const dist_dot); // Relative linear velocity. Unit: mm/s.
    void run(const uint8_t id, const float dist_dot); // Run one single motor.
    void run(const float dist_dot); // Run all motors.

    // void setTargetTorque(const Eigen::Matrix<float, N_CABLE, 1>& Tau);
    // void setTargetTorque(const float Tau);
    // Set through a homogeneous transformation matrix.
    // void setTargetPose(const Eigen::Matrix4f& P = Eigen::Matrix4f::Zero());
    // Give a series of xi.
    // void setTargetPath(const Eigen::Matrix<float, N_DIM, Eigen::Dynamic>& Xis);
    // Call after setting a target path to allocate time for each via point.
    // Unit: second.
    // void allocateTime(void);

    // Motor controller tracking procedure.
    // No feedforward.
    void trackX(void);
    void trackPFB(const bool wait);
    void trackVFB(void);
    // With feedforward.
    void trackPFBFW(const bool wait);
    void trackVFBFW(void);

    // void trackTorque(void);

    // Position feedforward.
    // void trackPath(void);
    // void trackPath2(void);

    // Velocity feedforward, no position feedback.
    // void trackTrajectory(void);

    // Velocity feedforward and position feedback.
    // void trackTrajectory2(void);

private:
    // Model.
    // Unit of length L1, L2, L3: mm.
    const float L1 = 256;
    const float L2 = 256;
    const float L3 = 256;

    // Return the end pose (forward kinematics).
    const Eigen::Matrix4f fk3cc(
        const float L1,
        const float L2,
        const float L3,
        const Eigen::Matrix<float, N_DIM, 1>& Xi
    );

    // Return the Jacobian matrix.
    const Eigen::Matrix<float, N_DIM, N_DIM> jacobian3cc(
        const float L1,
        const float L2,
        const float L3,
        const Eigen::Matrix<float, N_DIM, 1>& Xi
    );

    // Return the estimated xi through the command position feedback of motors.
    const Eigen::Matrix<float, N_DIM, 1> getXi2(void);
    
    // State variables.
    // Torque.
    float tau[N_CABLE] = {0.0}; // Need to be an array.
    // Desired torque.
    Eigen::Matrix<float, N_CABLE, 1> taud = Eigen::Matrix<float, N_CABLE, 1>::Zero();

    // For a desired xi.
    // Desired xi.
    // {0.8147, 0.9058, 0.1270, 0.9134, 0.6324, 0.0975};
    Eigen::Matrix<float, N_DIM, 1> xid = Eigen::Matrix<float, N_DIM, 1>::Zero();
    // Current xi.
    Eigen::Matrix<float, N_DIM, 1> xi = Eigen::Matrix<float, N_DIM, 1>::Zero();
    // This xi is estimated through absolute command position feedback of motors.
    Eigen::Matrix<float, N_DIM, 1> xi2 = Eigen::Matrix<float, N_DIM, 1>::Zero();

    // For a desired pose.
    // Desired transformation.
    Eigen::Matrix4f Td = Eigen::Matrix4f::Identity();
    // Body transformation.
    Eigen::Matrix4f Tb = Eigen::Matrix4f::Identity();
    // Weight matrix, for optimisation.
    const Eigen::DiagonalMatrix<float, N_DIM> W {5.0, 5.0, 5.0, 1.0, 1.0, 1.0};

    // Log files and write log function.
    std::ofstream file_log;
    void saveData(
        const double t,
        const Eigen::Matrix<float, N_DIM, 1>& mat_n_dim_by_1 = Eigen::Matrix<float, N_DIM, 1>::Zero(),
        const Eigen::Matrix<float, N_CABLE, 1>& mat_n_cable_by_1 = Eigen::Matrix<float, N_CABLE, 1>::Zero()
    );

    // // Force feedback PID controller.
    // CtrlPID<N_CABLE> ffbc {0.5, 0.0, 0.0, 1.0};
    // // Position feedback PID controller.
    // CtrlPID<N_DIM> pfbc {1, 0.0, 0.0, 1.0};
    // // Velocity feedforward and position feedback PID controller.
    // CtrlPID<N_DIM> vfwpfbc {1.0, 0.0, 0.0, 1.0};
    // // Number of inner loop control during one path tracking action.
    // const size_t NILC = 10;

    // Initial transformation.
    Eigen::Matrix4f invT_zero// = Eigen::Matrix4f::Identity();
    {
        {0.151374945569343,-0.0124623769613737,0.988397852594972,-218.786913752145},
        {0.988212752680851,-0.0211857279629794,-0.151613720914748,-110.545856654432},
        {0.0228293953668265,0.999697881407754,0.00910849142470263,-12.5438697597325},
        {0,0,0,1}
    };

    // Motion capture system setup.
    // Get body transformation with the motion capture system.
    ros::Subscriber tf_subscriber;
    void tfCallback(const geometry_msgs::PoseStamped& tfmsg);

    // Receive desired xi from the kinematics controller.
    ros::Subscriber xi_subscriber;
    void xiCallback(const std_msgs::Float32MultiArray& ximsg);
    Eigen::Matrix<float, N_DIM, 1> xi_diff_fw = Eigen::Matrix<float, N_DIM, 1>::Zero();
    Eigen::Matrix<float, N_DIM, 1> xi_dot_fw = Eigen::Matrix<float, N_DIM, 1>::Zero();

    // Get robot tip error.
    const Eigen::Matrix<float, 6, 1> getError(void);

    // Calculate the cable length difference.
    // Output: length difference. Unit: mm.
    // Input: xi difference.
    const Eigen::Matrix<float, N_CABLE, 1> getCableLengthDiff(
        const Eigen::Matrix<float, N_DIM, 1>& xi_diff
    );
    // Based on my theory, these two functions have the same expression.
    const Eigen::Matrix<float, N_CABLE, 1> getCableLengthDot(
        const Eigen::Matrix<float, N_DIM, 1>& xi_dot
    );
    // Maximum cable length derivative.
    // Calculated by element-wise VEL / RESOLUTION.
    Eigen::Array<float, N_CABLE, 1> L_DOT_MAX = Eigen::Matrix<float, N_CABLE, 1>::Zero();

    // Calculate U_DELTA and C_DELTA in MATLAB.
    const Eigen::Matrix<float, N_CABLE, N_DIM> PUTC {
        {0.0f,-42.0f,0.0f,0.0f,0.0f,0.0f},
        {36.3730669589464f,21.0000000000000f,0.0f,0.0f,0.0f,0.0f},
        {-36.3730669589464f,21.0000000000000f,0.0f,0.0f,0.0f,0.0f},
        {26.9970796068347f,-32.1738666109971f,26.9970796068347f,-32.1738666109971f,26.9970796068347f,-32.1738666109971f},
        {14.3648460196781f,39.4670900730082f,14.3648460196781f,39.4670900730082f,14.3648460196781f,39.4670900730082f},
        {-41.3619256265127f,-7.29322346201106f,-41.3619256265127f,-7.29322346201106f,-41.3619256265127f,-7.29322346201106f},
        {41.3619256265127f,-7.29322346201108f,41.3619256265127f,-7.29322346201108f,0.0f,0.0f},
        {-14.3648460196781f,39.4670900730082f,-14.3648460196781f,39.4670900730082f,0.0f,0.0f},
        {-26.9970796068347f,-32.1738666109971f,-26.9970796068347f,-32.1738666109971f,0.0f,0.0f}
    };

    // Torque sensors setup.
    ros::Publisher torque_publisher;
    std_msgs::Float32MultiArray torque_message;

    const float TORQUE_PERMITTED[N_MOTOR] = {
        5.5, 5.5, 5.5, 5.5, 5.5, 5.5, 5.5, 5.5, 5.5
    };

    const float TORQUE_OFFSET[N_MOTOR] = {
        // 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        -0.273189f, -0.864938f, -0.308825f,
        -0.155347f, -0.791518f, -0.142095f,
        -0.786404f, -0.479842f, -0.538369f
    };

    // Motor parameters.
    const uint8_t ID_MOTOR[N_MOTOR] = {
        1, 2, 3, 4, 5, 6, 7, 8, 9
    };
    // const uint8_t ID_DISTAL[3] = {4, 5, 6};
    // const uint8_t ID_MIDDLE[3] = {7, 8, 9};
    // const uint8_t ID_PROXIMAL[3] = {1, 2, 3};

    // Step per rev / (2 * _PI * PULLEY_RADIUS) per rev,
    // e.g. 2,400,000 / (2 * _PI * 42) = 200,000 / (7 * _PI),
    // where 2,400,000 = 1,000 * 30 * 80, PULLEY_RADIUS = 42 mm.
    // Minus sign: positive dist means positive cable length change.
    const float RESOLUTION[N_MOTOR] = {
         -200000/(7*_PI),  -200000/(7*_PI),  -200000/(7*_PI),
        -200000/(21*_PI), -200000/(21*_PI), -200000/(21*_PI),
        -200000/(21*_PI), -200000/(21*_PI), -200000/(21*_PI)
    };

    // Velocities. Unit: Hz.
    const float VEL1[N_MOTOR] = {
        30000, 30000, 30000,
        10000, 10000, 10000,
        10000, 10000, 10000
    };
    // const float VEL1[N_MOTOR] = {
    //     12000, 12000, 12000,
    //      4000,  4000,  4000,
    //      4000,  4000,  4000
    // };
    // const float VEL2[N_MOTOR] = {
    //     9000, 9000, 9000,
    //     3000, 3000, 3000,
    //     3000, 3000, 3000
    // };
    // const float VEL3[N_MOTOR] = {
    //     6000, 6000, 6000,
    //     2000, 2000, 2000,
    //     2000, 2000, 2000
    // };
    // const float VEL4[N_MOTOR] = {
    //     3000, 3000, 3000,
    //     1000, 1000, 1000,
    //     1000, 1000, 1000
    // };
    const float* VEL = VEL1;
    
    // Accelerations. Unit: Hz/s.
    const uint32_t ACC_START[N_MOTOR] = {
        300000, 300000, 300000,
        100000, 100000, 100000,
        100000, 100000, 100000
    };

    const uint32_t ACC_BRAKE[N_MOTOR] = {
        300000, 300000, 300000,
        100000, 100000, 100000,
        100000, 100000, 100000
    };
    
    
    // const uint32_t ACC_START[N_MOTOR] = {
    //     360000, 360000, 360000,
    //     120000, 120000, 120000,
    //     120000, 120000, 120000
    // };

    // const uint32_t ACC_BRAKE[N_MOTOR] = {
    //     360000, 360000, 360000,
    //     120000, 120000, 120000,
    //     120000, 120000, 120000
    // };
};

#endif