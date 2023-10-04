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
const float KR = 0.5;
const float KRDCR = 1.0/4.0;
const float CR = 2;

template <const size_t N_SF>
class SplineFunction
{
public:
    SplineFunction(const Eigen::Matrix<double, 1, Eigen::Dynamic>& t_vec,
        const Eigen::Matrix<double, N_SF, Eigen::Dynamic>& x_vec)
    : spline_(Eigen::SplineFitting<Eigen::Spline<double,N_SF>>::Interpolate(
            x_vec,
            // No more than cubic spline, but accept short vectors.
            std::min<int>(x_vec.cols()-1, 3),
            t_vec
            )
        )
    {
        return;
    }

    const Eigen::Matrix<float, N_SF, 1> operator()(const double t) const
    {
        return this->spline_(t).template cast<const float>();
    }


private:
    Eigen::Spline<double, N_SF> spline_;
};

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

class CtmMpt2 : protected CtmMpt
{
public:
    CtmMpt2();
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
    void move(const uint8_t id, const float dist, bool wait); // Move a single motor.
    // Function run() is dangerous.
    // Must call it together with snooze() and stop().
    // Current maximum value: 1.31947 mm/s.
    void run(const float *const dist_dot); // Relative linear velocity. Unit: mm/s.
    void run(const uint8_t id, const float dist_dot); // Run a single motor.
    void relax(const float dist = 10); // Set positive cable length to reduce the tension.
    // Do something else while snoozing, like publishing sensor data.
    // Unit: second.
    void snooze(const float tttt);

    void setTargetTorque(const Eigen::Matrix<float, N_CABLE, 1>& Tau);
    void setTargetTorque(const float Tau);
    // Set through a homogeneous transformation matrix.
    void setTargetPose(const Eigen::Matrix4f& P = Eigen::Matrix4f::Zero());
    // Give a series of xi.
    void setTargetPath(const Eigen::Matrix<float, N_DIM, Eigen::Dynamic>& Xis);
    // Call after setting a target path to allocate time for each via point.
    // Unit: second.
    void allocateTime(void);

    // void trackTorque(void);
    // void trackPose(void);
    void trackXi(void);

    // Position feedforward.
    // void trackPath(void);
    // void trackPath2(void);

    // Motor controller tracking procedure.
    void track(void);

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

    // For a desired path.
    Eigen::Matrix<float, N_DIM, Eigen::Dynamic> xis;
    Eigen::Matrix<float, 4, Eigen::Dynamic> Tds;
    size_t xis_cols = 0;
    size_t xis_index = -1;

    // For a trajectory.
    Eigen::Matrix<float, 1, Eigen::Dynamic> time_interval;

    // Log files and write log function.
    std::ofstream file_log;
    void saveData(
        const double t,
        const Eigen::Matrix<float, N_DIM, 1>& xi_diff = Eigen::Matrix<float, N_DIM, 1>::Zero(),
        const Eigen::Matrix<float, N_CABLE, 1>& diff = Eigen::Matrix<float, N_CABLE, 1>::Zero()
    );

    // Force feedback PID controller.
    CtrlPID<N_CABLE> ffbc {0.5, 0.0, 0.0, 1.0};
    // Position feedback PID controller.
    CtrlPID<N_DIM> pfbc {1, 0.0, 0.0, 1.0};
    // Velocity feedforward and position feedback PID controller.
    CtrlPID<N_DIM> vfwpfbc {1.0, 0.0, 0.0, 1.0};
    // Number of inner loop control during one path tracking action.
    const size_t NILC = 10;

    // Initial transformation.
    Eigen::Matrix4f invT_zero //= Eigen::Matrix4f::Identity();
    {
        {-0.892959269501999,-0.0017208783670654,0.450134181759287,21.3082667037461},
        {0.450136688146614,-0.0015485003565991,0.892958321609158,-253.278177506032},
        {-0.000839639717376681,0.999997320358555,0.00215737823610617,-12.6148643825625},
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
        3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5
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

    // 1 / PULLEY_RADIUS * Step per rev / (2 * _PI) per rev,
    // e.g. 2,400,000 / (2 * _PI * 42) = 200,000 / (7 * _PI),
    // where 2,400,000 = 1,000 * 30 * 80, PULLEY_RADIUS = 42 mm.
    // Minus sign: positive dist means positive cable length change.
    const float RESOLUTION[N_MOTOR] = {
         -200000/(7*_PI),  -200000/(7*_PI),  -200000/(7*_PI),
        -200000/(21*_PI), -200000/(21*_PI), -200000/(21*_PI),
        -200000/(21*_PI), -200000/(21*_PI), -200000/(21*_PI)
    };

    // Velocities. Unit: Hz.
    const float VEL[N_MOTOR] = {
        12000, 12000, 12000,
         4000,  4000,  4000,
         4000,  4000,  4000
    };
    
    // Accelerations. Unit: Hz/s.
    const uint32_t ACC_START[N_MOTOR] = {
        1800000, 1800000, 1800000,
         600000,  600000,  600000,
         600000,  600000,  600000
    };

    const uint32_t ACC_BRAKE[N_MOTOR] = {
        1800000, 1800000, 1800000,
         600000,  600000,  600000,
         600000,  600000,  600000
    };
};

#endif