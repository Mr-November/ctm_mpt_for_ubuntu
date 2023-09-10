#ifndef CTM_MPT2_H
#define CTM_MPT2_H

#include "ctm_mpt.h"
#include <ros/ros.h>
#include <Eigen/Dense>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#define _PI 3.1415926535897932

class CtrlPID
{
public:
    CtrlPID(const float kp, const float ki, const float kd,
        const float dt, const size_t n);

    ~CtrlPID();

    Eigen::MatrixXf pid(const Eigen::MatrixXf err);
    
private:
    float dt_;
    float one_over_dt_;

    size_t n_;

    float kp_;
    float ki_;
    float kd_;
    
    Eigen::MatrixXf err_sum_;
    Eigen::MatrixXf err_last_;

    bool is_first_run_ = true;
};

class CtmMpt2 : protected CtmMpt
{
public:
    CtmMpt2(const std::string& snsr_port_1,
            const std::string& snsr_port_2,
            const std::string& mtr_port,
            ros::NodeHandle* nh);

    virtual ~CtmMpt2();

public:
    void print(const uint8_t id); // Single motor status.
    void print(void); // A table of all motors.

    bool readTorque(void); // Read torque values.

    void stop(void); // Stop motors for emergency.
    void init(void); // Initialise motors and sensors.
    void reset(void); // Return to zero positions.
    void zero(void); // Adjust zero.
    void move(const float *const dist); // Relative linear distances. Unit: mm.
    void move(const uint8_t id, const float dist); // Move single motor.
    void relax(const float dist = 20); // Set positive cable length to reduce the tension.

    void setTargetPose(const Eigen::Matrix4f P);
    void trackPoint(void);

    // Control time interval. Unit: millisecond.
    // There are another time interval when declaring the controllers.
    // Please keep the values the same.
    const float CTI = 1000.0;

private:
    // Model.
    // Return the Jacobian matrix.
    // Unit of length L1, L2, L3: mm.
    Eigen::Matrix<float, 6, 6> jacobian3cc(const float L1 = 256,
                                           const float L2 = 256,
                                           const float L3 = 256);
    
    // State variables.
    float torque[9] = {0.0};
    Eigen::Matrix<float, 6, 1> xi = Eigen::MatrixXf::Zero(6, 1);//{0.8147, 0.9058, 0.1270, 0.9134, 0.6324, 0.0975};

    // Position PID controller.
    CtrlPID position_controller {0.1, 0.0, 0.0, 1000.0, 6};

    // Initial transformation.
    Eigen::Vector3f orig_zero {0.0, 0.0, 0.0};
    Eigen::Quaternionf quat_zero {1.0, 0.0, 0.0, 0.0};
    // A constant matrix calculated from orig_zero and quat_zero.
    Eigen::Matrix4f invT_zero = Eigen::Matrix4f::Zero(4, 4);

    // Body transformation.
    Eigen::Matrix4f Tb = Eigen::Matrix4f::Identity(4, 4);//Zero(4, 4);

    // Desired transformation.
    Eigen::Matrix4f Td = Eigen::Matrix4f::Identity(4, 4);//Zero(4, 4);

    // Motion capture system setup.
    ros::Subscriber tf_subscriber;

    // Get body transformation with the motion capture system.
    void mcsCallback(const geometry_msgs::PoseStamped& tfmsg);
    
    // Parameters.
    // Motor position.
    const uint8_t ID_ALL[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    const uint8_t ID_DISTAL[3] = {4, 5, 6};
    const uint8_t ID_MIDDLE[3] = {7, 8, 9};
    const uint8_t ID_PROXIMAL[3] = {1, 2, 3};
    // Number of motors.
    const size_t N_ALL = 9;
    // Number of cables.
    const size_t N_CABLE = 9;
    // Dimensionality of xi.
    const size_t N_DIM = 6;

    void getCableLengthDiff(float *const diff,
                            const Eigen::Matrix<float, 6, 1> xi_diff);

    // Calculate U_DELTA and C_DELTA in MATLAB.
    const Eigen::Matrix<float, 6, 9> U_DELTA {
            {42.00f,-20.999999999999989f,-21.000000000000018f, 7.293223462011078f,-39.467090073008158f, 32.173866610997067f,32.173866610997074f,-39.467090073008151f,  7.293223462011059f},
            { 0.00f, 36.373066958946424f,-36.373066958946417f,41.361925626512736f,-14.364846019678065f,-26.997079606834664f,26.997079606834649f, 14.364846019678094f,-41.361925626512743f},
            { 0.00f,               0.00f,               0.00f, 7.293223462011078f,-39.467090073008158f, 32.173866610997067f,32.173866610997074f,-39.467090073008151f,  7.293223462011059f},
            { 0.00f,               0.00f,               0.00f,41.361925626512736f,-14.364846019678065f,-26.997079606834664f,26.997079606834649f, 14.364846019678094f,-41.361925626512743f},
            { 0.00f,               0.00f,               0.00f,              0.00f,               0.00f,               0.00f,32.173866610997074f,-39.467090073008151f,  7.293223462011059f},
            { 0.00f,               0.00f,               0.00f,              0.00f,               0.00f,               0.00f,26.997079606834649f, 14.364846019678094f,-41.361925626512743f},
        };
    
    const Eigen::Matrix<float, 6, 6> C_DELTA {
        {0.0f, -1.0f,  0.0f,  0.0f,  0.0f,  0.0f},
        {1.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f},
        {0.0f,  0.0f,  0.0f, -1.0f,  0.0f,  0.0f},
        {0.0f,  0.0f,  1.0f,  0.0f,  0.0f,  0.0f},
        {0.0f,  0.0f,  0.0f,  0.0f,  0.0f, -1.0f},
        {0.0f,  0.0f,  0.0f,  0.0f,  1.0f,  0.0f},
        };

    // Torque sensors setup.
    ros::Publisher torque_publisher;

    const float TORQUE_PERMITTED[9] = {5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0};

    const float TORQUE_OFFSET[9] = {-0.318507f, -0.792682f, -0.344987f,
                                    -0.136032f, -0.793897f, -0.280762f,
                                    -0.835308f, -0.437068f, -0.496267f};

    // Motor parameters.
    // 1 / PULLEY_RADIUS * Step per rev / (2 * _PI) per rev,
    // e.g. 2,400,000 / (2 * _PI * 42) = 200,000 / (7 * _PI),
    // where 2,400,000 = 1,000 * 30 * 80, PULLEY_RADIUS = 42 mm.
    // Minus sign: positive dist means positive cable length change.
    const float RESOLUTION[9] = { -200000/(7*_PI),  -200000/(7*_PI),  -200000/(7*_PI),
                                 -200000/(21*_PI), -200000/(21*_PI), -200000/(21*_PI),
                                 -200000/(21*_PI), -200000/(21*_PI), -200000/(21*_PI)};

    // Velocities. Unit: Hz.
    const int32_t VEL[9] = {12000, 12000, 12000,
                             4000,  4000,  4000,
                             4000,  4000,  4000};
    
    // Accelerations. Unit: Hz/s.
    const uint32_t ACC_START[9] = {1800000, 1800000, 1800000,
                                    600000,  600000,  600000,
                                    600000,  600000,  600000};

    const uint32_t ACC_BRAKE[9] = {1800000, 1800000, 1800000,
                                    600000,  600000,  600000,
                                    600000,  600000,  600000};
};

#endif