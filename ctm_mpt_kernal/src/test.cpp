#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>
#include "ctm_mpt2.h"

void watchControlSignal(float* ctr, float* dst, float* src, float* err, size_t n);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    ros::Publisher trq_pub = nh.advertise<std_msgs::Float32MultiArray>("trq", 100);
    
    // cd /dev
    // ls -al ttyUSB*
    // sudo chmod a+rw ttyUSB*
    // 
    //            "sensor 1",     "sensor 2",        "motor".
    CtmMpt2 m("/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB0");
    
    const size_t N = 9;
    CntlrPID c(3, 0, 0, 0.02, N);
    float trq[N] = {0.0};
    float trq_expt[N] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    float trq_diff[N] = {0.0};
    float pid_out[N] = {0.0};

    std_msgs::Float32MultiArray msg;
    std_msgs::MultiArrayDimension dim;
    dim.label = "tau";
    dim.size = N;
    dim.stride = N;
    msg.layout.dim.push_back(dim);
    msg.layout.data_offset = 0;
    msg.data = std::vector<float>(N, 0.0);

    // m.print();
    // m.init();
    // m.zero();
    // m.print();
    float dist[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    ros::Rate loop_rate(2);
    std::getchar();
    while (ros::ok())
    {
        size_t i = 0;
        
        // Check if it is overloaded.
        if (m.read(trq))
        {
            return 1;
        }

        while (i < N)
        {
            msg.data.at(i) = trq[i];
            // For single motor pid debugging, use
            uint8_t id = 1;
            trq_diff[id-1] = trq_expt[id-1] - trq[id-1];
            // For multiple, use
            // trq_diff[i] = trq_expt[i] - trq[i];
            i++;
        }
        trq_pub.publish(msg);

        // c.pid(trq_diff, pid_out);
        // When tuning a PID, print the control signal first.
        // Check if the values are abnormal before continue.
        // watchControlSignal(pid_out, trq_expt, trq, trq_diff, N);
        // m.move(pid_out);

        m.print();
        std::getchar();
        m.move(dist);

        loop_rate.sleep();
    }

    return 0;
}

void watchControlSignal(float* out, float* dst, float* src, float* err, size_t n)
{
    size_t i = 0;

    std::cout << "Current ";
    for (i = 0; i < n; i++)
    {
        printf("%7.4f ", src[i]);
    }
    std::cout << std::endl;

    std::cout << "Desire  ";
    for (i = 0; i < n; i++)
    {
        printf("%7.4f ", dst[i]);
    }
    std::cout << std::endl;

    std::cout << "Error   ";
    for (i = 0; i < n; i++)
    {
        printf("%7.4f ", err[i]);
    }
    std::cout << std::endl;

    std::cout << "Output  ";
    for (i = 0; i < n; i++)
    {
        printf("%7.4f ", out[i]);
    }
    std::cout << std::endl;

    return;
}