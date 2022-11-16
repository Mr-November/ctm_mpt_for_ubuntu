#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>
#include "ctm_mpt2.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trq_cntlr");
    ros::NodeHandle nh;
    
    // cd /dev
    // ls -al ttyUSB*
    // sudo chmod a+rw ttyUSB*
    // 
    // CtmMpt2 m;
    // CtmMpt2 m("/dev/ttyUSB0");
    // CtmMpt2 m("/dev/ttyUSB2", "/dev/ttyUSB1");
    //            "sensor 1",     "sensor 2",        "motor".
    CtmMpt2 m("/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB0");

    const size_t N = 9;
    CntlrPID c(10, 0, 0, 0.02, N);

    float trq[N] = {0.0};
    float trq_expt[N] = {0.0};
    float trq_diff_mult_1k[N] = {0.0};
    float pid_out[N] = {0.0};
    size_t i = 0;

    // m.init();
    m.print();
    // float dist[] = {-30, -20, -10, -60, -50, -40, -90, -80, -70};
    // m.move(dist);

    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        m.read(trq);

        while (i < N)
        {
            // // For single motor pid debugging, use
            // uint8_t id = 1;
            // trq_diff_mult_1k[id-1] = 1000 * (trq_expt[id-1] - trq[id-1]);
            // // For multiple, use
            trq_diff_mult_1k[i] = 1000*(trq_expt[i] - trq[i]);
            i++;
        }
        
        // c.pid(trq_diff_mult_1k, pid_out);
        // m.move(pid_out);

        loop_rate.sleep();
    }

    return 0;
}