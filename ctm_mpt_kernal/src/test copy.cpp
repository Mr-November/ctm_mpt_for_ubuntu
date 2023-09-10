#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>
#include "ctm_mpt2.h"

void watchControlSignal(float* ctr, float* dst, float* src, float* err, size_t n);

int main(int argc, char** argv)
{
    // ROS initialisation.
    // *************************************************
	ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    ros::Publisher trq_pub = nh.advertise<std_msgs::Float32MultiArray>("trq", 100);
    
    const size_t N = 9;
    std_msgs::Float32MultiArray msg;
    std_msgs::MultiArrayDimension dim;
    dim.label = "tau";
    dim.size = N;
    dim.stride = N;
    msg.layout.dim.push_back(dim);
    msg.layout.data_offset = 0;
    msg.data = std::vector<float>(N, 0.0);
    // *************************************************
    


    // Tpye of manipulator.
    // 
    // cd /dev
    // ls -al ttyUSB*
    // sudo chmod a+rw ttyUSB*
    // *************************************************
    // CtmMpt2 m;
    CtmMpt2 m("/dev/ttyUSB0");
    // CtmMpt2 m("/dev/ttyUSB2", "/dev/ttyUSB1");
    //            "sensor 1",     "sensor 2",        "motor".
    // CtmMpt2 m("/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB0");
    // *************************************************
    


    // If use torque controller, uncomment this.
    // *************************************************
    CntlrPID c(3, 0, 0, 0.02, N);
    float trq[N] = {0.0};
    float trq_expt[N] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    float trq_diff[N] = {0.0};
    float pid_out[N] = {0.0};
    float dist[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // *************************************************



    // If use motor controller, uncomment this.
    // *************************************************
    // float trq[N] = {0.0};
    // float dist[N] = {0.0};
    // std::string fn("/home/ellipse/Desktop/catkin_ws/src/ctm_mpt/ctm_mpt_kernal/src/fd.txt");
    // std::ifstream fin(fn);
    // std::string str_dist;
    // std::string str_tot;
    // size_t tot = 0;
    // size_t cur = 0;

    // if (!fin.is_open())
    // {
    //     ROS_ERROR_STREAM("Unable to open \"" << fn << "\".");

    //     return 0;
    // }

    // fin >> str_tot;
    // tot = std::stoul(str_tot);
    // *************************************************

    // m.print();
    // m.init();
    // m.zero();
    // std::getchar();
    
    m.print();
    
    ros::Rate loop_rate(2);
    std::getchar();

    // while(1)
    // {
    //     float angle;
    //     uint8_t index;
    //     float dist[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    //     std::cout << "Motor ID: ";
    //     std::cin >> index;
    //     std::cout << "Angle: ";
    //     std::cin >> angle;
    //     dist[index-1] = angle;
    //     ROS_INFO("Motor %d rotates %.1f degree.\n", index, angle);

    //     // m.move(dist);
    // }

    while (ros::ok())
    {
        size_t i = 0;
        
        // Check if it is overloaded.
        // Necessary in each loop.
        if (m.read(trq))
        {
            return 1;
        }

        while (i < N)
        {
            msg.data.at(i) = trq[i]; // Necessary in each loop.
            // For single motor pid debugging, use
            // uint8_t id = 1;
            // trq_diff[id-1] = trq_expt[id-1] - trq[id-1];
            // For multiple, use
            trq_diff[i] = trq_expt[i] - trq[i];
            i++;
        }
        trq_pub.publish(msg); // Necessary in each loop.

        // c.pid(trq_diff, pid_out);
        // When tuning a PID, print the control signal first.
        // Check if the values are abnormal before continue.
        // watchControlSignal(pid_out, trq_expt, trq, trq_diff, N);
        // m.move(pid_out);

        // m.print();
        // std::getchar();
        // m.move(dist);

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