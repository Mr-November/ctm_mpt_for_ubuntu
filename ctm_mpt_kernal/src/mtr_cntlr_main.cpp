#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>
#include <string>
#include <fstream>
#include "ctm_mpt2.h"

void proc(size_t cur, size_t tot);
void watchControlSignal(float* out, size_t n);

int main(int argc, char** argv)
{
    // ROS initialisation.
    // *************************************************
	ros::init(argc, argv, "mtr_cntlr");
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
    // CtmMpt2 m("/dev/ttyUSB0");
    // CtmMpt2 m("/dev/ttyUSB2", "/dev/ttyUSB1");
    //            "sensor 1",     "sensor 2",        "motor".
    CtmMpt2 m("/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB0");
    // *************************************************
    


    // If use torque controller, uncomment this.
    // *************************************************
    // CntlrPID c(3, 0, 0, 0.02, N);
    // float trq[N] = {0.0};
    // float trq_expt[N] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    // float trq_diff[N] = {0.0};
    // float pid_out[N] = {0.0};
    // float dist[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // *************************************************



    // If use motor controller, uncomment this.
    // *************************************************
    float trq[N] = {0.0};
    float dist[N] = {0.0};
    std::string fn("/home/ellipse/Desktop/catkin_ws/src/ctm_mpt/ctm_mpt_kernal/src/fd.txt");
    std::ifstream fin(fn);
    std::string str_dist;
    std::string str_tot;
    size_t tot = 0;
    size_t cur = 0;

    if (!fin.is_open())
    {
        ROS_ERROR_STREAM("Unable to open \"" << fn << "\".");

        return 0;
    }

    fin >> str_tot;
    tot = std::stoul(str_tot);
    // *************************************************



    ros::Rate loop_rate(1);
    m.init();
    m.print();
    std::getchar();
    while (cur < tot)
    {
        size_t i = 0;
        
        // Check if it is overloaded.
        // Need in each loop.
        if (m.read(trq))
        {
            return 1;
        }

        for (i = 0; i < N; i++)
        {
            msg.data.at(i) = trq[i]; // Need in each loop.

            fin >> str_dist;
            dist[i] = std::stof(str_dist);
        }
        trq_pub.publish(msg); // Need in each loop.
        dist[6-1] = 0.0;

        m.print();
        watchControlSignal(dist, N);
        cur += 1;
        proc(cur, tot);
        std::getchar();
        m.move(dist);
        
        // loop_rate.sleep();
    }
    m.reset();
    m.print();
    fin.clear();
    fin.close();

    return 0;
}

void proc(size_t cur, size_t tot)
{
	size_t i = 0;
	size_t len = 20;
	float per = float(cur) / float(tot);
	size_t num = size_t(per * float(len));
	std::string info("[");
	char suffix[64];

	for (i = 0; i < num; i++)
	{
		info += "#";
	}
	for (i = 0; i < len-num; i++)
	{
		info += "-";
	}
	sprintf(suffix, "]%6.2f%% (%d/%d)", per*100, cur, tot);
	ROS_INFO_STREAM(info << suffix);

    return;
}

void watchControlSignal(float* out, size_t n)
{
    size_t i = 0;

    std::cout << std::endl
                << "id         1       2       3       4       5       6       7       8       9"
                << std::endl;

    std::cout << "out  ";
    for (i = 0; i < n; i++)
    {
        printf("%7.4f ", out[i]);
    }
    std::cout << std::endl;

    return;
}

