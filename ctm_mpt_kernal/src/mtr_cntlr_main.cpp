#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>
#include <string>
#include <fstream>
#include "ctm_mpt2.h"

void proc(size_t cur, size_t tot);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mtr_cntlr");
    ros::NodeHandle nh;
    
    // cd /dev
    // ls -al ttyUSB*
    // sudo chmod a+rw ttyUSB*
    // 
    CtmMpt2 m;
    // CtmMpt2 m("/dev/ttyUSB0");
    // CtmMpt2 m("/dev/ttyUSB2", "/dev/ttyUSB1");
    //            "sensor 1",     "sensor 2",        "motor".
    // CtmMpt2 m("/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB0");
    const size_t N = 9;
    std::string fn("/home/ellipse/Desktop/catkin_ws/src/ctm_mpt/ctm_mpt_kernal/src/fd.txt");
    std::ifstream fin(fn);
    std::string str_dist;
    std::string str_tot;
    size_t tot = 0;
    size_t cur = 0;
    size_t i = 0;
    float dist[N] = {0.0};
    
    if (!fin.is_open())
    {
        ROS_ERROR_STREAM("Unable to open \"" << fn << "\".");

        return 0;
    }

    fin >> str_tot;
    tot = std::stoul(str_tot);

    ros::Rate loop_rate(1);
    m.init();
    m.print();
    std::getchar();
    while (cur < tot)
    {
        for (i = 0; i < N; i++)
        {
            fin >> str_dist;
            dist[i] = std::stof(str_dist);
        }
        // m.move(dist);
        cur += 1;
        proc(cur, tot);
        std::getchar(); // For debug only.
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
	static bool cover = false;
	size_t i = 0;
	size_t len = 20;
	float per = float(cur) / float(tot);
	size_t num = size_t(per * float(len));
	std::string info("[");
	char suffix[64];

	if (!cover)
	{
		cover = !cover;
	}
	else
	{
		printf("\033[1A\033[K");
	}
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
