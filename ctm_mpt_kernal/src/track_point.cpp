#include "ctm_mpt2.h"
#include <iostream>
#include <Eigen/Dense>

int main(int argc, char** argv)
{
    // ROS initialisation.
    // *************************************************
	ros::init(argc, argv, "track_point_node");
    ros::NodeHandle nh;
    // *************************************************
    


    // Tpye of manipulator.
    // 
    // cd /dev
    // ls -al ttyUSB*
    // sudo chmod a+rw ttyUSB*
    // *************************************************
    //        "sensor 1",     "sensor 2",     "motor",       "ros handle".
    CtmMpt2 m("/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB0",        &nh);
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
    // m.zero();
    m.init();
    // m.print();
    // float dist[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // m.move(1, -30.0);
    // m.relax(20.0);
    // m.init();
    // m.print();

    m.setTargetPose(Eigen::Matrix4f::Identity(4, 4));
    
    ros::Rate loop_rate(1000.0 / m.CTI);
    while (ros::ok())
    {
        m.trackPoint();

        m.readTorque();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
