#include "ctm_mpt2.h"
#include <iostream>
#include <Eigen/Dense>

int main(int argc, char** argv)
{
    // ROS initialisation.
    // *************************************************
	ros::init(argc, argv, "track_path_node");
    ros::NodeHandle nh;
    // *************************************************
    


    // Tpye of manipulator.
    // 
    // cd /dev
    // ls -al ttyUSB*
    // sudo chmod a+rw ttyUSB*
    // *************************************************
    // CtmMpt2 m;
    //        "sensor 1",     "sensor 2",     "motor",       "ros handle".
    CtmMpt2 m("/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB0",        &nh);
    // *************************************************



    m.init();
    Eigen::Matrix<float, 30, 6> xis
    {
        
    };
    m.setTargetPath(xis.transpose());

    // ros::Rate loop_rate(1.0 / m.CTI);
    while (ros::ok())
    {
        // m.trackPath();
        m.trackPath2();
    }

    return 0;
}
