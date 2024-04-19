#include "ctm_mpt2.h"
#include <iostream>
#include <Eigen/Dense>

int main(int argc, char** argv)
{
    // ROS initialisation.
    // *************************************************
	ros::init(argc, argv, "motor_controller_node");
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
    ros::Duration(20).sleep();

    // Go to the first point.
    ros::spinOnce();
    m.trackX();
    // std::cout << "Here." << std::endl;
    // std::getchar();
    m.trackPFB(true);
    m.trackPFB(true);
    m.trackPFB(true);
    
    ROS_INFO("The motor controller is ready.");
    ros::Rate loop_rate(CR);
    while (ros::ok())
    {
        // ros::Time tt = ros::Time::now();

        // m.trackVFBFW();
        // m.trackVFB();

        m.trackPFBFW(false);
        // m.trackPFB(false);

        // std::cout << "Time for tracking: "
        //     << (ros::Time::now()-tt).toSec() << " sec" << std::endl;
        
        if(!loop_rate.sleep())
        {
            ROS_WARN("The motor controller did not meet the desired rate.");
        }
    }
    ROS_INFO("Motor controller ended.");

    return 0;
}
