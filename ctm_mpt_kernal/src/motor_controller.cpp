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



    ros::Duration(10).sleep();
    ros::Time tt = ros::Time::now();
    ros::spinOnce();
    std::cout << "spinOnce() ------------------------------- "
        << (ros::Time::now()-tt).toSec() << " sec" << std::endl;
    tt = ros::Time::now();
    m.trackXi();
    std::cout << "trackXi() ------------------------------- "
        << (ros::Time::now()-tt).toSec() << " sec" << std::endl;
    tt = ros::Time::now();
    m.readTorque();
    std::cout << "readTorque() ------------------------------- "
        << (ros::Time::now()-tt).toSec() << " sec" << std::endl;
    tt = ros::Time::now();
    m.print();
    std::cout << "print() ------------------------------- "
        << (ros::Time::now()-tt).toSec() << " sec" << std::endl;
    tt = ros::Time::now();






    ROS_INFO("The motor controller is ready.");
    ros::Rate loop_rate(CR);
    while (ros::ok())
    {
        ros::Time tt = ros::Time::now();
        ros::spinOnce();
        std::cout << "spinOnce() ------------------------------- "
            << (ros::Time::now()-tt).toSec() << " sec" << std::endl;
        tt = ros::Time::now();
        m.track();
        std::cout << "track() ------------------------------- "
            << (ros::Time::now()-tt).toSec() << " sec" << std::endl;
        tt = ros::Time::now();
        m.readTorque();
        std::cout << "readTorque() ------------------------------- "
            << (ros::Time::now()-tt).toSec() << " sec" << std::endl;
        if(!loop_rate.sleep())
        {
            ROS_WARN("The motor controller did not meet the desired rate.");
        }
    }

    return 0;
}
