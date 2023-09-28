#include "ctm_mpt2.h"
#include <iostream>
#include <Eigen/Dense>

int main(int argc, char** argv)
{
    // ROS initialisation.
    // *************************************************
	ros::init(argc, argv, "track_pose_node");
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

    // m.print();
    // m.zero();
    // m.init();
    // m.print();
    // float dist[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // m.move(1, -30.0);
    // m.relax(10);
    // m.init();
    // m.print();

    // m.setTargetXi(Eigen::Matrix<float, 6, 1> {0.8147, 0.9058, 0.1270, 0.9134, 0.6324, 0.0975});
    // m.trackXi();
    // m.init();

    // m.setTargetPose(Eigen::Matrix4f::Identity(4, 4));
    // Eigen::Matrix<float, 6, 3> xis {
    //     {0, 0, 0},
    //     {0, 2, 6},
    //     {0, 0, 8},
    //     {0, 0, 0},
    //     {0, 0, 0},
    //     {0, 0, 0},
    // };
    // m.setTargetPath(xis);
    // m.allocateTime();

    // Eigen::Matrix<float, 9, 1> td {0.15, 0.15, 0.15, 0.05, 0.05, 0.05, 0.0, 0.1, 0.1};
    // m.setTargetTorque(td);


    m.init();
    Eigen::Matrix<float, 6, 1> xi {-1.0719,-0.0582,0.9587,0.8018,0.2375,-0.0146};
    m.setTargetXi(xi);
    m.setTargetPose(xi);
    m.trackXi();
    
    ros::Rate loop_rate(1.0 / m.CTI);
    while (ros::ok())
    {
        m.trackPose();

    //     // m.trackTorque();
    //     // std::getchar();

        m.readTorque();

        // ros::spinOnce();

    //     loop_rate.sleep();
    }

    // m.init();
    // m.print();
    // m.print(6);
    // m.run(6, 1.31947);
    // m.snooze(10.0);
    // m.stop();
    // m.print();
    // m.print(6);

    return 0;
}
