#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <fstream>
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>

void goalsetCallback(const geometry_msgs::PoseStamped & msg)
{
	
	std::ofstream fout("/mnt/hgfs/matlab_link/3d_nav_goal");
	while(!fout.is_open())
	{
		ROS_INFO_STREAM("Unable to open 3d_nav_goal now.");
		sleep(0.1);
	}
	ROS_INFO_STREAM("Loading new targetpoint.");
	fout << msg.pose.orientation.w << " " ;
	fout << msg.pose.orientation.x << " " ;
	fout << msg.pose.orientation.y << " " ;
	fout << msg.pose.orientation.z << " " ;
	fout << msg.pose.position.x << " ";
	fout << msg.pose.position.y << " " ;
	fout << msg.pose.position.z << " " ;
	fout.close();
	sleep(0.01);
}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "matlab_link_node");
	ros::NodeHandle nh;
	
	ros::Subscriber sub = nh.subscribe("/3d_nav_goal", 100, goalsetCallback);
	ros::Rate looprate(1000);   
	while (ros::ok())
	{
		ros::spin();                
		looprate.sleep();
	}
	return 0;
}
		