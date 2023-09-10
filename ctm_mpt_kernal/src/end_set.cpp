#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <fstream>
#include <iostream>
#include <cmath>
#include <ctime>
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
#include "std_msgs/Float64MultiArray.h"
#include <vector>
#include <sstream>
#include <random>
#include "kinematics.h"
#define PL_Max 100
#define Delta 0.00001
#define R0 0.042

#define PI 3.14159265358

#define L_r 0.384
const double L = 0.384;
using namespace Eigen;
using namespace std;
Quaterniond Quater_temp;
Vector3d Origin_temp;
void myCallback(const geometry_msgs::PoseStamped& tfmsg) 
{ 
    Origin_temp << tfmsg.pose.position.x,
				   tfmsg.pose.position.y,
				   tfmsg.pose.position.z;
	Quater_temp.x() = tfmsg.pose.orientation.x;
	Quater_temp.y() = tfmsg.pose.orientation.y;
	Quater_temp.z() = tfmsg.pose.orientation.z; 	
	Quater_temp.w() = tfmsg.pose.orientation.w;		 
  	ROS_INFO("received feedback"); 
	cout << Quater_temp.w() <<endl;
	cout << Quater_temp.x() <<endl;
	cout << Quater_temp.y() <<endl;
	cout << Quater_temp.z() <<endl;
}

int main (int argc, char** argv)
{
    //Q->theta->
    ros::init(argc, argv, "set_zero_controller");
    const size_t NL = 9;
    const size_t N = 18;
	ros::NodeHandle nh;
	ros::Publisher jnt_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1000);
	double STD;
	bool ifSTD= ros::param::get("STD", STD);
    tf::TransformListener tf_listener;
    tf::StampedTransform transform;
    nav_msgs::Path path;
	ros::Rate loop_rate(1000);
	ros::Duration(5).sleep(); // Wait for RViz to open.
    ros::Time t_cur;
	path.header.stamp = t_cur;
    path.header.frame_id = "Ref";
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("history_path", 1000);
	ros::Publisher pose_e_pub = nh.advertise<geometry_msgs::PoseStamped>("expected_Pose", 1000);
	ros::Publisher pose_fb_pub = nh.advertise<geometry_msgs::PoseStamped>("feedback_Pose", 1000);
	ros::Publisher sim_error_pub = nh.advertise<geometry_msgs::Twist>("Sim/error", 1000);
	ros::Publisher dist_pub = nh.advertise<std_msgs::Float64MultiArray>("Dist", 1000);
	ros::Subscriber tf_subscriber;
	tf_subscriber = nh.subscribe("vrpn_client_node/End/pose",100,&myCallback);	
	geometry_msgs::Twist Error;
	VectorXd D = MatrixXd::Zero(9,1);
    MatrixXd T_p(4,4),T_fb(4,4),T_e(4,4),T_next(4,4),T_start(4,4),T_pxi(4,4),F(6,6);
    Vector3d Origin_fb;
    Quaterniond Quater_fb;
    Vector3d kp,ph,LL(L,L,L);
	default_random_engine e;
	e.seed(time(0));
	normal_distribution<double> u(0,STD);
	//adp val
    if (ros::ok())
	{
        //Draw open_loop control path
		std::string fq("/home/ellipse/Desktop/catkin_ws/src/ctm_mpt/ctm_mpt_kernal/src/zero_qs_1");
		std::string fx("/home/ellipse/Desktop/catkin_ws/src/ctm_mpt/ctm_mpt_kernal/src/zero_rs_1");
		
		std::string str_ang;
		std::string str_tot;
		while(1)
		{
			std::ofstream fiq(fq,ios_base::out);
			std::ofstream fix(fx,ios_base::out);
			std_msgs::Float64MultiArray dist_msg;
			dist_msg.data.resize(9);
			for (int i = 0;i<9;i++)
       			dist_msg.data[i] = 0;
			dist_pub.publish(dist_msg); 
/* 			fiq.clear();
			fix.clear();
			fiq.seekp(0, ios::beg);
			fix.seekp(0, ios::beg); */
			fiq << Quater_temp.w() << endl;
			fiq << Quater_temp.x() << endl;
			fiq << Quater_temp.y() << endl;
			fiq << Quater_temp.z() << endl;
			fix << Origin_temp[0] << endl;
			fix << Origin_temp[1] << endl;
			fix << Origin_temp[2] << endl;

			cout << Quater_temp.w() <<endl;
			cout << Quater_temp.x() <<endl;
			cout << Quater_temp.y() <<endl;
			cout << Quater_temp.z() <<endl;
			//ROS_INFO("End Pose set");
			loop_rate.sleep();
			
			ros::spinOnce();
			getchar();
			fiq.clear();
			fix.clear();
			fiq.close();
			fix.close();
		}
		

    }
}

