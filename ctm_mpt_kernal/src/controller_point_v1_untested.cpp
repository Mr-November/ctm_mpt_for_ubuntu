#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>
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
#include <vector>
#include <sstream>
#include <random>
#include "std_msgs/Float64MultiArray.h"
#include "kinematics.h"
#include "ctm_mpt2.h"

#define PL_Max 100
#define Delta 0.00001
#define R0 0.042
#define PI 3.14159265358
#define L 0.384
#define L_defalut 0.128
#define H_defalut 0.442

using namespace Eigen;
using namespace std;

const double L = 0.384;

VectorXd nonlinear_controller(VectorXd xi,MatrixXd Tfb,MatrixXd Te ,int JN,int PN,double Lp,double kp,double ki,double kd);
void motorinit(void);
void joint_init(*string joint_state_name);
double update_L(double L,double x,double y,bool clear);
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
  	ROS_INFO("received feedback :");
	cout <<  "x" << tfmsg.pose.position.x <<endl;
	cout <<  "y" << tfmsg.pose.position.y <<endl;
	cout <<  "z" << tfmsg.pose.position.z <<endl;
	
}

int main (int argc, char** argv)
{
    //Q->theta->
    ros::init(argc, argv, "vrt_controller");
	double KP,KI,KD,MAX_ERR,STD;
	int JN,PN,step;
	bool debug_mode;
	bool ifgetP = ros::param::get("P", KP);
	bool ifgetI = ros::param::get("I", KI);
	bool ifgetD = ros::param::get("D", KD);
	bool ifgetJN = ros::param::get("JN", JN);
	bool ifdebug = ros::param::get("DEBUG", debug_mode);
	bool ifgetPN = ros::param::get("PN", PN);
	bool ifgetstep = ros::param::get("step", step);
	bool ifSTD= ros::param::get("STD", STD);
    ros::param::get("MAX_ERR", MAX_ERR);
    if (!ifgetP)
		KP = 0;
	if (!ifgetI)
		KI = 0;
	if (!ifgetD)	
		KD = 0;
	if (!ifgetJN)	
		JN = 10;
	if (!ifgetJN)	
		PN = 10;
	if (!ifdebug)
		debug_mode=false;
    
    const size_t NL = 9;
    const size_t N = 18;

    ros::NodeHandle nh;
    ros::Publisher jnt_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1000);
	sensor_msgs::JointState joint_state;
    joint_state.position.resize(N);
	joint_state.name.resize(N);
    joint_init(joint_state.name);
    
    tf::TransformListener tf_listener;
    tf::StampedTransform transform;
    nav_msgs::Path path;
    std_msgs::Float64MultiArray dist_msg;

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

    VectorXd x0(6);
    Vector4d param;
    VectorXd Dist;//target motor angle
    float dist[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};//send to motor
    MatrixXd T_p(4,4),T_fb(4,4),T_e(4,4),T_next(4,4),T_start(4,4),T_pxi(4,4);
    Vector3d Origin_feedback,Origin_expect,Origin_next,Origin_zero;
    Quaterniond Quater_feedback,Quater_expect,Quater_next,Quater_zero;
	MatrixXd J_inverse,J_r;
    VectorXd dw(6),dxi(6),xi_r(6),xi(6),xi_n(6),dw_n(6),xi_p(6),xi_e(6),Td(6);//xi_r = xi+dxi error of w,xi,calculated form J_1 and error of T. y the controlled value , z the output value(same as y)
    VectorXd arc(6),jnt(18),Q(6),arc_ctr(6),dl(6),arc_ctr_des(6),err = MatrixXd::Zero(6,1),last_err= MatrixXd::Zero(6,1),sum_err = MatrixXd::Zero(6,1);
	double L_predict,H_predict;
    Vector3d kp,ph,L_Vector(L,L,L);

    if (ros::ok())
    {
         //init all file name
        std::string fn("/home/stalin/CTM/src/ctm_mpt_for_ubuntu-master/ctm_mpt_kernal/src/fq.txt");

		std::string fq("/home/stalin/CTM/src/ctm_mpt_for_ubuntu-master/ctm_mpt_kernal/src/star_qs_1");
		std::string fx("/home/stalin/CTM/src/ctm_mpt_for_ubuntu-master/ctm_mpt_kernal/src/star_rs_1");

		std::string fxi("/home/stalin/CTM/src/ctm_mpt_for_ubuntu-master/ctm_mpt_kernal/src/xi_rs");

		std::string f_zero_q("/home/stalin/CTM/src/ctm_mpt_for_ubuntu-master/ctm_mpt_kernal/src/zero_qs_1");
		std::string f_zero_x("/home/stalin/CTM/src/ctm_mpt_for_ubuntu-master/ctm_mpt_kernal/src/zero_rs_1");
	
		std::ifstream fixi(fxi);
		std::ifstream fiq(fq);
		std::ifstream fix(fx);
		std::ifstream fin(fn);
		std::ifstream fi_zero_x(f_zero_x);
		std::ifstream fi_zero_q(f_zero_q);
		std::string str_ang;
		std::string str_tot;
		size_t tot = 0;
		size_t cur = 0;
		size_t i = 0;
		float fwd[N] = {0.0};
        //read zero_point
		fi_zero_q >> str_ang;
		Quater_zero.w() = std::stof(str_ang);
		fi_zero_q >> str_ang;
		Quater_zero.x() = std::stof(str_ang);
		fi_zero_q >> str_ang;
		Quater_zero.y() = std::stof(str_ang);
		fi_zero_q >> str_ang;
		Quater_zero.z() = std::stof(str_ang);
		for (size_t i = 0;i<3;i++)
		{
			fi_zero_x >> str_ang;
			Origin_zero[i] = std::stof(str_ang);
		}
		Origin_zero[2] = 0;// the z of floor is zero

		//read expected_point
		fiq >> str_ang;
		Quater_e.w() = std::stof(str_ang);
		fiq >> str_ang;
		Quater_e.x() = std::stof(str_ang);
		fiq >> str_ang;
		Quater_e.y() = std::stof(str_ang);
		fiq >> str_ang;
		Quater_e.z() = std::stof(str_ang);
		for (size_t i = 0;i<3;i++)
		{
			fix >> str_ang;
			Origin_e[i] = std::stof(str_ang); //factor of expected point
		}
		Origin_e[2] += H_p;        

        CtmMpt2 m("/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB0");
		m.print();
		m.init ();
		ROS_INFO("Motors initalization done!\n");

        L_predict = L_defalut;
		H_predict = H_defalut;

		jnt = xi2jnt(xi,L,L,L);
		arc = xi2arc(xi,L,L,L);        

        D = jnt2dist(jnt,0,param);
        dist_msg.data.resize(9);

        for (int i = 0;i<9;i++)
		{
			dist[i] = D[i];
			dist_msg.data[i] = D[i];
		}
        dist_pub.publish(dist_msg); 
        m.move(dist);
        tot = step*PN;
		cur = 0;
		getchar();
        while (cur < tot)
	    {
            Origin_feedback = Origin_temp;
        	Quater_feedback = Quater_temp;
            cur ++;
            for (i = 0;i<N;i++)
            {
                joint_state.position[i] = jnt(i);
			    joint_state.header.stamp = ros::Time::now();
            }
			jnt_pub.publish(joint_state);
            param << 0.038,0.042,0.102/0.128*L_predict,0.026/0.128*L_predict;
			D = jnt2dist(jnt,0,param);

			for (int i = 0;i<9;i++)
			{
				dist[i] = D[i] - dist_msg.data[i];
       			dist_msg.data[i] = D[i];
			}
			dist_pub.publish(dist_msg); 
			m.move(dist);
            //Calculate predicted pose
            T_p = jnt2T(jnt,1);
            try
			{
				tf_listener.lookupTransform("/Ref", "/Tip", ros::Time(0), transform);
			}
			catch (tf::TransformException ex)
			{
				ROS_ERROR("%s",ex.what());
				ros::Duration(0.5).sleep();
				continue;
			}
            Origin_feedback << transform.getOrigin().x(),
                         transform.getOrigin().y(),
                         transform.getOrigin().z();
            Quater_feedback.x() = transform.getRotation().x();
            Quater_feedback.y() = transform.getRotation().y();
            Quater_feedback.z() = transform.getRotation().z();
            Quater_feedback.w() = transform.getRotation().w();

            T_fb = q2T(Quater_fb,Origin_fb);
            T_e = q2T(Quater_e,Origin_e); 
			dw_n = upvee((inv(T_fb)*T_e).log());
            //publish expect pose and feed back
            t_cur = ros::Time::now();
			geometry_msgs::PoseStamped pose_stamped;
			pose_stamped.pose.position.x = transform.getOrigin().x();
			pose_stamped.pose.position.y = transform.getOrigin().y();
			pose_stamped.pose.position.z = transform.getOrigin().z();
			pose_stamped.pose.orientation.x = transform.getRotation().x();
			pose_stamped.pose.orientation.y = transform.getRotation().y();
			pose_stamped.pose.orientation.z = transform.getRotation().z();
			pose_stamped.pose.orientation.w = transform.getRotation().w();
			path.header.stamp = t_cur;
			path.poses.push_back(pose_stamped);
			pose_fb_pub.publish(pose_stamped);
			path_pub.publish(path);

            geometry_msgs::PoseStamped pose_stamped1;
			pose_stamped1.pose.orientation.x = Quater_e.x();
			pose_stamped1.pose.orientation.y = Quater_e.y();
			pose_stamped1.pose.orientation.z = Quater_e.z();
			pose_stamped1.pose.orientation.w = Quater_e.w();	
			pose_stamped1.pose.position.x = T_e(0,3);
			pose_stamped1.pose.position.y = T_e(1,3);
			pose_stamped1.pose.position.z = T_e(2,3);
			pose_e_pub.publish(pose_stamped1);
			cout<<"L"<<L_p<<"H"<<H_p<<endl;
			cout<<"TpTfb"<<endl;
			cout<<T_e<<endl<<T_fb<<endl<<T_p<<endl;

 			dw_n = upvee((inv(T_fb)*T_e).log())/JN;
			xi = nonlinear_controller(xi,T_fb,T_e ,JN,PN,3*L_p,KP,KI,KD);
			jnt = xi2jnt(xi,L_p*3); 
			cout << "dw_n" << endl << dw_n.transpose()*JN << endl;
			if (debug_mode)
				getchar();
            loop_rate.sleep();
			ros::spinOnce();            
            
        }
        fiq.clear();
		fix.clear();
		fiq.close();
		fix.close();
		fin.close();
		fixi.close();
    }
}
void joint_init(*string joint_state_name)
{
	joint_state_name[1]="J1S1";
	joint_state_name[2]="S1J2";
	joint_state_name[3]="J2S2";
	joint_state_name[4]="S2J3";
	joint_state_name[5]="J3S3";
	joint_state_name[6]="S3J4";
	joint_state_name[7]="J4S4";
	joint_state_name[8]="S4J5";
	joint_state_name[9]="J5S5";
	joint_state_name[10]="S5J6";
	joint_state_name[11]="J6S6";
	joint_state_name[12]="S6J7";
	joint_state_name[13]="J7S7";
	joint_state_name[14]="S7J8";
	joint_state_name[15]="J8S8";
	joint_state_name[16]="S8J9";
	joint_state_name[17]="J9S9";
}
double update_L(double L,double x,double y,bool clear)
{
	static double K = 0,P=1e-3;
	L = L+K*(y-x*L);
	K = P*x/(1+x*P*x);
	P = (1-K*x)*P;
	if (clear)
	{
		K = 0;
		P = 1e-3;
	}
	return L;
}
VectorXd nonlinear_controller(VectorXd xi,MatrixXd Tfb,MatrixXd Te ,int JN,int PN,double Lp,double kp,double ki,double kd)
{
	static ros::NodeHandle nh;
	static ros::Publisher sim_error_pub = nh.advertise<geometry_msgs::Twist>("Sim/error", 1000);
	VectorXd xi_1 = xi,dw_n,dxi;
	
	VectorXd err = MatrixXd::Zero(6,1),last_err= MatrixXd::Zero(6,1);
	static VectorXd sum_err = MatrixXd::Zero(6,1);
	MatrixXd J_1(6,6);
	geometry_msgs::Twist Error;
	dw_n = upvee((inv(Tfb)*Te).log())/JN;

	for (size_t i = 0;i<JN;i++)
	{
		J_1 = inv(jacobian3cc(xi_1,Lp,Lp,Lp));
		dxi = J_1 * dw_n;
		xi_1 += dxi;
	}
	err = xi_1-xi;
	sum_err += err;
	xi += kp*err+kd*(err-last_err)+ki*sum_err;
	last_err = err;

	Error.angular.x = dw_n[0]*JN;
	Error.angular.y = dw_n[1]*JN;
	Error.angular.z = dw_n[2]*JN;

	Error.linear.x = dw_n[3]*JN;
	Error.linear.y = dw_n[4]*JN;
	Error.linear.z = dw_n[5]*JN;
			
	sim_error_pub.publish(Error);
	return xi;
}