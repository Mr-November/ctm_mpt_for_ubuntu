#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <fstream>
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <random>
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
#include "kinematics.h"
#define PL_Max 100
#define Delta 0.00001
#define R0 0.042

#define PI 3.14159265358

#define L_r 0.384

using namespace Eigen;
using namespace std;
const double L = 0.384;
VectorXd nonlinear_controller(VectorXd xi,MatrixXd Tfb,MatrixXd Te ,int JN,int PN,double Lp,double kp,double ki,double kd);
MatrixXd invA(MatrixXd A);
void proc(size_t cur, size_t tot);
double update_L(double L,double x,double y,bool clear);


enum Move_state_t{
	STOP,
	PLAN,
	CONTROL
};
int end_time;
int main (int argc, char** argv)
{
    //Q->theta->
    ros::init(argc, argv, "vrt_controller");
	double KP,KI,KD,MAX_ERR,MIN_ERR,STD;
	int JN,PN,step,MAX_LENGTH;
	double Rate = 0;
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
	ros::param::get("MIN_ERR", MIN_ERR);
	ros::param::get("loop_rate",Rate);
	ros::param::get("MAX_LENGTH",MAX_LENGTH);
	
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
	joint_state.name[0]="BJ1";
	joint_state.name[1]="J1S1";
	joint_state.name[2]="S1J2";
	joint_state.name[3]="J2S2";
	joint_state.name[4]="S2J3";
	joint_state.name[5]="J3S3";
	joint_state.name[6]="S3J4";
	joint_state.name[7]="J4S4";
	joint_state.name[8]="S4J5";
	joint_state.name[9]="J5S5";
	joint_state.name[10]="S5J6";
	joint_state.name[11]="J6S6";
	joint_state.name[12]="S6J7";
	joint_state.name[13]="J7S7";
	joint_state.name[14]="S7J8";
	joint_state.name[15]="J8S8";
	joint_state.name[16]="S8J9";
	joint_state.name[17]="J9S9";

    tf::TransformListener tf_listener;
    tf::StampedTransform transform;
    nav_msgs::Path path;
	ros::Rate loop_rate(Rate);
	ros::Duration(5).sleep(); // Wait for RViz to open.
    ros::Time t_cur;
	path.header.stamp = t_cur;
    path.header.frame_id = "Ref";
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("history_path", 1000);
	ros::Publisher pose_e_pub = nh.advertise<geometry_msgs::PoseStamped>("expected_Pose", 1000);
	ros::Publisher pose_fb_pub = nh.advertise<geometry_msgs::PoseStamped>("feedback_Pose", 1000);
	ros::Publisher sim_error_pub = nh.advertise<geometry_msgs::Twist>("Sim/error", 1000);
	ros::Publisher dist_pub = nh.advertise<std_msgs::Float64MultiArray>("Dist", 1000);
	
	geometry_msgs::Twist Error;
	VectorXd D(9);
    MatrixXd T_p(4,4),T_fb(4,4),T_e(4,4),T_next(4,4),T_start(4,4),T_pxi(4,4),A(6,6),B(6,6),F(6,6);
    Vector3d Origin_fb,Origin_e,Origin_next;
    Quaterniond Quater_fb,Quater_e,Quater_next;
    MatrixXd J_1,J_r;
    VectorXd dw(6),dxi(6),xi_r(6),xi(6),xi_n(6),dw_n(6),xi_p(6),xi_e(6),Td(6);//xi_r = xi+dxi error of w,xi,calculated form J_1 and error of T. y the controlled value , z the output value(same as y)
    VectorXd arc(6),jnt(18),Q(6),arc_ctr(6),dl(6),arc_ctr_des(6),err = MatrixXd::Zero(6,1),last_err= MatrixXd::Zero(6,1),sum_err = MatrixXd::Zero(6,1);
	double L_p,H_p;
    Vector3d kp,ph,LL(L,L,L);
	VectorXd goalxi(6),goaljnt(18),last_goal = MatrixXd::Zero(6,1);
	MatrixXd A_path = MatrixXd::Zero(6,4);
	default_random_engine e;
	e.seed(time(0));
	normal_distribution<double> u(0,STD);
	//adp val
    if (ros::ok())
	{
        //Draw open_loop control path
        std::string fn("/home/ellipse/Desktop/catkin_ws/src/ctm_mpt/ctm_mpt_kernal/src/fq.txt");
		std::string fq("/home/ellipse/Desktop/catkin_ws/src/ctm_mpt/ctm_mpt_kernal/src/star_qs_1");
		std::string fx("/home/ellipse/Desktop/catkin_ws/src/ctm_mpt/ctm_mpt_kernal/src/star_rs_1");
		std::string fxi("/home/ellipse/Desktop/catkin_ws/src/ctm_mpt/ctm_mpt_kernal/src/xi_rs");
		std::string fgoal("/mnt/hgfs/matlab_link/3d_nav_xi");
		
		std::ifstream fixi(fxi);
		std::ifstream fiq(fq);
		std::ifstream fix(fx);
		std::ifstream fin(fn);
		std::string str_ang;
		std::string str_tot;
		size_t tot = 0;
		size_t cur = 0;
		size_t i = 0;
		float fwd[N] = {0.0};
		
/* 		if (!fin.is_open()||!fiq.is_open()||!fix.is_open()||!fixi.is_open())
		{
			ROS_ERROR_STREAM("Unable to open \"" << fn << "\".");
			return 0;
		} */

        //simluation

		L_p = 0.128;
		H_p = 0.442;
 		

		// initialized for test

/*  		xi << 1.09516470919101,
			-1.13616303315901,
			-0.789389721731943,
			1.52657966054361,
			-0.186096767380782,
			-0.224583658450246; */
		xi << 0,0,0,0,0,0;
		jnt = xi2jnt(xi,3*L_p);
		arc = xi2arc(xi,L,L,L);
		
		Vector4d param;
		param << 0.038,0.042,0.102,0.026;
		
		D = jnt2dist(jnt,0,param);

		tot = step*PN;
		cur = 0;

		getchar();

		int cur_tick = 0;
		size_t move_state = STOP;
        while (1)
	    {
			cout<<move_state<<endl;
 			t_cur = ros::Time::now();
			for (i = 0;i<N;i++)
            {
                joint_state.position[i] = jnt(i);
			    joint_state.header.stamp = ros::Time::now();
            }
			param << 0.038,0.042,0.102/0.128*L_p,0.026/0.128*L_p;
			jnt_pub.publish(joint_state);
			D = jnt2dist(jnt,0,param);
			std_msgs::Float64MultiArray dist_msg;
			dist_msg.data.resize(9);
			for (int i = 0;i<9;i++)
       			dist_msg.data[i] =  D[i];
			dist_pub.publish(dist_msg); 
		
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
			cur ++;


 			Origin_fb << transform.getOrigin().x(),
                         transform.getOrigin().y(),
                         transform.getOrigin().z();
			cout<<Origin_fb<<endl;
			Origin_fb[0] +=u(e);
			Origin_fb[1] +=u(e);
			Origin_fb[2] +=u(e);
			cout<<Origin_fb<<endl;
            Quater_fb.x() = transform.getRotation().x();
            Quater_fb.y() = transform.getRotation().y();
            Quater_fb.z() = transform.getRotation().z();
            Quater_fb.w() = transform.getRotation().w();
			cout<<Quater_fb<<endl;
            Quater_fb.x() += u(e);
            Quater_fb.y() += u(e);
            Quater_fb.z() += u(e);
            Quater_fb.w() += u(e);	
			cout<<Quater_fb<<endl;
			
			//add noise
            T_fb = q2T(Quater_fb,Origin_fb);
			T_p = jnt2T(jnt,1);
			
			//Adp_update(L)
			if(T_p(0,3)!=0&&T_p(1,3)!=0)
			{
				if (cur < 3)
				{
					L_p = T_fb(0,3)/T_p(0,3)/2+T_fb(1,3)/T_p(1,3)/2;
					H_p = T_fb(2,3)-T_p(2,3)*L_p; 
				}
				else if(T_p(0,3)!=0&&T_p(1,3)!=0)
				{
					L_p = update_L(L_p,T_p(0,3),T_fb(0,3),false);
					L_p = update_L(L_p,T_p(1,3),T_fb(1,3),false);
					H_p = T_fb(2,3)-T_p(2,3)*L_p;
				}
			}
			else
			{
				L_p = 0.128;
				H_p = 0.442;
			}
			

			if (move_state == STOP)//read goal and plan
			{
				std::ifstream figoal(fgoal);
				if (!figoal.is_open())
				{
					ROS_ERROR_STREAM("Unable to open \"" << fgoal << "\".");
					getchar();
				}

				for (size_t i = 0;i<6;i++)
				{
					figoal >> str_ang;
					goalxi(i) = std::stof(str_ang);
				}
				
				if (goalxi != last_goal)
				{
					last_goal = goalxi;
					goaljnt = xi2jnt(goalxi,3*L_p);
					cout<<goaljnt<<endl;
					MatrixXd XX(6,2),TT(2,2);
					XX  = MatrixXd::Zero(6,2);
					TT << step*step*step,3*step*step,step*step,2*step;
					cout<<goalxi<<endl;
					cout<<TT<<endl;
					XX.block(0,0,6,1) = goalxi;
					XX.block(0,1,6,1) = -xi;
					A_path.block(0,3,6,1) = xi;
					A_path.block(0,2,6,1) = MatrixXd::Zero(6,1);
					A_path.block(0,0,6,2) = XX*invA(TT);
					move_state = PLAN;
					cur_tick = 0;

					figoal >> str_ang;
					Quater_e.w() = std::stof(str_ang);
					figoal >> str_ang;
					Quater_e.x() = std::stof(str_ang);
					figoal >> str_ang;
					Quater_e.y() = std::stof(str_ang);
					figoal >> str_ang;
					Quater_e.z() = std::stof(str_ang);
					for (size_t i = 0;i<3;i++)
					{
						figoal >> str_ang;
						Origin_e[i] = std::stof(str_ang)*0.384;
					}
					Origin_e[2]+=H_p;
					cout<<"OQ"<<endl;
					cout<<Origin_e<<endl;
					cout<<Quater_e<<endl;
					T_next = q2T(Quater_e,Origin_e); 
					cout<<"XX"<<endl;
					cout<<XX<<endl;

				}
				figoal.clear();
				cout<<A_path<<endl;
				last_goal = goalxi;
				figoal.close();
			}
			if (move_state == PLAN)//on the way to goal
			{
				static int p_tick = 0;
				Vector4d t_step(p_tick*p_tick*p_tick,p_tick*p_tick,p_tick,1);
				xi = A_path*t_step;
				jnt = xi2jnt(xi,3*L_p);
				p_tick++;
				if (p_tick >= step)
				{
					move_state = CONTROL;
					p_tick = 0;
				}
			}
			if (move_state == CONTROL)
			{
				static int keep_draw = 0;
				VectorXd dw(6);
				xi = nonlinear_controller(xi,T_fb,T_next ,JN,PN,3*L_p,KP,KI,KD);
				jnt = xi2jnt(xi,3*L_p);
				dw = upvee((inv(T_fb)*T_next).log());
				for (size_t i = 0;i<3;i++)
				{
					dw(i) = 0;
				}
				if (dw.transpose()*dw < MIN_ERR)
				{
					//break;
				}
				if (dw.transpose()*dw > MAX_ERR)
					break;
				keep_draw++;
				if (keep_draw>10*step)
					break;
			}
			if (move_state == 4)
			{
				static int keep_draw = 0;
				keep_draw++;
				if (keep_draw>100)
					break;
			}
			//judge_reach_target
			//
			cout<<"L"<<L_p<<" H"<<H_p<<endl;
			cout<<"TeTfbTp"<<endl;
			cout<<T_next<<endl<<T_fb<<endl<<T_p<<endl;

			if (move_state!=STOP)
			{
				VectorXd dw_n(6);
				dw_n = upvee((inv(T_fb)*T_next).log());
				Error.angular.x = dw_n[0];
				Error.angular.y = dw_n[1];
				Error.angular.z = dw_n[2];
				Error.linear.x = dw_n[3];
				Error.linear.y = dw_n[4];
				Error.linear.z = dw_n[5];
				sim_error_pub.publish(Error);
			}

			if (debug_mode)
				getchar();

			geometry_msgs::PoseStamped pose_stamped;
			pose_stamped.pose.position.x = T_next(0,3);;
			pose_stamped.pose.position.y = T_next(1,3);;
			pose_stamped.pose.position.z = T_next(2,3);;
			pose_stamped.pose.orientation.x = Quater_e.x();
			pose_stamped.pose.orientation.y = Quater_e.y();
			pose_stamped.pose.orientation.z = Quater_e.z();
			pose_stamped.pose.orientation.w = Quater_e.w();
			pose_e_pub.publish(pose_stamped);

			pose_stamped.pose.position.x = transform.getOrigin().x();
			pose_stamped.pose.position.y = transform.getOrigin().y();
			pose_stamped.pose.position.z = transform.getOrigin().z();
			pose_stamped.pose.orientation.x = transform.getRotation().x();
			pose_stamped.pose.orientation.y = transform.getRotation().y();
			pose_stamped.pose.orientation.z = transform.getRotation().z();
			pose_stamped.pose.orientation.w = transform.getRotation().w();
			path.header.stamp = ros::Time::now();
            path.poses.push_back(pose_stamped);
			path_pub.publish(path);
			pose_fb_pub.publish(pose_stamped);
			loop_rate.sleep();

			if (cur>MAX_LENGTH)
				break;
        }
		cout<<end_time<<endl;
		fiq.clear();
		fix.clear();
		fiq.close();
		fix.close();
		fin.close();
		fixi.close();
    }
}
MatrixXd invA(MatrixXd A)// pseudo inverse, safer
{
	MatrixXd invAA;
	try
	{
		invAA = (A.transpose()*A).inverse()*A.transpose();
	}
	catch (tf::TransformException ex)
	{
		ROS_WARN("Matrix is Not Invertible,return 0 Matrix instead");
		invAA = MatrixXd::Identity(A.cols(),A.rows());
	}
    return invAA;
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
	dw_n = upvee((inv(Tfb)*Te).log())/JN;

	for (size_t i = 0;i<JN;i++)
	{
		J_1 = invA(jacobian3cc(xi_1,Lp,Lp,Lp));
		dxi = J_1 * dw_n;
		xi_1 += dxi.transpose();
	}
	err = xi_1-xi;
	if (PN == 0)
	{
		err(0)=0;
		err(1)=0;
		err(2)=0;
	}
	
	sum_err += err;
	xi += kp*err+kd*(err-last_err)+ki*sum_err;
	last_err = err;

	return xi;
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
		cover = !cover;
	else
		printf("\033[1A\033[K");
	for (i = 0; i < num; i++)
		info += "#";
	for (i = 0; i < len-num; i++)
		info += "-";
	//sprintf(suffix, "]%6.2f%% (%d/%d)", per*100, cur, tot);
	ROS_INFO_STREAM(info << suffix);

	return;
}

//仿真			实际
