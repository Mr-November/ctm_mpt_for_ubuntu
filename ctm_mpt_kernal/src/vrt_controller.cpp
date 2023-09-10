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

#include "kinematics.h"
#define PL_Max 100
#define Delta 0.00001
#define R0 0.042

#define PI 3.14159265358

#define L_r 0.384

using namespace Eigen;
using namespace std;
const double L = 0.384;

bool Ctrb_Judge(MatrixXd A,MatrixXd B);
MatrixXd Calculate_F(MatrixXd A,MatrixXd B1,MatrixXd B2,MatrixXd C1,MatrixXd D11,MatrixXd D12);
MatrixXd invA(MatrixXd A);
bool Stop_Flag(MatrixXd P,MatrixXd last_P);
void proc(size_t cur, size_t tot);
double update_L(double L,double x,double y,bool clear);



int main (int argc, char** argv)
{
    //Q->theta->
    ros::init(argc, argv, "vrt_controller");
	double KP,KI,KD;
	int JN,PN,step;
	bool debug_mode;
	bool ifgetP = ros::param::get("P", KP);
	bool ifgetI = ros::param::get("I", KI);
	bool ifgetD = ros::param::get("D", KD);
	bool ifgetJN = ros::param::get("JN", JN);
	bool ifdebug = ros::param::get("DEBUG", debug_mode);
	bool ifgetPN = ros::param::get("PN", PN);
	bool ifgetstep = ros::param::get("step", step);
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
	ros::Rate loop_rate(100);
	ros::Duration(5).sleep(); // Wait for RViz to open.
    ros::Time t_cur;
	path.header.stamp = t_cur;
    path.header.frame_id = "Ref";
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("history_path", 1000);
	ros::Publisher sim_error_pub = nh.advertise<geometry_msgs::Twist>("Sim/error", 1000);
	geometry_msgs::Twist Error;
	Matrix3d D;
    MatrixXd T_p(4,4),T_fb(4,4),T_e(4,4),T_next(4,4),T_start(4,4),T_pxi(4,4),A(6,6),B(6,6),F(6,6);
	MatrixXd C1 = MatrixXd::Identity(6,6);
	MatrixXd B1 = MatrixXd::Ones(6,1);
	MatrixXd D12 = 0.0001* MatrixXd::Identity(6,6);
	MatrixXd D11 = MatrixXd::Zero(6,1);
    Vector3d Origin_fb,Origin_e,Origin_next;
    Quaterniond Quater_fb,Quater_e,Quater_next;
    MatrixXd J_1,J_r;
    VectorXd dw(6),dxi(6),xi_r(6),xi(6),xi_n(6),dw_n(6),xi_p(6),xi_e(6),Td(6);//xi_r = xi+dxi error of w,xi,calculated form J_1 and error of T. y the controlled value , z the output value(same as y)
    VectorXd arc(6),jnt(18),Q(6),arc_ctr(6),dl(6),arc_ctr_des(6),err = MatrixXd::Zero(6,1),last_err= MatrixXd::Zero(6,1),sum_err = MatrixXd::Zero(6,1);
	double L_p,H_p;
    Vector3d kp,ph,LL(L,L,L);

	//adp val
    if (ros::ok())
	{
        //Draw open_loop control path
        std::string fn("/home/stalin/CTM/src/ctm_mpt_for_ubuntu-master/ctm_mpt_kernal/src/fq.txt");
		std::string fq("/home/stalin/CTM/src/ctm_mpt_for_ubuntu-master/ctm_mpt_kernal/src/star_qs");
		std::string fx("/home/stalin/CTM/src/ctm_mpt_for_ubuntu-master/ctm_mpt_kernal/src/star_rs");
		std::string fxi("/home/stalin/CTM/src/ctm_mpt_for_ubuntu-master/ctm_mpt_kernal/src/xi_rs");
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
        fin >> str_tot;
		tot = std::stoul(str_tot);
		#ifdef DRAW_PATH
        while (cur < tot)
		{
			for (i = 0; i < N; i++)
			{
				fin >> str_ang;
				fwd[i] = std::stof(str_ang);
				joint_state.position[i] = fwd[i];
				jnt(i) = fwd[i];
			}
			T_p = jnt2T(jnt,1);
			
			joint_state.header.stamp = ros::Time::now();
			jnt_pub.publish(joint_state);
			cur += 1;
			proc(cur, tot);
			// std::getchar(); // For debug only.
			loop_rate.sleep();
			// Publish history path.
			tf::StampedTransform transform;
			try
			{
				tf_listener.lookupTransform("/Ref", "/Tip", ros::Time(0), transform);
			}
			catch (tf::TransformException ex)
			{
				ROS_ERROR("%s",ex.what());
				ros::Duration(0.5).sleep();
				cur-=1;
				continue;
			}
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
			path_pub.publish(path);

			Origin_fb << transform.getOrigin().x(),
                        transform.getOrigin().y(),
                        transform.getOrigin().z();
			
            Quater_fb.x() = transform.getRotation().x(),
            Quater_fb.y() = transform.getRotation().y(),
            Quater_fb.z() = transform.getRotation().z(),
            Quater_fb.w() = transform.getRotation().w();

            T_fb = q2T(Quater_fb,Origin_fb);

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
				Origin_e[i] = std::stof(str_ang)*0.384;
			}
			for (size_t i = 0;i<6;i++)
			{
				fixi >> str_ang;
				xi_e[i] = std::stof(str_ang);
			}
			Origin_e[2] += 0.442;
			T_e = q2T(Quater_e,Origin_e);
			dw_n = upvee((inv(T_fb)*T_e).log())/JN;
					Error.angular.x = dw_n[0]*JN;
					Error.angular.y = dw_n[1]*JN;
					Error.angular.z = dw_n[2]*JN;

					Error.linear.x = dw_n[3]*JN;
					Error.linear.y = dw_n[4]*JN;
					Error.linear.z = dw_n[5]*JN;
			
		sim_error_pub.publish(Error);
		}
		fin.clear();
		fin.close();
		fiq.clear();
		fix.clear();
		#endif
        //simluation

		L_p = 0.128;
		H_p = 0.442;
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
			Origin_e[i] = std::stof(str_ang)*0.384;
		}

		
		Origin_e[2] += H_p;
		T_start = q2T(Quater_e,Origin_e); 
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
			Origin_e[i] = std::stof(str_ang)*0.384;
		}
		Origin_e[2] += H_p;
		T_next = q2T(Quater_e,Origin_e); 
		Td = upvee((inv(T_start)*T_next).log())/PN;

		// initialized for test

 		xi << 1.09516470919101,
			-1.13616303315901,
			-0.789389721731943,
			1.52657966054361,
			-0.186096767380782,
			-0.224583658450246;
/* 		xi <<-0.175425275578762,
			-0.233611919529856,
			1.60552765158577,
			-0.455238823721488,
			-1.31427634215090,
			0.858315440606158; */
		jnt = xi2jnt(xi,L,L,L);
		arc = xi2arc(xi,L,L,L);
		
		Vector4d param;
		param << 0.038,0.042,0.102,0.026;
		
		//D = jnt2dist(jnt,0,param);

		tot = step*PN;
		cur = 0;
		int cur_tick = 0;
        while (cur < tot)
	    {
            for (i = 0;i<N;i++)
            {
                joint_state.position[i] = jnt(i);
			    joint_state.header.stamp = ros::Time::now();
            }
			jnt_pub.publish(joint_state);
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
			cur ++;
			cur_tick++;
            Origin_fb << transform.getOrigin().x(),
                        transform.getOrigin().y(),
                        transform.getOrigin().z();
			
            Quater_fb.x() = transform.getRotation().x(),
            Quater_fb.y() = transform.getRotation().y(),
            Quater_fb.z() = transform.getRotation().z(),
            Quater_fb.w() = transform.getRotation().w();
			//add noise
            T_fb = q2T(Quater_fb,Origin_fb);
			T_e = T_start*(uphat(cur_tick*Td).exp());
			//read expected T
			if (cur_tick == PN-1)
			{
				T_e = T_next;
				T_start = T_next; 
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
					Origin_e[i] = std::stof(str_ang)*0.384;
				}
				Origin_e[2] += H_p;
				T_next = q2T(Quater_e,Origin_e); 
				Td = upvee((inv(T_start)*T_next).log())/PN;
				cur_tick = 0;
			}
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
			path_pub.publish(path);
			//Origin_e[2] -= H_p;
			//Adp_update(L)
			if (cur < 3)
			{
				L_p = T_fb(0,3)/T_p(0,3)/2+T_fb(1,3)/T_p(1,3)/2;
				H_p = T_fb(2,3)-T_p(2,3)*L_p; 
			}
			else
			{
				L_p = update_L(L_p,T_p(0,3),T_fb(0,3),false);
				L_p = update_L(L_p,T_p(1,3),T_fb(1,3),false);
				H_p = T_fb(2,3)-T_p(2,3)*L_p;
			}
			
			cout<<"L"<<L_p<<" H"<<H_p<<endl;


			cout<<"TpTfb"<<endl;
			cout<<T_e<<endl<<T_fb<<endl<<T_p<<endl;

            /* dw = upvee((T_fb*inv(T_e)).log());
            dxi = J_1*dw;//6*1
            xi_r = xi + dxi;
			cout<<"x"<<xi_r<<endl<<xi<<endl; */

			//J_r = invA(jacobian3cc(L_p,xi_r));

 			dw_n = upvee((inv(T_fb)*T_e).log())/JN;
			VectorXd xi_1 = xi;
			
			for (size_t i = 0;i<JN;i++)
			{
				J_1 = invA(jacobian3cc(xi_1,3*L_p,3*L_p,3*L_p));
				dxi = J_1 * dw_n;
				xi_n = xi_1 + dxi;
				xi_1 = xi_n;
			}
			err = xi_n-xi;
			sum_err+=err;
			xi += KP*err+KD*(err-last_err)+KI*sum_err;
			last_err = err;
			jnt = xi2jnt(xi,L_p*3);
			Error.angular.x = dw_n[0]*JN;
			Error.angular.y = dw_n[1]*JN;
			Error.angular.z = dw_n[2]*JN;

			Error.linear.x = dw_n[3]*JN;
			Error.linear.y = dw_n[4]*JN;
			Error.linear.z = dw_n[5]*JN;
			
			sim_error_pub.publish(Error);
			//D = jnt2dist(xi2jnt(xi_n,L),0,param);
			cout<<"dw_n"<<endl<<dw_n.transpose()*JN<<endl;
			if (debug_mode)
				getchar();
			/* B = MatrixXd::Zero(6,6);
				A = MatrixXd::Identity(6,6);
				for (size_t i = 0;i<3;i++)
				{
					arc_ctr(2*i) = xi_r(2*i)*xi_r(2*i)+xi_r(2*i+1)*xi_r(2*i+1);
					arc_ctr(2*i+1) = atan2(2*i,xi_r(2*i+1))+2*PI/9*i;
					arc_ctr_des(2*i) = xi(2*i)*xi(2*i)+xi(2*i+1)*xi(2*i+1);
					arc_ctr_des(2*i+1) = atan2(2*i,xi(2*i+1))+2*PI/9*i;

					double l = LL(i),l1 = D(i,0),l2 = D(i,1),l3 = D(i,2);
					A.block(2*i,2*i,2,2) = MatrixXd::Identity(2,2);
					B.block(2*i,2*i,2,2) << -(3*l-2*l1-l2)/R0/R0/4.5,-(3*l-2*l2-l1)/R0/R0/4.5, 2*sqrt(3)*(l-l2)/(l1+2*l2-3*l), -2*sqrt(3)*(l-l1)/(l1+2*l2-3*l);
					
				} */
				//F = Calculate_F(A,B1,B,C1,D11,D12);
				//dl = F*(arc_ctr-arc_ctr_des);
				//using adp control
				/* for (size_t i = 0; i<3;i++)
				{
					D(i,0) += dl(2*i);
					D(i,1) += dl(2*i+1);
					D(i,2) = 3*LL(i) - D(i,0) - D(i,1);
				} */
			//cal control result
			//arc = dist2arc(L_r,R0,D);
			//jnt = xi2jnt(xi,L_r);
            loop_rate.sleep();
        }
		fiq.clear();
		fix.clear();
		fiq.close();
		fix.close();
		fin.close();
		fixi.close();
    }
}
MatrixXd Calculate_F(MatrixXd A,MatrixXd B1,MatrixXd B2,MatrixXd C1,MatrixXd D11,MatrixXd D12)
{// may have some problem but dont affect temp control result
    if (!Ctrb_Judge(A,B2))
        return MatrixXd::Zero(A.rows(),A.cols());
    MatrixXd P = 0.00*MatrixXd::Identity(B2.cols(),B2.rows());
    MatrixXd last_P = P;
	for (size_t pk = 0;pk<PL_Max;pk++)
    {
		cout<<pk<<endl;
		cout<<P<<endl;
		getchar();
        MatrixXd BB1 = B1.transpose()*P*A+D11.transpose()*C1;
        MatrixXd BB2 = B2.transpose()*P*A+D12.transpose()*C1;
        MatrixXd BB3 = B1.transpose()*P*B2+D11.transpose()*D12;
        MatrixXd theta1 = -B1.transpose()*P*B1-D11.transpose()*D11;
        theta1 += MatrixXd::Identity(theta1.cols(),theta1.rows());
        MatrixXd theta2 = D12.transpose()*D12 + B2.transpose()*P*B2 + BB3.transpose()*invA(theta1)*BB3;
        MatrixXd theta3 = BB2 + BB3.transpose()*invA(theta1)*BB1;
        P = A.transpose()*P*A + C1.transpose()*C1 + BB1.transpose()*invA(theta1)*BB1 - theta3.transpose()*invA(theta2)*theta3 + Delta*MatrixXd::Identity(P.rows(),P.cols());
        
		if (Stop_Flag(P,last_P)|| pk == PL_Max-1)
            return (-invA(theta2)*theta3);
        last_P = P;
    }
    return MatrixXd::Zero(B2.rows(),B2.cols());
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
bool Ctrb_Judge(MatrixXd A,MatrixXd B)
{
    MatrixXd CA(B.rows(),A.rows()*B.cols());
    MatrixXd TempA = MatrixXd::Identity(A.rows(),A.cols());
    for (size_t i = 0;i<A.rows();i++)
    {
        CA.block(0,i*A.rows(),B.rows(),B.cols()) = TempA*B;
        TempA *= B;
    }
    JacobiSVD<Eigen::MatrixXd> svd(CA);
	
	//if (svd.rank() == A.rows())
    	return true;
	//else 
	//	return false;
}

bool Stop_Flag(MatrixXd P,MatrixXd last_P)
{
    MatrixXd Sub = P-last_P;
    double max_ele = 0;
    for (size_t i = 0;i<Sub.rows();i++)
		for (size_t j = 0;j<Sub.cols();j++)
            if (max_ele < abs(Sub(i,j)))
                max_ele = abs(Sub(i,j));
	if (max_ele>10e-10)
        return false;
    else
        return true;
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
//链接: https://pan.baidu.com/s/14S3LnADV0oGc0tb-7c9Z7A 提取码: 24ra