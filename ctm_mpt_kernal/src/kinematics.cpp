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
#include <Eigen/Eigenvalues>
#include "kinematics.h"
using namespace Eigen;
using namespace std;
#define PI 3.14159265358

MatrixXd R2T(Matrix3d R,double L);

double mod(double x1,double x2);
MatrixXd diag(MatrixXd source);
MatrixXd Matrix_Sqrt(MatrixXd source);
Matrix3d z2rot(VectorXd z,VectorXd& q,size_t index);

MatrixXd uphat(VectorXd V)
{
	if (V.size() == 3)
	{
		MatrixXd M(3,3);
		M <<    0,  	-V(2),   V(1),
          		V(2),      0,  	-V(0),
         		-V(1),   V(0),      0;
		 return M;
	}
	else if(V.size() == 6)
	{
		MatrixXd M(4,4);
		M <<    0,     -V(2),   V(1),   V(3),
          		V(2),   0,     -V(0),   V(4),
         	   -V(1),   V(0),   0,   	V(5),
             	0,      0,     	0,      0;
		return M;
	}
	ROS_ERROR("Vector Size Not Right");
	return MatrixXd::Identity(4,4);
}
VectorXd upvee(MatrixXd M)
{
	
	if (M.cols()==3)
	{
		Vector3d V;
		V << -M(1,2),M(0,2),-M(0,1);
		return V;
	}
		
	else if (M.cols() == 4)
	{
		VectorXd V(6);
		V << -M(1,2),M(0,2),-M(0,1),M(0,3),M(1,3),M(2,3);
		return V;
	}
	return MatrixXd::Zero(3,1);
}
MatrixXd forward_kinematics(Vector3d kp, Vector3d ph,double L1,double L2,double L3)
{
	Vector3d L;
	L<<L1,L2,L3;
	MatrixXd T = MatrixXd::Identity(4,4);
	MatrixXd Ti(4,4);
	for (size_t i = 0;i<3;i++)
	{

		VectorXd X0(6);
		if (abs(kp(i)*L(i)) > 0.001)
			X0 << -kp(i)*L(i)*sin(ph(i)),kp(i)*L(i)*cos(ph(i)),0,0,0,L(i);
		else
			X0 << 0,0,0,0,0,L(i);
		Ti = uphat(X0);
		T = T*Ti.exp();
	}
	return T;
}
VectorXd arc2xi(VectorXd arc,double L1,double L2,double L3)
{
	VectorXd Xi(6);
	Xi << -L1*arc(0)*sin(arc(1)),
       L1*arc(0)*cos(arc(1)),
      -L2*arc(2)*sin(arc(3)),
       L2*arc(2)*cos(arc(3)),
      -L3*arc(4)*sin(arc(5)),
       L3*arc(4)*cos(arc(5));
	return Xi;
}
VectorXd xi2jnt(VectorXd xi,double L1,double L2,double L3)
{
	VectorXd q(18);
	Vector3d L(L1,L2,L3);
	MatrixXd Ttsf(4,4);
	MatrixXd Tstd = MatrixXd::Identity(4,4);
	MatrixXd Tcur = MatrixXd::Identity(4,4);
	MatrixXd Rtsf = MatrixXd::Identity(3,3);
	
	for (size_t i = 0;i<9;i++)
	{
		if ( i%3 == 0 )
		{
			VectorXd xtemp(6);
			xtemp << xi[2*i/3]/3,xi[2*i/3+1]/3, 0, 0, 0, L(i/3);
			Ttsf = uphat(xtemp).exp();
		}
		Tstd *= Ttsf;
		if (i!=0)
			Tcur *= R2T(Rtsf,L(i/3));
		Rtsf = z2rot(Tcur.inverse()*Tstd.block(0,3,4,1),q,i);
	}
	for (size_t i = 0;i<3;i++)
	{
		if (abs(xi(2*i)) <= 1e-7&&abs(xi(2*i+1))<= 1e-7)
		{
			for (size_t j = 0;j<6 ;j++)
			{
				q(6*i+j) = 0;
			}
		}
	}
	
	return q;
}
VectorXd xi2jnt(VectorXd xi,double L)
{
	return xi2jnt(xi,L,L,L);
}
VectorXd jnt2dist(VectorXd q,size_t noise_type, Vector4d param)
{
	Matrix3d R[9];
	MatrixXd D = MatrixXd::Zero(3,9);
	VectorXd dist(9);
	for (size_t i = 0;i<9;i++)
	{
		if (i%2==1)
		{
			AngleAxisd Anglex(AngleAxisd(q(2*i),Vector3d::UnitX()));
			AngleAxisd Angley(AngleAxisd(q(2*i+1),Vector3d::UnitY()));
			R[i] = Anglex.matrix()*Angley.matrix();
		}
		else
		{
			AngleAxisd Anglex(AngleAxisd(q(2*i+1),Vector3d::UnitX()));
			AngleAxisd Angley(AngleAxisd(q(2*i),Vector3d::UnitY()));
			R[i] = Angley.matrix()*Anglex.matrix();
		}
	}
	MatrixXd h(3,9),hbar(3,9);
	for (size_t j = 0;j<9;j++)
	{
		h(0,j) = param[1]*cos(PI*j/4.5);
		h(1,j) = param[1]*sin(PI*j/4.5);
		hbar(0,j) = h(0,j);
		hbar(1,j) = h(1,j);
		h(2,j) = param[3]/2;
		hbar(2,j) = -param[3]/2;
	}
	for (size_t i = 0;i<3;i++)
	{
		for (size_t j = 0;j<3;j++)
		{
			D.block(i,0,1,9) += Matrix_Sqrt(diag((R[3*i+j]*h-hbar).transpose()*(R[3*i+j]*h-hbar)));
		}
		D.block(i,0,1,9) -= 3*(h.block(2,0,1,9)-hbar.block(2,0,1,9));
	}
	
	D.block(1,0,1,9) += D.block(0,0,1,9);
	D.block(2,0,1,9) += D.block(1,0,1,9);
	cout<<D<<endl;
	dist << D(0,1),D(0,4),D(0,7),D(2,2),D(2,5),D(2,8),D(1,3),D(1,6),D(1,0);
	return dist;
}
VectorXd dist2arc(double L,double r0,Matrix3d dist)
{
	VectorXd arc(6);
	Vector3d L1,L2,L3;
	L1 = dist.block(0,0,1,3).transpose();
	L2 = dist.block(1,0,1,3).transpose(); 
	L3 = dist.block(2,0,1,3).transpose();
	for (size_t i = 0;i < 3;i++)
	{
		arc(i) = atan2((L3(i)+L2(i)-2*L1(i)),sqrt(3)*(L2(i)-L3(i)))+2*PI/9*i;
		if (arc(i) > PI)
			arc(i)-=2*PI;
		arc(i+1) = (2*sqrt(L1(i)*L1(i)+L2(i)*L2(i)+L3(i)*L3(i)-L1(i)*L2(i)-L1(i)*L3(i)-L2(i)*L3(i))/r0/(L1(i)+L2(i)+L3(i)));
	}
	return arc;
}
Matrix3d z2rot(VectorXd z,VectorXd& q,size_t index)
{
	Matrix3d R;
	z.normalize();
	Matrix3d Rotx,Roty;
	double ty,tx;
	if (index%2 == 1)//xy
	{
		ty = asin (z(0));
		tx = atan2(-z(1),z(2));		
		Rotx << 1,0,0,
		0,cos(tx),-sin(tx),
		0,sin(tx),cos(tx);
		Roty << cos(ty),0,sin(ty),
		0,1,0,
		-sin(ty),0,cos(ty);
		R = Rotx*Roty;
		q(2*index) = tx;
		q(2*index+1) = ty;
	}
	else //yx
	{
		tx = asin (-z(1));
		ty = atan2(z(0),z(2));
		Rotx << 1,0,0,0,cos(tx),-sin(tx),0,sin(tx),cos(tx);
		Roty << cos(ty),0,sin(ty),0,1,0,-sin(ty),0,cos(ty);
		R = Roty*Rotx;
		q(2*index) = ty;
		q(2*index+1) = tx;
	}
	
	return R;
}
//MatrixXd q2rot(Matrix3d q)
MatrixXd R2T(Matrix3d R,double L)
{
	MatrixXd T = MatrixXd::Identity(4,4);
	VectorXd LL(6);
	LL << 0,0,0,0,0,L; 
	T.block(0,0,3,3) = R;
	T*= uphat(LL).exp();
	return T;
}
MatrixXd diag(MatrixXd source)
{
	MatrixXd Diag = MatrixXd::Zero(1,source.cols());
	if (source.cols() == source.rows())
		for (size_t j = 0; j < source.cols(); j++)
			Diag(j) = source(j,j);
	return Diag;
}
MatrixXd Matrix_Sqrt(MatrixXd source)
{
	for (size_t i = 0;i<source.rows();i++)
		for (size_t j = 0;j<source.cols();j++)
			source(i,j) = sqrt(source(i,j));

	return source;
}
MatrixXd jacobian3cc(VectorXd xi,double L1,double L2,double L3)
{
	MatrixXd J3 = jaco_c12(xi[4],xi[5], L3);
	VectorXd temp_uphat(6);
	temp_uphat << xi[4],xi[5], 0, 0, 0, L3;
	
	MatrixXd T3 = uphat(temp_uphat).exp();
	MatrixXd invT3 = inv(T3);
	MatrixXd J2 = jaco_c12(xi[2],xi[3], L2); 
	VectorXd J2_c1 = upvee(invT3 * uphat(J2.block(0,0,6,1)) * T3);
	VectorXd J2_c2 = upvee(invT3 * uphat(J2.block(0,1,6,1)) * T3);
	J2.block(0,0,6,1) = J2_c1;
	J2.block(0,1,6,1) = J2_c2;

	temp_uphat << xi[2],xi[3], 0, 0, 0, L2;
	MatrixXd T2 = uphat(temp_uphat).exp();
	MatrixXd invT2 = inv(T2);
	MatrixXd J1 = jaco_c12(xi[0],xi[1], L1); 
	VectorXd J1_c1 = upvee(invT3 * invT2 * uphat(J1.block(0,0,6,1)) * T2 * T3);
	VectorXd J1_c2 = upvee(invT3 * invT2 * uphat(J1.block(0,1,6,1)) * T2 * T3);
	J1.block(0,0,6,1) = J1_c1;
	J1.block(0,1,6,1) = J1_c2;
	
	MatrixXd J(6,6);
	J.block(0,0,6,2) = J1;
	J.block(0,2,6,2) = J2;
	J.block(0,4,6,2) = J3;
	return J;

}
MatrixXd jacobian3cc(VectorXd xi,double L)
{
	return jacobian3cc(xi,L,L,L);
}
MatrixXd jaco_c12(double w1,double w2, double L)
{
	Vector3d w;
	w << w1,w2,0;
	MatrixXd Jc = MatrixXd::Zero(6,3);
	double n = w.norm();
	if (n == 0)
	{
		Jc.block(0,0,3,3) = MatrixXd::Identity(3,3);
		Jc.block(3,0,3,3) << 0,0.5,0,-0.5,0,0,0,0,0;
	}
	else
	{
		double n2 = n*n;
		double n3 = n*n*n;
		double M = (1 - cos(n)) /n2;
		double N = (n - sin(n)) /n3;
		double p1M = w1/n2 - w1*N - 2*w1*M/n2;
		double p2M = w2/n2 - w2*N - 2*w2*M/n2;
		double p1N = w1*M/n2 - 3*w1*N/n2;
		double p2N = w2*M/n2 - 3*w2*N/n2;
		Matrix3d pwJleftwv;
		pwJleftwv<<           p1M*w2,           p2M*w2 + M,    0,
                           -p1M*w1 - M,              -p2M*w1,    0,
                     -p1N*n2 - 2*N*w1,    -p2N*n2 - 2*N*w2,    0;
		Jc.block(0,0,3,3) = MatrixXd::Identity(3,3) - M*uphat(w) + N*(uphat(w)*uphat(w));
		Jc.block(3,0,3,3) = uphat(w).exp().transpose()*L*pwJleftwv;
	}
	return Jc.block(0,0,6,2);
}
MatrixXd inv(MatrixXd T)
{
	MatrixXd invT = MatrixXd::Identity(4,4);
	invT.block(0,0,3,3) = T.block(0,0,3,3).transpose();
	invT.block(0,3,3,1) = -T.block(0,0,3,3).transpose()*T.block(0,3,3,1);
	return invT;
}
MatrixXd q2T(Quaterniond q,Vector3d o)
{
	MatrixXd T = MatrixXd::Identity(4,4);
    T.block(0,0,3,3) = q.toRotationMatrix();
    T.block(0,3,3,1) = o;
	return T;
}
VectorXd xi2arc(VectorXd xi,double L1,double L2,double L3)
{
	VectorXd arc(6);
	arc(0) = mod(sqrt(xi(0)*xi(0) + xi(1)*xi(1) ), 2*PI) / L1,
	arc(1) = atan2(-xi(0), xi(1)),
	arc(2) = mod(sqrt(xi(2)*xi(2)  + xi(3)*xi(3) ), 2*PI)/L2,
	arc(3) = atan2(-xi(2), xi(3)),
	arc(4) = mod(sqrt(xi(4)*xi(4)  + xi(5)*xi(5) ), 2*PI) / L3;
	arc(5) = atan2(-xi(4), xi(5));
	return arc;
}
VectorXd xi2arc(VectorXd xi, double L)
{
	return xi2arc(xi,L,L,L);
}
MatrixXd jnt2T(VectorXd jnt,VectorXd L)
{

	MatrixXd T = MatrixXd::Identity(4,4);
	MatrixXd dK = MatrixXd::Identity(4,4);
	MatrixXd Rotx(4,4),Roty(4,4);
	double tx,ty;
	
	for (size_t i = 0;i<9;i++)
	{
		if (i%2 == 0)
		{
			ty = jnt(2*i);
			tx = jnt(2*i+1);
			Rotx << 1,0,0,0,0,cos(tx),-sin(tx),0,0,sin(tx),cos(tx),0,0,0,0,1;
			Roty << cos(ty),0,sin(ty),0,0,1,0,0,-sin(ty),0,cos(ty),0,0,0,0,1;
			dK(2,3) = L(i); 
			T = T*Roty*Rotx*dK;
			
		}
		else
		{
			ty = jnt(2*i+1);
			tx = jnt(2*i);
			Rotx << 1,0,0,0,0,cos(tx),-sin(tx),0,0,sin(tx),cos(tx),0,0,0,0,1;
			Roty << cos(ty),0,sin(ty),0,0,1,0,0,-sin(ty),0,cos(ty),0,0,0,0,1;
			dK(2,3) = L(i); 
			T = T*Rotx*Roty*dK;
		}
	}
	return T;
}
MatrixXd jnt2T(VectorXd jnt,double L)
{
	VectorXd LL(9);
	LL<<L,L,L,L,L,L,L,L,L;
	return jnt2T(jnt,LL);
}
MatrixXd xi2T(VectorXd xi, double L1,double L2,double L3)
{
	VectorXd X0(6);
	Vector3d L;
	L << L1,L2,L3;
	MatrixXd Ti,T=MatrixXd::Identity(4,4);
	for (size_t i = 0;i<3;i++)
	{

		X0 << xi(2*i),xi(2*i+1),0,0,0,L(i);
		Ti = uphat(X0);
		T = T*Ti.exp();
	}
	return T;
}
MatrixXd xi2T(VectorXd xi, double L)
{
	return xi2T(xi,L,L,L);
}
double mod(double x1,double x2)
{
	int k = x1/x2;
	return x1-k*x2;
}
/* #define _TEST */
#ifdef _TEST
//test, disable in final version
int main (void)
{
	VectorXd arc(6);
	MatrixXd T;
	Matrix3d D;
	MatrixXd J;
	arc << 1,2,1,2,1,2;
	arc *= PI/2;
	srand(time(0));
	arc *= double(rand())/double(RAND_MAX);
	VectorXd q(18);
	VectorXd Xi(6);
	Xi << 0.00293195155861062,
		0.00293209911804849,
		-0.308962346414838,
		-0.519154472891102,
		0.357002819823276,
		-0.817500789875600;//arc2xi(1,1,1,arc);
	Vector3d kp,ph,L;
	L << 1,1,1;
	kp << Xi(0),Xi(2),Xi(4);
	ph << Xi(1),Xi(3),Xi(5);
	q = xi2jnt(Xi,1);
	T = forward_kinematics(kp,ph,L);
	Vector4d param ;
	param << 0.038,0.042,0.102,0.026;
	D = jnt2dist(q,0,param);
	J = jacobian3cc(1,Xi);
	std::cout << q << std::endl;
	std::cout << J << std::endl;
	std::cout << D << std::endl;
}
#endif
