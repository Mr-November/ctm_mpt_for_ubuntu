#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/MatrixFunctions>
using namespace Eigen;
VectorXd arc2xi(VectorXd arc,double L1,double L2,double L3);
MatrixXd forward_kinematics(Vector3d kp, Vector3d ph,double L1,double L2,double L3);
VectorXd xi2jnt(VectorXd xi,double L1,double L2,double L3);
VectorXd xi2jnt(VectorXd xi,double L);
VectorXd jnt2arc(VectorXd jnt,double L);
VectorXd jnt2dist(VectorXd q,size_t noise_type, Vector4d param);
MatrixXd jacobian3cc(VectorXd xi,double L1,double L2,double L3);
MatrixXd jacobian3cc(VectorXd xi,double L);
MatrixXd jaco_c12(double w1,double w2, double L);
VectorXd dist2arc(double L,double r0,Matrix3d dist);
MatrixXd uphat(VectorXd V);
VectorXd upvee(MatrixXd M);
MatrixXd inv(MatrixXd T);
MatrixXd q2T(Quaterniond q,Vector3d o);
VectorXd xi2arc(VectorXd xi, double L);
VectorXd xi2arc(VectorXd xi, double L1,double L2,double L3);
MatrixXd jnt2T(VectorXd jnt,double L);
MatrixXd jnt2T(VectorXd jnt,VectorXd L);
MatrixXd xi2T(VectorXd xi, double L);
MatrixXd xi2T(VectorXd xi, double L1,double L2,double L3);
#endif