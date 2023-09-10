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

using namespace std;
using namespace Eigen;

int main (int argc, char** argv)
{
    Vector4d param;
   VectorXd D;
    VectorXd q(18);
    param << 0.038,0.042,0.102,0.026;
    q << -0.188487330000000,	0.123931510000000,	0.475757250000000,	-0.256258920000000,	-0.574335500000000,	0.220797710000000,	0.244328830000000,	0.107556130000000,	0.587016020000000,	-0.201588880000000,	-0.367420880000000,	0.335219860000000,	0.499318400000000,	-0.0571375300000000	,-0.115655160000000,	-0.117963390000000,	-0.152269460000000,	-0.0427534000000000;
    D = jnt2dist(q,0,param);
    cout<<D<<endl;
}