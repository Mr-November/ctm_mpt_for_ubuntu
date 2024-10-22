#include "vrpn_client_ros/vrpn_client_ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vrpn_client_node");
  ros::NodeHandle nh, private_nh("~");
  vrpn_client_ros::VrpnClientRos client(nh, private_nh);
  ros::spin();
  return 0;
}

