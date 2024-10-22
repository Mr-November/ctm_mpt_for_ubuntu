#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

void myCallback(const geometry_msgs::PoseStamped& tfmsg) 
{
  ROS_INFO("Received value is: %f", tfmsg.pose.position.x);

  return;
} 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vrpn_test_node");
  ros::NodeHandle nh;
  ros::Subscriber tf_subscriber = nh.subscribe("vrpn_client_node/RigidBodyTest/pose", 10, &myCallback);
  
  ros::spin();
// test
  return 0;
}

