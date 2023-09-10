#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <fstream>

#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

void proc(size_t cur, size_t tot);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "vrt_cntlr_2");
	ros::NodeHandle nh;

	const size_t N = 18;

	tf::TransformListener tf_listener;
	ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("history_path", 1000);
	ros::Time t_cur;
    t_cur = ros::Time::now();
    nav_msgs::Path path;
    path.header.stamp = t_cur;
    path.header.frame_id = "Ref";

	ros::Rate loop_rate(100);
	ros::Duration(10).sleep(); // Wait for RViz to open.
	if (ros::ok())
	{
			try
			{
				tf_listener.lookupTransform("/Ref", "/Tip", ros::Time(0), transform);
			}
			catch (tf::TransformException ex)
			{
				ROS_ERROR("%s",ex.what());
				ros::Duration(0.5).sleep();
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
	}

	return 0;
}