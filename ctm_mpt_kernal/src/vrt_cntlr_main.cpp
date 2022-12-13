#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <fstream>

void proc(size_t cur, size_t tot);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "vrt_cntlr");
	ros::NodeHandle nh;

	const size_t N = 18;

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

	ros::Rate loop_rate(10);
	ros::Duration(4).sleep(); // Wait for RViz to open.
	while (ros::ok())
	{
		std::string fn("/home/ellipse/Desktop/catkin_ws/src/ctm_mpt/ctm_mpt_kernal/src/fq.txt");
		std::ifstream fin(fn);
		std::string str_ang;
		std::string str_tot;
		size_t tot = 0;
		size_t cur = 0;
		size_t i = 0;
		float fwd[N] = {0.0};
		
		if (!fin.is_open())
		{
			ROS_ERROR_STREAM("Unable to open \"" << fn << "\".");

			return 0;
		}

		fin >> str_tot;
		tot = std::stoul(str_tot);

		while (cur < tot)
		{
			for (i = 0; i < N; i++)
			{
				fin >> str_ang;
				fwd[i] = std::stof(str_ang);
				joint_state.position[i] = fwd[i];
			}
			joint_state.header.stamp = ros::Time::now();
			jnt_pub.publish(joint_state);
			cur += 1;
			proc(cur, tot);
			// std::getchar(); // For debug only.
			loop_rate.sleep();
		}
		fin.clear();
		fin.close();
	}

	return 0;
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
	{
		cover = !cover;
	}
	else
	{
		printf("\033[1A\033[K");
	}
	for (i = 0; i < num; i++)
	{
		info += "#";
	}
	for (i = 0; i < len-num; i++)
	{
		info += "-";
	}
	sprintf(suffix, "]%6.2f%% (%d/%d)", per*100, cur, tot);
	ROS_INFO_STREAM(info << suffix);

	return;
}