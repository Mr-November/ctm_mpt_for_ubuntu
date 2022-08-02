#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <iostream>

void gen(float* dst, float t);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ctm_mpt_virtual_controller");
	ros::NodeHandle nh;

	ros::Publisher pos_pub = nh.advertise<std_msgs::Int32MultiArray>("jnt_pos", 1000);
	std_msgs::Int32MultiArray msg;
    std_msgs::MultiArrayDimension dim;
    dim.label = "ang";
    dim.size = 18;
    dim.stride = 18;
    msg.layout.dim.push_back(dim);
    msg.layout.data_offset = 0;
    msg.data = std::vector<int32_t>(18, 0);

	ros::Publisher jnt_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1000);
	sensor_msgs::JointState joint_state;
	joint_state.position.resize(18);
	joint_state.name.resize(18);
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
	while (ros::ok())
	{
		float fwd[18] = {0.0};
		uint8_t i = 0;
		static float proc = 0.0;
		static float dir = 1.0;

		if (proc <= 0)
		{
			dir = 1.0;
		}
		else if (proc > 1)
		{
			dir = -1.0;
		}
		proc += dir * 0.01;
		gen(fwd, proc);

		joint_state.header.stamp = ros::Time::now();

		for (i = 0; i < 18; i++)
		{
			msg.data.at(i) = fwd[i];
			joint_state.position[i] = fwd[i];
		}

		pos_pub.publish(msg);
		jnt_pub.publish(joint_state);
		ROS_INFO_STREAM("Procedure = " << proc << ".");
		loop_rate.sleep();
	}

	return 0;
}

void gen(float* dst, float t)
{
	uint8_t i = 0;
	
	for (i = 0; i < 18; i += 2)
	{
		*(dst + i) = -0.35 + 0.7 * t;
	}
	for (i = 1; i < 18; i += 2)
	{
		*(dst + i) = -0.7 * t * (t - 1);
	}

	return;
}