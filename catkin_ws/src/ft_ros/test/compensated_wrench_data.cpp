/**
 * @file compensated_wrench_data.cpp
 * @brief This is a test file
 * @details 
 *  It is used to record force torque sensor data
 * @mainpage FT Sensor
 * @author Xiangchi Chen
 * @email chenxiangchi@zju.edu.cn
 * @version 1.1.0
 * @date 11.17 2020
 */
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include <geometry_msgs/WrenchStamped.h>

using namespace std;
geometry_msgs::WrenchStamped wrench_data;
geometry_msgs::Pose pose;
void wrenchCallback(const geometry_msgs::WrenchStamped &input)
{
	wrench_data = input;
}
void toolposeCallback(const geometry_msgs::PoseStampedConstPtr &input)
{
	pose = input->pose;
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "wrench_data_get", ros::init_options::AnonymousName);
	ros::AsyncSpinner spinner(2);
	ros::NodeHandle nh;
	ros::Subscriber wrench_sub;
	ros::Subscriber tool_pose_sub;
	ros::Publisher wrench_pub;
	wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>("raw_wrench_data", 100);
	wrench_sub = nh.subscribe("netft_data", 100, wrenchCallback);
	tool_pose_sub = nh.subscribe("ur16e_tool_pose", 100, toolposeCallback);
	spinner.start();
	ros::Rate loop_rate(500);
	geometry_msgs::WrenchStamped wrench;
	char str[1] = {0};
	int i;
	i = 0;
	ofstream out("/home/tencent1/wrench_data.txt");
	while (ros::ok())
	{
		std::cout << 1 << std::endl;
		out << "data" << i << ":\n"
			<< wrench_data.wrench << std::endl;
		i++;
		wrench_pub.publish(wrench_data);
		loop_rate.sleep();
	}
	out.close();
	cin.get();
}