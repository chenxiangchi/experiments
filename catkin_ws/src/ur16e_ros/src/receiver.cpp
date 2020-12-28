/**
 * @file receiver.cpp
 * @brief UR16e ros realtime status receiver
 * @details 
 *  This program will subscribe 3 kinds of UR16e status and transform its type into ros type 
 *  Tool Velocity Receiver: Publish "/ur16e_tool_vel" 
 *  Tool Pose Receiver: Publish "/ur16e_tool_pose 
 *  Joint State Receiver: Publish "/ur16e_joint_states" 
 * @mainpage UR16e Receiver
 * @author Xiangchi Chen
 * @email chenxiangchi@zju.edu.cn
 * @version 1.0.0
 * @date 11.17 2020
 */

#include <ros/ros.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

/// IP address of UR16e host
#define IP_ADDRESS "192.168.3.1"
/// Pi
#define PI 3.1415926

int main(int argc, char **argv)
{
  /// Init a ROS node "ur_receiver"
  ros::init(argc, argv, "ur_receiver");
  ros::NodeHandle nh;

  /// IP address of UR16e host
  std::string ip_address;
  nh.param<std::string>("ip_address", ip_address, IP_ADDRESS);
  nh.getParam("ip_address", ip_address);

  /// Init UR realtime receiver object with robot host
  ur_rtde::RTDEReceiveInterface receiver(ip_address);
  ros::AsyncSpinner spin(2);
  spin.start();

 

  /// Frequency of published ur16e data
  ros::Rate loop_rate(500);

  /// Wait until receiver is connected
  while (!receiver.isConnected())
    loop_rate.sleep();
  ROS_INFO_STREAM("Robot Receiver Connected!");

  ros::Publisher tool_twist_pub, //< Jool velocity publisher
      tool_pose_pub,             //< Jool pose publisher
      joint_state_pub;           //< Joints's state publisher

  /// Init ROS publishers
  tool_twist_pub = nh.advertise<geometry_msgs::TwistStamped>("/ur16e_tool_twist", 2);
  tool_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ur16e_tool_pose", 2);
  joint_state_pub = nh.advertise<sensor_msgs::JointState>("/ur16e_joint_states", 2);

  geometry_msgs::TwistStamped twist; //< Tool velocity
  geometry_msgs::PoseStamped pose;   //< Tool Pose
  double theta;                      //< Rotation angle

  while (ros::ok())
  {
    //Ensure that the connection is OK
    while (!receiver.isConnected())
    {
      ROS_WARN_STREAM_ONCE("Robot disconnected!");
      if (receiver.reconnect())
      {
        ROS_INFO_STREAM("Robot Reconnected!");
      }
    }

    /// Current tool speed
    twist.header.frame_id = "base_link";
    twist.header.stamp = ros::Time::now();
    twist.twist.linear.x = receiver.getActualTCPSpeed()[0];
    twist.twist.linear.y = receiver.getActualTCPSpeed()[1];
    twist.twist.linear.z = receiver.getActualTCPSpeed()[2];
    twist.twist.angular.x = receiver.getActualTCPSpeed()[3];
    twist.twist.angular.y = receiver.getActualTCPSpeed()[4];
    twist.twist.angular.z = receiver.getActualTCPSpeed()[5];

    /// Current tool pose
    pose.header.frame_id = "base_link";
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = receiver.getActualTCPPose()[0];
    pose.pose.position.y = receiver.getActualTCPPose()[1];
    pose.pose.position.z = receiver.getActualTCPPose()[2];
    theta = sqrt(pow(receiver.getActualTCPPose()[3], 2) + pow(receiver.getActualTCPPose()[4], 2) + pow(receiver.getActualTCPPose()[5], 2));
    pose.pose.orientation.x = receiver.getActualTCPPose()[3] / theta * sin(theta / 2);
    pose.pose.orientation.y = receiver.getActualTCPPose()[4] / theta * sin(theta / 2);
    pose.pose.orientation.z = receiver.getActualTCPPose()[5] / theta * sin(theta / 2);
    pose.pose.orientation.w = cos(theta / 2);

    /// Current joints' states
    sensor_msgs::JointState joint_states;
    joint_states.header.frame_id = "base_link";
    joint_states.header.stamp = ros::Time::now();

    /// Joints' position
    joint_states.position.push_back(receiver.getActualQ()[0]);
    joint_states.position.push_back(receiver.getActualQ()[1]);
    joint_states.position.push_back(receiver.getActualQ()[2]);
    joint_states.position.push_back(receiver.getActualQ()[3]);
    joint_states.position.push_back(receiver.getActualQ()[4]);
    joint_states.position.push_back(receiver.getActualQ()[5]);
    /// Joints' velocity
    joint_states.velocity.push_back(receiver.getActualQd()[0]);
    joint_states.velocity.push_back(receiver.getActualQd()[1]);
    joint_states.velocity.push_back(receiver.getActualQd()[2]);
    joint_states.velocity.push_back(receiver.getActualQd()[3]);
    joint_states.velocity.push_back(receiver.getActualQd()[4]);
    joint_states.velocity.push_back(receiver.getActualQd()[5]);

    /// Publish UR16e status
    tool_twist_pub.publish(twist);
    tool_pose_pub.publish(pose);
    joint_state_pub.publish(joint_states);

    loop_rate.sleep();
  }
}