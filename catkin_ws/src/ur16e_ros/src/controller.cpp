/**
 * @file controller.cpp
 * @brief UR16e ros realtime controller
 * @details 
 *  This file contains 4 kinds of UR16e controller:
 *  Tool Velocity Controller: Subscribe "/tool_vel_cmd" topic and control Manipulator with tcp velocity 
 *  Tool Pose Controller: Subscribe "/tool_pose_cmd" topic and control Manipulator with tcp pose 
 *  Joint Velocity Controller: Subscribe "/joint_vel_cmd" topic and control Manipulator with joints' velocity 
 *  Joint Position Controller: Subscribe "/joint_pose_cmd" topic and control Manipulator with joints' position 
 * @mainpage UR16e Controller
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
/// Default ur16e acceleration scale
#define acc 1.0
/// Default ur16e velocity scale
#define vel 1.0

/**
 * @brief UR16e controller 
 *  Tool Velocity Controller: Subscribe "/tool_vel_cmd" topic and control Manipulator with tcp velocity
 *  Tool Pose Controller: Subscribe "/tool_pose_cmd" topic and control Manipulator with tcp pose
 *  Joint Velocity Controller: Subscribe "/joint_vel_cmd" topic and control Manipulator with joints' velocity
 *  Joint Position Controller: Subscribe "/joint_pose_cmd" topic and control Manipulator with joints' position
 */
class URController
{
public:
  /// Constructor with robot's host
  URController(std::string ip_address) : controller(ip_address)
  {
    /// Init subscribers and publishers
    tool_vel_cmd_sub = nh.subscribe("/tool_twist_cmd", 1, &URController::toolvelCallback, this);
    tool_pose_cmd_sub = nh.subscribe("/tool_pose_cmd", 1, &URController::toolposeCallback, this);
    joint_vel_cmd_sub = nh.subscribe("/joint_twist_cmd", 1, &URController::jointvelCallback, this);
    joint_pose_cmd_sub = nh.subscribe("/joint_pose_cmd", 1, &URController::jointposeCallback, this);
    controller.setPayload(5);
  } 
public:
  /**
  * @brief Tool velocity command function
  * @detail
  *  Used to control UR16e with tool velocity
  * @param[in] &msg        Desired tool velocity 
  */
  void toolvelCallback(const geometry_msgs::TwistConstPtr msg)
  {
    if (lock == 0)
    {
      lock = 1;
      std::vector<double> cmd;
      cmd.push_back(msg->linear.x);
      cmd.push_back(msg->linear.y);
      cmd.push_back(msg->linear.z);
      cmd.push_back(msg->angular.x);
      cmd.push_back(msg->angular.y);
      cmd.push_back(msg->angular.z);
      controller.speedL(cmd, acc, 0.0);
    }
    lock = 0;
  }

  /**
  * @brief Tool pose command function
  * @detail
  *  Used to control UR16e with tool pose
  * @param[in] &msg        Desired tool pose
  */
  void toolposeCallback(const geometry_msgs::PoseConstPtr msg)
  {
    if (lock == 0)
    {
      lock = 1;
      std::vector<double> cmd;
      double theta;
      theta = 2 * acos(msg->orientation.w);
      cmd.push_back(msg->position.x);
      cmd.push_back(msg->position.y);
      cmd.push_back(msg->position.z);
      cmd.push_back(msg->orientation.x / sin(theta / 2) * theta);
      cmd.push_back(msg->orientation.y / sin(theta / 2) * theta);
      cmd.push_back(msg->orientation.z / sin(theta / 2) * theta);
      controller.moveL(cmd, 0.1, 0.1);
    }
    lock = 0;
  }

  /**
  * @brief Joints' velocity command function
  * @detail
  *  Used to control UR16e with joints' velocity
  * @param[in] &msg        Desired joints' velocity 
  */
  void jointvelCallback(const sensor_msgs::JointStateConstPtr msg)
  {
    if (lock == 0)
    {
      lock = 1;
      std::vector<double> cmd;
      cmd.push_back(msg->velocity[0]);
      cmd.push_back(msg->velocity[1]);
      cmd.push_back(msg->velocity[2]);
      cmd.push_back(msg->velocity[3]);
      cmd.push_back(msg->velocity[4]);
      cmd.push_back(msg->velocity[5]);
      controller.speedJ(cmd, 3.0, 0.01);
    }
    lock = 0;
  }

  /**
  * @brief Joints' pose command function
  * @detail
  *  Used to control UR16e with joints' pose
  * @param[in] &msg        Desired joints' pose
  */
  void jointposeCallback(const sensor_msgs::JointStateConstPtr msg)
  {
    if (lock == 0)
    {
      lock = 1;
      std::vector<double> cmd;
      cmd.push_back(msg->position[0]);
      cmd.push_back(msg->position[1]);
      cmd.push_back(msg->position[2]);
      cmd.push_back(msg->position[3]);
      cmd.push_back(msg->position[4]);
      cmd.push_back(msg->position[5]);
      controller.moveJ(cmd, 0.3, 0.3);
    }
    lock = 0;
  }

public:
  /// Subscribers for control command
  ros::Subscriber tool_vel_cmd_sub, //< Tool velocity command subscriber
      tool_pose_cmd_sub,            //< Tool pose command subscriber
      joint_vel_cmd_sub,            //< Joints' velocity command subscriber
      joint_pose_cmd_sub;           //< Joints' position command subscriber
  /// Universal robot realtime controller API
  ur_rtde::RTDEControlInterface controller;
  /// Ros nodehandle
  ros::NodeHandle nh;
  /// Flag to ensure that only one ur16e command can be executed in the same time
  bool lock;
};

/**
 * @brief UR16e controller main function
 *  Start the UR16e control loop 
 *  If no command subscribed,the controller will set manipulator speed to zero
 *  The controller will reconnect when crashing.
 */
int main(int argc, char **argv)
{
  /// Init a ROS node "ur_controller"
  ros::init(argc, argv, "ur_controller");
  ros::NodeHandle nh;

  ros::AsyncSpinner spin(4);
  spin.start();
  /// Control loop's rate
  ros::Rate loop_rate(500);

  /// IP address of UR16e host
  std::string ip_address;
  nh.param<std::string>("ip_address", ip_address, IP_ADDRESS);
  nh.getParam("ip_address", ip_address);
  URController ur_controller(ip_address);

 

  /// Wait until controller is connected
  while (!ur_controller.controller.isConnected())
    loop_rate.sleep();
  ROS_INFO_STREAM("Robot Controller Connected!");

  while (ros::ok())
  {
    /// Ensure that the connection is OK
    while (!ur_controller.controller.isConnected())
    {
      ROS_WARN_STREAM_ONCE("Robot Controller Disconnected!");
      if (ur_controller.controller.reconnect())
      {
        ROS_INFO_STREAM("Robot Controller Reconnected!");
      }
    }
    /// If connection crashed,ur16e will stop moving
    if (!ur_controller.tool_vel_cmd_sub.getNumPublishers() && !ur_controller.tool_pose_cmd_sub.getNumPublishers() && !ur_controller.joint_vel_cmd_sub.getNumPublishers() && !ur_controller.joint_pose_cmd_sub.getNumPublishers())
    {
      ur_controller.controller.speedStop();
    }
    loop_rate.sleep();
  }
}
