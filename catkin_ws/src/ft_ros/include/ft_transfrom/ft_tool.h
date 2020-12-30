/**
 * @file ft_tool.h
 * @brief Head document of Ati force torque sensor delta 330 data process program
 * @details 
 *  This document contains a FTProcess class for force troque data process
 * @mainpage FT Sensor
 * @author Xiangchi Chen
 * @email chenxiangchi@zju.edu.cn
 * @version 1.1.0
 * @date 11.17 2020
 */
#ifndef ft_tool
#define ft_tool

#include <ros/ros.h>
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include <geometry_msgs/WrenchStamped.h>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"
#include "std_msgs/Float64.h"

/**
 * @brief Eigen Library
 * Used to matrix operation
*/
using namespace Eigen;

typedef Matrix<double, 6, 1> Vector6d;

/**
 * @brief Class for force torque data process
 * The class contains several functions:
 * Gravity compensation function
 * Bias compensation function
 * Low pass filter function
 * Zero drift compensation function
 * Set Deadzone function
 */
class FTProcessor
{

public:
  /// Listener to listen the ft_publisher
  tf::TransformListener ft_lis;
  /// Keep the transform
  tf::StampedTransform ft_transform;
  /// Wrench data
  geometry_msgs::WrenchStamped wrench_data;
  /// Temp wrench data
  geometry_msgs::WrenchStamped _wrench_data;
  /// Transformed force
  Matrix<double, 3, 1> transformed_force;
  /// Transformed torque
  Matrix<double, 3, 1> transformed_torque;
  /// Deadzone
  geometry_msgs::Wrench Deadzone;
  /// The error from the sensor
  geometry_msgs::Wrench wrench_error;
  // Old wrench for step error
  geometry_msgs::Wrench old_wrench;
  /// Transform loop rate
  ros::Rate rate;
  /// Filter Factor
  double wrench_filter_factor;
  /// Current wrench for filter
  Vector6d wrench_external;
  /// Previous wrench for filter
  Vector6d wrench_external_old;
  /// UR16e tool pose
  geometry_msgs::PoseStamped tool_pose;
  /// Bias matrix of gripper
  Matrix<double, 3, 1> gripper_bias_matrix;
  /// Gravity matrix
  Matrix<double, 3, 1> gripper_gravity_matrix;
  /// Rotation Matrix from sensor frame to base frame
  Matrix<double, 3, 3> T_sensor2base;
  /// Compensated Force
  Matrix<double, 3, 1> wrench_force;
  /// Compensated Torque
  Matrix<double, 3, 1> wrench_torque;
  /// Bias matrix of sensor, sensor center of gravity w.r.t. sensor frame
  Matrix<double, 3, 1> sensor_bias_matrix;
  /// Rotation matrix from sensor frame to toolframe
  Matrix<double,3 ,3> T_sensor2tool;
  /// Quaternion from tool frame to base frame
  Quaterniond Q_tool2base;
  /// Process flag
  bool flag;


  /// Constructor with cutoff frequency : 2hz
  FTProcessor() : rate(500), ft_lis(ros::Duration(10)), wrench_filter_factor(0.01)
  {
    init();
  }

  /**
  * @brief Init function
  * @detail
  *  Init the FTProcessor coefficient,ros publishers and ros subscribers
  */
  void init();

  /**
  * @brief Transform loop  
  * @detail
  *  Transform loop  
  */
  void spin();

protected:
  /// Ros nodehandle
  ros::NodeHandle n_ftp;
  /// Subscribe the data from sensor
  ros::Subscriber ft_wrench_sub;
  /// Publish transformed wrench data
  ros::Publisher ft_wrench_pub;
  /// Subscribe tool pose data
  ros::Subscriber tool_pose_sub;

private:
  /**
  * @brief Transform wrench function
  * @detail
  *  Transform wrench_force to tool frame
  */
  void transformWrench();

  /**
  * @brief Wrench callback
  * @detail
  *  Subscribe raw wrench data from /netft_data topic
  * @param[in] &input        raw wrench data with stamp
  */
  void wrenchCallback(const geometry_msgs::WrenchStamped &_wrench);

  /**
  * @brief Low pass filter function
  * @detail
  *  Process sensor data with low pass filter
  *  wrench_external = (1 - wrench_filter_factor) * wrench_external_old + wrench_filter_factor * wrench_external;
  *  wrench_filter_factor = 1/(1 + (1/2*pi*T*fc)) 
  */
  void lowpassFilter();

  /**
  * @brief Drift ompensation funtion 
  * @detail
  *  Zero the oringinal data of the force torque sensor
  * @param[in] _wrench_data        temp wrench data
  */
  void driftCompensation(geometry_msgs::WrenchStamped _wrench_data);

  /**
  * @brief Set Deadzone funtion 
  * @detail
  *  Set Deadzone for the wrench data
  *  If the data is within Deadzone,set zero
  * @param[in] _wrench_data        temp wrench data
  * @param[in] _Deadzone       desired Deadzone
  * @param[out] setDeadzone()      wrench data after being set Deadzone
  */
  geometry_msgs::Wrench setDeadzone(geometry_msgs::Wrench _wrench_data, const geometry_msgs::Wrench _Deadzone);

  /**
  * @brief Gripper bias compensation funtion 
  * @detail
  *  Gravity compensation for gripper to get true force torque data
  *  Force = mass * T_sensor2mecanum.T * gravity_matrix
  *  Torque = mass * bias_matrix X (T_sensor2mecanum.T * gravity_matrix)
  */
  void gripperbiasCompensation();

  /**
  * @brief Sensor bias compensation funtion 
  * @detail
  *  Bias compensation for the bias of force torque sensor to get true force torque data
  *  Force = mass * T_sensor2mecanum.T * gravity_matrix
  *  Torque = mass * bias_matrix X (T_sensor2mecanum.T * gravity_matrix)
  */
  void sensorbiasCompensation();

  /**
  * @brief Pose callback
  * @detail
  *  Subscribe UR16e pose from /ur16e_tool_pose topic
  * @param[in] &msg        ur16e tool pose with stamp
  */
  void poseCallback(const geometry_msgs::PoseStamped &msg);
};

#endif
