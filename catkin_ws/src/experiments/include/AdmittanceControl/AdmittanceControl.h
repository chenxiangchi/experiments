#include <ros/ros.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <iostream>
#include <thread>
#include <chrono>
#include <fstream>

#define HOME_PATH getenv("HOME")
#define LOOP_RATE 500.0
#define IP_ADDRESS "192.168.3.1"

using namespace Eigen;
using namespace std;
using namespace std::chrono;

class AdmittanceControl
{   
public:
    /// Constructor
    AdmittanceControl(string ip_address):controller(ip_address),receiver(ip_address),loop_rate(LOOP_RATE)
    {
        Init();
    }

public:
    /// Loop rate
    ros::Rate loop_rate;
    /// Control interface
    ur_rtde::RTDEControlInterface controller;
    /// Data receive interface
    ur_rtde::RTDEReceiveInterface receiver;
    /// Current tool twist
    Matrix<double,6,1> current_tool_twist;
    /// Current filtered wrench data
    Matrix<double,6,1> current_filtered_wrench_data;
    /// Temp current raw wrench data
    Matrix<double,6,1> current_raw_wrench_data;
    /// Temp current filtered wrench data
    Matrix<double,6,1> _current_filtered_wrench_data;
    /// Current raw wrench data
    Matrix<double,6,1> _current_raw_wrench_data;
    /// Compensated wrench data for temperature drift
    Matrix<double,6,1> compensated_wrench_data;
    ///
    double sigma_temp;
    /// Current tool acceleration
    Matrix<double,6,1> current_tool_acc;
    /// Current tool pose
    Matrix<double,6,1> current_tool_pose;
    /// Expected wrench data
    Matrix<double,6,1> expected_wrench_data;
    /// Expected tool twist
    Matrix<double,6,1> expected_tool_twist;
    /// Expected tool acceleration
    Matrix<double,6,1> expected_tool_acc;
    /// Inverse mass matrix
    Matrix<double,6,6> M_inv_matrix;
    /// Dadmpling matrix
    Matrix<double,6,6> D_matrix;
    /// K matrix
    Matrix<double,6,6> K_matrix;
    /// Ros nodehandle
    ros::NodeHandle nh;
    /// Transformed wrench data subscriber
    ros::Subscriber transformed_wrench_data_sub;
    /// Raw wrench data subscriber
    ros::Subscriber raw_wrench_data_sub;
    /// Control interval
    double timer;
    /// Origin joints' posistion
    vector<double> origin_q;
public:

    MatrixXd Vector2Eigen(vector<double> input);

    void UpdateData();

    void Init();

    void TransformedWrenchCallback(const geometry_msgs::WrenchStamped &input);
    
    void RawWrenchCallback(const geometry_msgs::WrenchStamped &input);

    void Execute();
    
    void TempComp();

    void Spin();
};