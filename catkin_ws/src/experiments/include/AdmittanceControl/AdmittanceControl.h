#include <ros/ros.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>

#define LOOP_RATE 500.0
using namespace Eigen;

class AdmittanceControl
{   
public:
    /// Constructor
    AdmittanceControl(std::string ip_address):controller(ip_address),receiver(ip_address),loop_rate(LOOP_RATE)
    {

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
    /// Current wrench data
    Matrix<double,6,1> current_wrench_data;
    /// Current tool acceleration
    Matrix<double,6,1> current_tool_acc;
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
    ros::Subscriber wrench_data_sub;
    /// Control interval
    double timer;
public:

    MatrixXd Vector2Eigen(std::vector<double> input);

    void UpdateData();

    void Init();

    void WrenchCallback(const geometry_msgs::WrenchStamped &input);
    
    void Execute();

    void Spin();
};