
/**
 * @file ft_tool.cpp
 * @brief Function implement of Ati force torque sensor delta 330 data process program
 * @details 
 *  The process program runs as below: 
 *  Bias compensation--->Gravity compensation--->Transform wrench frame--->
 *  Zero compensation--->Low pass filter--->Set threshold
 * @mainpage FT Sensor
 * @author Xiangchi Chen
 * @email chenxiangchi@zju.edu.cn
 * @version 1.1.0
 * @date 10.17 2020
 */
#include <ft_transfrom/ft_tool.h>

/// Transform wrench_force to tool frame
void FTProcessor::transformWrench()
{   
    
    Matrix<double, 3, 1> temp_force, temp_torque;
    temp_force << wrench_data.wrench.force.x, wrench_data.wrench.force.y, wrench_data.wrench.force.z;
    temp_torque << wrench_data.wrench.torque.x, wrench_data.wrench.torque.y, wrench_data.wrench.torque.z;
    transformed_force = T_sensor2tool * temp_force;
    transformed_torque = T_sensor2tool * temp_torque;

    wrench_data.wrench.force.x = transformed_force(0, 0);
    wrench_data.wrench.force.y = transformed_force(1, 0);
    wrench_data.wrench.force.z = transformed_force(2, 0);
    wrench_data.wrench.torque.x = transformed_torque(0, 0);
    wrench_data.wrench.torque.y = transformed_torque(1, 0);
    wrench_data.wrench.torque.z = transformed_torque(2, 0);
}

/// Subscribe UR16e pose from /ur16e_tool_pose topic
void FTProcessor::poseCallback(const geometry_msgs::PoseStamped &msg)
{
    tool_pose = msg;
}

/// Subscribe raw wrench data from /netft_data topic
void FTProcessor::wrenchCallback(const geometry_msgs::WrenchStamped &input)
{
    // Obtain wrench_data(Need to be setZero and filtered)
    _wrench_data = input;
    //_wrench_data.wrench.force.z = - _wrench_data.wrench.force.z;
}

/// Process sensor data with low pass filter
///  wrench_external = (1 - wrench_filter_factor) * wrench_external_old + wrench_filter_factor * wrench_external;
///  wrench_filter_factor = 1/(1 + (1/2*pi*T*fc))
void FTProcessor::lowpassFilter()
{
    wrench_external << wrench_data.wrench.force.x, wrench_data.wrench.force.y,
        wrench_data.wrench.force.z, wrench_data.wrench.torque.x,
        wrench_data.wrench.torque.y, wrench_data.wrench.torque.z;
    wrench_external << (1 - wrench_filter_factor) * wrench_external_old +
                           wrench_filter_factor * wrench_external;
    wrench_data.wrench.force.x = wrench_external[0];
    wrench_data.wrench.force.y = wrench_external[1];
    wrench_data.wrench.force.z = wrench_external[2];
    wrench_data.wrench.torque.x = wrench_external[3];
    wrench_data.wrench.torque.y = wrench_external[4];
    wrench_data.wrench.torque.z = wrench_external[5];
    wrench_external_old = wrench_external;
}

/// Zero the oringinal data of the force torque sensor
void FTProcessor::driftCompensation(geometry_msgs::WrenchStamped _wrench_data)
{
    wrench_data.wrench.force.x -= wrench_error.force.x;
    wrench_data.wrench.force.y -= wrench_error.force.y;
    wrench_data.wrench.force.z -= wrench_error.force.z;
    wrench_data.wrench.torque.x -= wrench_error.torque.x;
    wrench_data.wrench.torque.y -= wrench_error.torque.y;
    wrench_data.wrench.torque.z -= wrench_error.torque.z;
}

/// Set threshod for the wrench data
geometry_msgs::Wrench FTProcessor::setDeadzone(geometry_msgs::Wrench _wrench_data, const geometry_msgs::Wrench _Deadzone)
{
    if (fabs(_wrench_data.force.x) < _Deadzone.force.x)
        _wrench_data.force.x = 0;
    if (fabs(_wrench_data.force.y) < _Deadzone.force.y)
        _wrench_data.force.y = 0;
    if (fabs(_wrench_data.force.z) < _Deadzone.force.z)
        _wrench_data.force.z = 0;
    if (fabs(_wrench_data.torque.x) < _Deadzone.torque.x)
        _wrench_data.torque.x = 0;
    if (fabs(_wrench_data.torque.y) < _Deadzone.torque.y)
        _wrench_data.torque.y = 0;
    if (fabs(_wrench_data.torque.z) < _Deadzone.torque.z)
        _wrench_data.torque.z = 0;
    return _wrench_data;
}

/// Gravity compensation for gripper to get true force torque data
void FTProcessor::gripperbiasCompensation()
{
    /// Compensated for systemic error,params are obtained from experiments
    /// Sensor raw data when gripper z-axis is toward the ground 
    wrench_data.wrench.force.x += 1.36;
    wrench_data.wrench.force.y += 0.36;
    wrench_data.wrench.force.z += 4;
    wrench_data.wrench.torque.x -= 0.167;
    wrench_data.wrench.torque.y += 0.236;
    wrench_data.wrench.torque.z += 0;
    /// Get gravity compensated data
    wrench_force = T_sensor2base.transpose() * gripper_gravity_matrix;
    wrench_torque = gripper_bias_matrix.cross(T_sensor2base.transpose() * gripper_gravity_matrix);
    /// Compensate
    wrench_data.wrench.force.x -= wrench_force(0, 0);
    wrench_data.wrench.force.y -= wrench_force(1, 0);
    wrench_data.wrench.force.z -= wrench_force(2, 0);
    wrench_data.wrench.torque.x -= wrench_torque(0, 0);
    wrench_data.wrench.torque.y -= wrench_torque(1, 0);
    wrench_data.wrench.torque.z -= wrench_torque(2, 0);
}

/// Bias compensation for the bias of force torque sensor to get true force torque data
void FTProcessor::sensorbiasCompensation()
{
    /// Compensated for systemic error,params are obtained from experiments
    /// Sensor raw data when ATI-Delta-330 z-axis is toward the ground  
    wrench_data.wrench.force.x -= 13.16;
    wrench_data.wrench.force.y += 2.6;
    wrench_data.wrench.force.z += 8;
    wrench_data.wrench.torque.x += 0.23;
    wrench_data.wrench.torque.y += 0.99;
    wrench_data.wrench.torque.z += 0.1;
    wrench_force = T_sensor2base.transpose() * sensor_bias_matrix;
    wrench_data.wrench.force.x -= wrench_force(0, 0);
    wrench_data.wrench.force.y -= wrench_force(1, 0);
    wrench_data.wrench.force.z -= wrench_force(2, 0);
}
/// Init the FTProcessor coefficient,ros publishers and ros subscribers
void FTProcessor::init()
{
    wrench_external_old << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    Deadzone.force.x = 1;
    Deadzone.force.y = 1;
    Deadzone.force.z = 0;
    Deadzone.torque.x = 0.1;
    Deadzone.torque.y = 0.1;
    Deadzone.torque.z = 0.1;
    T_sensor2tool << 0,1,0,-1,0,0,0,0,1;
    /// Init coefficient For gravity compensation;Params are obtained from experiments
    flag = 0;
    gripper_bias_matrix << 0, 0, 0.06;
    sensor_bias_matrix << 0, 0, -3; 
    /// m*g = 2.92 * (-9.81) = -28.7
    gripper_gravity_matrix << 0, 0, -28.7;
    /// Init the subscribers and publishers
    ft_wrench_sub = n_ftp.subscribe("/ft_sensor/netft_data", 100, &FTProcessor::wrenchCallback, this);
    tool_pose_sub = n_ftp.subscribe("/ur16e/tool_pose", 100, &FTProcessor::poseCallback, this);
    ft_wrench_pub = n_ftp.advertise<geometry_msgs::WrenchStamped>("/ft_sensor/transformed_wrench_data", 125);
    /// Waiting for wrench data
    while (ft_wrench_sub.getNumPublishers())
        sleep(1);
    ROS_INFO_STREAM("Connected to the ft_publihser");
    /// Sleep 2 seconds to get stable data
    sleep(2);
}

/// Transform loop
void FTProcessor::spin()
{   
    while (ros::ok())
    {
        wrench_data = _wrench_data;
        if (wrench_data.wrench.force.x != 0.0 && wrench_data.wrench.force.y != 0.0)
        {
            Quaterniond _Q_tool2base(tool_pose.pose.orientation.w, tool_pose.pose.orientation.x, tool_pose.pose.orientation.y, tool_pose.pose.orientation.z);
            /// Get rotation matrix from sensor frame to world frame
            Q_tool2base = _Q_tool2base;
            /// Get current rotation matrix from ur16e tool frame to base frame
            T_sensor2base =  Q_tool2base.matrix() * T_sensor2tool;
            /// Bias Compensation
            //sensorbiasCompensation();
            /// Gravity Compensation 1st edition
            //gripperbiasCompensation();
            /// Transform wrench_data to base frame
            //transformWrench();
            /// Get systemic error
            if (!wrench_error.force.x && flag == 0 && ros::ok())
            {
                wrench_error = wrench_data.wrench;
                flag = 1;
            }

            /// Compensate for the zero drift
            driftCompensation(wrench_data);
            /// Filter high frequency noise
            lowpassFilter();
            /// Set threshold
            wrench_data.wrench = setDeadzone(wrench_data.wrench, Deadzone);
            /// Publish Transformed wrench data
            ft_wrench_pub.publish(wrench_data);

        }
        rate.sleep();
    }
}