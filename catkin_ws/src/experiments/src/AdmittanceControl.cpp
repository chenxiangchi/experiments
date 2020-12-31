#include <AdmittanceControl/AdmittanceControl.h>

void AdmittanceControl::Init()
{
    transformed_wrench_data_sub = nh.subscribe("/ft_sensor/transformed_wrench_data", 1, &AdmittanceControl::TransformedWrenchCallback, this);
    raw_wrench_data_sub = nh.subscribe("/ft_sensor/netft_data", 1, &AdmittanceControl::RawWrenchCallback, this);
    timer = 1 / LOOP_RATE;
    expected_wrench_data << 0.0, 0.0, -5.0 , 0.0, 0.0, 0.0;
    M_inv_matrix << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    D_matrix << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 40.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    tool_twist_pub = nh.advertise<geometry_msgs::TwistStamped>("/ur16e_tool_twist", 2);
    tool_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ur16e_tool_pose", 2);
    joint_state_pub = nh.advertise<sensor_msgs::JointState>("/ur16e_joint_states", 2);

    vector<double> _origin_q = {-0.020349804555074513, -2.3013864956297816, -2.1782941818237305, -0.23279936731372075, 1.5696759223937988, -3.157480780278341};
    origin_q.assign(_origin_q.begin(), _origin_q.end());
    current_tool_twist.setZero();
    current_tool_acc.setZero();
    expected_tool_twist.setZero();
    expected_tool_acc.setZero();
    sigma_temp = 1000;
}

void AdmittanceControl::TempComp()
{
    compensated_wrench_data.setZero();
    for (int i = 0; i < sigma_temp; i++)
        compensated_wrench_data += _current_filtered_wrench_data;
    compensated_wrench_data = compensated_wrench_data * (1 / sigma_temp);
}

void AdmittanceControl::TransformedWrenchCallback(const geometry_msgs::WrenchStamped &input)
{
    _current_filtered_wrench_data << input.wrench.force.x, input.wrench.force.y, input.wrench.force.z,
        input.wrench.torque.x, input.wrench.torque.y, input.wrench.torque.z;
     /// Current tool speed
    twist.header.frame_id = "base_link";
    twist.header.stamp = ros::Time::now();
    twist.twist.linear.x = receiver.getActualTCPSpeed()[0];
    twist.twist.linear.y = receiver.getActualTCPSpeed()[1];
    twist.twist.linear.z = receiver.getActualTCPSpeed()[2] * 300;
    twist.twist.angular.x = receiver.getActualTCPSpeed()[3];
    twist.twist.angular.y = receiver.getActualTCPSpeed()[4];
    twist.twist.angular.z = receiver.getActualTCPSpeed()[5];

    /// Current tool pose
    pose.header.frame_id = "base_link";
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = receiver.getActualTCPPose()[0];
    pose.pose.position.y = receiver.getActualTCPPose()[1];
    pose.pose.position.z = receiver.getActualTCPPose()[2] * 100;
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

}

void AdmittanceControl::RawWrenchCallback(const geometry_msgs::WrenchStamped &input)
{
    _current_raw_wrench_data << input.wrench.force.x, input.wrench.force.y, input.wrench.force.z,
        input.wrench.torque.x, input.wrench.torque.y, input.wrench.torque.z;
}
MatrixXd AdmittanceControl::Vector2Eigen(vector<double> input)
{
    int row = input.size();
    Matrix<double, Dynamic, 1> temp_matrix;
    temp_matrix.resize(row, 1);
    for (int i = 0; i < input.size(); i++)
        temp_matrix(i, 0) = input[i];
    return temp_matrix;
}

void AdmittanceControl::UpdateData()
{
    /// Get current tool twist
    current_tool_twist = Vector2Eigen(receiver.getActualTCPSpeed());

    current_raw_wrench_data = _current_raw_wrench_data;

    current_filtered_wrench_data = _current_filtered_wrench_data;

    current_tool_pose = Vector2Eigen(receiver.getActualTCPPose());

    //cout << "current_tool_twist:\n"
        // << current_tool_twist << endl;
    //cout << "raw wrench data:\n" << current_raw_wrench_data << endl;
    //cout << "transformed wrench data:\n" << current_filtered_wrench_data << endl;
}

void AdmittanceControl::Execute()
{
    Matrix<double, 6, 1> temp_wrench;

    temp_wrench = current_filtered_wrench_data - compensated_wrench_data;

    current_tool_acc = M_inv_matrix * ((expected_wrench_data - temp_wrench) - D_matrix * (current_tool_twist - expected_tool_twist));
    current_tool_twist = current_tool_twist + current_tool_acc * timer;
    vector<double> cmd = {0.0, 0.0, current_tool_twist(2, 0), 0.0, 0.0, 0.0};
    controller.speedL(cmd, 1.0, 0.0);
}

void AdmittanceControl::Spin()
{

    while (ros::ok())
    {
        controller.speedStop();
        sleep(1);
        controller.moveJ(origin_q, 0.1, 1);
        sleep(2);
        /// Naming rule is Color_Diameter(mm)_Length(mm)_Quantity
        string home_path = HOME_PATH;
        cout << "Please enter the type of target:"
             << endl;
        string file_name;
        cin >> file_name;
        cout << "The name of target is: " << file_name << endl;
        string exp_files = home_path + "/admittance_data/" + file_name + ".txt";
        std::ofstream recorder(exp_files.c_str());
        recorder << "Type:" << file_name << "\n"
                 << "Data:" << endl;
        long long unsigned int count = 0;
        /// Compensate for temperature drift
        TempComp();

        UpdateData();
        int count2 = 0 ;
        
        while (fabs((current_filtered_wrench_data - compensated_wrench_data)(2, 0)) < 0.03 && ros::ok() || count2 < 200)
        {std::cout << compensated_wrench_data(2,0) << std::endl;
            UpdateData();
            vector<double> cmd = {0.0, 0.0, -0.005, 0.0, 0.0, 0.0};
            controller.speedL(cmd, 1.0, 0.0);
            loop_rate.sleep();
            count2 ++;
        }

        while (ros::ok())
        {

            auto t_start = high_resolution_clock::now();
            UpdateData();
            Execute();
            auto t_stop = high_resolution_clock::now();
            auto t_duration = std::chrono::duration<double>(t_stop - t_start);
            if (t_duration.count() < timer)
            {
                std::this_thread::sleep_for(std::chrono::duration<double>(timer - t_duration.count()));
            }

            recorder << "Num" << count << endl;
            recorder << "Tool_pose_z: "
                     << current_tool_pose(2, 0) << endl;
            recorder << "Tool twist_z: "
                     << current_tool_twist(2, 0) << endl;

            recorder << "Tool_acc_z: "
                     << current_tool_acc(2, 0) << endl;
            recorder << "Wrench data_z: "
                     << current_raw_wrench_data(2, 0) << endl;
            count++;
            if (count > 1000)
                break;
            //loop_rate.sleep();
        }
    }
}
