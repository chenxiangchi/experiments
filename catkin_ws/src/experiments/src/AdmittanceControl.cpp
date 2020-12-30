#include <AdmittanceControl/AdmittanceControl.h>

void AdmittanceControl::Init()
{
    transformed_wrench_data_sub = nh.subscribe("/ft_sensor/transformed_wrench_data", 1, &AdmittanceControl::TransformedWrenchCallback, this);
    raw_wrench_data_sub = nh.subscribe("/ft_sensor/netft_data", 1, &AdmittanceControl::RawWrenchCallback, this);
    timer = 1 / LOOP_RATE;
    expected_wrench_data << 0.0, 0.0, -1.0, 0.0, 0.0, 0.0;
    M_inv_matrix << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    D_matrix << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 40.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    vector<double> _origin_q = {-0.020349804555074513, -2.0127326450743617, -2.1184072494506836, -0.5813568395427247, 1.5691380500793457, -3.1572888533221644};
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
    for(int i = 0; i < sigma_temp; i++)
        compensated_wrench_data += _current_raw_wrench_data; 
    compensated_wrench_data = compensated_wrench_data * (1 / sigma_temp);
}

void AdmittanceControl::TransformedWrenchCallback(const geometry_msgs::WrenchStamped &input)
{
    _current_filtered_wrench_data << input.wrench.force.x, input.wrench.force.y, input.wrench.force.z,
        input.wrench.torque.x, input.wrench.torque.y, input.wrench.torque.z;
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

    //cout <<"current_tool_twist:\n" << current_tool_twist<<endl;
    //cout << "raw wrench data:\n" << current_raw_wrench_data << endl;
    //cout << "transformed wrench data:\n" << current_filtered_wrench_data << endl;
}

void AdmittanceControl::Execute()
{
    current_tool_acc = M_inv_matrix * ((expected_wrench_data - (current_raw_wrench_data - compensated_wrench_data)) - D_matrix * (current_tool_twist - expected_tool_twist));
    current_tool_twist = current_tool_twist + current_tool_acc * timer;
    vector<double> cmd = {0.0, 0.0, current_tool_twist(2, 0), 0.0, 0.0, 0.0};
    //controller.speedL(cmd,1.0,0.0);
}

void AdmittanceControl::Spin()
{

    while (ros::ok())
    {
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
        TempComp();
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
            count ++;
            //std::cout << (current_raw_wrench_data - compensated_wrench_data) << std::endl;
            if(fabs(current_raw_wrench_data(2,0) - compensated_wrench_data(2,0)) > 0.17)
                cout <<count<<endl;
            //loop_rate.sleep();
        }
    }
}
