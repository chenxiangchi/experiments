#include <AdmittanceControl/AdmittanceControl.h>

void AdmittanceControl::Init()
{
    wrench_data_sub = nh.subscribe("transformed_wrench_data",1,&WrenchCallback,this);
    timer = 1 / LOOP_RATE;
}   

void AdmittanceControl::WrenchCallback(const geometry_msgs::WrenchStamped &input)
{
    current_wrench_data << input.wrench.force.x,input.wrench.force.y,input.wrench.force.z,
                            input.wrench.torque.x,input.wrench.torque.y,input.wrench.torque.z;
}

MatrixXd AdmittanceControl::Vector2Eigen(std::vector<double> input)
{

    Matrix<double, Dynamic, 1> temp_matrix;

    for (int i = 0; i <= input.size(); i++)
        temp_matrix(i, 1) = input[i];
    return temp_matrix;
}

void AdmittanceControl::UpdateData()
{
    current_tool_twist = Vector2Eigen(receiver.getActualTCPSpeed());
}

void AdmittanceControl::Execute()
{
    current_tool_acc = M_inv_matrix * ((expected_wrench_data - current_wrench_data) - D_matrix * (current_tool_twist - expected_tool_twist));
    current_tool_twist = current_tool_twist + current_tool_acc * timer; 
}

void AdmittanceControl::Spin()
{

}
