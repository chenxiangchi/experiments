/**
 * @file ft_main.cpp
 * @brief Program entrance
 * @mainpage FT Sensor
 * @author Xiangchi Chen
 * @email chenxiangchi@zju.edu.cn
 * @version 1.1.0
 * @date 11.17 2020
 */
#include <ft_transfrom/ft_tool.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Force_torque_sensor_node");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    FTProcessor ft_node;
    ft_node.spin();
}