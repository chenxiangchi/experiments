#include <AdmittanceControl/AdmittanceControl.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_node");
    ros::NodeHandle nh;
    string ip_address;
    ros::AsyncSpinner spin(4);
    spin.start();
    nh.param<string>("ip_address", ip_address, IP_ADDRESS);
    nh.getParam("ip_address", ip_address);
    AdmittanceControl test(ip_address);
    test.Spin();

}