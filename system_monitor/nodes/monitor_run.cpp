#include "monitor.h"
#include <chrono>
#include <ctime>

using namespace std;

void enumerate_ports()
{
    vector<serial::PortInfo> devices_found = serial::list_ports();
    vector<serial::PortInfo>::iterator iter = devices_found.begin();
    while( iter != devices_found.end() )
    {
        serial::PortInfo device = *iter++;
        ROS_INFO( "(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),device.hardware_id.c_str() );
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "monitor_node");
    
    ros::AsyncSpinner spinner(2);
    spinner.start();

    Monitor monitor;
    
    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        monitor.run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
