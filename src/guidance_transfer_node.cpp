//
// Created by kehan on 19-6-19.
//

#include "ros/ros.h"
#include "DJIGuidanceROSHardware.h"

using namespace std;
using namespace vwpp;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "guidance_transfer_node");

    DJIGuidanceROSHardware dji_guidance_ros_hardware;

    if (dji_guidance_ros_hardware.Prepare() != 0)
    {
        return -1;
    }

    while (ros::ok())
    {
        dji_guidance_ros_hardware.Run();
        ros::spinOnce();
    }

    if (dji_guidance_ros_hardware.Release() != 0)
    {
        return -1;
    }

    ros::Duration(0.1).sleep();
    return 0;
}
