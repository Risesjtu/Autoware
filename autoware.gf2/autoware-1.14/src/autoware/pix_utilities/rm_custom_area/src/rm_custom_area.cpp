#include <ros/ros.h>
#include <iostream>
#include "rm_custom_area/rm_custom_area_core.h"


int main(int argc,char** argv)
{
    ros::init(argc, argv, "rm_custom_area");
    KillStopLine kill_stop_line;
    kill_stop_line.MainLoop();
    return 0;
}