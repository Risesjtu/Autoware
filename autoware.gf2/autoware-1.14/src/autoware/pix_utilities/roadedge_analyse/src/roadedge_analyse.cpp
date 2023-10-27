#include <ros/ros.h>
#include <iostream>
#include "roadedge_analyse/roadedge_analyse_core.h"


int main(int argc,char** argv)
{
    ros::init(argc, argv, "roadedge_analyse");
    roadEdgeAnaly road;
    road.MainLoop();
    return 0;
}