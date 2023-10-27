#include <ros/ros.h>
#include <iostream>
#include "scenario_manager/scenario_manager_core.h"


int main(int argc,char** argv)
{
    ros::init(argc, argv, "scenario_manager");
    ScenarioManager scenario_manager;
    scenario_manager.MainLoop();
    return 0;
}