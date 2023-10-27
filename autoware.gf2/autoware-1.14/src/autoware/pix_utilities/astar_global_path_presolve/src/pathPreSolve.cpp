#include <ros/ros.h>
#include <iostream>
#include "astar_global_path_presolve/pathPreSolve_core.h"

int main(int argc, char **argv){
  ros::init(argc,argv,"pathPreSolve");
  pathPreSolve path_solve;
  path_solve.MainLoop();  // 主循环
  return 0;
  
}