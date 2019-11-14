//
// Created by naivehobo on 11/7/19.
//

#include <ros/ros.h>

#include "planner/rrt_planner/RRTPlanner.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "planner_node");

  auto planner = RRTPlanner();

  ros::spin();

  return 0;
}