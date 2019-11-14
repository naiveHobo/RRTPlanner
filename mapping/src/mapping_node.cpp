//
// Created by naivehobo on 11/9/19.
//

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "mapping/Map.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "mapping_node");

  auto map = Map();

  ros::spin();

  return 0;
}