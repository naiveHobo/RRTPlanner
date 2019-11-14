//
// Created by naivehobo on 11/10/19.
//

#include "planner/Planner.h"


Planner::Planner() : private_nh_("~") {
  private_nh_.param<std::string>("map_topic", map_topic_, "map");
  private_nh_.param<std::string>("pose_topic", pose_topic_, "pose");
  private_nh_.param<std::string>("goal_topic", goal_topic_, "goal");

  pose_set_ = false;

  map_sub_ = nh_.subscribe(map_topic_, 1, &Planner::mapCallback, this);
  goal_sub_ = nh_.subscribe(goal_topic_, 1, &Planner::goalCallback, this);
  pose_sub_ = nh_.subscribe(pose_topic_, 1, &Planner::poseCallback, this);

  ROS_INFO("Planner node initialized");
}

void Planner::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map) {
  map_ = cv::Mat::zeros(map->info.height, map->info.width, CV_8UC3);
  for (int i = 0; i < map->info.height; i++) {
    for (int j = 0; j < map->info.width; j++) {
      unsigned char value = map->data[i * map->info.width + j] != 0 ? (unsigned char) 255 : (unsigned char) 0;
      map_.at<cv::Vec3b>(i, j) = cv::Vec3b(value, value, value);
    }
  }
  map_.copyTo(display_map_);
  ROS_INFO("Received map");
}

void Planner::goalCallback(const geometry_msgs::Pose2D::ConstPtr &goal) {
  goal_.x = (int) goal->x;
  goal_.y = (int) goal->y;
  ROS_INFO("Received goal: (%d, %d)", goal_.x, goal_.y);
  if(pose_set_ && !map_.empty())
    getPlan();
}

void Planner::poseCallback(const geometry_msgs::Pose2D::ConstPtr &pose) {
  pose_.x = (int) pose->x;
  pose_.y = (int) pose->y;
  pose_set_ = true;
  ROS_INFO("Received goal: (%d, %d)", pose_.x, pose_.y);
}
