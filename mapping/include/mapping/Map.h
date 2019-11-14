//
// Created by naivehobo on 11/9/19.
//

#ifndef MAPPING_MAP_H
#define MAPPING_MAP_H

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


enum BrushState {
  READY,
  ACTIVE,
  DELETE,
  MAP,
  POSE,
  GOAL
};

struct Brush {
  std::string window;
  cv::Mat *canvas;
  cv::Scalar *color;
  cv::Point pos;
  int width;
  BrushState state;
};


class Map {

 public:

  Map();
  ~Map();

  void markObstacles();

  void markStartingPosition();

  void markGoal();

  void markWaypoint(int x, int y);

 private:

  void publishMap();
  void publishGoal();
  void publishPose();

  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal);

  static void onMouse(int event, int x, int y, int flags, void* ptr);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher map_pub_;
  ros::Publisher goal_pub_;
  ros::Publisher pose_pub_;

  ros::Subscriber goal_sub_;

  std::string map_topic_;
  std::string pose_topic_;
  std::string goal_topic_;

  std::string save_map_;

  cv::Mat map_;
  std::string window_;

  geometry_msgs::Pose2D pose_;
  geometry_msgs::Pose2D goal_;

  int height_;
  int width_;
  float resolution_;
};

#endif //MAPPING_MAP_H
