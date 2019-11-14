//
// Created by naivehobo on 11/9/19.
//

#include "mapping/Map.h"

Map::Map() : private_nh_("~") {

  private_nh_.param<std::string>("map_topic", map_topic_, "map");
  private_nh_.param<std::string>("pose_topic", pose_topic_, "pose");
  private_nh_.param<std::string>("goal_topic", goal_topic_, "goal");
  private_nh_.param("height", height_, 500);
  private_nh_.param("width", width_, 500);
  private_nh_.param("resolution", resolution_, 0.1f);
  private_nh_.param<std::string>("save_map", save_map_, "none");

  std::string map_path;
  private_nh_.param<std::string>("load_map", map_path, "none");

  window_ = "Map";

  if(map_path == "none")
    map_ = cv::Mat::zeros(height_, width_, CV_8UC3);
  else {
    map_ = cv::imread(map_path);
    height_ = map_.rows;
    width_ = map_.cols;
    ROS_INFO("Loaded map from: %s", map_path.c_str());
  }

  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(map_topic_, 1);
  goal_pub_ = nh_.advertise<geometry_msgs::Pose2D>(goal_topic_, 1);
  pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>(pose_topic_, 1);

  goal_sub_ = nh_.subscribe("move_base_simple/goal", 1, &Map::goalCallback, this);

  ROS_INFO("Mapping node initialized");

  while(ros::ok()) {
    markObstacles();
  }
}

Map::~Map() {
  cv::destroyWindow(window_);
}

void Map::markObstacles() {

  ROS_INFO("Mark obstacles on the map");
  ROS_INFO("Press 'q' to quit");
  ROS_INFO("Press 's' to save and move on");
  ROS_INFO("Press numbers to the keyboard to change width of paint brush");
  ROS_INFO("Right-click to toggle between eraser and paint brush");

  char key;

  Brush brush;
  brush.canvas = &map_;
  brush.window = window_;
  brush.state = BrushState::READY;
  brush.color = new cv::Scalar(255, 255, 255);
  brush.width = 5;

  cv::namedWindow(window_);
  cv::setMouseCallback(window_, onMouse, &brush);

  do {
    cv::imshow(window_, map_);
    key = (char) (cv::waitKey(0) & 0xFF);

    if(key == 'q') {
      ros::shutdown();
      return;
    }

    for (int i = 1; i < 10; i++) {
      if (key - '0' == i) {
        brush.width = i;
        break;
      }
    }
  } while (key != 's');

  if(save_map_ != "none") {
    cv::imwrite(save_map_, map_);
    ROS_INFO("Saved map as: %s", save_map_.c_str());
  }

  publishMap();
  markStartingPosition();
}

void Map::markStartingPosition() {

  Brush brush;
  brush.canvas = &map_;
  brush.window = window_;
  brush.state = BrushState::POSE;
  brush.color = new cv::Scalar(0, 255, 0);
  brush.width = 4;

  cv::setMouseCallback(window_, onMouse, &brush);

  while(brush.state == BrushState::POSE) {
    cv::imshow(window_, map_);
    cv::waitKey(100);
  }

  pose_.x = brush.pos.x;
  pose_.y = brush.pos.y;

  publishPose();
  markGoal();
}

void Map::markGoal() {

  Brush brush;
  brush.canvas = &map_;
  brush.window = window_;
  brush.state = BrushState::GOAL;
  brush.color = new cv::Scalar(0, 0, 255);
  brush.width = 4;

  cv::setMouseCallback(window_, onMouse, &brush);

  while(brush.state == BrushState::GOAL) {
    cv::imshow(window_, map_);
    cv::waitKey(100);
  }

  goal_.x = brush.pos.x;
  goal_.y = brush.pos.y;

  publishGoal();
}

void Map::markWaypoint(int x, int y) {
  cv::circle(map_, cv::Point(x, y), 2, cv::Scalar(255, 0, 0), -1);
  cv::imshow(window_, map_);
}

void Map::onMouse(int event, int x, int y, int, void *ptr) {

  auto brush = (Brush *) ptr;

  switch (event) {
    case CV_EVENT_LBUTTONDOWN:
      if (brush->state == BrushState::READY || brush->state == BrushState::DELETE) {
        brush->state = BrushState::ACTIVE;
        brush->pos.x = x;
        brush->pos.y = y;
        cv::circle(*brush->canvas, cv::Point(x, y), brush->width, *brush->color, -1);
        cv::imshow(brush->window, *brush->canvas);
      }
      else if (brush->state == BrushState::POSE) {
        brush->state = BrushState::GOAL;
        brush->pos.x = x;
        brush->pos.y = y;
        cv::circle(*brush->canvas, cv::Point(x, y), brush->width, *brush->color, -1);
        cv::imshow(brush->window, *brush->canvas);
      }
      else if (brush->state == BrushState::GOAL) {
        brush->state = BrushState::MAP;
        brush->pos.x = x;
        brush->pos.y = y;
        cv::circle(*brush->canvas, cv::Point(x, y), brush->width, *brush->color, -1);
        cv::imshow(brush->window, *brush->canvas);
      }
      break;
    case CV_EVENT_LBUTTONUP:
      if (brush->state == BrushState::ACTIVE) {
        if (brush->color->val[0] == 0)
          brush->state = BrushState::DELETE;
        else
          brush->state = BrushState::READY;
      }
      break;
    case CV_EVENT_MOUSEMOVE:
      if (brush->state == BrushState::ACTIVE) {
        cv::line(*brush->canvas, brush->pos, cv::Point(x, y), *brush->color, brush->width);
        brush->pos.x = x;
        brush->pos.y = y;
        cv::imshow(brush->window, *brush->canvas);
      }
      break;
    case CV_EVENT_RBUTTONDOWN:
      if (brush->state == BrushState::READY) {
        brush->state = BrushState::DELETE;
        brush->color = new cv::Scalar(0, 0, 0);
      } else {
        brush->state = BrushState::READY;
        brush->color = new cv::Scalar(255, 255, 255);
      }
      break;
    default:break;
  }
}

void Map::publishMap() {

  nav_msgs::OccupancyGrid grid;

  grid.info.map_load_time = ros::Time::now();
  grid.header.frame_id = "map";
  grid.header.stamp = ros::Time::now();

  grid.info.width = (unsigned int) width_;
  grid.info.height = (unsigned int) height_;
  grid.info.resolution = resolution_;

  grid.info.origin.position.x = -width_ * resolution_ / 2.0;
  grid.info.origin.position.y = -height_ * resolution_ / 2.0;
  grid.info.origin.position.z = 0.0;

  grid.info.origin.orientation.x = 0.0;
  grid.info.origin.orientation.y = 0.0;
  grid.info.origin.orientation.z = 0.0;
  grid.info.origin.orientation.w = 1.0;

  grid.data.resize(grid.info.width * grid.info.height);

  for(unsigned int i = 0; i < grid.info.height; i++) {
    for (unsigned int j = 0; j < grid.info.width; j++) {
      int val = map_.at<cv::Vec3b>(i, j).val[0] > 0 ? 100 : 0;
      grid.data[i * grid.info.width + j] = (char) val;
    }
  }

  map_pub_.publish(grid);
  ROS_INFO("Published map!");
}

void Map::publishPose() {
  pose_pub_.publish(pose_);
  ROS_INFO("Published starting pose: (%lf, %lf)", pose_.x, pose_.y);
}

void Map::publishGoal() {
  goal_pub_.publish(goal_);
  ROS_INFO("Published goal: (%lf, %lf)", goal_.x, goal_.y);
}

void Map::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal) {
  goal_.x = (goal->pose.position.x * 2.0 / resolution_) + (width_ / 2.0);
  goal_.y = (goal->pose.position.y * 2.0 / resolution_) + (height_ / 2.0);
  ROS_INFO("Recieved goal: (%lf, %lf)", goal->pose.position.x, goal->pose.position.y);
  cv::circle(map_, cv::Point((int)goal_.x, (int)goal_.y), 4, cv::Scalar(0, 0, 255), -1);
  cv::imshow(window_, map_);
  cv::waitKey(100);
  publishGoal();
}
