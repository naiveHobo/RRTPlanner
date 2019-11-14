//
// Created by naivehobo on 11/7/19.
//

#include "planner/rrt_planner/RRTPlanner.h"

RRTPlanner::RRTPlanner() {
  private_nh_.param("max_vertices", max_vertices_, 1500);
  private_nh_.param("step_size", step_, 20);

  cv::namedWindow("Path");
}

Vertex RRTPlanner::findClosestPoint(Vertex point) {
  auto minDist = DBL_MAX;
  double dist;
  Vertex closest;

  for (auto &node : graph_->getNodes()) {

    auto root = node->getRoot();

    if (root == point)
      continue;

    dist = euclideanDistance(root, point);

    if (dist < minDist) {
      minDist = dist;
      closest = root;
    }
  }

  return closest;
}

std::vector<Vertex> RRTPlanner::connectPoints(Vertex a, Vertex b) {
  std::vector<Vertex> newPoints;
  newPoints.emplace_back(b);

  cv::LineIterator linePoints(display_map_, cv::Point(b.x, b.y), cv::Point(a.x, a.y));
  cv::Point point;
  Vertex v;
  bool blocked = false;

  for (int i = 0; i < linePoints.count; i++, linePoints++) {
    point = linePoints.pos();

    v.x = point.x;
    v.y = point.y;

    if (isPointOccupied(v)) {
      blocked = true;
      break;
    }

    if (euclideanDistance(v, newPoints[newPoints.size() - 1]) > step_)
      newPoints.emplace_back(v);
  }

  if (!blocked) {
    if (newPoints[newPoints.size() - 1] != a)
      newPoints.emplace_back(a);
  }

  return newPoints;
}

bool RRTPlanner::findInPoints(std::vector<Vertex> &points, Vertex point) {
  for (auto &p : points) {
    if (p == point)
      return true;
  }
  return false;
}

void RRTPlanner::addToGraph(std::vector<Vertex> &newPoints, Vertex point) {
  if (newPoints.size() > 1) {
    int p = 0;
    for (p = 0; p < newPoints.size() - 1; p++) {
      graph_->add(newPoints[p], newPoints[p + 1]);

      if (p != 0)
        cv::circle(display_map_, cv::Point(newPoints[p].x, newPoints[p].y), 1, cv::Scalar(0, 255, 255), -1);

      cv::line(display_map_,
               cv::Point(newPoints[p].x, newPoints[p].y),
               cv::Point(newPoints[p + 1].x, newPoints[p + 1].y),
               cv::Scalar(255, 0, 0),
               1);
    }

    if (findInPoints(newPoints, point))
      cv::circle(display_map_, cv::Point(point.x, point.y), 1, cv::Scalar(0, 255, 0), -1);
    else
      cv::circle(display_map_, cv::Point(newPoints[p + 1].x, newPoints[p + 1].y), 1, cv::Scalar(255, 0, 0), -1);
  }
}

std::vector<Vertex> RRTPlanner::getPath(Vertex src, Vertex dest) {
  auto srcNode = graph_->find(src);
  auto destNode = graph_->find(dest);
  std::vector<Vertex> path;
  graph_->getPath(srcNode, destNode, path);
  std::reverse(path.begin(), path.end());
  return path;
}

std::vector<Vertex> RRTPlanner::getPlan() {

  cv::circle(display_map_, cv::Point(pose_.x, pose_.y), 3, cv::Scalar(0, 255, 0), -1);
  cv::circle(display_map_, cv::Point(goal_.x, goal_.y), 3, cv::Scalar(0, 0, 255), -1);

  std::vector<Vertex> newPoints;

  // Check if goal is reachable from starting position
  newPoints = connectPoints(goal_, pose_);
  if (findInPoints(newPoints, goal_)) {
    for (int i = 0; i < newPoints.size() - 1; i++) {
      cv::line(display_map_,
               cv::Point(newPoints[i].x, newPoints[i].y),
               cv::Point(newPoints[i + 1].x, newPoints[i + 1].y),
               cv::Scalar(0, 0, 255),
               1);
      if (i != 0)
        cv::circle(display_map_, cv::Point(newPoints[i].x, newPoints[i].y), 1, cv::Scalar(0, 255, 255), -1);
    }
    cv::imshow("Path", display_map_);
    cv::waitKey(1);
    return newPoints;
  }

  cv::circle(display_map_, cv::Point(pose_.x, pose_.y), 3, cv::Scalar(0, 255, 0), -1);
  cv::circle(display_map_, cv::Point(goal_.x, goal_.y), 3, cv::Scalar(0, 0, 255), -1);

  graph_.reset(new Graph());
  graph_->add(pose_);

  // random integer generator
  std::random_device device;
  std::mt19937 eng(device());

  std::uniform_int_distribution<int> distX(0, display_map_.cols - 1);
  std::uniform_int_distribution<int> distY(0, display_map_.rows - 1);

  bool occupied;

  int totalPoints = 0;

  Vertex randomPoint;
  Vertex closestPoint;

  while (graph_->find(goal_) == nullptr && graph_->getSize() < max_vertices_ && totalPoints < 30000) {

    if (totalPoints % 100 == 0)
      ROS_INFO("Random points generated: %d", totalPoints);

    // keep sampling till sampled point is not occupied by an obstacle
    occupied = true;
    while (occupied) {
      randomPoint.x = distX(eng);
      randomPoint.y = distY(eng);
      if (!isPointOccupied(randomPoint) && graph_->find(randomPoint) == nullptr)
        occupied = false;
    }

    if (totalPoints == 500) {
      randomPoint.x = 10;
      randomPoint.y = 10;
    }

    // find closest point in the graph to the sampled point
    closestPoint = findClosestPoint(randomPoint);
    newPoints = connectPoints(randomPoint, closestPoint);
    addToGraph(newPoints, randomPoint);

    totalPoints += 1;

    closestPoint = findClosestPoint(goal_);
    newPoints = connectPoints(goal_, closestPoint);
    addToGraph(newPoints, goal_);

    cv::imshow("Path", display_map_);
    cv::waitKey(1);
  }

  std::vector<Vertex> path;
  if (graph_->find(goal_) != nullptr) {
    ROS_INFO("Goal found!");
    path = getPath(pose_, goal_);
    for (int i = 0; i < path.size() - 1; i++)
      cv::line(display_map_, cv::Point(path[i].x, path[i].y), cv::Point(path[i+1].x, path[i+1].y), cv::Scalar(0, 0, 255), 1);
  } else
    ROS_INFO("Could not reach goal");

  ROS_INFO("Total vertex in graph: %d", graph_->getSize());

  cv::imshow("Path", display_map_);
  cv::waitKey(100);

  return path;
}

double RRTPlanner::euclideanDistance(Vertex &a, Vertex &b) {
  return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

bool RRTPlanner::isPointOccupied(Vertex v) {
  cv::Vec3b vec = display_map_.at<cv::Vec3b>(v.y, v.x);
  return (vec.val[0] == 255 && vec.val[1] == 255 && vec.val[2] == 255);
}
