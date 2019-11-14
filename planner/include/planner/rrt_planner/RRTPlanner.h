//
// Created by naivehobo on 11/7/19.
//

#ifndef RRT_PLANNER_RRTPLANNER_H
#define RRT_PLANNER_RRTPLANNER_H

#include <ros/ros.h>

#include <planner/Planner.h>
#include <planner/Graph.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>
#include <memory>
#include <random>
#include <cmath>


class RRTPlanner : public Planner {

 public:

  RRTPlanner();

  Vertex findClosestPoint(Vertex point);

  bool findInPoints(std::vector<Vertex> &newPoints, Vertex point);

  std::vector<Vertex> connectPoints(Vertex a, Vertex b);

  void addToGraph(std::vector<Vertex>& points, Vertex point);

  std::vector<Vertex> getPlan() override;

  std::vector<Vertex> getPath(Vertex src, Vertex dest);

 private:

  std::shared_ptr<Graph> graph_;

  double euclideanDistance(Vertex& a, Vertex& b);

  bool isPointOccupied(Vertex v);

  int max_vertices_;
  int step_;

};

#endif //RRT_PLANNER_RRTPLANNER_H
