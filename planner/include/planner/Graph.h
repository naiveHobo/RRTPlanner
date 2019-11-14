//
// Created by naivehobo on 11/12/19.
//

#ifndef PLANNER_GRAPH_H
#define PLANNER_GRAPH_H

#include <vector>
#include <memory>

class Vertex {

 public:

  Vertex() {
    x = 0;
    y = 0;
  };

  Vertex(int x, int y) {
    this->x = x;
    this->y = y;
  }

  bool operator==(const Vertex &rhs) {
    return (rhs.x == x && rhs.y == y);
  }

  bool operator!=(const Vertex &rhs) {
    return !(*this == rhs);
  }

  int x;
  int y;
};

class Node {

 public:

  Node() {
    root_ = Vertex(0, 0);
    links_.resize(0);
  }

  explicit Node(const Vertex &root) {
    root_ = Vertex(root);
    links_.resize(0);
  }

  std::shared_ptr<Node> add(const Vertex &link) {
    auto node = find(link);
    if (node == nullptr) {
      node.reset(new Node(link));
      links_.push_back(node);
    }
    return node;
  }

  std::shared_ptr<Node> add(const std::shared_ptr<Node> &link) {
    auto node = find(link->getRoot());
    if (node == nullptr) {
      node = link;
      links_.push_back(node);
    }
    return node;
  }

  std::shared_ptr<Node> find(const Vertex &vertex) {
    for (auto &node : links_) {
      if (node->getRoot() == vertex)
        return node;
    }
    return nullptr;
  }

  Vertex getRoot() {
    return root_;
  }

  std::vector<std::shared_ptr<Node>> getLinks() {
    return links_;
  }

  bool operator==(const Node &rhs) {
    return (this->root_ == rhs.root_);
  }

 private:

  Vertex root_;
  std::vector<std::shared_ptr<Node>> links_;

};

class Graph {

 public:

  Graph();

  std::shared_ptr<Node> add(const Vertex &vertex);

  void add(const Vertex &src, const Vertex &dest);

  std::shared_ptr<Node> find(const Vertex &vertex);

  std::shared_ptr<Node> find(const std::shared_ptr<Node> &node);

  int getSize();

  std::vector<std::shared_ptr<Node>> getNodes();

  bool getPath(std::shared_ptr<Node> src, std::shared_ptr<Node> dest, std::vector<Vertex>& path);

 private:

  std::vector<std::shared_ptr<Node>> nodes_;

  int size_;

};

#endif //PLANNER_GRAPH_H
