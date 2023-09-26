#pragma once

#include <nav2_costmap_2d/costmap_2d.hpp>

#include <utility>


namespace parallel_curves {

class CollisionDetector {
typedef std::array<double,2> Point;
 public:
  explicit CollisionDetector(nav2_costmap_2d::Costmap2D* costmap) : costmap_(costmap) {};

  bool collisionFree(const Point& pt);

  bool collisionFree(const Point &pt1, const Point &pt2);

  void clearRobotCell(const Point& start);
 private:
  std::vector<std::array<int, 2>> bresenham(const std::array<uint, 2>& start, const std::array<uint, 2>& end);
 private:
  nav2_costmap_2d::Costmap2D* costmap_{nullptr};
};

}  