#include <parallel_curves/collision_detector.h>
#include <nav2_costmap_2d/cost_values.hpp>

namespace parallel_curves {

bool CollisionDetector::collisionFree(const Point& pt) 
{
  // no collision in case of no costmap loaded
  if (costmap_ == nullptr)
    return true;

  uint mx, my;
  
  if (!costmap_->worldToMap(pt[0], pt[1], mx, my))
    return false;

  if (costmap_->getCost(mx, my) != nav2_costmap_2d::FREE_SPACE)
    return false;

  return true;
}

bool CollisionDetector::collisionFree(const Point &a, const Point &b) 
{
  if (costmap_ == nullptr) 
    return true;

  if (!collisionFree(a))
    return false;

  if (!collisionFree(b))
    return false;

  uint ax, ay;
  if (!costmap_->worldToMap(a[0], a[1], ax, ay))
    return false;

  uint bx, by;
  if (!costmap_->worldToMap(b[0], b[1], bx, by))
    return false;
  
  for (const auto& p : bresenham({ax, ay}, {bx, by})) 
    if (costmap_->getCost(p[0], p[1]) != nav2_costmap_2d::FREE_SPACE)
      return false;

  return true;
}

std::vector<std::array<int, 2>> CollisionDetector::bresenham(const std::array<uint, 2>& start, const std::array<uint, 2>& end) 
{
  int dx = end[0] - start[0];
  int dy = end[1] - start[1];
  int dx1 = std::abs(dx);
  int dy1 = std::abs(dy);
  int px = 2 * dy1 - dx1;
  int py = 2 * dx1 - dy1;
  int x, y, xe, ye;

  if (dy1 <= dx1) 
  {
      if (dx >= 0) 
      {
          x = start[0];
          y = start[1];
          xe = end[0];
      } 
      else 
      {
          x = end[0];
          y = end[1];
          xe = start[0];
      }

      std::array<int, 2> pt = {x, y};
      std::vector<std::array<int, 2>> points;
      points.push_back(pt);

      for (int i = 0; x < xe; ++i) 
      {
          x = x + 1;
          if (px < 0) 
          {
              px = px + 2 * dy1;
          } 
          else 
          {
              if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0)) 
              {
                  y = y + 1;
              } 
              else 
              {
                  y = y - 1;
              }
              px = px + 2 * (dy1 - dx1);
          }
          pt = {x, y};
          points.push_back(pt);
      }
      return points;
  } 
  else 
  {
      if (dy >= 0) 
      {
          x = start[0];
          y = start[1];
          ye = end[1];
      } 
      else 
      {
          x = end[0];
          y = end[1];
          ye = start[1];
      }

      std::array<int, 2> pt = {x, y};
      std::vector<std::array<int, 2>> points;
      points.push_back(pt);

      for (int i = 0; y < ye; ++i) 
      {
          y = y + 1;
          if (py <= 0) 
          {
              py = py + 2 * dx1;
          } 
          else 
          {
              if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0)) 
              {
                  x = x + 1;
              } 
              else 
              {
                  x = x - 1;
              }
              py = py + 2 * (dx1 - dy1);
          }
          pt = {x, y};
          points.push_back(pt);
      }
      return points;
  }
}

void CollisionDetector::clearRobotCell(const Point& start)
{
  unsigned int mx, my;
  
  if (!costmap_->worldToMap(start[0], start[1], mx, my))
    return;

  //set the associated costs in the cost map to be free
  costmap_->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
}

} 
