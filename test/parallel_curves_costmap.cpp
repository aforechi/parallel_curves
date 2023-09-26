/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <string>
//#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>
#include <parallel_curves/parallel_curves.h>
#include "map/read_pgm.h"

namespace parallel_curves {

class ParallelCurvesCostmap : public parallel_curves::ParallelCurves 
{
public:
    ~ParallelCurvesCostmap() {
      if (costmap_ != nullptr)
        delete costmap_;
    }
    ParallelCurvesCostmap() : costmap_(nullptr), 
      parallel_curves::ParallelCurves() {}

    ParallelCurvesCostmap(u_char* grid, int sx, int sy, double cs, double mr) : 
      costmap_(grid), 
      size_x(sx), size_y(sy), 
      parallel_curves::ParallelCurves() {
        _cell_size = cs;
         _max_range = mr;
        _min_distance_between_nodes = 2 * _cell_size;
        _max_iterations_inside_the_circle = 200;
    }
 
protected:
    bool collisionFree(const Point& pt) 
    {
      // no collision in case of no costmap loaded
      if (costmap_ == nullptr)
        return true;

      auto m = to_cell_id(pt);
      int mx(m[0]), my(m[1]);
      
      if (mx < 0 || mx >= size_x ||
          my < 0 || my >= size_y) 
          return false;

      // getCost returns unsigned char
      u_char cost = costmap_[(size_y - my - 1) * size_x + mx];

      if (cost == 0u)
        return false;

      return true;
    }

    bool directPath(const Point& a, const Point& b) {
      if (costmap_ == nullptr) 
        return true;
      
      if (!collisionFree(a))
        return false;

      if (!collisionFree(b))
        return false;

      if ((a[0] < 0) || (a[0] > size_x*_cell_size) || 
          (a[1] < 0) || (a[1] > size_y*_cell_size) || 
          (b[0] < 0) || (b[0] > size_x*_cell_size) || 
          (b[1] < 0) || (b[1] > size_y*_cell_size))
        return false;

      std::vector<u_char> grid1d_isobstacle = this->get_ravel();
      for (int i : this->line_w(a, b)) 
          if (grid1d_isobstacle[i] == 0) 
              return false;
      return true;
    }

    std::array<int, 2> to_cell_id(const Point& pt) 
    {
        int x = std::min(static_cast<int>(pt[0] / _cell_size), size_x-1);
        int y = std::min(static_cast<int>(pt[1] / _cell_size), size_y-1);
        return {x, y};
    }

    std::vector<int> line_w(const Point& p1, const Point& p2) 
    {
        auto cell1 = to_cell_id(p1);
        auto cell2 = to_cell_id(p2);
        std::vector<std::array<int, 2>> points = bresenham(cell1, cell2);

        std::vector<int> z;
        z.reserve(points.size());
        for (const auto& pt : points) 
            z.push_back(static_cast<int>((size_y - pt[1] - 1) * size_x + pt[0]));

        return z;
    }

    std::vector<u_char> get_ravel() const 
    {
        std::vector<u_char> ravel;
        ravel.reserve(size_x * size_y);
        for (int i=0; i<size_y; i++) 
            for (int j=0; j<size_x; j++) 
                ravel.push_back(costmap_[i * size_x + j]);
        return ravel;
    }

    std::vector<std::array<int, 2>> bresenham(const std::array<int, 2>& start, const std::array<int, 2>& end) 
    {
        int dx = end[0] - start[0];
        int dy = end[1] - start[1];
        int dx1 = std::abs(dx);
        int dy1 = std::abs(dy);
        int px = 2 * dy1 - dx1;
        int py = 2 * dx1 - dy1;
        int x, y, xe, ye;

        if (dy1 <= dx1) {
            if (dx >= 0) {
                x = start[0];
                y = start[1];
                xe = end[0];
            } else {
                x = end[0];
                y = end[1];
                xe = start[0];
            }

            std::array<int, 2> pt = {x, y};
            std::vector<std::array<int, 2>> points;
            points.push_back(pt);

            for (int i = 0; x < xe; ++i) {
                x = x + 1;
                if (px < 0) {
                    px = px + 2 * dy1;
                } else {
                    if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0)) {
                        y = y + 1;
                    } else {
                        y = y - 1;
                    }
                    px = px + 2 * (dy1 - dx1);
                }
                pt = {x, y};
                points.push_back(pt);
            }
            return points;
        } else {
            if (dy >= 0) {
                x = start[0];
                y = start[1];
                ye = end[1];
            } else {
                x = end[0];
                y = end[1];
                ye = start[1];
            }

            std::array<int, 2> pt = {x, y};
            std::vector<std::array<int, 2>> points;
            points.push_back(pt);

            for (int i = 0; y < ye; ++i) {
                y = y + 1;
                if (py <= 0) {
                    py = py + 2 * dx1;
                } else {
                    if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0)) {
                        x = x + 1;
                    } else {
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

private:
    u_char* costmap_{nullptr};
    int size_x{0};
    int size_y{0};
};

}

std::string get_map_path()
{
  std::string package_name = "parallel_curves";
  return /*ament_index_cpp::get_package_share_directory(package_name) +*/ "/test/map/";
}

// Load a willow garage costmap and return a ParallelCurvesCostmap instance using it.
parallel_curves::ParallelCurvesCostmap* make_willow_nav()
{
  int sx,sy;

  std::string path = get_map_path() + "willow_costmap.pgm";

  u_char *cmap = readPGM( path.c_str(), sx, sy );
  if( cmap == NULL )
  {
    return NULL;
  }
  parallel_curves::ParallelCurvesCostmap* nav = new parallel_curves::ParallelCurvesCostmap(cmap,sx,sy,1,80);

  return nav;
}

parallel_curves::ParallelCurvesCostmap* make_maze_nav()
{
  int sx,sy;

  std::string path = get_map_path() + "obstacles.pgm";

  u_char *cmap = readPGM( path.c_str(), sx, sy );
  if( cmap == NULL )
  {
    return NULL;
  }
  parallel_curves::ParallelCurvesCostmap* nav = new parallel_curves::ParallelCurvesCostmap(cmap,sx,sy,0.25,8.0);

  return nav;
}

TEST(PathCalc, oscillate_in_pinch_point)
{
  parallel_curves::ParallelCurvesCostmap* nav = make_willow_nav();
  ASSERT_TRUE( nav != NULL );

  parallel_curves::Point start = {428, 746};
  parallel_curves::Point goal = {350, 450};

  auto path = nav->plan( start, goal );

  EXPECT_TRUE( path.size() > 0 );
  
  delete nav;
}

TEST(PathCalc, maze_simple)
{
  parallel_curves::ParallelCurvesCostmap* nav = make_maze_nav();
  ASSERT_TRUE( nav != NULL );

  parallel_curves::Point start = {4.6, 2.4};
  parallel_curves::Point goal =  {1.6, 8.0};

  auto path = nav->plan( start, goal );

  EXPECT_EQ(path.size(), 5);

  parallel_curves::Point path_correct[] = {{4.6, 2.4}, 
                          {0.49667637112999685, 3.3799159131053544}, 
                          {1.8389143651466706, 5.762720418426307}, 
                          {4.129126211818629, 5.958916806033631}, 
                          {1.6, 8}};

  for(int i=0; i<path.size(); i++)
      EXPECT_TRUE(path[i] == path_correct[i]);

  delete nav;
}

TEST(PathCalc, easy_nav_should_always_work)
{
  parallel_curves::ParallelCurvesCostmap* nav = make_willow_nav();
  ASSERT_TRUE( nav != NULL );

  parallel_curves::Point start = {350, 400};
  parallel_curves::Point goal = {350, 450};

  auto path = nav->plan( start, goal );

  EXPECT_TRUE( path.size() > 0 );

  delete nav;
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}