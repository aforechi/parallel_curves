#pragma once

#include <cmath>
#include <climits>
#include <vector>
#include <algorithm>
#include <array>
#include <iostream>

#include "environment.h"

class Grid {
public:
    Environment& env;
    double cell_size;
    int nx, ny;
    double cell_dia;
    std::vector<std::vector<int>> grid;

    Grid(Environment& env, double cell_size = 0.25)
        : env(env), cell_size(cell_size) 
    {
        nx = static_cast<int>(env.lx / cell_size);
        ny = static_cast<int>(env.ly / cell_size);
        cell_dia = std::sqrt(2 * cell_size * cell_size);

        get_obstacle_occupancy();
    }

    void get_obstacle_occupancy() {
        // Fill grid with obstacles.

        grid.resize(nx, std::vector<int>(ny, 0));

        for (const auto& ob : env.obs) {
            auto p1 = to_cell_id({ob.x, ob.y});
            auto p2 = to_cell_id({ob.x + ob.w, ob.y + ob.h});
            double x1 = p1[0];
            double y1 = p1[1];
            double x2 = p2[0];
            double y2 = p2[1];

            if (std::fmod(ob.x + ob.w, cell_size) == 0) {
                x2 -= 1;
            }

            if (std::fmod(ob.y + ob.h, cell_size) == 0) {
                y2 -= 1;
            }

            for (int i = x1; i <= x2; ++i) {
                for (int j = y1; j <= y2; ++j) {
                    grid[i][j] = 1;
                }
            }
        }
    }

    void save_costmap(const char *fname) {
        char fn[4096];
        sprintf(fn,"%s.pgm",fname);
        FILE* fp = fopen(fn,"wb");
        if (!fp)
        {
            std::cout << "Can't open file " << fn << std::endl;
            return;
        }
        fprintf(fp,"P5\n%d\n%d\n%d\n", nx, ny, UCHAR_MAX);
        
        int k = 0;
        char grid_flatenned[nx*ny];
        for (int j=ny-1; j>=0; j--)  //origin is row zero
            for (int i=0; i<nx; i++) //origin is col zero
                grid_flatenned[k++] = (grid[i][j] == 1 ? 0u : 255u);
        
        fwrite(grid_flatenned,1,nx*ny,fp);
        fclose(fp);
    }

    std::array<int, 2> to_cell_id(const std::array<double, 2>& pt) {
        // Convert point into grid index.
        int x = std::min(static_cast<int>(pt[0] / cell_size), nx - 1);
        int y = std::min(static_cast<int>(pt[1] / cell_size), ny - 1);
        return {x, y};
    }

    std::vector<int> line_w(const std::array<double, 2>& p1, const std::array<double, 2>& p2) {
        auto cell1 = to_cell_id(p1);
        auto cell2 = to_cell_id(p2);
        std::vector<std::array<int, 2>> points = bresenham(cell1, cell2);

        std::vector<int> z;
        z.reserve(points.size());
        for (const auto& pt : points) {
            z.push_back(static_cast<int>(pt[0] * ny + pt[1]));
        }

        return z;
    }

    std::vector<int> get_ravel() const {
        // Ravel the grid (superclass)
        std::vector<int> ravel;
        ravel.reserve(nx * ny);
        for (const auto& row : grid) {
            for (const auto& val : row) {
                ravel.push_back(val);
            }
        }
        return ravel;
    }

    std::vector<std::array<int, 2>> bresenham(const std::array<int, 2>& start, const std::array<int, 2>& end) {
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
};
