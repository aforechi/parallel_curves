#pragma once

#include <cmath>
#include <vector>
#include <array>

#include "obstacle.h"

class Environment {
public:
    double lx, ly;
    std::vector<Obstacle> obs;

    Environment(std::vector<std::array<double, 4>> obs = {}, double lx = 10.0, double ly = 10.0)
        : lx(lx), ly(ly) 
    {
        for (const auto& ob : obs) {
            this->obs.emplace_back(Obstacle(ob[0], ob[1], ob[2], ob[3]));
        }
    }

    bool rectangle_inbounds(const std::array<std::array<double, 2>, 4>& rect, double safe_dis = 0.05) {
        // Check if a rectangle target is within the map bounds.
        for (const auto& v : rect) {
            if (v[0] < safe_dis || v[0] > lx - safe_dis || v[1] < safe_dis || v[1] > ly - safe_dis) {
                return false;
            }
        }
        return true;
    }

};
