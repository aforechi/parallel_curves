#pragma once

#include <cmath>
#include <array>

class Obstacle {
public:
    double x, y, w, h, safe_dis;
    std::array<std::array<double, 2>, 4> obs;

    Obstacle(double x, double y, double w, double h, double safe_dis = 0.1)
        : x(x), y(y), w(w), h(h), safe_dis(safe_dis) 
    {
        obs = {{
            {{x - safe_dis, y - safe_dis}},
            {{x + w + safe_dis, y - safe_dis}},
            {{x + w + safe_dis, y + h + safe_dis}},
            {{x - safe_dis, y + h + safe_dis}}
        }};
    }

};
