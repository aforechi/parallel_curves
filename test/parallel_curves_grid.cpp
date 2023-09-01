#include <gtest/gtest.h>
#include <parallel_curves/parallel_curves.h>
#include "map/environment.h"
#include "map/grid.h"

class ParallelCurvesCpp : public parallel_curves::ParallelCurves 
{
public:
    ParallelCurvesCpp() : grid(nullptr), parallel_curves::ParallelCurves() {}
    ParallelCurvesCpp(Grid* grid) : grid(grid), parallel_curves::ParallelCurves() {
        _cell_size = grid->cell_size;
        _min_distance_between_nodes = 2 * _cell_size;
        _max_iterations_inside_the_circle = 200;
        _max_range = 8.0;
    }
 
protected:
    
    bool validate_endpoint(const Point& point) {
        auto cell_id = grid->to_cell_id(point);
        if (grid->grid[cell_id[0]][cell_id[1]] != 0) {
            return false;
        }
        return true;
    }

    bool directPath(const Point& start, const Point& target) {
        if (start[0] < 0 || start[0] > grid->env.lx ||
            start[1] < 0 || start[1] > grid->env.ly ||
            target[0] < 0 || target[0] > grid->env.lx ||
            target[1] < 0 || target[1] > grid->env.ly) {
            return false;
        }

        if (!validate_endpoint(start)) {
            return false;
        }

        if (!validate_endpoint(target)) {
            return false;
        }

        // Ray tracing
        std::vector<int> grid1d_isoccupied = grid->get_ravel();
        for (int i : grid->line_w(start, target)) {
            if (grid1d_isoccupied[i] != 0) {
                return false;
            }
        }

        return true;
    }
private:
    Grid* grid;
};

typedef std::array<double,2> Point;

TEST(ParallelCurves, planning)
{
    Point start_pos = {4.6, 2.4};
    Point end_pos = {1.6, 8};

    std::vector<std::array<double, 4> > obs = {
        {2, 3, 6, 0.1},
        {2, 3, 0.1, 1.5},
        {4.3, 0, 0.1, 1.8},
        {6.5, 1.5, 0.1, 1.5},
        {0, 6, 3.5, 0.1},
        {5, 6, 5, 0.1}
    };

    auto env = Environment(obs);

    auto grid = Grid(env);

    auto pcurve = new ParallelCurvesCpp(&grid);

    auto path = pcurve->plan(start_pos, end_pos);

    Point path_correct[] = {{4.6, 2.4}, 
                            {0.49667637112999685, 3.3799159131053544}, 
                            {1.8389143651466706, 5.762720418426307}, 
                            {4.129126211818629, 5.958916806033631}, 
                            {1.6, 8}};

    EXPECT_EQ(path.size(), 5);
    for(int i=0; i<path.size(); i++)
        EXPECT_TRUE(path[i] == path_correct[i]);

    grid.save_costmap("/home/avelino/catkin_ws/src/parallel_curves/test/map/obstacles");

    delete pcurve;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}