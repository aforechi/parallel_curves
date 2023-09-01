#include <parallel_curves/parallel_curves.h>
#include <iostream>
#include <iomanip>

namespace parallel_curves 
{

    std::vector<ParallelCurves::Point> ParallelCurves::findIntersectionsBetweenCircles(Point center0, double radius0, Point center1, double radius1, double delta)
    {

        double cx0 = center0[0];
        double cy0 = center0[1];
        double cx1 = center1[0];
        double cy1 = center1[1];

        std::vector<ParallelCurves::Point> intersections;

        // Find the distance between the centers.
        auto dx = cx0 - cx1;
        auto dy = cy0 - cy1;
        double dist = std::sqrt(dx * dx + dy * dy);
        
        // See how many solutions there are.
        if (dist > radius0 + radius1)
            // No solutions, the circles are too far apart.
            return intersections;

        else if (dist < std::abs(radius0 - radius1))
            //  No solutions, one circle contains the other.
            return intersections;

        else if (almostEqual(dist, 0.0) && almostEqual(radius0, radius1))
        //else if ((dist == 0.0) && (radius0 == radius1))
            //  No solutions, the circles coincide.
            return intersections;

        // Find a and h
        double a = (radius0 * radius0 - radius1 * radius1 + dist * dist) / (2.0 * dist);
        //double h = std::sqrt(radius0 * radius0 - a * a);
        double h = 0;
        if (!almostEqual(radius0 * radius0, a * a))
           h = std::sqrt(radius0 * radius0 - a * a);

        // Find P2
        double cx2 = cx0 + a * (cx1 - cx0) / dist;
        double cy2 = cy0 + a * (cy1 - cy0) / dist;

        // Get the points P3
        ParallelCurves::Point intersection1 = {cx2 + h * (cy1 - cy0) / dist, cy2 - h * (cx1 - cx0) / dist};
        ParallelCurves::Point intersection2 = {cx2 - h * (cy1 - cy0) / dist, cy2 + h * (cx1 - cx0) / dist};

        intersections.push_back(intersection1);

        // See if we have 2 solutions
        if (std::fabs(dist - radius0 - radius1) > delta)
            intersections.push_back(intersection2);
        
        return intersections;
    }

    std::vector<ParallelCurves::Point> ParallelCurves::findTangentPoints(ParallelCurves::Point center, double radius, ParallelCurves::Point external_point, double distance, double delta)
    {
        // Find the points of intersection between the original circle and the circle with center external_point and radius dist.
        return findIntersectionsBetweenCircles(center, radius, external_point, distance, delta);
    }

    bool ParallelCurves::containWaypointNearTo(const Point& other_node)
    {
        for(auto node : this->G->getNodes())
            if (distanceBetweenPoints(node.center(), other_node) < this->_min_distance_between_nodes)
                return true;
        return false;
    }

    void ParallelCurves::addWaypointNode(const Node& origin_node, const Point& waypoint, const Node& target_node, double target_radius, double distance) 
    {
        if (directPath(origin_node.center(), waypoint))
        {
            if (!containWaypointNearTo(waypoint)) 
            {
                Node waypoint_node(waypoint);
                waypoint_node = G->addNode(waypoint_node); //TODO target_circle=dict(center=target_node, radius=target_radius))
                G->addEdge(origin_node, waypoint_node, distance);   
                S.push_back(waypoint_node);
            } 
        }
    }

    void ParallelCurves::addTangentNodes(const Node& origin_node, const Node& target_node, double target_radius) 
    {
        double distance = distanceBetweenExternalPointAndTangentPoints(target_node.center(), target_radius, origin_node.center());
        std::vector<ParallelCurves::Point> tangent_points = findTangentPoints(target_node.center(), target_radius, origin_node.center(), distance, _cell_size);
        
        if (tangent_points.size() == 2) 
        {
            addWaypointNode(origin_node, tangent_points[0], target_node, target_radius, distance);
            addWaypointNode(origin_node, tangent_points[1], target_node, target_radius, distance);
            //addWaypointNode(origin_node, tangent_points[0], origin_node, distance, distance);
            //addWaypointNode(origin_node, tangent_points[1], origin_node, distance, distance);
        }
    }

    void ParallelCurves::addSecantNodes(const Node& origin_node, const Node& target_node, double target_radius) 
    {
        if (target_radius > 0.0) 
        {
            double angular_offset = std::atan2(origin_node.center()[1] - target_node.center()[1], origin_node.center()[0] - target_node.center()[0]);
            for (double arc : {-M_PI / 4.0, 0.0, M_PI / 4.0}) 
            {
                Point secant_point = {target_node.center()[0] + target_radius * std::cos(arc + angular_offset), 
                                      target_node.center()[1] + target_radius * std::sin(arc + angular_offset)};
                double distance = distanceBetweenPoints(origin_node.center(), secant_point);
                addWaypointNode(origin_node, secant_point, target_node, target_radius, distance);
            }
        }
    }
    
    void ParallelCurves::build(const Node& start_node, const Node& target_node) 
    {
        S.push_back(start_node);
        
        while (S.size() > 0) 
        {
        
            auto origin_node = S.back(); S.pop_back();

            //std::cerr << std::fixed << origin_node.center()[0] << " " << origin_node.center()[1] << std::endl;

            auto iterations_inside_the_circle = 0;

            //found a path from origin to the target point
            if (directPath(origin_node.center(), target_node.center())) 
            {
                G->addEdge(origin_node, target_node, distanceBetweenPoints(origin_node.center(), target_node.center()));
                break;
            }

            //for(double target_radius = _max_range; target_radius >= 0.0; target_radius -= _cell_size)                      
            for(double target_radius = 0.0; target_radius < _max_range; target_radius += _cell_size)
            {
                if (insideTheCircle(origin_node.center(), target_node.center(), target_radius)) 
                {
                    addSecantNodes(origin_node, target_node, target_radius);
                    //TODO check if can be removed as max_range already works as limitation. The difference is that the continue jumps the found path below
                    if (iterations_inside_the_circle < _max_iterations_inside_the_circle) 
                    {
                        iterations_inside_the_circle += 1;
                        continue;
                    }
                }
                else 
                {
                    addTangentNodes(origin_node, target_node, target_radius);
                }
                //found a path from origin to at least one tangent point
                if (G->getNeighbors(origin_node).size() > 0) 
                {
                    for (const auto &node : G->getNeighbors(origin_node))
                        if (directPath(node.first.center(), target_node.center()))
                            if (node.first != target_node)
                                G->addEdge(node.first, target_node, distanceBetweenPoints(node.first.center(), target_node.center()));
                    break; //FIXME shouldn't be here
                }
            }
        }
        S.clear();
    }

    std::vector<Node> ParallelCurves::findShortestPath(const Node& start_node, const Node& target_node) 
    {
        std::vector<Node> waypoints;
        if (G->size() > 0)
            waypoints = G->shortestPath(start_node, target_node);
        return waypoints;
    }

    double ParallelCurves::findPathDistance(const Node& start_node, const Node& target_node) 
    {
        double distance = INF;
        if (G->size() > 0)
            distance = G->shortestDistance(start_node, target_node);
        return distance;
    }

    std::vector<Node> ParallelCurves::findPathSmooth(const Node& start_node, const Node& target_node) 
    {
        build(start_node, target_node);
        auto main_path = findShortestPath(start_node, target_node);
        for(int i=1; i < main_path.size()-1; i++)
            for(int j=i+1; j < main_path.size(); j++)
                build(main_path[i], main_path[j]);
        auto main_path_after_start = std::vector<Node>(main_path.begin()+1, main_path.end());
        for(auto current_node : main_path_after_start)
            build(start_node, current_node);
        auto path = findShortestPath(start_node, target_node);
        return path;
    }

    std::vector<Point> ParallelCurves::plan(const Point& start, const Point& target) 
    {
        Node start_node, target_node;

        G = std::make_unique<Graph>();
        start_node = G->addNode(start); //TODO target_circle=dict(center=target_node, radius=0))
        target_node = G->addNode(target); //TODO target_circle=dict(center=target_node, radius=0))
        auto waypoints_forward = findPathSmooth(start_node, target_node);
        double distance_forward = findPathDistance(start_node, target_node);
        auto path_forward = std::move(G);

        G = std::make_unique<Graph>();
        start_node = G->addNode(start); //TODO target_circle=dict(center=target_node, radius=0))
        target_node = G->addNode(target); //TODO target_circle=dict(center=target_node, radius=0))
        auto waypoints_backward = findPathSmooth(target_node, start_node);
        double distance_backward = findPathDistance(target_node, start_node);
        auto path_backward = std::move(G);
        
        if (std::min(distance_forward, std::min(distance_backward, INF)) == INF)
        {
            return std::vector<Point>(0);
        }
        else if (distance_forward <= distance_backward) 
        {
            G = std::move(path_forward);
            std::vector<Point> waypoints(waypoints_forward.size());
            std::copy(waypoints_forward.begin(), waypoints_forward.end(), waypoints.begin());
            return waypoints;
        }
        else 
        {
            G = std::move(path_backward);
            std::vector<Point> waypoints(waypoints_backward.size());
            std::reverse_copy(waypoints_backward.begin(), waypoints_backward.end(), waypoints.begin());
            return waypoints;
        }
    }
}