#include <parallel_curves/shortest_path.h>

#include <vector>
#include <string>
#include <cmath>
#include <memory>

using parallel_curves::Graph;
using parallel_curves::Node;

#pragma once

namespace parallel_curves {

    /**
    * @class ParallelCurves
    * @brief Provides a simple yet effective global planner that will compute a valid goal point for the local planner by finding tangent segments allong the parallel curves centered at the goal point.
    */
    class ParallelCurves {
    public:
        /**
        * @brief  Constructor
        */
        ParallelCurves() : _cell_size(0.0), _max_range(0.0), _min_distance_between_nodes(0.0), _max_iterations_inside_the_circle(0) {}
        /**
        * @brief Destructor
        */
        ~ParallelCurves() {}
        
    protected:
        
        virtual bool directPath(const Point& start, const Point& target) = 0;

    private:
        inline double distanceBetweenPoints(Point pt1, Point pt2){
            return std::sqrt(std::pow(pt2[0] - pt1[0], 2.0) + std::pow(pt2[1] - pt1[1], 2.0));
        }

        inline bool almostEqual(double x, double y){
            double maxXYOne = std::max( { 1.0, std::fabs(x) , std::fabs(y) } ) ;
            return std::fabs(x - y) <= std::numeric_limits<double>::epsilon()*maxXYOne ;
        }

        /**
         * @brief: Find the distance from the external point to the tangent points of the circle with center and radius.
        */
        inline double distanceBetweenExternalPointAndTangentPoints(Point center, double radius, Point external_point){
            double dx = center[0] - external_point[0];
            double dy = center[1] - external_point[1];
            double d2 = dx * dx + dy * dy;
            double r2 = radius * radius;

            if (d2 < r2)
                return std::numeric_limits<double>::infinity();
            
            return std::sqrt(d2 - r2);            
        }

        /**
         * @brief: Return true if the point is inside the circle defined by center and radius. And false otherwise.
        */
        inline bool insideTheCircle(Point pt, Point center, double radius){
            double dx = center[0] - pt[0];
            double dy = center[1] - pt[1];
            double d2 = dx * dx + dy * dy;
            double r2 = radius * radius;
            return (d2 <= r2);
        }

        /** 
         * @brief Find the points where the two circles intersect.
         *   @see: http://www.vb-helper.com/howto_net_circle_circle_intersection.html
         */
        std::vector<Point> findIntersectionsBetweenCircles(Point center0, double radius0, Point center1, double radius1, double delta);

        /**
        * Find the tangent points for this circle and external point.
        * Return true if we find the tangents, false if the point is inside the circle.
        * @see: http://www.vb-helper.com/howto_net_find_circle_tangents.html
        **/
        std::vector<Point> findTangentPoints(Point center, double radius, Point external_point, double distance, double delta);

        bool containWaypointNearTo(const Point& other_node);

        void addWaypointNode(const Node& origin_node, const Point& waypoint, const Node& target_node, double target_radius, double distance) ;

        void addTangentNodes(const Node& origin_node, const Node& target_node, double target_radius);

        void addSecantNodes(const Node& origin_node, const Node& target_node, double target_radius);

        void build(const Node& start_node, const Node& target_node);

        double findPathDistance(const Node& start_node, const Node& target_node);

        std::vector<Node> findShortestPath(const Node& start_node, const Node& target_node);

        std::vector<Node> findPathSmooth(const Node& start_node, const Node& target_node);

        std::vector<Node> findPathSmoothDistance(const Point& start, const Point& target, double & shortest);

    public:
        /**
         * Find a path between two configurations
         * @param start_node: start configuration
         * @param target_node: target configuration
        */
        std::vector<Point> plan(const Point& start_node, const Point& target_node);
        std::vector<double> radius() const {return R;}
        bool isforward() const {return forward;}
         
    protected:
        double _cell_size;
        double _min_distance_between_nodes;
        double _max_range;
        int _max_iterations_inside_the_circle;

    private:
        bool forward{true};
        std::vector<Node> S;
        std::vector<double> R;
        std::unique_ptr<Graph> G{nullptr};
    };
};
