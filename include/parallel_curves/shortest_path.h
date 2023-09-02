#include<queue>
#include<vector>
#include<utility>
#include<limits>
#include<stdexcept>
#include<algorithm>
#include<functional>

#pragma once

namespace parallel_curves
{
    #define INF std::numeric_limits<double>::infinity()

    typedef std::array<double,2> Point;
    typedef std::array<double,3> Circle;

    class Node
    {
    private:
        int id;
        Point pt;
        //std::vector<Circle> annotations; 
    public:
        Node() : id(-1) {}
        Node(int id): id(id) {}
        Node(Point pt): id(0), pt(pt) {}
        Node(int id, Point pt): id(id), pt(pt) {}

        constexpr int node() const {return id;}

        constexpr Point center() const {return pt;}

        operator Point() const 
        {
            return pt;
        }

        constexpr bool operator<(const Node& that) const
        {
            return this->id < that.id;
        }

        constexpr bool operator==(const Node& that) const
        {
            return (this->pt[0] == that.pt[0]) && (this->pt[1] == that.pt[1]);
        }

        constexpr bool operator!=(const Node& that) const
        {
            return (this->pt[0] != that.pt[0]) || (this->pt[1] != that.pt[1]);
        }

        constexpr bool operator()(const Node& lhs, const Node& rhs) const
        {
            return lhs == rhs;
        }
    };

    typedef std::pair<Node, double> NodeWeightPair;
    typedef std::pair<double, Node> WeightNodePair;

    // This class represents a directed graph using
    // adjacency list representation
    class Graph 
    {
    private:
        // In a weighted graph, we need to store vertex
        // and weight pair for every edge
        std::vector<std::vector<NodeWeightPair> > adj;
        std::vector<Node> nodes;
    public:
        /**
         * Deallocates memory for adjacency list
        */
        ~Graph()
        {
            for(int i=0; i < adj.size(); i++)
                adj[i].clear();
            adj.clear();
            nodes.clear();
        }

        /**
         * Returns the no. of vertices
        */
        int size() const {return nodes.size();}

        const std::vector<Node>& getNodes()
        {
            return nodes;
        }

        const std::vector<NodeWeightPair>& getNeighbors(const Node& u)
        {
            return adj[u.node()];
        }

        /**
         * To get a node
        */
        Node getNode(int index) const
        {
            if (index < 0 || index >= size())
                throw std::out_of_range("Node index invalid!");
            return nodes[index];
        }

        /**
         * To find a node
        */
        int findNode(const Node& v) const
        {
            const auto it = std::find_if(nodes.begin(), nodes.end(), std::bind(std::equal_to<Node>(), std::placeholders::_1, v));
            if (it == nodes.end()) //if not found
                return -1;
            return it->node();
        }

        /**
         * To add a node
        */
        Node addNode(const Node& v)
        {
            auto index = findNode(v);
            if (index < 0)
                return addNode(v.center());
            else
                return getNode(index);
        }

        /**
         * To add a node from point
        */
        Node addNode(const Point& pt)
        {
            Node v_new(nodes.size(), pt);
            adj.push_back(std::vector<NodeWeightPair>());
            nodes.push_back(v_new);
            return v_new;
        }

        /**
         * To add an edge for a directed graph
        */
        void addEdge(const Node& first, const Node& second, double weight)
        {
            adj[first.node()].push_back(std::make_pair(second, weight));
        }
    
        /**
         * Find Dijkstra's shortest path using a priority_queue in STL
         * @returns the shortest paths from src to all other vertices
        */ 
        std::pair<std::vector<double>, std::vector<Node> > dijkstra(const Node& source) const
        {
            // Create a priority queue to store vertices that are being preprocessed. This is weird syntax in C++.
            std::priority_queue<WeightNodePair, std::vector<WeightNodePair>, std::greater<WeightNodePair> > pq;

            // Create a vector for distances and initialize all distances as infinite (INF)
            std::vector<double> dist(size(), INF);

            // Create a vector for path and initialize all nodes as invalid (-1)
            std::vector<Node> path(size(), -1);

            // Insert source itself in priority queue and initialize its distance as 0.
            pq.push(std::make_pair(0.0, source));
            dist[source.node()] = 0.0;

            // Looping untill priority queue becomes empty (or all distances are not finalized)
            while (!pq.empty()) 
            {
                // The first vertex in pair is the minimum distance vertex, extract it from priority queue.
                // vertex label is stored in second of pair (it has to be done this way to keep the vertices
                // sorted distance (distance must be first item in pair)
                auto u = pq.top().second;
                pq.pop();

                // Get all adjacent of u.
                for (auto x : adj[u.node()]) 
                {
                    // Get vertex label and weight of current adjacent of u.
                    auto v = x.first;
                    double weight = x.second;

                    // If there is shorted path to v through u.
                    if (dist[v.node()] > dist[u.node()] + weight) 
                    {
                        // Updating distance of v
                        dist[v.node()] = dist[u.node()] + weight;
                        pq.push(std::make_pair(dist[v.node()], v));
                        path[v.node()] = u;
                    }
                }
            }

            return std::make_pair(dist, path);
        }

        std::vector<Node> shortestPath(const Node& source, const Node& target) const 
        {
            auto results = dijkstra(source);
            std::vector<Node> paths = results.second;
            std::vector<Node> shortest;
            
            Node next = target;
            do {
                shortest.push_back( next );
                next = paths[next.node()];
            } while ( (next.node() >= 0) && (next.node() != source.node()) );
            shortest.push_back(source);
            std::reverse(shortest.begin(), shortest.end());
            
            return shortest;
        }

        double shortestDistance(const Node& source, const Node& target) const
        {
            auto results = dijkstra(source);
            std::vector<double> dist = results.first;
            return dist[target.node()];
        }

    };

}