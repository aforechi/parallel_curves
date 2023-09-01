#include <gtest/gtest.h>
#include <parallel_curves/shortest_path.h>
using parallel_curves::Graph;
using parallel_curves::Node;
using parallel_curves::Point;

TEST(ShortestPath, basicAlloc)
{
  int V = 9;
  Graph g;
  
  for(double i=0; i<V; i++)
  {
    Point p = {i,i};
    g.addNode(p);
  }
  EXPECT_EQ(g.size(), V);
}

TEST(ShortestPath, basicCopy)
{
  int V = 2;
  Graph g;
  
  for(double i=0; i<V; i++)
  {
    Point p = {i,i};
    g.addNode(p);
  }

  Graph h(g);

  for(int i=0; i<V; i++)
    EXPECT_EQ(g.getNode(i), h.getNode(i));

  EXPECT_EQ(g.size(), V);
}

TEST(ShortestPath, basicGraph)
{
  int V = 9;
  Graph g;

  for(int i=0; i<V; i++)
    g.addNode(Node(i, {(double)i,(double)i}));

  g.addEdge(g.getNode(0), g.getNode(1), 4);
  g.addEdge(g.getNode(0), g.getNode(7), 8);
  g.addEdge(g.getNode(1), g.getNode(2), 8);
  g.addEdge(g.getNode(1), g.getNode(7), 11);
  g.addEdge(g.getNode(2), g.getNode(3), 7);
  g.addEdge(g.getNode(2), g.getNode(8), 2);
  g.addEdge(g.getNode(2), g.getNode(5), 4);
  g.addEdge(g.getNode(3), g.getNode(4), 9);
  g.addEdge(g.getNode(3), g.getNode(5), 14);
  g.addEdge(g.getNode(4), g.getNode(5), 10);
  g.addEdge(g.getNode(5), g.getNode(6), 2);
  g.addEdge(g.getNode(6), g.getNode(7), 1);
  g.addEdge(g.getNode(6), g.getNode(8), 6);
  g.addEdge(g.getNode(7), g.getNode(8), 7);

  auto results = g.dijkstra(g.getNode(0));
  auto dist = results.first;

  int expected_dist[] = {0, 4, 12, 19, 28, 16, 18, 8, 14};

  for (int i = 0; i < V; ++i)
    EXPECT_EQ(dist[i], expected_dist[i]);
}

TEST(ShortestPath, shortestPath)
{
  Graph g;

  Node nodes[] = {
    Node(0, {4.6, 2.4}),
    Node(1, {0.49667637112999685, 3.3799159131053544}),
    Node(2, {3.411114196338082, 5.93061232055889}),
    Node(3, {1.8389143651466706, 5.762720418426307}),
    Node(4, {7.241896934056941, 3.446814413459639}),
    Node(5, {1.8920064462903754, 5.265547178076597}),
    Node(6, {1.9450985274340802, 4.768373937726887}),
    Node(7, {4.129126211818629, 5.958916806033631}),
    Node(8, {1.9981906085777847, 4.271200697377177}),
    Node(9, {4.518222552098418, 5.644904006961882}),
    Node(10, {5.101867062518101, 5.173884808354258}),
    Node(11, {5.685511572937784, 4.702865609746635}),
    Node(12, {6.269156083357467, 4.231846411139012}),
    Node(13, {6.658252423637258, 3.9178336120672625}),
    Node(14, {1.6, 8})
  };

  for(int i=0; i<=14; i++)
    g.addNode(nodes[i]);

  g.addEdge(nodes[0], nodes[1], 4.218708333127569);
  g.addEdge(nodes[0], nodes[1], 4.2187083331275685);
  g.addEdge(nodes[1], nodes[2], 3.8729833462074175);
  g.addEdge(nodes[1], nodes[2], 3.8729833462074175);
  g.addEdge(nodes[1], nodes[2], 3.8729833462074175);
  g.addEdge(nodes[1], nodes[3], 2.7348418863180615);
  g.addEdge(nodes[1], nodes[3], 2.7348418863180615);
  g.addEdge(nodes[2], nodes[3], 1.5811388300841898);
  g.addEdge(nodes[2], nodes[3], 1.5811388300841898);
  g.addEdge(nodes[2], nodes[4], 4.565539226303439);
  g.addEdge(nodes[2], nodes[4], 4.565539226303439);
  g.addEdge(nodes[3], nodes[5], 0.5000000000000002);
  g.addEdge(nodes[3], nodes[6], 1.0000000000000004);
  g.addEdge(nodes[3], nodes[7], 2.2986002969517654);
  g.addEdge(nodes[3], nodes[8], 1.5000000000000007);
  g.addEdge(nodes[3], nodes[9], 2.6818972887635004);
  g.addEdge(nodes[3], nodes[10], 3.315657955967777);
  g.addEdge(nodes[3], nodes[11], 3.989937630398227);
  g.addEdge(nodes[3], nodes[12], 4.687282465135125);
  g.addEdge(nodes[3], nodes[13], 5.1603901645411945);
  g.addEdge(nodes[3], nodes[4], 5.878404652946996);
  g.addEdge(nodes[3], nodes[4], 5.878404652946996);
  g.addEdge(nodes[3], nodes[4], 5.878404652946996);
  g.addEdge(nodes[7], nodes[14], 3.25);
  g.addEdge(nodes[9], nodes[14], 3.7500000000000004);
  g.addEdge(nodes[10], nodes[14], 4.5);
  g.addEdge(nodes[11], nodes[14], 5.25);
  g.addEdge(nodes[12], nodes[14], 5.999999999999999);
  g.addEdge(nodes[13], nodes[14], 6.500000000000001);
  g.addEdge(nodes[4], nodes[14], 7.250000000000001);
  g.addEdge(nodes[4], nodes[14], 7.250000000000001);
  g.addEdge(nodes[4], nodes[14], 7.250000000000001);
  g.addEdge(nodes[4], nodes[14], 7.250000000000001);

  auto dist = g.shortestDistance(nodes[0], nodes[14]);
  
  EXPECT_DOUBLE_EQ(dist, 12.502150516397394);

  auto path = g.shortestPath(nodes[0], nodes[14]);
  Node path_correct[] = {nodes[0], nodes[1], nodes[3], nodes[7], nodes[14]};

  EXPECT_EQ(path.size(), 5);
  for(int i=0; i<path.size(); i++)
    EXPECT_TRUE(path[i].node() == path_correct[i].node());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

