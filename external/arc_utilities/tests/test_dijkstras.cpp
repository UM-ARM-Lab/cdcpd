
#include <gtest/gtest.h>

#include <arc_utilities/dijkstras.hpp>
#include <arc_utilities/eigen_helpers.hpp>

#define DIFF 0.0000001

using namespace arc_dijkstras;
using namespace arc_utilities;
using namespace EigenHelpers;
typedef Graph<std::vector<double>> GraphD;

void addEdge(GraphD &g, int64_t n1, int64_t n2) {
  double d = Distance(g.getNode(n1).getValue(), g.getNode(n2).getValue());
  g.addEdgesBetweenNodes(n1, n2, d);
}

double DistanceHeuristic(std::vector<double> q1, std::vector<double> q2) { return Distance(q1, q2); }

TEST(DijkstrasTestSuite, AStarSimple) {
  GraphD g;

  int64_t start = g.addNode(std::vector<double>{0, 0});
  int64_t goal = g.addNode(std::vector<double>{0, 1});
  addEdge(g, start, goal);
  ASSERT_EQ(g.getEdge(start, goal).getWeight(), 1.0) << "No edge added from start to goal";

  int64_t n1 = g.addNode(std::vector<double>{0.5, 0.5});
  addEdge(g, start, n1);
  addEdge(g, goal, n1);

  int64_t n2 = g.addNode(std::vector<double>{-0.5, 0.5});
  addEdge(g, start, n2);
  addEdge(g, goal, n2);
  addEdge(g, n1, n2);

  int64_t n3 = g.addNode(std::vector<double>{10, 10});
  addEdge(g, start, n3);
  addEdge(g, goal, n3);
  addEdge(g, n1, n3);
  addEdge(g, n2, n3);

  auto result = SimpleGraphAstar<std::vector<double>>::PerformAstar(g, start, goal, &DistanceHeuristic, true);
  EXPECT_EQ(result.second, 1.0) << "AStar did not find optimal path cost of 1.0";
  auto path = result.first;
  ASSERT_EQ(result.first.size(), (size_t)2) << "AStar did not find optimal path of length 2";
  EXPECT_EQ(result.first.front(), start) << "AStar's path does not begin at start node";
  EXPECT_EQ(result.first.back(), goal) << "AStar's path does not end at goal node";
}

TEST(DijkstrasTestSuite, AStarNoSolution) {
  GraphD g;

  int64_t start = g.addNode(std::vector<double>{0, 0});
  int64_t goal = g.addNode(std::vector<double>{0, 1});

  int64_t n1 = g.addNode(std::vector<double>{0.5, 0.5});
  addEdge(g, start, n1);

  int64_t n2 = g.addNode(std::vector<double>{-0.5, 0.5});
  addEdge(g, start, n2);
  addEdge(g, n1, n2);

  int64_t n3 = g.addNode(std::vector<double>{10, 10});
  addEdge(g, start, n3);
  addEdge(g, n1, n3);
  addEdge(g, n2, n3);

  auto result = SimpleGraphAstar<std::vector<double>>::PerformAstar(g, start, goal, &DistanceHeuristic, true);
  EXPECT_GE(result.second, std::numeric_limits<double>::max()) << "AStar did not return cost=INF when no path exists";
  auto path = result.first;
  ASSERT_EQ(result.first.size(), (size_t)0) << "AStar returned a path when non exists";
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  // ros::init(argc, argv, "tester");
  // ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
