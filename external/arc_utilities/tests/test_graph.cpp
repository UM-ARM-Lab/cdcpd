
#include <gtest/gtest.h>

#include <arc_utilities/graph.hpp>

#define DIFF 0.0000001

using namespace arc_dijkstras;
using namespace arc_utilities;
typedef Graph<std::vector<double>> GraphD;

// Declare a test
TEST(GraphTestSuite, construction) {
  GraphD g;

  EXPECT_EQ(g.getNodes().size(), (size_t)0) << "Empty graph has nodes";
  g.addNode(std::vector<double>{0, 0});
  EXPECT_EQ(g.getNodes().size(), (size_t)1) << "After adding nodes, wrong number appear in graph";
  g.addNode(std::vector<double>{1, 1});
  EXPECT_EQ(g.getNodes().size(), (size_t)2) << "After adding nodes, wrong number appear in graph";
  g.addNode(std::vector<double>{3, 0});
  EXPECT_EQ(g.getNodes().size(), (size_t)3) << "After adding nodes, wrong number appear in graph";

  EXPECT_EQ(g.getNode(0).getOutEdges().size(), (size_t)0) << "Node has edge before any added";
  EXPECT_EQ(g.getNode(0).getInEdges().size(), (size_t)0) << "Node has edge before any added";

  GraphEdge& e1 = g.addEdgeBetweenNodes(0, 1, 1.0);
  ASSERT_EQ(g.getNode(0).getOutEdges().size(), (size_t)1) << "Node has wrong number of out edges";
  ASSERT_EQ(g.getNode(1).getInEdges().size(), (size_t)1) << "Node has wrong number of in edges";
  EXPECT_EQ(e1, g.getNode(0).getOutEdges()[0]) << "Edge returned and edge lookup are different";

  g.addEdgeBetweenNodes(1, 2, 1.0);
  g.addEdgeBetweenNodes(0, 2, 1.0);
  EXPECT_EQ(g.getNode(0).getOutEdges().size(), (size_t)2) << "Node has wrong number of out edges";
}

TEST(GraphTestSuite, serialization) {
  Graph<Eigen::Vector2d> test_graph(2);
  test_graph.addNode(Eigen::Vector2d(0, 0));
  test_graph.addNode(Eigen::Vector2d(1, 1));
  test_graph.addEdgesBetweenNodes(0, 1, 1.0);

  // Define the graph value serialization function
  const auto value_serializer_fn = [](const Eigen::Vector2d& value, std::vector<uint8_t>& buffer) {
    const uint64_t start_buffer_size = buffer.size();
    uint64_t running_total = 0;

    running_total += arc_utilities::SerializeFixedSizePOD<double>(value(0), buffer);
    running_total += arc_utilities::SerializeFixedSizePOD<double>(value(1), buffer);

    const uint64_t end_buffer_size = buffer.size();
    const uint64_t bytes_written = end_buffer_size - start_buffer_size;

    assert(running_total == bytes_written);

    return bytes_written;
  };

  // Define the graph value serialization function
  const auto value_deserializer_fn = [](const std::vector<uint8_t>& buffer, const uint64_t current) {
    uint64_t current_position = current;

    // Deserialze 2 doubles
    std::pair<double, uint64_t> x = DeserializeFixedSizePOD<double>(buffer, current_position);
    current_position += x.second;
    std::pair<double, uint64_t> y = DeserializeFixedSizePOD<double>(buffer, current_position);
    current_position += y.second;

    const Eigen::Vector2d deserialized(x.first, y.first);

    // Figure out how many bytes were read
    const uint64_t bytes_read = current_position - current;
    return std::make_pair(deserialized, bytes_read);
  };

  // Serialze the graph
  std::vector<uint8_t> buffer;
  test_graph.serializeSelf(buffer, value_serializer_fn);

  auto deserialized_result = Graph<Eigen::Vector2d>::Deserialize(buffer, 0, value_deserializer_fn);
  auto deserialized_test_graph = deserialized_result.first;

  EXPECT_TRUE(deserialized_test_graph.checkGraphLinkage()) << "Deserialized graph linkage failed";
  ASSERT_EQ(test_graph.getNodes().size(), deserialized_test_graph.getNodes().size())
      << "Test and deserialized graph have different numbers of nodes";

  for (int64_t n_id = 0; n_id < (int64_t)test_graph.getNodes().size(); n_id++) {
    const auto n = test_graph.getNode(n_id);
    const auto other_n = deserialized_test_graph.getNode(n_id);
    EXPECT_EQ(n.getValue(), other_n.getValue()) << "Test and deserialized have different node values";
    ASSERT_EQ(n.getOutEdges().size(), other_n.getOutEdges().size())
        << "Test and deserialized node has different number of out edges";
    ASSERT_EQ(n.getInEdges().size(), other_n.getInEdges().size())
        << "Test and deserialized node has different number of in edges";
    for (int64_t e_id = 0; e_id < (int64_t)n.getOutEdges().size(); e_id++) {
      EXPECT_EQ(n.getOutEdges()[e_id], other_n.getOutEdges()[e_id]) << "Test and deserialized out edge are different";
    }
    for (int64_t e_id = 0; e_id < (int64_t)n.getInEdges().size(); e_id++) {
      EXPECT_EQ(n.getInEdges()[e_id], other_n.getInEdges()[e_id]) << "Test and deserialized in edge are different";
    }
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  // ros::init(argc, argv, "tester");
  // ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
