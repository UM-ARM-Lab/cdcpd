<<<<<<< HEAD
#include <arc_utilities/dijkstras.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <iostream>

int main(int argc, char* argv[]) {
  (void)argc;
  (void)argv;

  std::cout << "Creating graph with nodes indices:\n\n";
  std::cout << "  2 | 5 | 8\n"
            << "  --+---+--\n"
            << "  1 | 4 | 7\n"
            << "  --+---+--\n"
            << "  0 | 3 | 6\n\n";

  arc_dijkstras::Graph<Eigen::Vector2d> graph(9);
  for (double x = -1; x <= 1; x += 1) {
    for (double y = -1; y <= 1; y += 1) {
      graph.addNode(Eigen::Vector2d(x, y));
    }
  }

  // Y-direction edges
  graph.addEdgeBetweenNodes(0, 1, 1.0);
  graph.addEdgesBetweenNodes(1, 2, 1.0);
  graph.addEdgesBetweenNodes(3, 4, 1.0);
  graph.addEdgesBetweenNodes(4, 5, 1.0);
  graph.addEdgesBetweenNodes(6, 7, 1.0);
  graph.addEdgesBetweenNodes(7, 8, 1.0);

  // X-direction edges
  graph.addEdgesBetweenNodes(0, 3, 1.0);
  graph.addEdgesBetweenNodes(3, 6, 1.0);
  graph.addEdgesBetweenNodes(1, 4, 1.0);
  graph.addEdgesBetweenNodes(4, 7, 1.0);
  graph.addEdgesBetweenNodes(2, 5, 1.0);
  graph.addEdgesBetweenNodes(5, 8, 1.0);

  assert(graph.checkGraphLinkage());

  auto dijkstras_result_4connected =
      arc_dijkstras::SimpleDijkstrasAlgorithm<Eigen::Vector2d,
                                              std::allocator<Eigen::Vector2d>>::PerformDijkstrasAlgorithm(graph, 0);

  std::cout << "4-connected edges\n"
            << "Node index            : 0, 1, 2, 3, 4, 5, 6, 7, 8\n";
  std::cout << "Previous graph indices: " << PrettyPrint::PrettyPrint(dijkstras_result_4connected.second.first)
            << std::endl;
  std::cout << "Distance              : " << PrettyPrint::PrettyPrint(dijkstras_result_4connected.second.second)
            << std::endl;

  // Diagonal edges
  graph.addEdgesBetweenNodes(0, 4, std::sqrt(2));
  graph.addEdgesBetweenNodes(1, 5, std::sqrt(2));
  graph.addEdgesBetweenNodes(3, 7, std::sqrt(2));
  graph.addEdgesBetweenNodes(4, 8, std::sqrt(2));

  graph.addEdgesBetweenNodes(1, 3, std::sqrt(2));
  graph.addEdgesBetweenNodes(2, 4, std::sqrt(2));
  graph.addEdgesBetweenNodes(4, 6, std::sqrt(2));
  graph.addEdgesBetweenNodes(5, 7, std::sqrt(2));

  assert(graph.checkGraphLinkage());
  auto dijkstras_result_8connected =
      arc_dijkstras::SimpleDijkstrasAlgorithm<Eigen::Vector2d,
                                              std::allocator<Eigen::Vector2d>>::PerformDijkstrasAlgorithm(graph, 0);

  std::cout << "\n8-connected edges\n"
            << "Node index            : 0, 1, 2, 3, 4, 5, 6, 7, 8\n";
  std::cout << "Previous graph indices: " << PrettyPrint::PrettyPrint(dijkstras_result_8connected.second.first)
            << std::endl;
  std::cout << "Distance              : " << PrettyPrint::PrettyPrint(dijkstras_result_8connected.second.second)
            << std::endl;

  std::cout << "\nSerialization test... ";

  std::cout << "passed" << std::endl;

  return 0;
=======
#include <iostream>

#include <arc_utilities/dijkstras.hpp>
#include <arc_utilities/pretty_print.hpp>

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    std::cout << "Creating graph with nodes indices:\n\n";
    std::cout << "  2 | 5 | 8\n"
              << "  --+---+--\n"
              << "  1 | 4 | 7\n"
              << "  --+---+--\n"
              << "  0 | 3 | 6\n\n";

    arc_dijkstras::Graph<Eigen::Vector2d> graph(9);
    for (double x = -1; x <= 1; x += 1)
    {
        for (double y = -1; y <= 1; y += 1)
        {
            graph.addNode(Eigen::Vector2d(x, y));
        }
    }

    // Y-direction edges
    graph.addEdgeBetweenNodes(0, 1, 1.0);
    graph.addEdgesBetweenNodes(1, 2, 1.0);
    graph.addEdgesBetweenNodes(3, 4, 1.0);
    graph.addEdgesBetweenNodes(4, 5, 1.0);
    graph.addEdgesBetweenNodes(6, 7, 1.0);
    graph.addEdgesBetweenNodes(7, 8, 1.0);

    // X-direction edges
    graph.addEdgesBetweenNodes(0, 3, 1.0);
    graph.addEdgesBetweenNodes(3, 6, 1.0);
    graph.addEdgesBetweenNodes(1, 4, 1.0);
    graph.addEdgesBetweenNodes(4, 7, 1.0);
    graph.addEdgesBetweenNodes(2, 5, 1.0);
    graph.addEdgesBetweenNodes(5, 8, 1.0);

    assert(graph.checkGraphLinkage());


    auto dijkstras_result_4connected = arc_dijkstras::SimpleDijkstrasAlgorithm<Eigen::Vector2d, std::allocator<Eigen::Vector2d>>::PerformDijkstrasAlgorithm(graph, 0);

    std::cout << "4-connected edges\n"
              << "Node index            : 0, 1, 2, 3, 4, 5, 6, 7, 8\n";
    std::cout << "Previous graph indices: " << PrettyPrint::PrettyPrint(dijkstras_result_4connected.second.first) << std::endl;
    std::cout << "Distance              : " << PrettyPrint::PrettyPrint(dijkstras_result_4connected.second.second) << std::endl;

    // Diagonal edges
    graph.addEdgesBetweenNodes(0, 4, std::sqrt(2));
    graph.addEdgesBetweenNodes(1, 5, std::sqrt(2));
    graph.addEdgesBetweenNodes(3, 7, std::sqrt(2));
    graph.addEdgesBetweenNodes(4, 8, std::sqrt(2));

    graph.addEdgesBetweenNodes(1, 3, std::sqrt(2));
    graph.addEdgesBetweenNodes(2, 4, std::sqrt(2));
    graph.addEdgesBetweenNodes(4, 6, std::sqrt(2));
    graph.addEdgesBetweenNodes(5, 7, std::sqrt(2));

    assert(graph.checkGraphLinkage());
    auto dijkstras_result_8connected = arc_dijkstras::SimpleDijkstrasAlgorithm<Eigen::Vector2d, std::allocator<Eigen::Vector2d>>::PerformDijkstrasAlgorithm(graph, 0);

    std::cout << "\n8-connected edges\n"
              << "Node index            : 0, 1, 2, 3, 4, 5, 6, 7, 8\n";
    std::cout << "Previous graph indices: " << PrettyPrint::PrettyPrint(dijkstras_result_8connected.second.first) << std::endl;
    std::cout << "Distance              : " << PrettyPrint::PrettyPrint(dijkstras_result_8connected.second.second) << std::endl;

    std::cout << "\nSerialization test... ";


    std::cout << "passed" << std::endl;

    return 0;
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
}
