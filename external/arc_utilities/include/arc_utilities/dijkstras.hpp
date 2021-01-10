#ifndef DIJKSTRAS_HPP
#define DIJKSTRAS_HPP

#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/graph.hpp>
#include <functional>
#include <limits>
#include <queue>
#include <random>
#include <stdexcept>
#include <unordered_map>

namespace arc_dijkstras {
template <typename NodeValueType, typename Allocator = std::allocator<NodeValueType>>
class SimpleDijkstrasAlgorithm {
 protected:
  class CompareIndexFn {
   public:
    constexpr bool operator()(const std::pair<int64_t, double>& lhs, const std::pair<int64_t, double>& rhs) const {
      return lhs.second > rhs.second;
    }
  };

  SimpleDijkstrasAlgorithm() {}

 public:
  typedef Graph<NodeValueType, Allocator> GraphType;
  typedef std::pair<GraphType, std::pair<std::vector<int64_t>, std::vector<double>>> DijkstrasResult;

  static DijkstrasResult PerformDijkstrasAlgorithm(const GraphType& graph, const int64_t start_index) {
    if ((start_index < 0) || (start_index >= (int64_t)graph.getNodes().size())) {
      throw std::invalid_argument("Start index out of range");
    }
    GraphType working_copy = graph;
    // Setup
    std::vector<int64_t> previous_index_map(working_copy.getNodes().size(), -1);
    std::vector<double> distances(working_copy.getNodes().size(), std::numeric_limits<double>::infinity());
    std::priority_queue<std::pair<int64_t, double>, std::vector<std::pair<int64_t, double>>, CompareIndexFn> queue;
    std::unordered_map<int64_t, uint32_t> explored(graph.getNodes().size());
    for (size_t idx = 0; idx < working_copy.getNodes().size(); idx++) {
      working_copy.getNode((int64_t)idx).setDistance(std::numeric_limits<double>::infinity());
      queue.push(std::make_pair((int64_t)idx, std::numeric_limits<double>::infinity()));
    }
    working_copy.getNode(start_index).setDistance(0.0);
    previous_index_map[(size_t)start_index] = start_index;
    distances[(size_t)start_index] = 0.0;
    queue.push(std::make_pair(start_index, 0.0));
    while (queue.size() > 0) {
      const std::pair<int64_t, double> top_node = queue.top();
      const int64_t& top_node_index = top_node.first;
      const double& top_node_distance = top_node.second;
      queue.pop();
      if (explored[top_node.first] > 0) {
        // We've already been here
        continue;
      } else {
        // Note that we've been here
        explored[top_node.first] = 1;
        // Get our neighbors
        const std::vector<GraphEdge>& neighbor_edges = working_copy.getNode(top_node_index).getInEdges();
        // Go through our neighbors
        for (size_t neighbor_idx = 0; neighbor_idx < neighbor_edges.size(); neighbor_idx++) {
          const int64_t neighbor_index = neighbor_edges[neighbor_idx].getFromIndex();
          const double neighbor_edge_weight = neighbor_edges[neighbor_idx].getWeight();
          const double new_neighbor_distance = top_node_distance + neighbor_edge_weight;
          // Check against the neighbor
          const double stored_neighbor_distance = working_copy.getNode(neighbor_index).getDistance();
          if (new_neighbor_distance < stored_neighbor_distance) {
            // We've found a better way to get to this node
            // Check if it's already been explored
            if (explored[neighbor_index] > 0) {
              // If it's already been explored, we just update it in place
              working_copy.getNode(neighbor_index).setDistance(new_neighbor_distance);
            } else {
              // If it hasn't been explored, we need to update it and add it to the queue
              working_copy.getNode(neighbor_index).setDistance(new_neighbor_distance);
              queue.push(std::make_pair(neighbor_index, new_neighbor_distance));
            }
            // Update that we're the best previous node
            previous_index_map[(size_t)neighbor_index] = top_node_index;
            distances[(size_t)neighbor_index] = new_neighbor_distance;
          } else {
            // Do nothing
            continue;
          }
        }
      }
    }
    return std::make_pair(working_copy, std::make_pair(previous_index_map, distances));
  }

  static uint64_t SerializeDijstrasResult(
      const DijkstrasResult& result, std::vector<uint8_t>& buffer,
      const std::function<uint64_t(const NodeValueType&, std::vector<uint8_t>&)>& value_serializer) {
    const uint64_t start_buffer_size = buffer.size();
    // Serialize the graph
    result.first.SerializeSelf(buffer, value_serializer);
    // Serialize the previous index
    const auto index_serializer =
        std::bind(arc_utilities::SerializeFixedSizePOD<int64_t>, std::placeholders::_1, std::placeholders::_2);
    SerializeVector(result.second.first, index_serializer);
    // Serialze the distances
    const auto distance_serializer =
        std::bind(arc_utilities::SerializeFixedSizePOD<double>, std::placeholders::_1, std::placeholders::_2);
    SerializeVector(result.second.second, distance_serializer);
    // Figure out how many bytes were written
    const uint64_t end_buffer_size = buffer.size();
    const uint64_t bytes_written = end_buffer_size - start_buffer_size;
    return bytes_written;
  }

  static std::pair<DijkstrasResult, uint64_t> DijstrasResult(
      const std::vector<uint8_t>& buffer, const uint64_t current,
      const std::function<std::pair<NodeValueType, uint64_t>(const std::vector<uint8_t>&, const uint64_t)>&
          value_deserializer) {
    using namespace arc_utilities;
    uint64_t current_position = current;
    // Deserialize the graph itself
    std::pair<DijkstrasResult, uint64_t> deserialized;
    const auto graph_deserialized = GraphType::Deserialize(buffer, current_position, value_deserializer);
    deserialized.first.first = graph_deserialized.first;
    current_position += graph_deserialized.second;
    // Deserialize the previous index
    const auto index_deserializer =
        std::bind(DeserializeFixedSizePOD<int64_t>, std::placeholders::_1, std::placeholders::_2);
    const auto prev_index_deserialized = DeserializeVector<int64_t>(buffer, current_position, index_deserializer);
    deserialized.first.second.first = prev_index_deserialized.first;
    current_position += prev_index_deserialized.second;
    // Deserialize the distances
    const auto distance_deserializer =
        std::bind(DeserializeFixedSizePOD<double>, std::placeholders::_1, std::placeholders::_2);
    const auto distance_deserialized = DeserializeVector<double>(buffer, current_position, distance_deserializer);
    deserialized.first.second.second = distance_deserialized.first;
    current_position += distance_deserialized.second;
    // Figure out how many bytes were read
    deserialized.second = current_position - current;
    return deserialized;
  }
};

template <typename NodeValueType, typename Allocator = std::allocator<NodeValueType>>
class SimpleGraphAstar {
 protected:
  SimpleGraphAstar() {}

 public:
  typedef Graph<NodeValueType, Allocator> GraphType;

  static arc_helpers::AstarResult PerformLazyAstar(
      const GraphType& graph, const int64_t start_index, const int64_t goal_index,
      const std::function<bool(const GraphType&, const GraphEdge&)>& edge_validity_check_fn,
      const std::function<double(const GraphType&, const GraphEdge&)>& distance_fn,
      const std::function<double(const NodeValueType&, const NodeValueType&)>& heuristic_fn,
      const bool limit_pqueue_duplicates) {
    using namespace arc_helpers;
    // Enforced sanity checks
    if ((start_index < 0) || (start_index >= (int64_t)graph.getNodes().size())) {
      throw std::invalid_argument("Start index out of range");
    }
    if ((goal_index < 0) || (goal_index >= (int64_t)graph.getNodes().size())) {
      throw std::invalid_argument("Goal index out of range");
    }
    if (start_index == goal_index) {
      throw std::invalid_argument("Start and goal indices must be different");
    }
    // Make helper function
    const auto heuristic_function = [&](const int64_t node_index) {
      return heuristic_fn(graph.getNode(node_index).getValue(), graph.getNode(goal_index).getValue());
    };
    // Setup
    std::priority_queue<AstarPQueueElement, std::vector<AstarPQueueElement>, CompareAstarPQueueElementFn> queue;

    // Optional map to reduce the number of duplicate items added to the pqueue
    // Key is the node index in the provided graph
    // Value is cost-to-come
    std::unordered_map<int64_t, double> queue_members_map;

    // Key is the node index in the provided graph
    // Value is a pair<backpointer, cost-to-come>
    // backpointer is the parent index in the provided graph
    std::unordered_map<int64_t, std::pair<int64_t, double>> explored;

    // Initialize
    queue.push(AstarPQueueElement(start_index, -1, 0.0, heuristic_function(start_index)));
    if (limit_pqueue_duplicates) {
      queue_members_map[start_index] = 0.0;
    }

    // Search
    while (queue.size() > 0) {
      // Get the top of the priority queue
      const arc_helpers::AstarPQueueElement n = queue.top();
      queue.pop();

      if (n.id() == goal_index) {
        // Solution found
        explored[n.id()] = std::make_pair(n.backpointer(), n.costToCome());
        break;
      }

      if (limit_pqueue_duplicates) {
        queue_members_map.erase(n.id());
      }

      if (explored.count(n.id()) && n.costToCome() >= explored[n.id()].second) {
        continue;
      }

      // Add to the explored list
      explored[n.id()] = std::make_pair(n.backpointer(), n.costToCome());

      // Explore and add the children
      for (const GraphEdge& current_out_edge : graph.getNode(n.id()).getOutEdges()) {
        // Get the next potential child node
        const int64_t child_id = current_out_edge.getToIndex();

        if (!edge_validity_check_fn(graph, current_out_edge)) {
          continue;
        }

        // Compute the cost-to-come for the new child
        const double child_cost_to_come = n.costToCome() + distance_fn(graph, current_out_edge);

        if (explored.count(child_id) && child_cost_to_come >= explored[child_id].second) {
          continue;
        }

        if (limit_pqueue_duplicates && queue_members_map.count(child_id) &&
            child_cost_to_come >= queue_members_map[child_id]) {
          continue;
        }

        const double child_value = child_cost_to_come + heuristic_function(child_id);
        queue.push(AstarPQueueElement(child_id, n.id(), child_cost_to_come, child_value));
      }
    }
    return ExtractAstarResult(explored, start_index, goal_index);
  }

  static arc_helpers::AstarResult PerformLazyAstar(
      const GraphType& graph, const int64_t start_index, const int64_t goal_index,
      const std::function<bool(const NodeValueType&, const NodeValueType&)>& edge_validity_check_fn,
      const std::function<double(const NodeValueType&, const NodeValueType&)>& distance_fn,
      const std::function<double(const NodeValueType&, const NodeValueType&)>& heuristic_fn,
      const bool limit_pqueue_duplicates) {
    const auto edge_validity_check_function = [&](const GraphType& search_graph, const GraphEdge& edge) {
      return edge_validity_check_fn(search_graph.getNode(edge.getFromIndex()).getValue(),
                                    search_graph.getNode(edge.getToIndex()).getValue());
    };
    const auto distance_function = [&](const GraphType& search_graph, const GraphEdge& edge) {
      return distance_fn(search_graph.getNode(edge.getFromIndex()).getValue(),
                         search_graph.getNode(edge.getToIndex()).getValue());
    };
    return PerformLazyAstar(graph, start_index, goal_index, edge_validity_check_function, distance_function,
                            heuristic_fn, limit_pqueue_duplicates);
  }

  static arc_helpers::AstarResult PerformAstar(
      const GraphType& graph, const int64_t start_index, const int64_t goal_index,
      const std::function<double(const NodeValueType&, const NodeValueType&)>& heuristic_fn,
      const bool limit_pqueue_duplicates) {
    const auto edge_validity_check_function = [&](const GraphType& search_graph, const GraphEdge& edge) {
      UNUSED(search_graph);
      if (edge.getValidity() == EDGE_VALIDITY::INVALID) {
        return false;
      }

      return edge.getWeight() < std::numeric_limits<double>::infinity();
    };
    const auto distance_function = [&](const GraphType& search_graph, const GraphEdge& edge) {
      UNUSED(search_graph);
      return edge.getWeight();
    };
    return PerformLazyAstar(graph, start_index, goal_index, edge_validity_check_function, distance_function,
                            heuristic_fn, limit_pqueue_duplicates);
  }
};

template <typename NodeValueType, typename Allocator = std::allocator<NodeValueType>>
class GraphRandomWalk {
 protected:
  GraphRandomWalk() {}

 public:
  typedef Graph<NodeValueType, Allocator> GraphType;
  template <typename Generator>
  static std::vector<int64_t> PerformRandomWalk(const GraphType& graph, const int64_t start_index,
                                                const int64_t goal_index, Generator& generator) {
    std::uniform_int_distribution<int64_t> uniform_int_distribution;

    std::vector<int64_t> path(1, start_index);

    while (path.back() != goal_index) {
      // Collect data from the current node
      const int64_t curr_index = path.back();
      const auto& out_edges = graph.getNode(curr_index).getOutEdges();
      const auto num_edges = out_edges.size();

      // Determine which node to step to next
      std::uniform_int_distribution<int64_t>::param_type params(0, (int64_t)(num_edges - 1));
      uniform_int_distribution.param(params);
      const int64_t next_step = uniform_int_distribution(generator);
      const auto next_index = out_edges.at(next_step).getToIndex();

      // If the next index is somewhere we've been already, then "trim" the loop off
      const auto it = std::find(path.begin(), path.end(), next_index);
      path.erase(it, path.end());

      // (Re)add the new index to the path
      path.push_back(next_index);
    }

    return path;
  }
};
}  // namespace arc_dijkstras

#endif  // DIJKSTRAS_HPP
